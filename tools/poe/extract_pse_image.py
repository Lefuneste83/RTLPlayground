#!/usr/bin/env python3
"""
extract_pse_image.py - find and extract Realtek PSE (PoE controller) firmware
images from a switch's OEM firmware.

VERSION = "1.0.0"

Realtek RTL8238B / RTL8238C PoE controllers run a volatile firmware image that
the switch uploads over I2C at every power-on. The OEM switch firmware carries
that image (sometimes one per supported PSE chip); RTLPlayground does not ship
it, because it is Realtek code. This tool finds the image(s) in a file you
already have, verifies them, and writes them out. See doc/poe.md.

Input: any of
  - an OEM update file as downloaded from the vendor (e.g. *_V100.9.9_*.bin),
  - a raw dump of the switch's SPI flash (e.g. read with a SOIC clip).
The images are found by their container header, not by offset or size, so the
file layout and firmware version do not matter.

Container format (little-endian), as found in all files seen so far:
  +0   u16  0x0000
  +2   u16  0x8239            signature
  +4   u16  chip id           0x000f = RTL8238B, 0x000b = RTL8238C
  +6   u16[4] section lengths
  +14  u16  0x0000
  +16  sections, then a 4-byte trailer
  total length = 16 + sum(sections) + 4, a multiple of 4; the trailer makes the
  whole container sum to zero over its little-endian 32-bit words.

Usage:
  python3 tools/poe/extract_pse_image.py OEM_FILE [OEM_FILE ...]
        list the images found and write each valid one to stock_fw/
        as pse_rtl8238b.bin / pse_rtl8238c.bin (pse_chip_XXXX.bin if unknown)
  ... --use rtl8238b      also copy that chip's image to stock_fw/pse_image.bin,
                          the file the build embeds
  ... --list              only list, write nothing
  ... -o DIR              write to DIR instead of stock_fw/
  ... --force             overwrite existing files with different content
  python3 tools/poe/extract_pse_image.py --selftest
        check the parser on synthetic data (no firmware needed)
"""

import argparse
import hashlib
import os
import struct
import sys

SIGNATURE = b'\x00\x00\x39\x82'
HEADER_LEN = 16
TRAILER_LEN = 4
MAX_LEN = 0x10000

CHIPS = {
    0x000f: 'rtl8238b',
    0x000b: 'rtl8238c',
}

# Images seen so far, identified by the SHA-256 of the whole container.
# Only hashes are listed here; the images themselves are never part of the repo.
KNOWN = {
    '95d687e2a03d2d916a72319385eea67f030022df9b374059b12e7042b58cc5c1':
        ('rtl8238b', 21164, 'keepLink KP-9000-9XHPML-X-AC V100.9.5; '
                            'ZX-AFGW-SWTG218ANS V100.9.9 and V200.1.30'),
    '401d91bd543feb573ef9ecb99644c456c6f382cecbfcc346ffe89ea4f0d5676b':
        ('rtl8238c', 8028, 'keepLink KP-9000-9XHPML-X-AC V100.9.5; '
                           'ZX-AFGW-SWTG218ANS V200.1.30'),
}


def chip_name(chip_id):
    return CHIPS.get(chip_id, 'chip_%04x' % chip_id)


def find_images(data):
    """Yield dicts describing every container candidate in data."""
    pos = data.find(SIGNATURE)
    while pos >= 0:
        img = parse_at(data, pos)
        if img:
            yield img
        pos = data.find(SIGNATURE, pos + 1)


def parse_at(data, pos):
    if pos + HEADER_LEN > len(data):
        return None
    chip_id, s0, s1, s2, s3, pad = struct.unpack_from('<6H', data, pos + 4)
    length = HEADER_LEN + s0 + s1 + s2 + s3 + TRAILER_LEN
    if pad != 0 or length > MAX_LEN or length % 4 or pos + length > len(data):
        return None
    blob = data[pos:pos + length]
    checksum_ok = sum(struct.unpack('<%dI' % (length // 4), blob)) & 0xffffffff == 0
    sha = hashlib.sha256(blob).hexdigest()
    return {
        'offset': pos, 'chip_id': chip_id, 'chip': chip_name(chip_id),
        'length': length, 'checksum_ok': checksum_ok, 'sha256': sha,
        'known': KNOWN.get(sha), 'blob': blob,
    }


def describe(img):
    state = 'checksum OK' if img['checksum_ok'] else 'BAD CHECKSUM'
    if img['known']:
        origin = 'known image'
    elif img['checksum_ok']:
        origin = 'new image, not in the known list'
    else:
        origin = 'not usable'
    return ('  0x%07x  %-9s (id 0x%04x)  %5d bytes  %s  %s\n'
            '             sha256 %s'
            % (img['offset'], img['chip'], img['chip_id'], img['length'], state,
               origin, img['sha256']))


def write_file(path, blob, force):
    if os.path.exists(path):
        with open(path, 'rb') as f:
            if f.read() == blob:
                print('  %s: already up to date' % path)
                return True
        if not force:
            print('  %s exists with different content; use --force to replace it' % path)
            return False
    with open(path, 'wb') as f:
        f.write(blob)
    print('  wrote %s (%d bytes)' % (path, len(blob)))
    return True


def selftest():
    def container(chip_id, sections):
        body = b''.join(bytes((i * 7 + n) & 0xff for i in range(n)) for n in sections)
        head = SIGNATURE + struct.pack('<6H', chip_id, *sections, 0)
        partial = head + body + b'\0' * TRAILER_LEN
        s = sum(struct.unpack('<%dI' % (len(partial) // 4), partial)) & 0xffffffff
        return head + body + struct.pack('<I', (-s) & 0xffffffff)

    a = container(0x000f, [100, 20, 8, 4])
    b = container(0x000b, [40, 0, 0, 12])
    bad = bytearray(container(0x000f, [8, 8, 8, 8]))
    bad[20] ^= 0xff
    data = b'\xff' * 37 + a + b'\x00\x00\x39\x82garbage' + b + bytes(bad) + b'\xff' * 5
    imgs = list(find_images(data))
    assert [(i['chip'], i['length'], i['checksum_ok']) for i in imgs] == [
        ('rtl8238b', len(a), True), ('rtl8238c', len(b), True), ('rtl8238b', len(bad), False)], imgs
    assert imgs[0]['blob'] == a and imgs[1]['blob'] == b
    print('selftest OK: %d containers found, checksum check works' % len(imgs))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('files', nargs='*', help='OEM update file(s) or raw flash dump(s)')
    ap.add_argument('-o', '--outdir', default='stock_fw', help='output directory (default: stock_fw)')
    ap.add_argument('--use', metavar='CHIP', help='also copy this chip\'s image to <outdir>/pse_image.bin '
                    '(rtl8238b or rtl8238c: read the marking on your PoE controller)')
    ap.add_argument('--list', action='store_true', help='only list the images, write nothing')
    ap.add_argument('--force', action='store_true', help='overwrite files with different content')
    ap.add_argument('--selftest', action='store_true', help='test the parser on synthetic data')
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return 0
    if not args.files:
        ap.error('give at least one OEM firmware file')

    found = {}
    for path in args.files:
        with open(path, 'rb') as f:
            data = f.read()
        imgs = list(find_images(data))
        print('%s: %d bytes, %d PSE image(s)' % (path, len(data), len(imgs)))
        for img in imgs:
            print(describe(img))
            if img['checksum_ok']:
                found.setdefault(img['chip'], {})[img['sha256']] = img

    if not found:
        print('No valid PSE image found. This file may not contain one (PoE handled '
              'in hardware, or a different PSE vendor), or it is packed/encrypted.')
        return 1

    conflict = [c for c, v in found.items() if len(v) > 1]
    for c in conflict:
        print('Warning: several different %s images found; nothing written for that chip. '
              'Run on one file at a time.' % c)

    if args.list:
        return 0

    os.makedirs(args.outdir, exist_ok=True)
    ok = True
    print('Writing to %s/:' % args.outdir)
    for chip, imgs in sorted(found.items()):
        if chip in conflict:
            continue
        img = next(iter(imgs.values()))
        ok &= write_file(os.path.join(args.outdir, 'pse_%s.bin' % chip), img['blob'], args.force)

    if args.use:
        use = args.use.lower()
        if use not in found or use in conflict:
            print('No single %s image found; pse_image.bin not written.' % use)
            return 1
        img = next(iter(found[use].values()))
        ok &= write_file(os.path.join(args.outdir, 'pse_image.bin'), img['blob'], args.force)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
