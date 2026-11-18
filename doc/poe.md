# PoE on RTL837x switches (Realtek RTL8238B / RTL8238C)

Many RTL8372/RTL8373 PoE switches power their ports with a Realtek RTL8238B or RTL8238C PSE controller. The PSE has no flash of its own. At every power-on, the switch firmware uploads a firmware image to it over I2C. Without that upload, the ports stay unpowered. This is why PoE goes dead after the switch is flashed with RTLPlayground and the stock firmware is gone.

RTLPlayground performs the same upload. The image itself is Realtek code, so it is not part of this repository. You extract it from your switch's own OEM firmware with `tools/poe/extract_pse_image.py`, and the build embeds it into your image.

## 1. Check that your switch uses this PSE

- Open the case and read the marking on the PoE controller(s), usually near the port magnetics: `RTL8238B` or `RTL8238C`. Note which one; you need it in step 3.
- Boards supported so far: see the machine files that define `POE_CHIP_RTL8238` in `machine/`. Their `.poe` entry in the PCB file gives the I2C addresses (0x20/0x21), the port count and the power budget.
- The driver has been tested on an RTL8238B board (ZX-AFGW-SWTG218ANS-150, PCB AFG-SWTG118ANS V1.2). RTL8238C boards use the same upload sequence in the OEM firmware, but have not been tested with this driver yet. Reports are welcome.

## 2. Get the OEM firmware

Any of these works:

- The vendor's update file for your model, as offered on the vendor's site or by the seller (for example `..._V100.9.9_ZX-AFGW-SWTG218ANS_1018.bin`).
- A dump of the switch's SPI flash, made before flashing RTLPlayground (with a SOIC clip and a programmer, or one saved earlier).

The tool finds the image by its header, so the file layout and the firmware version do not matter. Many OEM firmwares carry both a B and a C image and pick one at run time.

## 3. Extract the image

From the repository root:

```
python3 tools/poe/extract_pse_image.py path/to/oem_firmware.bin --use rtl8238b
```

Use `--use rtl8238c` if your controller is a C. The tool:

- lists every image found, with its offset, chip, size and SHA-256;
- checks each image's checksum (the container's 32-bit little-endian words sum to zero);
- marks each image as a known one or as a new one;
- writes `stock_fw/pse_rtl8238b.bin` and/or `stock_fw/pse_rtl8238c.bin`;
- with `--use`, also copies the chosen one to `stock_fw/pse_image.bin`, the file the build embeds.

Example with the V200.1.30 firmware of the ZX-AFGW-SWTG218ANS:

```
...V200.1.30_ZX-AFGW-SWTG218ANS_0302.bin: 1029076 bytes, 2 PSE image(s)
  0x00e72f8  rtl8238b  (id 0x000f)  21164 bytes  checksum OK  known image
             sha256 95d687e2a03d2d916a72319385eea67f030022df9b374059b12e7042b58cc5c1
  0x00ec5a4  rtl8238c  (id 0x000b)   8028 bytes  checksum OK  known image
             sha256 401d91bd543feb573ef9ecb99644c456c6f382cecbfcc346ffe89ea4f0d5676b
```

Other options: `--list` only lists; `-o DIR` writes elsewhere; `--force` replaces existing files with different content; `--selftest` checks the parser without any firmware.

So far the same two images have been found in every firmware examined:

| Chip     | Size    | SHA-256 (first 16 hex) | Found in |
|----------|---------|------------------------|----------|
| RTL8238B | 21164 B | `95d687e2a03d2d91`     | ZX-AFGW-SWTG218ANS V100.9.9 and V200.1.30, keepLink KP-9000-9XHPML-X-AC V100.9.5 |
| RTL8238C | 8028 B  | `401d91bd543feb57`     | ZX-AFGW-SWTG218ANS V200.1.30, keepLink KP-9000-9XHPML-X-AC V100.9.5 |

If the tool reports a new image with a good checksum, it will most likely work too. Please report it (vendor, model, firmware version, chip and hash) so it can be added to the list.

If no image is found, the firmware may not drive a Realtek PSE (another PSE vendor, or PoE managed in hardware), or the file is compressed or encrypted.

### Container format

As seen in all files so far, little-endian:

| Offset | Size  | Content |
|--------|-------|---------|
| 0      | 2     | `00 00` |
| 2      | 2     | `39 82` signature |
| 4      | 2     | chip ID: `0x000f` RTL8238B, `0x000b` RTL8238C |
| 6      | 4 × 2 | section lengths |
| 14     | 2     | `00 00` |
| 16     | …     | sections, then a 4-byte trailer that makes the container's 32-bit words sum to zero |

Total length = 16 + sum of the section lengths + 4.

## 4. Build and flash

Build as usual for your machine, for example:

```
make MACHINE=REALHD_PSW8_25MG_V1_2
```

For a machine that defines `POE_CHIP_*`, the Makefile embeds `stock_fw/pse_image.bin` at 0x60000 (override with `PSE_IMAGE=...`). The web UI then has to end below that address. If the file is missing, the build prints `WARNING: stock_fw/pse_image.bin missing - PoE firmware NOT embedded, PoE will stay off` and the switch runs without PoE.

## 5. Check that it works

On a cold boot, the serial console shows:

```
PoE: PSE image 21164 bytes, OK, uploading (takes a few seconds)
PoE: download status ... => ACCEPTED
PoE: ports enabled, mask 0x...
```

On a warm reboot (`reset`), the PSE keeps power and keeps running, so the upload is skipped and powered devices are not power-cycled:

```
PoE: PSE firmware already running, kept (no power cycle)
```

Without a serial console, send `poe info` and `poe status` over HTTP to the `/cmd` endpoint; the answer comes back in the response.

## 6. Use

Console and `/cmd` commands:

| Command | Effect |
|---------|--------|
| `poe status` | per-port admin state, power, class, voltage, current |
| `poe info` | image and boot diagnostics |
| `poe port N on\|off` | enable or disable PoE on port N |
| `poe port N reset` | power-cycle port N |
| `poe watchdog N <1-40\|off>` | power-cycle port N when it is powered but has had no Ethernet link for that many minutes |
| `poe global on\|off` | all ports |
| `poe load` | force a new upload (power-cycles every powered device) |
| `poe poll on\|off` | periodic host poll of the PSE, off by default |

The web UI shows a PoE box on the dashboard (budget, total power, per-port state) and PoE controls on the Ports page. Disabled ports and watchdog settings are saved with the configuration as `poe port N off` and `poe watchdog N M` lines.

## 7. Troubleshooting

| Console output | Likely cause |
|----------------|--------------|
| `NOT FOUND or invalid at 0x60000` | `stock_fw/pse_image.bin` was missing at build time, or is not a PSE image. |
| `0xf4 NACK` or `no ACK from PSE address` | Wrong I2C pins or addresses in the board's `.poe` entry, or no PSE on this board. |
| `=> REJECTED` | The image does not match the chip: an RTL8238C image on an RTL8238B, or the reverse. Check the marking and rerun the tool with the other `--use`. |
| Ports power the wrong devices | Channel order differs from port order on this board. Set `ch_rev` in the board's `.poe` entry. |

## Licence

The PSE images are Realtek firmware, distributed inside the vendors' switch firmware. Do not commit them to this repository or attach them to issues. Share the SHA-256 instead, which the tool prints.
