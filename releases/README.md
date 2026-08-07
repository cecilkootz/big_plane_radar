# Release Artifacts

This directory is populated by local builds. Use `bash build_arduino_highperf.sh`
for release-equivalent builds or `UPLOAD=1 bash build_arduino_highperf.sh` for
direct flashing.

The expected release files are:

- `big_plane_radar.ino.merged.bin`

Use `big_plane_radar.ino.merged.bin` for browser-based flashing tools at offset
`0x0`.

Manual flashing example:

```sh
esptool.py --chip esp32s3 --port /dev/cu.usbmodemXXXX --baud 921600 \
  write_flash 0x0 releases/big_plane_radar.ino.merged.bin
```
