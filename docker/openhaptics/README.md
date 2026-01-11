# OpenHaptics SDK Files (Backup)

These SDK files are backup copies from the JHU repository.

## Source

**Original Repository**: https://github.com/jhu-cisst-external/phantom-omni-1394-drivers

The files in this directory are copies of those provided by Johns Hopkins University (JHU)
for the Phantom Omni FireWire driver installation.

## Files

| File | Description |
|------|-------------|
| `OpenHapticsAE_Linux_v3_0.zip` | OpenHaptics Academic Edition 3.0 SDK |
| `Linux_JUJU_PDD_64-bit.tgz` | JUJU FireWire driver (replaces vendor's libPHANToMIO) |

## Why These Files?

- **OpenHaptics 3.4** (current version) only supports USB devices (Geomagic Touch)
- **OpenHaptics AE 3.0** uses `libPHANToMIO.so` which works with FireWire devices
- **JUJU driver** uses `libraw1394.so.11` (modern Linux firewire-core stack)

## Configuration

Set `OPENHAPTICS_SDK_DIR` in `.env.local` to specify SDK location:

```bash
# Option 1: Use JHU repository (recommended)
OPENHAPTICS_SDK_DIR=/path/to/phantom-omni-1394-drivers/files

# Option 2: Use this backup directory (default)
# OPENHAPTICS_SDK_DIR=openhaptics
```

## License

- OpenHaptics AE is the Academic Edition - commercial use requires separate licensing
- JUJU driver origin is documented in the JHU repository

## References

- [JHU Phantom Omni 1394 Drivers](https://github.com/jhu-cisst-external/phantom-omni-1394-drivers)
- [sawSensablePhantom](https://github.com/jhu-saw/sawSensablePhantom)
- [3D Systems OpenHaptics](https://www.3dsystems.com/haptics-devices/openhaptics)
