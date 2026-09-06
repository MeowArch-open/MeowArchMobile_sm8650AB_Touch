#!/usr/bin/env python3
"""Volume-down + power = screenshot, the way Android does it.

Why a daemon and not a KDE global shortcut: KDE's shortcut system binds a
modifier plus a key, and neither volume-down nor power is a modifier. The two
keys also arrive on *separate* input devices here -- the PMIC exposes pwrkey and
resin as one device each, quite apart from gpio-keys -- so nothing but a reader
watching several descriptors at once can see the chord at all.

How the power key is handled, which is the part with a real trade-off:

  * pwrkey is grabbed with EVIOCGRAB, so logind and powerdevil never see it.
    That is deliberate and load-bearing: logind's default HandlePowerKey is
    poweroff, so without the grab the first press of this chord would shut the
    phone down. Grabbing means no logind.conf edit is needed, but it also means
    THIS daemon now owns every power-key press -- hence --single-press below.
    If the daemon dies, the grab is released and logind's poweroff comes back,
    so the unit restarts itself.

  * resin (volume down) is NOT grabbed, so Plasma still gets it and the volume
    still goes down. The visible cost is that taking a screenshot also drops the
    volume one notch. Grabbing it instead would mean re-injecting volume keys
    through uinput to keep the volume working, which is a lot of machinery for a
    cosmetic gain.

Keys are found by keycode, not by device name, because the name a PMIC key
driver picks is not stable across kernels while KEY_POWER and KEY_VOLUMEDOWN are.

Runs as a --user unit: it needs the session's D-Bus to talk to Spectacle, and
the user is in the `input` group so no privileges are required.
"""

import fcntl
import glob
import os
import select
import struct
import subprocess
import sys
import time

EV_KEY = 0x01
KEY_VOLUMEDOWN = 114
KEY_POWER = 116

EVENT_FMT = 'qqHHi'
EVENT_SIZE = struct.calcsize(EVENT_FMT)


def _ioc(direction, typ, nr, size):
    """Compute the ioctl numbers rather than hardcoding them. evdev's EVIOCGNAME
    handler trusts _IOC_SIZE(cmd) for how much it may write, so a hardcoded
    constant that disagrees with the buffer actually passed is a buffer overrun
    waiting for a long device name."""
    return (direction << 30) | (size << 16) | (ord(typ) << 8) | nr


NAME_LEN = 256
KEYBITS_LEN = 0x2ff // 8 + 1

EVIOCGNAME = _ioc(2, 'E', 0x06, NAME_LEN)
EVIOCGRAB = _ioc(1, 'E', 0x90, 4)


def eviocgbit(ev, nbytes):
    return _ioc(2, 'E', 0x20 + ev, nbytes)

SCREENSHOT = ['spectacle', '--fullscreen', '--background']
SINGLE_PRESS = ['loginctl', 'lock-session']


def device_keys(fd):
    """Keycodes a device can emit, via EVIOCGBIT."""
    buf = bytearray(KEYBITS_LEN)
    try:
        fcntl.ioctl(fd, eviocgbit(EV_KEY, KEYBITS_LEN), buf)
    except OSError:
        return set()
    return {c for c in range(0x2ff) if (buf[c // 8] >> (c % 8)) & 1}


def device_name(fd):
    buf = bytearray(NAME_LEN)
    try:
        fcntl.ioctl(fd, EVIOCGNAME, buf)
    except OSError:
        return '?'
    return buf.split(b'\0')[0].decode(errors='replace')


def find_devices():
    """Map keycode -> (path, fd, name) for the two keys we care about. A device
    carrying both would be handled fine; here they are always separate."""
    found = {}
    for path in sorted(glob.glob('/dev/input/event*')):
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue
        keys = device_keys(fd)
        wanted = keys & {KEY_POWER, KEY_VOLUMEDOWN}
        if not wanted:
            os.close(fd)
            continue
        name = device_name(fd)
        for code in wanted:
            found[code] = (path, fd, name)
        print(f"  {path} {name!r} -> {sorted(wanted)}", flush=True)
    return found


def run(cmd, what):
    print(f"  {what}: {' '.join(cmd)}", flush=True)
    try:
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as e:
        print(f"  {what} failed: {e}", file=sys.stderr, flush=True)


def main():
    single_press = SINGLE_PRESS
    if '--no-single-press' in sys.argv:
        single_press = None

    print("=== looking for the power and volume-down devices ===", flush=True)
    devs = find_devices()
    for code, label in ((KEY_POWER, 'power'), (KEY_VOLUMEDOWN, 'volume down')):
        if code not in devs:
            sys.exit(f"no device reports {label} (keycode {code}) -- is the DT "
                     "with pwrkey/resin enabled actually flashed?")

    pwr_path, pwr_fd, _ = devs[KEY_POWER]
    vol_path, vol_fd, _ = devs[KEY_VOLUMEDOWN]

    # Grab only the power key. See the module docstring for why this asymmetry.
    try:
        fcntl.ioctl(pwr_fd, EVIOCGRAB, 1)
        print(f"  grabbed {pwr_path} (logind and powerdevil will not see it)", flush=True)
    except OSError as e:
        sys.exit(f"cannot grab {pwr_path}: {e} -- refusing to run, because an "
                 "ungrabbed power key means logind powers the phone off on the "
                 "first chord")

    fds = {pwr_fd, vol_fd}
    vol_down_held = False
    # A chord fires once; the power key has to be released before it can fire
    # again, otherwise holding both would spray screenshots.
    armed = True
    last_shot = 0.0

    print("=== watching: volume-down + power = screenshot ===", flush=True)
    while True:
        ready, _, _ = select.select(list(fds), [], [], None)
        for fd in ready:
            try:
                data = os.read(fd, EVENT_SIZE * 64)
            except BlockingIOError:
                continue
            except OSError as e:
                sys.exit(f"read error, letting systemd restart us: {e}")

            for i in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                _, _, etype, code, value = struct.unpack(
                    EVENT_FMT, data[i:i + EVENT_SIZE])
                if etype != EV_KEY:
                    continue

                if code == KEY_VOLUMEDOWN:
                    # value 1 = press, 2 = autorepeat, 0 = release
                    vol_down_held = value != 0
                    continue

                if code != KEY_POWER:
                    continue

                if value == 0:
                    armed = True
                    continue
                if value != 1 or not armed:
                    continue

                armed = False
                now = time.monotonic()
                if vol_down_held:
                    if now - last_shot < 1.0:
                        continue
                    last_shot = now
                    run(SCREENSHOT, "screenshot")
                elif single_press:
                    run(single_press, "power key alone")


if __name__ == '__main__':
    main()
