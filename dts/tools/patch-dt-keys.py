#!/usr/bin/env python3
"""Give zorn its power key and volume-down key.

Runs on the output of patch-dt-touch.py, so all three sets of changes compose:

    ./patch-dt-display.py     # -> zorn-display.dtb
    ./patch-dt-touch.py       # -> zorn-touch.dtb
    ./patch-dt-keys.py        # -> zorn-keys.dtb

Today the kernel sees exactly one key. EVIOCGBIT on every event device reports:

    /dev/input/event0 'gpio-keys'  keycodes: [115] -> VOLUMEUP

because gpio-keys in the DT has a single child, key-volume-up. The other two
keys are not on GPIOs at all -- they hang off the PMIC's PON block, which the
booted DT describes but leaves switched off:

    pon@1300 (qcom,pmk8350-pon)
        pwrkey  linux,code = <0x74>   status = "disabled"
        resin                         status = "disabled"   <- no linux,code

So two changes:

  1. pwrkey status disabled -> okay. It already carries linux,code = 0x74
     (KEY_POWER), so nothing else is needed.

  2. resin status disabled -> okay, plus linux,code = <0x72> (KEY_VOLUMEDOWN).
     RESIN is the PMIC's reset-input line; on this phone, as on every Qualcomm
     reference board, volume-down is wired to it -- that is what makes the
     volume-down + power EDL combination work in the bootloader. The stock
     Android DT says linux,code = <0x72> for it, and upstream's own
     sm8650-hdk.dts / sm8650-qrd.dts both do exactly:
         &pon_resin { linux,code = <KEY_VOLUMEDOWN>; status = "okay"; };

CONFIG_INPUT_PM8941_PWRKEY=y is already built in and provides both
qcom,pmk8350-pwrkey and qcom,pmk8350-resin, so no module work either.

Note the two keys arrive as their own input devices, separate from gpio-keys --
anything watching for a chord has to read all of them at once.
"""

import argparse
import importlib.util
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

KEY_VOLUMEDOWN = 0x72
KEY_POWER = 0x74

PON_NODE = 'pon@1300 {'
PWRKEY_NODE = 'pwrkey {'
RESIN_NODE = 'resin {'


def load_display_helpers():
    """Reuse patch-dt-display.py's node machinery rather than a second, subtly
    different copy of it. Only the definitions above main() are run."""
    path = HERE / 'patch-dt-display.py'
    src = path.read_text().split('def main()')[0]
    spec = importlib.util.spec_from_loader('dtdisp', loader=None)
    mod = importlib.util.module_from_spec(spec)
    mod.__file__ = str(path)
    exec(compile(src, str(path), 'exec'), mod.__dict__)
    return mod


D = load_display_helpers()


def add_keycode(s: str, node: str, code: int, what: str) -> str:
    """Add linux,code to a node that has none. resin needs this; pwrkey has it."""
    i, j = D.node_span(s, node)
    head = D.own_props(s, i, j)
    m = re.search(r'linux,code = <([^>]*)>;', head)
    if m:
        have = int(m.group(1), 0)
        if have == code:
            print(f"  {what}: already has linux,code = {code:#x}")
            return s
        sys.exit(f"{what} already has linux,code = {have:#x}, expected {code:#x} "
                 "-- refusing to guess which one is right")

    # Put it next to compatible, where dtc would have emitted it.
    mc = re.search(r'(\n([\t ]*)compatible = [^;]*;)', head)
    if not mc:
        sys.exit(f"{what}: no compatible property to anchor on")
    indent = mc.group(2)
    head_new = head[:mc.end()] + f'\n{indent}linux,code = <{code:#x}>;' + head[mc.end():]
    print(f"  {what}: added linux,code = {code:#x}")
    return s[:i] + head_new + s[i:j][len(head):] + s[j:]


def verify(s: str) -> None:
    """Both keys must end up enabled and carrying the right keycode, because a
    resin left without linux,code binds fine and then reports nothing at all --
    a silent failure that looks identical to a dead button."""
    print("=== verifying ===")
    expect = {PWRKEY_NODE: ('power key', KEY_POWER),
              RESIN_NODE: ('volume down (resin)', KEY_VOLUMEDOWN)}
    bad = False
    for node, (what, code) in expect.items():
        i, j = D.node_span(s, node)
        head = D.own_props(s, i, j)
        ok_status = 'status = "okay"' in head
        m = re.search(r'linux,code = <([^>]*)>;', head)
        got = int(m.group(1), 0) if m else None
        ok_code = got == code
        bad |= not (ok_status and ok_code)
        print(f"  {what:22s} status={'okay' if ok_status else 'NOT OKAY'}  "
              f"linux,code={got if got is None else hex(got)}"
              f"{'' if ok_code else f'  EXPECTED {code:#x}'}")
    # And they must still be inside the PON block, not spliced somewhere else.
    i, j = D.node_span(s, PON_NODE)
    for node, (what, _) in expect.items():
        if node not in s[i:j]:
            sys.exit(f"{what} is not inside pon@1300 any more")
    print("  both are children of pon@1300")
    if bad:
        sys.exit("refusing to write a tree where a key would bind but report nothing")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dtb', default=str(HERE / 'zorn-touch.dtb'),
                    help='input dtb (default: zorn-touch.dtb)')
    ap.add_argument('--out', default='zorn-keys.dtb')
    args = ap.parse_args()

    print(f"=== decompiling {args.dtb} ===")
    dts = D.sh('dtc', '-I', 'dtb', '-O', 'dts', args.dtb).stdout.decode()

    print("=== patching ===")
    dts = D.enable(dts, PWRKEY_NODE, 'pwrkey (KEY_POWER)')
    dts = D.enable(dts, RESIN_NODE, 'resin (KEY_VOLUMEDOWN)')
    dts = add_keycode(dts, RESIN_NODE, KEY_VOLUMEDOWN, 'resin')

    verify(dts)

    out_dts = Path(args.out).with_suffix('.dts')
    out_dts.write_text(dts)

    print(f"=== recompiling -> {args.out} ===")
    r = subprocess.run(['dtc', '-I', 'dts', '-O', 'dtb', '-o', args.out, str(out_dts)],
                       capture_output=True)
    if r.returncode:
        sys.exit(r.stderr.decode())

    print(f"\nWrote {args.out} ({Path(args.out).stat().st_size} bytes) and {out_dts}")
    print("\nNext:")
    print(f"  GRUB_DEFAULT=2 ./make-esp.sh knat/Image \"\" {args.out}")
    print("  fastboot flash esp esp-new.img")
    print("\nAfter the reboot there should be three key devices, not one:")
    print("  gpio-keys (VOLUMEUP), pmic pwrkey (POWER), pmic resin (VOLUMEDOWN)")
    print("Check with: ./zorn-keys-check.sh")


if __name__ == '__main__':
    main()
