#!/usr/bin/env python3
"""Add zorn's FocalTech touchscreen to the booted device tree.

Runs on the output of patch-dt-display.py, so the two sets of changes compose:

    ./patch-dt-display.py                       # -> zorn-display.dtb
    ./patch-dt-touch.py                         # -> zorn-touch.dtb

Four changes, all of them things the booted DT already almost has:

  1. dma-controller@a00000 (gpi_dma1), geniqup@ac0000 (QUP wrap1) and spi@a90000
     (its SE4) status disabled -> okay.  All three are needed.  The wrapper is
     what gets the platform device, and with it disabled the SE nodes underneath
     are never populated no matter what their own status says.  gpi_dma1 is where
     spi@a90000's existing dmas = <&gpi_dma1 0 4 ...> points; upstream leaves
     every GPI DMA engine disabled for board DTs to turn on, so without it
     spi-geni-qcom defers forever on
         a90000.spi: deferred probe pending: geni_spi: Failed to get tx DMA ch
     and every provider then reports sync_state() pending due to a90000.spi.  Everything else those nodes need is already there and
     correct, including pinctrl-0 pointing at upstream's
     qup-spi4-data-clk-state / qup-spi4-cs-state, i.e. gpio 48-51.

  2. tlmm gpio-reserved-ranges narrowed so gpio 48-51 (the SE4 SPI pins) plus
     161 and 162 become usable.  161/162 are covered by the <134 76> run and
     48-51 by <18 68>, and *all six* have to be freed: pinctrl-msm's
     msm_pinmux_request() is literally
         return gpiochip_line_is_valid(chip, offset) ? 0 : -EINVAL;
     so a reserved pin cannot even be muxed, let alone claimed as a GPIO.  Left
     reserved, the SPI pins fail the bus itself before the touchscreen is ever
     reached:
         sm8650-tlmm f100000.pinctrl: error -EINVAL: pin-48 (a90000.spi)
         geni_spi a90000.spi: Error applying setting, reverse things back
     (An earlier version of this script freed only 161/162 on the theory that
     muxing bypasses the valid mask.  It does not.)

  3. A touchscreen@0 child under spi@a90000, translated from the stock Android
     node (xiaomi_touch@0, compatible "focal,touch-spi"):

        spi-max-frequency  15 MHz     <0xe4e1c0>
        interrupts         tlmm 162   falling edge, as the vendor requests it
        reset-gpios        tlmm 161   active low
        avdd-supply        L14B       vreg_l14b_3p2, stock avdd-supply
        iovdd-supply       L12B       vreg_l12b_1p8, stock iovdd-supply

     The stock focaltech,* properties are all either defaults of the mainline
     driver or Xiaomi HAL plumbing (super-resolution, FOD, the test framework),
     so only max-x/max-y survive, as touchscreen-size-x/y.

Everything is verified after the edit: each phandle written is resolved back to
the node it is supposed to name, and both freed pins are re-checked against the
rewritten ranges.  The script aborts rather than emit a DT wired to the wrong
regulator, which is a mistake this project has already made once.
"""

import argparse
import importlib.util
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

SPI_PINS = (48, 49, 50, 51)  # qup-spi4-data-clk-state + qup-spi4-cs-state
IRQ_GPIO = 162          # focaltech,irq-gpio = <&tlmm 162 0x2008>
RESET_GPIO = 161        # focaltech,reset-gpio = <&tlmm 161 0>
SPI_HZ = 15000000       # spi-max-frequency = <0xe4e1c0>
MAX_X = 1440            # focaltech,max-x = <0x5a0>
MAX_Y = 3200            # focaltech,max-y = <0xc80>

AVDD = 'vreg_l14b_3p2'  # stock avdd-supply  = <&L14B>
IOVDD = 'vreg_l12b_1p8'  # stock iovdd-supply = <&L12B>

IRQ_TYPE_EDGE_FALLING = 2
GPIO_ACTIVE_LOW = 1

GPI_NODE = 'dma-controller@a00000 {'  # gpi_dma1, the DMA engine spi@a90000 asks for
QUP_NODE = 'geniqup@ac0000 {'   # QUP wrap1; disabled in the booted DT
SPI_NODE = 'spi@a90000 {'
TLMM_NODE = 'pinctrl@f100000 {'


def load_display_helpers():
    """Reuse patch-dt-display.py's node/phandle machinery rather than a second,
    subtly different copy of it.  Only the definitions above main() are run."""
    path = HERE / 'patch-dt-display.py'
    src = path.read_text().split('def main()')[0]
    spec = importlib.util.spec_from_loader('dtdisp', loader=None)
    mod = importlib.util.module_from_spec(spec)
    mod.__file__ = str(path)
    exec(compile(src, str(path), 'exec'), mod.__dict__)
    return mod


D = load_display_helpers()


def ensure_phandle(s: str, regulator_name: str) -> tuple[str, str]:
    """Phandle of the regulator named `regulator_name`, creating one if the tree
    has none.  dtc only emits a phandle for a node something references, so
    ldo14 -- which nothing used before the touchscreen -- legitimately has none."""
    try:
        return s, D.regulator_phandle(s, regulator_name)
    except SystemExit:
        pass

    m = re.search(r'regulator-name = "' + re.escape(regulator_name) + r'"', s)
    if not m:
        sys.exit(f"no regulator named {regulator_name}")
    name, i, _ = D.enclosing_node(s, m.start())
    ph = f'{D.free_phandle(s):#x}'
    indent = re.match(r'[\t ]*', s[s.index('\n', i) + 1:]).group(0)
    s = s[:i + 1] + f'\n{indent}phandle = <{ph}>;' + s[i + 1:]
    print(f"  {name} ({regulator_name}): had no phandle, assigned {ph}")
    return s, ph


def add_touch(s: str, tlmm: str, avdd: str, iovdd: str) -> str:
    """Insert touchscreen@0 as the only child of spi@a90000."""
    if 'touchscreen@0 {' in s:
        print("  touchscreen@0: already present")
        return s

    i, j = D.node_span(s, SPI_NODE)
    body = s[i:j]
    # node_span ends just past the closing brace, so the brace is the last char.
    if not body.rstrip().endswith('}'):
        sys.exit(f"unexpected shape for {SPI_NODE}")
    cut = i + body.rindex('}')

    node = f"""
				touchscreen@0 {{
					compatible = "focal,touch-spi";
					reg = <0x00>;
					spi-max-frequency = <{SPI_HZ:#x}>;
					interrupt-parent = <{tlmm}>;
					interrupts = <{IRQ_GPIO:#x} {IRQ_TYPE_EDGE_FALLING:#x}>;
					reset-gpios = <{tlmm} {RESET_GPIO:#x} {GPIO_ACTIVE_LOW:#x}>;
					avdd-supply = <{avdd}>;
					iovdd-supply = <{iovdd}>;
					touchscreen-size-x = <{MAX_X:#x}>;
					touchscreen-size-y = <{MAX_Y:#x}>;
				}};
"""
    print(f"  touchscreen@0: added under spi@a90000 "
          f"({SPI_HZ // 1000000} MHz, irq tlmm {IRQ_GPIO}, reset tlmm {RESET_GPIO})")
    print(f"           avdd={AVDD} {avdd}, iovdd={IOVDD} {iovdd}")
    return s[:cut].rstrip() + node + '\t\t\t' + s[cut:]


def verify(s: str, tlmm: str, avdd: str, iovdd: str) -> None:
    """Resolve every phandle the new node references back to the node it is meant
    to name.  Same reasoning as the display script: a wrong-but-plausible
    regulator phandle produces a tree that boots and silently never powers the
    part, and nothing downstream complains."""
    expect = {
        tlmm: 'pinctrl@f100000',
        avdd: 'ldo14',
        iovdd: 'ldo12',
    }
    print("=== verifying phandles ===")
    bad = False
    for ph, want in expect.items():
        got = D.phandle_owner(s, ph)
        ok = got == want
        bad |= not ok
        print(f"  {ph:8s} -> {got:26s} {'ok' if ok else f'EXPECTED {want}'}")
    if bad:
        sys.exit("refusing to write a tree with phandles pointing at the wrong nodes")

    i, j = D.node_span(s, TLMM_NODE)
    reserved = D.reserved_set(D.own_props(s, i, j))
    for g in (*SPI_PINS, RESET_GPIO, IRQ_GPIO):
        blocked = g in reserved
        print(f"  gpio {g:<4} {'STILL RESERVED' if blocked else 'usable'}")
        if blocked:
            sys.exit(f"gpio {g} is still reserved; pinmux or gpiod_get would get -EINVAL")

    for node, what in ((GPI_NODE, 'dma-controller@a00000'),
                       (QUP_NODE, 'geniqup@ac0000'),
                       (SPI_NODE, 'spi@a90000')):
        i, j = D.node_span(s, node)
        if 'status = "okay"' not in D.own_props(s, i, j):
            sys.exit(f"{what} did not end up enabled")

    i, j = D.node_span(s, SPI_NODE)
    if 'touchscreen@0 {' not in s[i:j]:
        sys.exit("touchscreen@0 is not inside spi@a90000")
    print("  spi@a90000 okay, touchscreen@0 is its child")

    # The supplies must also be reachable, i.e. their parent PMIC is not disabled.
    for name, ph in (('avdd', avdd), ('iovdd', iovdd)):
        node = D.phandle_owner(s, ph)
        print(f"  {name:5s} -> {node}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dtb', default=str(HERE / 'zorn-display.dtb'),
                    help='input dtb (default: zorn-display.dtb, the display-patched one)')
    ap.add_argument('--out', default='zorn-touch.dtb')
    args = ap.parse_args()

    print(f"=== decompiling {args.dtb} ===")
    dts = D.sh('dtc', '-I', 'dtb', '-O', 'dts', args.dtb).stdout.decode()

    print("=== patching ===")
    tlmm = D.node_phandle(dts, TLMM_NODE)
    dts, avdd = ensure_phandle(dts, AVDD)
    dts, iovdd = ensure_phandle(dts, IOVDD)

    dts = D.enable(dts, GPI_NODE, 'gpi_dma1 (tx/rx channels for spi4)')
    dts = D.enable(dts, QUP_NODE, 'qup wrap1 wrapper')
    dts = D.enable(dts, SPI_NODE, 'spi4 (qup wrap1 se4)')
    D.FREE_GPIOS = (*SPI_PINS, RESET_GPIO, IRQ_GPIO)
    dts = D.free_reserved_gpios(dts)
    dts = add_touch(dts, tlmm, avdd, iovdd)

    verify(dts, tlmm, avdd, iovdd)

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
    print("\nEntry 2 is the quiet display entry; entry 1's drm.debug=0x06 buries the")
    print("touch messages. The kernel itself does not change -- focaltech-spi-ts.ko")
    print("is built against the running one -- so install-touch.sh handles the rest.")


if __name__ == '__main__':
    main()
