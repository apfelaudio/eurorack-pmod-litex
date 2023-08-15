#!/usr/bin/env python3

# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["riscv", "nextpnr-ecp5", "yosys"]

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv

from litex_boards.targets.lambdaconcept_ecpix5 import *

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=lambdaconcept_ecpix5.Platform, description="LiteX SoC on ECPIX-5.")
    parser.add_target_argument("--device",          default="85F",            help="ECP5 device (45F or 85F).")
    parser.add_target_argument("--sys-clk-freq",    default=75e6, type=float, help="System clock frequency.")

    args = parser.parse_args()

    soc = BaseSoC(
        device                 = args.device,
        sys_clk_freq           = args.sys_clk_freq,
        toolchain              = args.toolchain,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

if __name__ == "__main__":
    main()
