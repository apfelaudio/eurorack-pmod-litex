# `eurorack-pmod` LiteX examples

Basic example of using `eurorack-pmod` inside a LiteX environment.

It consists of a basic SoC that includes one instance of `eurorack-pmod` that we can read and write raw samples from using a soft RISCV CPU.

# Dependencies

- [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build)
- [RISCV toolchain](https://xpack.github.io/dev-tools/riscv-none-elf-gcc/install/)

# Building


```
<from this repository>
git submodule update --init --recursive
python3 example-colorlight-i5.py --cpu-type vexriscv --cpu-variant imac --csr-svd build/csr.svd --build
```

The bitstream is built in CI inside `.github/workflows/main.yml`, it may be useful to look at if you are missing some dependencies.
