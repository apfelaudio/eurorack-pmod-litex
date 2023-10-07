# 2 cores fits on i5
python3 example-colorlight-i5.py --ecppack-compress --cpu-type vexriscv_smp --cpu-variant standard --csr-svd build/colorlight_i5/csr.svd --build --lx-ignore-deps --timer-uptime --cpu-count 1 --with-rvc --wishbone-force-32b --with-wishbone-memory --without-out-of-order-decoder --cpu-count 2 --without-coherent-dma --icache-width 32 --dcache-width 32 --icache-size 2048 --dcache-size 2048
# 1 core runs smoothly
python3 example-colorlight-i5.py --ecppack-compress --cpu-type vexriscv_smp --cpu-variant standard --csr-svd build/colorlight_i5/csr.svd --build --lx-ignore-deps --timer-uptime --cpu-count 1 --with-rvc --wishbone-force-32b --with-wishbone-memory --without-out-of-order-decoder
