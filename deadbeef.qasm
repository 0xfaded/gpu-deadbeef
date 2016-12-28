.include "vc4.qinc"

.set out_addr, r0

mov vw_setup, vpm_setup(1, 1, h32(0, 0))
mov vpm, 0xdeadbeef
mov -, vw_wait

mov vw_setup, vdw_setup_0(1, 4, dma_h32(0 ,0))
mov out_addr, unif
mov vw_addr, out_addr
mov -, vw_wait

nop; thrend
nop
nop
