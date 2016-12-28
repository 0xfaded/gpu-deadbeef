Absolute Minimal Raspberry Pi GPU Example: 0xdeadbeef
====

This repository serves as a starting point for writing GPU code for the Raspberry Pi.
The example loads an immediate into a register and writes it back out to memory,
yet as simple as this may be, it requires basic understanding of how data flow through
the GPU pipeline.


0. [Required Reading](#required-reading)
1. [Data Flow](#data-flow)
2. [VPM Write from QPU](#vpm-qpu)
3. [VPM DMA Write to Memory](#vpm-dma)
4. [Terminating](#terminating)
5. [Setup Macros](#setup-macros)
6. [Assembling](#assembling)
7. [Running](#running)
8. [Next Steps](#next-steps)

Any work with the VideoCore necessitates the acknowledgement of several awe inspiring individuals.

 * [Herman Hermitage](https://github.com/hermanhermitage/videocoreiv-qpu) for reverse engineering and documenting the VideoCore.
 * [Andrew Holme](http://www.aholme.co.uk/) for [hello_fft](https://github.com/raspberrypi/userland/tree/master/host_applications/linux/apps/hello_pi/hello_fft) which serves as the GPU Rosetta Stone.
 * [Marcel Müller](http://maazl.de/) for the [`vc4asm`](http://maazl.de/project/vc4asm/doc/index.html) assembler
 * And the others whose work proceeded the above.

<a name="required-reading"></a>
Required Reading
---
 * [Broadcom Documentation](https://ia800408.us.archive.org/30/items/pdfy-rXdEJ1TyaZLR_L12/VideoCoreIV-AG100-R.pdf)
 * [`vc4asm` Reference](http://maazl.de/project/vc4asm/doc/index.html)

<a name="data-flow"></a>
Data Flow
---

The first figure in the Broadcom documentation shows a block and arrow diagram of the GPU components.
Of the arrows, the `General DMA Write Data` provides a path out of the system.
It flows from the *Vertex Pipe Memory (VPM) Direct Memory Access (DMA) Writer* and
is the escape hatch for our data. Working backwards, we see that it writes from the VPM.

The VPM is a two dimensional block of memory manipulated by both horizontal and vertical read and write
operations at programmed offsets.
This minimal example uses only the top horizontal row,
but even so, the control registers must be setup to operate in this layout.

For simplicity, I have chosen to load the VPM memory with an immediate.
This will be performed by one of the *Quad Processor Units (QPU)*.

Our objective then is to have a QPU write a constant across the top of the VPM,
and then DMA write that row back to user memory. Simple, right?

<a name="vpm-qpu"></a>
VPM Write from QPU
---
The orientation and position of VPM reads and writes is configured by writing to
the registers documented as `VPMVCD_RD_SETUP` and `VPMVCD_WR_SETUP` respectively.
The `vc4asm` assembler assignees these registers the less unwieldy names
`vr_setup` and `vw_setup`.

It must be understood that these setup registers configure both access from the
QPUs as well as the DMA units, and the "QPU Control of VCD and VDW" section
of the Broadcom documentation should be read very carefully.

In particular, the type of read and write is configured by the most significant
bits (MSB) of the setup registers. The `VPM Generic Block (Read|Write) Setup Format`
is for reads and writes by the QPU and is specified by a profile id of 0 in the MSB.
Other profiles configure load and stores using the DMA units.
Also note that regardless of the profile,
only one VPM read and one VPM write operation may happen concurrently.
Hence it makes sense that the setup registers are shared between the QPU and DMA units.

With the above understanding, we can now configure a write from the QPU by writing
the below value to the `vw_setup` register.

    VPM Generic Block Profile    Packed   (y, x) = (0, 0)
      |                             |         |
      |  Unused  Stride  Horizontal | 32-bit  |
      |       |       |       |     |   |     |
      31:30   29:18   17:12   11   10   9:8   7:0
    |     0 |     0 |     1 |  1 |  0 |   3 |   0 |

     See Broadcom documentation
         Table 32: VPM Generic Block Write Setup Format

For our purposes, we only care that our write is horizontal from `(y, x)` coordinate `(0, 0)`.
As a side note, we set `Stride` to `1` because setting it to `0` would have translated as a
stride of `64`. Since we only make one write, it doesn't actually affect us.

After setup, the write is initiated by a QPU instruction writing to the `VPM_WRITE` register.
The `VPM_READ` register is mapped to the same address, therefore the assembler names the
common register address `vpm`.

As aforementioned, the QPU may not make another write until this write completes.
Somewhat confusingly, the term write is used both for a QPU to VPM write and a
VPM to memory write via DMA. This despite in the former case data enters the VPM,
and in the latter data leaves the VPM.
It means both our data operations are writes,
and therefore we must wait for the first to complete before initiating the second.
Polling `VPM_ST_BUSY` (`vw_busy`) is an option, however it is simpler to stall the QPU
by reading `VPM_ST_WAIT` (`vw_wait`).

    mov vw_setup, 0x00001b00
    mov vpm, 0xdeadbeef
    mov -, vw_wait

In the above code, the hyphen (-) indicates that no destination is specified for the move.
As a syntactic sugar, `vc4asm` provides a semantically equivalent pseudo-instruction `read`.

    read vw_wait

Note: Writing to `vpm` will increment the write position by `Stride`, in our case to the next row.
If `vpm` is written to again without modifying `vw_setup`, it is considered part of the same
write and it is not necessary to stall on `vw_wait` until all mov instructions have been issued.

    mov vw_setup, 0x00001b00
    mov vpm, 0xdeadbeef
    mov vpm, 0xfaded000
    mov vpm, 0xf00bad00
    mov -, vw_wait

The above will load three rows into the VPM as below.

    0 0xdeadbeef, 0xdeadbeef, 0xdeadbeef, 0xdeadbeef
    1 0xfaded000, 0xfaded000, 0xfaded000, 0xfaded000
    2 0xf00bad00, 0xf00bad00, 0xf00bad00, 0xf00bad00

<a name="vpm-dma"></a>
VPM DMA Write to Memory
---

Configuring a DMA write to memory uses the same setup register as above,
but with a different profile id.

    VCD DMA Store (VDW) Basic Profile   (y, x) = (0, 0)
       |                                    |
       |   Units   Depth   Laned  Horizontal|   32-bit
       |       |       |       |     |      |     |
       31:30   29:23   22:16   15   14   13:3   2:0
     |     2 |     1 |     4 |  0 |  1 |    0 |   0 |
    
     See Broadcom documentation
         Table 34: VCD DMA Store (VDW) Basic Setup Format 

Broadcom documentation better explains that `Units` should be interprented
as number of 2D rows, and `Depth` considered as row length in 32-bit
words. If bits `2:0` specified a different data width, then row length would
be in multiples of that width instead.
In our case, we want to write out the top 16-byte row, thus we set `Units` to `1` and
`Depth` to `4`.

Finally, we need a memory address to write to.
The uniform register can be used to pass a vector of values from the
outside world to the QPU's. More on this later, but for now it
suffices that we can read an address from the special register `unif`.

    .set out_addr, r0

    mov vw_setup, 0x80844000

    mov out_addr, unif
    mov vw_addr, out_addr

    mov -, vw_wait

For clarity we first moved the value into a register we aliased as `out_addr`,
however this was unessecary. The value could have been moved directly from
`unif` to `vw_addr`.

<a name="terminating"></a>
Terminating
---

There are two ways to execute our program, either by directly probing memory mapped registers
or by using a mailbox system.
I have chosen the direct method and therefore terminate by signaling `THREAD_END` (`thrend`).
To be precise, `thrend` is a signaling bit set on an instruction, usually a nop.
Because the QPU is prefetching instructions, two extra instructions will be executed before the
processor finally stops. Therefore it is typical to see the `thrend` followed by two nops.


    nop; thrend
    nop
    nop

If the mailbox system was used, then the program must interupt the host by writing to
`HOST_INT`. I do not fully understand this mechanism, but it should be noted that the
host refered to is not the ARM processor, but the VideoCore itself.
It also seems to be good practice to issue the `thrend` signal reguardless.

    mov interrupt, 1
    nop; thrend
    nop
    nop

<a name="setup-macros"></a>
Setup Macros
---
The `vc4asm` distribution includes a file `share/vc4.qinc` which defines some handy macros
for composing the constants loaded into the setup registers.
These can be best understood by cross referencing the `vc4.qinc` source with the
Broadcom documentation; as a starting point our program can be rewritten as below.

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

<a name="assembling"></a>
Assembling
---

`vc4asm` is available from http://maazl.de/project/vc4asm/doc/index.html.
We will assemble and output the instructions as an array of uint32_t
values which can be `#include`'d, or even copy pasted, into C code.

The included binaries run on the raspberry pi, or alternatively can be recompiled for
another architecture and run there.

    cp ./path/to/vc4asm/share/vc4.qinc .
    ./path/to/vc4asm -C deadbeef.c -V deadbeef.qasm

Side Note: I had difficulty with the `-I` flag to add include directories so I resorted to
copying `share/vc4.qinc` into my source directory.

<a name="running"></a>
Running
---
`deadbeef.c` contains a bare-bones main function that will run
the assembled program.
The repository also contains `mailbox.c`, `gpu_fft.h`, and `gpu_fft_base.c`
which are copied directly from [hello_fft](https://github.com/raspberrypi/userland/tree/master/host_applications/linux/apps/hello_pi/hello_fft).
These contain the actual machinery to load our program onto the GPU and run it.
The only intricacies that really need understanding are the memory layout
and uniforms mechanism.

Any memory that the GPU will DMA must be specially mapped into our user side
address space using the mailbox mechanism.
Thankfully, `mailbox.c` makes this as simple as opening the mailbox and requestion a memory
block of a particular size.

The memory will be mapped in at a virtual address which differs from the bus address.
It is important to understand that our user side program must reference this memory
using our virtual address, but the QPU program must use the bus address.
`gpu_fft_base.c` defines a function `gpu_fft_ptr_inc` which abstracts the dual addressing.

    unsigned gpu_fft_ptr_inc (
        struct GPU_FFT_PTR *ptr,
        int bytes);
The `ptr` struct will always reference a user side address. The return value is the
bus address *before* incrementing by `bytes` bytes.

We can now layout the GPU memory, which we will split into four sections.

    size  = info_bytes +        // header
            out_bytes +         // scratch memory
            code_bytes +        // shader, aligned
            unif_bytes;         // uniforms

    ret = gpu_fft_alloc(mb, size, &ptr);

The header bytes must be laid out as a `struct GPU_FFT_BASE` (technically only true
if executing using the mailbox mechanism, which we are not), so the first thing
we do with this allocated memory is cast it as base.

    base = (struct GPU_FFT_BASE *)ptr.arm.vptr;

    // Header
    gpu_fft_ptr_inc(&ptr, info_bytes);

We then increment the pointer over the header region.
Next comes `1k` of scratch memory initialised to `0xff`.
We will only write to 16 of these bytes, however the excess allocation guards against
overrun errors.

    out_user_ptr = ptr.arm.vptr;
    memset(ptr.arm.vptr, 0xff, out_bytes);
    out_bus_ptr = gpu_fft_ptr_inc(&ptr, out_bytes);

This time we saved the result of `gpu_fft_ptr_inc`, which is the bus address
of `out_bytes`. We will need to supply this address to the GPU program so it knows
where to DMA write the result to. As before we skip over the next `out_bytes` to reach
the next section.

    memcpy(ptr.arm.vptr, code, code_bytes);
    base->vc_code = gpu_fft_ptr_inc(&ptr, code_bytes);

The code section is next, and as before we save the bus address which will be where the
QPU execution starts.

Finally we write the uniforms, of which in this simple example there is only one.
Usually we would have one uniform vector for each QPU used, causing the same
program to run in parallel with different uniforms.
See [`gpu_fft.c`](https://github.com/raspberrypi/userland/blob/master/host_applications/linux/apps/hello_pi/hello_fft/hello_fft.c)
for a better example. At any rate, the bus base address of the uniforms must be saved and passed to the GPU.

    uniform_user_ptr = ptr.arm.uptr;
    *uniform_user_ptr = out_bus_ptr;
    base->vc_unifs[0] = gpu_fft_ptr_inc(&ptr, unif_bytes);

With the GPU memory set up, we at last compile and run the program. On the Raspberry Pi, run:

    > gcc -Wall deadbeef.c mailbox.c  gpu_fft_base.c  gpu_fft_print.c -o deadbeef.bin -lm -ldl
    > sudo ./deadbeef.bin 
    ffffffff ffffffff ffffffff ffffffff
    deadbeef deadbeef deadbeef deadbeef

And there you have it, some hardware accelerated dead beef.

<a name="next-steps"></a>
Next Steps
---

The `vc4asm` distribution includes a file `samples/simple/smitest.asm` which has examples
using other parts of the GPU such as the Texture Memory Lookup Unit for DMA'ing memory into
the VPM.

The only two other examples I've found are hello_fft and the [DeepBelief](https://petewarden.com/2014/08/07/how-to-optimize-raspberry-pi-code-using-its-gpu/) implementation. Hopefully this repository has provided a less intimidating starting point.
