/*
BCM2835 "DEADBEEF"
Copyright (c) 2016, Carl Chatfield
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "mailbox.h"
#include "gpu_fft.h"

unsigned code[] = {
    0x00101a00, 0xe0021c67,
    0xdeadbeef, 0xe0020c27,
    0x159f2fc0, 0x100009e7,
    0x80844000, 0xe0021c67,
    0x15827d80, 0x10020827,
    0x159e7000, 0x10021ca7,
    0x159f2fc0, 0x100009e7,
    0x009e7000, 0x300009e7,
    0x009e7000, 0x100009e7,
    0x009e7000, 0x100009e7
};

unsigned gpu_fft_base_exec_direct (struct GPU_FFT_BASE *base, int num_qpus);

int main(int argc, char *argv[]) {
    unsigned info_bytes, out_bytes, code_bytes, unif_bytes;
    unsigned size, *uniform_user_ptr, *out_user_ptr;
    unsigned out_bus_ptr;
    int ret;
    int mb = mbox_open();

    struct GPU_FFT_BASE *base;
    struct GPU_FFT_PTR ptr;

    info_bytes = 4096;
    out_bytes = 1024;
    code_bytes = sizeof(code);
    unif_bytes = sizeof(unsigned);

    size  = info_bytes +        // header
            out_bytes +         // scratch memory
            code_bytes +        // shader, aligned
            unif_bytes;         // uniforms

    ret = gpu_fft_alloc(mb, size, &ptr);
    if (ret) {
        fprintf(stderr, "Could not allocate memory %d: %d\n", ret, errno);
        return -1;
    }

    base = (struct GPU_FFT_BASE *) ptr.arm.vptr;

    // Header
    gpu_fft_ptr_inc(&ptr, info_bytes);

    // Scratch bytes
    out_user_ptr = ptr.arm.vptr;
    memset(ptr.arm.vptr, 0xff, out_bytes);
    out_bus_ptr = gpu_fft_ptr_inc(&ptr, out_bytes);

    // Shader code
    memcpy(ptr.arm.vptr, code, code_bytes);
    base->vc_code = gpu_fft_ptr_inc(&ptr, code_bytes);

    // Write our single uniform, the output address
    uniform_user_ptr = ptr.arm.uptr;
    *uniform_user_ptr = out_bus_ptr;
    base->vc_unifs[0] = gpu_fft_ptr_inc(&ptr, unif_bytes);

    fprintf(stderr, "%08x %08x %08x %08x\n", out_user_ptr[0], out_user_ptr[1], out_user_ptr[2], out_user_ptr[3]);
    gpu_fft_base_exec_direct(base, 1);
    fprintf(stderr, "%08x %08x %08x %08x\n", out_user_ptr[0], out_user_ptr[1], out_user_ptr[2], out_user_ptr[3]);

    gpu_fft_base_release(base); // Videocore memory lost if not freed !

    return 0;
}
