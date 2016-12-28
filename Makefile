all: *.c *.h
	gcc -Wall deadbeef.c mailbox.c  gpu_fft_base.c  -o deadbeef.bin -lm -ldl

clean:
	rm deadbeef.bin
