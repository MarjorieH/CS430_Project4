all: raytrace.c
	gcc raytrace.c -o raytrace

clean:
	rm -rf raytrace *~

test:
	./raytrace 100 150 input.json output.ppm
