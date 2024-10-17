CFLAGS=-Wall -Wstrict-prototypes -O2

test_CAN: test_CAN.o
	gcc -pthread -o test_CAN test_CAN.o -L/usr/lib -lwdt_dio

test_CAN.o: test_CAN.c
	gcc -c $(CFLAGS) -I../../include test_CAN.c

clean:
	rm -f test_CAN.o test_CAN
