gateway: gateway.o rfm69.o
	cc -o gateway gateway.o rfm69.o -pthread -lwiringPi

gateway.o: gateway.c rfm69.h rfm69config.h
	gcc -Wall -c gateway.c

rfm69.o: rfm69.c rfm69.h rfm69config.h
	gcc -Wall -c rfm69.c
