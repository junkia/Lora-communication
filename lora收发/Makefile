INC = ./

default:
	arm-linux-gnueabihf-gcc -c -I $(INC) llcc68.c
	arm-linux-gnueabihf-gcc -c -I $(INC) llcc68_driver_version.c
	arm-linux-gnueabihf-gcc -c -I $(INC) radio.c
	arm-linux-gnueabihf-gcc -c -I $(INC) main.c
	arm-linux-gnueabihf-gcc -o lorareceive_434 *.o 

clean:
	rm *.o
	rm lorasend_434
	rm lorareceive_434
	rm lorasend_495
	rm lorareceive_495