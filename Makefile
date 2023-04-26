FLAG = -lpthread -Wall
OBJ = main.c \
	./mdio/mdio-tool.c \
	./serial/serial.c \
	./nmea/nmea.c
OUT = main.o

all:
	gcc -o $(OUT) $(OBJ) $(FLAG)
send:
	@sudo ./$(OUT) s

recv:
	@sudo ./$(OUT) r

clean:
	@rm *.o
