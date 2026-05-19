CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -std=c11
LDFLAGS = -lgpiod -lpthread

TARGET  = fadec
SRC     = fadec.c unstart.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC) fadec.h
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)
