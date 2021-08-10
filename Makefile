CC = gcc
CFLAGS = -g
CLIBS =
INCLUDE = $(wildcard ./*.h)
SOURCES = $(wildcard ./*.c)

TARGET = oncecom
OBJECTS = $(patsubst %.c,%.o,$(SOURCES))

$(TARGET) : $(OBJECTS)
	$(CC) $(CFLAGS) $^ -o $@ $(CLIBS)
$(OBJECTS) : %.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@

.PHONY : clean

clean:
	rm -rf *.o $(TARGET)
