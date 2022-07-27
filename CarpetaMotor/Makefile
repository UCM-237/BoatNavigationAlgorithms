CC=g++
CFLAGS=  -g -Wall -g
CLIBS= -lpthread -std=c++0x
TARGET= prueba
all: $(TARGET)

$(TARGET): $(TARGET).cpp
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).cpp  $(CLIBS)

debug:
	$(CC) -debug -Wall $(CFLAGS) -o $(TARGET) $(TARGET).cpp  $(CLIBS)


clean:
	$(RM) $(TARGET)