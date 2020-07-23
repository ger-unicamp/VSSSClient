SRCDIR = net pb util

CPPFILES = $(shell find $(SRCDIR) -name '*.cpp')
CPPFILES += main.cpp

CCFILES = $(shell find $(SRCDIR) -name '*.cc')

HFILES = $(shell find $(SRCDIR) -name '*.h')

OBJFILES = $(CPPFILES:.cpp=.o)
OBJFILES += $(CCFILES:.cc=.o)

CFLAGS = -g -std=c++11 -march=native -Ofast -Wall -Wextra -I./

LDFLAGS = -lprotobuf -lpthread -L/usr/local/lib/

TARGET = vss.exe

all: $(TARGET)

$(TARGET): $(OBJFILES)
	g++ $^ -o $@ $(LDFLAGS)

%.o: %.cpp $(HFILES)
	g++ -c $< -o $@ $(CFLAGS)

source:
	@echo $(CPPFILES) $(CCFILES)

.PHONY: clean

clean:
	rm -f $(OBJFILES) $(TARGET)
