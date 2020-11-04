# Common source files
SRCDIR = net pb util strategy
CPPFILES = $(shell find $(SRCDIR) -name '*.cpp')
CCFILES = $(shell find $(SRCDIR) -name '*.cc')
HFILES = $(shell find $(SRCDIR) -name '*.h')
OBJFILES = $(CPPFILES:.cpp=.o)
OBJFILES += $(CCFILES:.cc=.o)

# Common flags
CFLAGS = -g -std=c++11 -march=native -Ofast -I./
LDFLAGS = -lprotobuf -lpthread -lm -L/usr/local/lib/

# Main files
MAIN_TARGET = main.cpp
MAIN_OBJ = $(MAIN_TARGET:.cpp=.o)
TARGET = vss.exe

# Training source files
CPPTRAIN = $(shell find train -name '*.cpp')
OBJTRAIN = $(CPPTRAIN:.cpp=.o)
TRAIN = train.exe

all: $(TARGET)

$(TARGET): $(OBJFILES) $(MAIN_OBJ)
	g++ $^ -o $@ $(LDFLAGS)

$(TRAIN): $(OBJFILES) $(OBJTRAIN)
	g++ $^ -o $@ $(LDFLAGS)

%.o: %.cpp $(HFILES)
	g++ -c $< -o $@ -Wall -Wextra $(CFLAGS)

%.o: %.cc
	g++ -c $< -o $@ $(CFLAGS)

source:
	@echo $(CPPFILES) $(CCFILES)

.PHONY: clean clean_train

clean:
	rm -f $(OBJFILES) $(TARGET)

clean_train:
	rm -f $(OBJTRAIN) $(TRAIN)