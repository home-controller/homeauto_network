# Tool definitions
CC = gcc -H
CXX = g++ -H

# Settings
SRC_DIR = ./src
TEST_DIR = ./tests
BUILD_DIR = ./build
#LIB_DIR = /home/jmnc2/doc/src/Arduino/libraries/
LIB_DIR = ../libraries/**/src
NAME = app.elf

# Search path for header files
#CFLAGS += -I$(SRC_DIR)/average
#CFLAGS += -I/home/jmnc2/doc/src/Arduino/libraries/defs/src
#CXXFLAGS += -I/home/jmnc2/doc/src/Arduino/libraries
CXXFLAGS += -I$(LIB_DIR)

CFLAGS += -I$(LIB_DIR)
#CFLAGS += -I/home/jmnc2/doc/src/Arduino/libraries/defs/src

# List module source files
CSOURCES = $(SRC_DIR)/main.cpp
CSOURCES += $(wildcard $(LIB_DIR)/defs/src/*.cpp)
#CSOURCES += $(wildcard $(SRC_DIR)/average/*.c)

# Compiler flags
CXXFLAGS += -v -Wall
CFLAGS += -Wall
#CFLAGS += -v
#CFLAGS += -H

# Linker flags
LDFLAGS +=

# Generate names for output object files (*.o)
COBJECTS = $(patsubst %.cpp, %.o, $(CSOURCES))

# Default rule: build application
.PHONY: all
all: $(NAME)

# Build components
$(COBJECTS) : %.o : %.cpp
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# print includes:
.PHONY: print_includes
print_includes:
	@echo $(CFLAGS) | tr " " "\n" 
# | grep "^-I"


# Build the target application
.PHONY: $(NAME)
$(NAME): $(COBJECTS)
	$(CC) $(COBJECTS) -o $(BUILD_DIR)/$(NAME) $(LDFLAGS)

# Remove compiled object files
.PHONY: clean
clean:
	rm -f $(COBJECTS)

# Run tests
.PHONY: test
test:
	make -C $(TEST_DIR)
	
# Clean tests
.PHONY: test_clean
test_clean:
	make -C $(TEST_DIR) clean

.PHONY: edit
edit:
	code-insiders --extensions-dir="../vscode/insiders/extensions" homeauto_network.code-workspace

#list conected devises
.PHONY: list
list:
	pio device list

#Build docs
.PHONY: docs
docs:
	doxygen -u
	doxygen

#Add udev rules
.PHONY: udev
udev:
#	#curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
	@echo "Adding user to uucp and lock groups..."
	@sudo usermod -a -G uucp,lock $(shell whoami) || { echo "Failed to add user to groups."; exit 1; }
	@echo "Reloading udev rules..."
	@sudo udevadm control --reload-rules || { echo "Failed to reload udev rules."; exit 1; }
	@echo "Triggering udev..."
	@sudo udevadm trigger || { echo "Failed to trigger udev."; exit 1; }
	@echo "Finished."
