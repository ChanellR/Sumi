# Thanks to Job Vranish (https://spin.atomicobject.com/2016/08/26/makefile-c-projects/)
TARGET_EXEC := Sumi

BUILD_DIR := ./build
INC_DIR := ./include
SRC_DIRS := ./src

# Find all the C and C++ files we want to compile
# Note the single quotes around the * expressions. The shell will incorrectly expand these otherwise, but we want to send the * directly to the find command.
SRCS := $(wildcard $(SRC_DIRS)/*) $(wildcard ./imgui/*.cpp)

# Prepends BUILD_DIR and appends .o to every src file
# As an example, ./your_dir/hello.cpp turns into ./build/./your_dir/hello.cpp.o
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

# Every folder in ./src will need to be passed to GCC so that it can find header files
INC_DIRS := ./imgui $(wildcard $(INC_DIR)/*) 
# Add a prefix to INC_DIRS. So moduleA would become -ImoduleA. GCC understands this -I flag
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CPPFLAGS := -g -std=c++17 $(INC_FLAGS)
LDFLAGS := -L./lib -lglfw3dll

# The final build step.
$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# Build step for C source
$(BUILD_DIR)/%.c.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# Build step for C++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

run: $(BUILD_DIR)/$(TARGET_EXEC)
	$(BUILD_DIR)/$(TARGET_EXEC)

#avoids imgui compiled files
.PHONY: clean
clean:
	rm -r $(BUILD_DIR)/src
	rm -r $(BUILD_DIR)/Sumi