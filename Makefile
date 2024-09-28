# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -fprofile-arcs -ftest-coverage -pg

# Source file directories
GRAPH_DIR = Graph_DIR
SERVER_DIR = Server_DIR

# Source files for Graph
GRAPH_SOURCES = $(wildcard $(GRAPH_DIR)/*.cpp)

# Source files for Server
SERVER_SOURCES = $(wildcard $(SERVER_DIR)/*.cpp)

# Client source file
CLIENT_SOURCE = pollclient.cpp

# Object files
GRAPH_OBJECTS = $(patsubst $(GRAPH_DIR)/%.cpp,$(GRAPH_DIR)/%.o,$(GRAPH_SOURCES))
SERVER_OBJECTS = $(patsubst $(SERVER_DIR)/%.cpp,$(SERVER_DIR)/%.o,$(SERVER_SOURCES))
CLIENT_OBJECT = $(CLIENT_SOURCE:.cpp=.o)

# Executable names
GRAPH_EXECUTABLE = Graph
SERVER_EXECUTABLE = server
CLIENT_EXECUTABLE = client

# Default target
all: $(GRAPH_EXECUTABLE) $(SERVER_EXECUTABLE) $(CLIENT_EXECUTABLE)

# Rule to link the Graph program
$(GRAPH_EXECUTABLE): $(GRAPH_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Rule to link the Server program
$(SERVER_EXECUTABLE): $(SERVER_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Rule to link the Client program
$(CLIENT_EXECUTABLE): $(CLIENT_OBJECT)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Rules to compile source files to object files
$(GRAPH_DIR)/%.o: $(GRAPH_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(SERVER_DIR)/%.o: $(SERVER_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(CLIENT_OBJECT): $(CLIENT_SOURCE)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean target
clean:
	rm -f $(GRAPH_OBJECTS) $(SERVER_OBJECTS) $(CLIENT_OBJECT) $(GRAPH_EXECUTABLE) $(SERVER_EXECUTABLE) $(CLIENT_EXECUTABLE)

# Phony targets
.PHONY: all clean