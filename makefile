CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++11
OBJS = graph.o algorithms.o

all: Main

Main: main.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) -o Main main.cpp $(OBJS)
	./Main

test: test.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) -o test test.cpp $(OBJS)
	./test

valgrind: test
	valgrind --leak-check=full ./test

graph.o: graph.cpp graph.h
	$(CXX) $(CXXFLAGS) -c graph.cpp

algorithms.o: algorithms.cpp algorithms.h
	$(CXX) $(CXXFLAGS) -c algorithms.cpp

clean:
	rm -f *.o Main test
