test: mkdir test.o kalman.o
	g++ -o bin/test bin/test.o bin/kalman.o

kalman.o: mkdir src/kalman_filter.cpp include/kalman_filter/kalman_filter.hpp
	g++ -c src/kalman_filter.cpp -o bin/kalman.o -I./include -I/usr/include/eigen3

test.o: mkdir src/test.cpp include/kalman_filter/kalman_filter.hpp
	g++ -c src/test.cpp -o bin/test.o -I./include -I/usr/include/eigen3

mkdir:
	-mkdir -p bin

clean:
	-rm -r bin/
