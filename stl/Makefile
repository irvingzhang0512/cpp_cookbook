all: test_string test_vector

test_vector: vector_cookbook.cc
	g++ vector_cookbook.cc -std=c++11 -o test_vector

test_string: string_cookbook.cc
	g++ string_cookbook.cc -std=c++11 -o test_string

.PHONY: clean
clean:
	rm test_*