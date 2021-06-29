all: test_string test_vector test_map test_set test_queue test_thread

test_vector: vector_cookbook.cc
	g++ vector_cookbook.cc -std=c++11 -o test_vector

test_string: string_cookbook.cc
	g++ string_cookbook.cc -std=c++11 -o test_string

test_map: map_cookbook.cc
	g++ map_cookbook.cc -std=c++11 -o test_map

test_set: set_cookbook.cc
	g++ set_cookbook.cc -std=c++11 -o test_set

test_queue: queue_cookbook.cc
	g++ queue_cookbook.cc -std=c++11 -o test_queue

test_thread: thread_cookbook.cc
	g++ thread_cookbook.cc -std=c++11 -o test_thread -lpthread

.PHONY: clean
clean:
	rm test_*