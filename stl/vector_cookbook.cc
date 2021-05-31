#include <assert.h>

#include <iostream>
#include <vector>

void initialize() {
  std::vector<int> vec1;
  std::vector<int> vec2(10);  // number of elements
  std::vector<int> vec3(
      10, 2);  // number of elements -> 10, init value for all elements 2
  std::vector<int> vec4(vec3.begin(), vec3.begin() + 3);  // {2, 2, 2}
  std::cout << vec4.size() << " " << vec4[0] << std::endl;
  std::vector<int> vec5(vec3);
  std::vector<int> vec6{1, 2, 3, 4};
  std::vector<int> vec8 = {1, 2, 3, 4};

  // array to vector & vector to array
  int array[5] = {1, 2, 3, 4, 5};
  std::vector<int> vec7(array, array + 5);  // {1, 2, 3, 4, 5}
  std::cout << vec7.size() << " " << vec7[0] << std::endl;
  int *p = vec6.data();
}

void iter() {
  std::vector<int> vec = {1, 2, 3, 4, 5};

  // there are four types of iter
  // begin/end, rbegin/rend, cbegin/cend, crbegin, crend
  // `r` refers to `reverse`, which means loop from right to left
  // `c` refers to `const`, which means we cannot modify the value of the
  // elements
  for (auto it = vec.begin(); it < vec.end(); it++) std::cout << *it << " ";
  std::cout << std::endl;

  // loop and remove
  // example: remove integers that can be divided by 2
  auto it = vec.begin();
  while (it < vec.end()) {
    if ((*it) % 2 == 0)
      it = vec.erase(it);
    else
      it++;
  }
  for (auto it = vec.begin(); it < vec.end(); it++) std::cout << *it << " ";
  std::cout << std::endl;
}

void capacity() {
  std::vector<int> vec(10);
  std::cout << std::endl << "capacity" << std::endl;

  std::cout << "std::vector<int> vec(10)" << std::endl;
  std::cout << "size " << vec.size() << std::endl;
  std::cout << "capacity " << vec.capacity() << std::endl;
  std::cout << "empty " << vec.empty() << std::endl;
  std::cout << "max_size " << vec.max_size() << std::endl;
  vec.push_back(1);
  std::cout << "push_back" << std::endl;
  std::cout << "size " << vec.size() << std::endl;
  std::cout << "capacity " << vec.capacity() << std::endl;

  vec.clear();
  std::cout << "clear" << std::endl;
  std::cout << "size " << vec.size() << std::endl;
  std::cout << "capacity " << vec.capacity() << std::endl;
  std::cout << "empty " << vec.empty() << std::endl;
  std::cout << "max_size " << vec.max_size() << std::endl;
  vec.push_back(1);
  std::cout << "push_back" << std::endl;
  std::cout << "size " << vec.size() << std::endl;
  std::cout << "capacity " << vec.capacity() << std::endl;
}

void modify() {
  std::vector<int> vec = {1, 2, 3, 4, 5};
  std::vector<int> vec2 = {4, 5, 6};

  // replacing all elements with new vector
  vec.assign(5, 2);  // {2, 2, 2, 2, 2}
  vec.assign({4, 5, 6});
  vec.assign(vec2.begin(), vec2.end());

  vec.push_back(10);
  vec.pop_back();  // remove the last element, void func

  // insert
  vec.insert(vec.begin(), 0);
  for (auto it = vec.begin(); it < vec.end(); it++) std::cout << *it << " ";
  std::cout << std::endl;
  vec.insert(vec.end(), 1);
  for (auto it = vec.begin(); it < vec.end(); it++) std::cout << *it << " ";
  std::cout << std::endl;
  vec.insert(vec.begin(), 5, 1);  // add {1, 1, 1, 1, 1}
  vec.insert(vec.begin(), vec2.begin(), vec2.end());
  vec.insert(vec.begin(), {1, 2, 1});

  // erase
  // please note that erase return a iterator that points to the next value
  auto next_iter = vec.erase(vec.begin());
  assert((*next_iter) == 2);
  next_iter = vec.erase(vec.begin(), vec.begin() + 2);
  assert((*next_iter) == 4);

  // emplace: create an element at the target position(no need to copy)
  // insert: create an element, copy to target place
  // emplace: add ONE element
  // insert: add one OR MORE elements
  vec.emplace(vec.begin(), 10);
  vec.emplace_back(10);

  vec.clear();
}

void element_access() {
  std::vector<int> vec = {1, 2, 3, 4, 5};
  std::cout << vec[4] << std::endl;
  vec[4] = 100;
  std::cout << vec[4] << std::endl;
  std::cout << "size " << vec.size() << std::endl;
  std::cout << "capacity " << vec.capacity() << std::endl;

  // no bound-check for []
  // Portable programs should never call this function with an argument n that
  // is out of range, since this causes undefined behavior.
  std::cout << vec[100] << std::endl;
  std::cout << "vec[100]" << std::endl;
  std::cout << "size " << vec.size() << std::endl;
  std::cout << "capacity " << vec.capacity() << std::endl;

  // there is bound check
  std:: cout << vec.at(4) << std::endl;
  vec.at(4) = 200;
  std:: cout << vec.at(4) << std::endl;
  // std:: cout << vec.at(200) << std::endl;

  int first_element = vec.front();
  int last_element = vec.back();
}

int main() {
  initialize();
  iter();
  capacity();
  modify();
  element_access();
  return 0;
}