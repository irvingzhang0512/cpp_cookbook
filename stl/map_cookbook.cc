#include <assert.h>

#include <iostream>
#include <map>
#include <string>

void constructor() {
  // empty map
  std::map<int, std::string> map1;
  map1[1] = "one";
  map1[2] = "two";
  map1[3] = "three";
  map1[4] = "four";

  // create by another map
  std::map<int, std::string> map2(map1);

  // create by iter
  std::map<int, std::string> map3(map1.begin(), map1.end());

  std::map<int, std::string> map4 = {
      {1, "one"}, {2, "two"}, {3, "three"}, {4, "four"}};
}

void modify() {
  std::map<int, std::string> m = {
      {1, "one"}, {2, "two"}, {3, "three"}, {4, "four"}};
  std::map<int, std::string> m2 = {{8, "eight"}, {9, "nine"}};

  // insert
  std::pair<std::map<int, std::string>::iterator, bool> ret =
      m.insert(std::pair<int, std::string>(5, "five"));
  assert(ret.second == true);
  // Please note that: insert will fail if key already exists.
  ret = m.insert(std::pair<int, std::string>(1, "ONE"));
  assert(m[1].compare("one") == 0);  // m[1] doesn't change
  assert(ret.second == false);
  m.insert(m2.begin(), m2.end());                            // insert range
  m.insert(m.end(), std::pair<int, std::string>(6, "six"));  // insert with hint

  // emplace
  // difference between insert and emplace:
  // insert: either copies or moves existing objects into the container
  // emplace: The element is constructed in-place by calling
  //  allocator_traits::construct with args forwarded.
  m.emplace(1, "one");
  ret = m.emplace(1, "ONE");
  assert(m[1].compare("one") == 0);  // m[1] doesn't change
  assert(ret.second == false);       // fail to insert

  // emplace_hint, emplace with hink position
  m.emplace_hint(m.end(), 7, "seven");

  // erase
  m.erase(1);                     // erase by key
  m.erase(m.begin());             // erase by iter
  m.erase(m.begin(), m.find(5));  // erase by iter range

  // clear
  m.clear();
}

void iter() {
  std::map<int, std::string> m = {
      {1, "one"}, {2, "two"}, {3, "three"}, {4, "four"}};

  // find, get iter by key
  auto iter = m.find(2);

  // iter, c == const, r == reverse
  // cannot compare iters with < <= > >=
  // begin/end cbegin/cend rbegin/rend crbegin/crend
  for (auto it = m.begin(); it != m.end(); it++) {
    std::cout << (*it).first << " " << (*it).second << std::endl;
    ;
  }
}

void element_ops() {
  std::map<int, std::string> m = {
      {1, "one"}, {2, "two"}, {3, "three"}, {4, "four"}};

  // access by []
  std::string value = m[1];
  m[1] = "ONE";

  // access by at
  value = m.at(1);
  m.at(1) = "one";
}

void other() {
  std::map<int, std::string> m = {
      {1, "one"}, {2, "two"}, {3, "three"}, {4, "four"}};

  // cnt could only be 0 or 1, whether this map contains this key
  int cnt = m.count(2);
}

int main(int argc, char** argv) {
  constructor();
  modify();
  element_ops();
  iter();
  other();
  return 0;
}