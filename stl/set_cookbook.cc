#include <assert.h>

#include <iostream>
#include <set>

void constructor() {
  int arr[5] = {1, 2, 3, 4, 5};       // construct with array
  std::set<int> iset1(arr, arr + 5);  // construct with array
  std::set<int> iset2(iset1);         // construct with another set
  std::set<int> iset3(iset1.begin(), iset1.end());  // construct with iter range
  std::set<int> iset4({1, 2, 3, 4, 5});             // construct with array

  std::set<std::string> sset1({"1", "2", "3"});
  std::set<std::string> sset2(sset1);
  std::set<std::string> sset3(sset1.begin(), sset1.end());
}

void modify() {
  std::set<int> iset({1, 2, 3, 4, 5});  // construct with array
  std::set<int> iset2({10, 11, 12});    // construct with array

  // insert
  std::pair<std::set<int>::iterator, bool> ret = iset.insert(6);
  assert(ret.second == true);
  ret = iset.insert(6);
  assert(ret.second == false);
  iset.insert(iset.end(), 7);               // insert with hint position
  iset.insert(iset2.begin(), iset2.end());  // insert with iter range
  int arr[5] = {13, 14, 15};
  iset.insert(arr, arr + 3);  // insert with arr range

  // emplace
  // Difference between insert and emplace
  // emplace: The element is constructed in-place by calling
  //  allocator_traits::construct with args forwarded. insert: either copies or
  // moves existing objects into the container.
  ret = iset.emplace(16);
  assert(ret.second == true);
  ret = iset.insert(16);
  assert(ret.second == false);

  // emplace_hint: emplace with hint position
  iset.emplace_hint(iset.end(), 17);

  // erase
  auto iter_after_erased_element = iset.erase(iset.begin());  // erase with iter
  int num_elements_erased = iset.erase(2);  // erase with value
  iter_after_erased_element =
      iset.erase(iset.begin(), iset.find(5));  // erase with iter range

  // clear
  iset.clear();
}

void iter() {
  std::set<int> iset({1, 2, 3, 4, 5});  // construct with array

  // iter all loops
  // c for const, r for reverse
  // begin/end, rbegin/rend, cbegin/cend, crbegin/crend
  for (auto it = iset.begin(); it != iset.end(); it++) {
    std::cout << (*it) << std::endl;
  }
}

void other() {
  std::set<int> iset({1, 2, 3, 4, 5});  // construct with array

  // find: get iter by value
  auto iter = iset.find(3);

  // count: whether a value exists or not
  // cnt == 1 or 0
  int cnt = iset.count(2);

  // empty
  iset.empty();

  // size
  iset.size();
}

int main(int argc, char** argv) {
  constructor();
  modify();
  iter();
  other();
  return 0;
}