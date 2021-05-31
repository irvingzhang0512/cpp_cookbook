#include <assert.h>

#include <iostream>
#include <string>

void initialize() {
  // Inialize with construct
  std::string str1;
  std::string str2("char arrays");
  str1 = std::string(str2, 1, 5);  // substring "har a"
  assert(str1.compare("har a") == 0);
  const char *const_chars = "const chars";
  str1 = std::string(const_chars);
  str1 = std::string(const_chars, 3);  // substring "cha"
  assert(str1.compare("con") == 0);
  str1 = std::string(10, 'a');  // string with 10 'a'
  assert(str1.compare("aaaaaaaaaa") == 0);

  // Inialize with =
  str1 = 'c';
  str1 = "1234567890";
  std::string str3 = str1;

  // substring
  // substr(pos, size=npos)
  str1 = str2.substr(5);
  assert(str1.compare("arrays") == 0);
  str1 = str2.substr(5, 5);
  assert(str1.compare("array") == 0);
}

void string_operations() {
  std::string str1 = "12345";
  std::string str2 = "45678";

  // compare two strings
  // if values are the same, then res = 0
  // str1 > str2 means the first different char that does not match is greater
  // than str2, or all compared characters are match but the length of str1 is
  // longer
  int res = str1.compare(str2);

  // convert from string to char*
  // c_str() ends with '\0'
  const char *chars1 = str1.data();   // string to char array
  const char *chars2 = str1.c_str();  // string to char array
  char *chars_normal = (char *)str1.data();
  chars_normal = (char *)str1.c_str();
}

void find() {
  std::string str1 = "12345";
  std::string str2 = "45678";

  // find
  // if no match, then found == std::string::npos, aka -1
  std::size_t found = str1.find('1');
  found = str1.find('1', 2);
  found = str1.find("12");
  found = str1.find("34", 2);

  // rfind, find from right to left
  // just like `find`
  found = str1.rfind('1');
  found = str1.rfind('1', 2);
  found = str1.rfind("12");
  found = str1.rfind("34", 2);

  // find_first_of
  // search the first character that matches any of the specified augments
  // in this case, "Pl**s*, r*pl*c* th* v*w*ls *n th*s s*nt*nc* by *st*r*sks."
  std::string str("Please, replace the vowels in this sentence by asterisks.");
  found = str.find_first_of("aeiou");
  while (found != std::string::npos) {
    str[found] = '*';
    found = str.find_first_of("aeiou", found + 1);
  }

  // find_last_of: search from the end
  // find_first_not_of: search first character that doesn't match any of the
  // args find_last_not_of: search from the end
}

void modify() {
  const char* chars = "123456";
  std::string str1 = "12345";
  std::string str2 = "45678";

  // append
  str1.push_back('6');
  str1 += str2;
  str1 += '6';
  str1 += chars;
  str1.append(str2);
  str1.append(str2, 1, 2);
  str1.append(chars);
  str1.append(chars, 2);
  str1.append(5, 'a');

  // assign: assign a new value to replace the current string
  str1.assign(str1);
  str1.assign(str1, 2, 3);
  str1.assign(chars);
  str1.assign(chars, 3);
  str1.assign('a', 5);

  // insert
  // the insert position is the first character index of the interted string
  // so 0 <= pos <= origin_string.length
  str1.insert(2, str2);
  str1.insert(2, str2, 2, 3);
  str1.insert(2, chars);
  str1.insert(2, chars, 2);
  str1.insert(2, 5, 'a');

  // erase(pos, len)
  str1.erase(2, 5);

  // remove the last char
  str1.pop_back();

  // replace ONE substring with a new string
  // Doesn't like python, couldn't replace a specific str with a new string
  str1.replace(1, 3, str2);
  str1.replace(1, 3, str2, 2, 3);
  str1.replace(1, 3, chars);
  str1.replace(1, 3, chars, 2);
  str1.replace(1, 3, 5, 'a');
  str1.replace(str1.begin() + 2, str1.end(), str2);

  // swap the whole string betwwen two strings
  str1.swap(str2);

  // element accesss
  str1[1] = 'b';
  char b = str1[3];
  b = str1.at(3);
  b = str1.front();
  b = str1.back();
}

void iter() {
  std::string str = "12345";

  // there are four types of iter
  // begin/end, rbegin/rend, cbegin/cend, crbegin, crend
  // `r` refers to `reverse`, which means loop from right to left
  // `c` refers to `const`, which means we cannot modify the value of the elements
  for (auto it = str.begin(); it < str.end(); it++) {
    std::cout << *it << std::endl;
  }
}

void capacity() {
  std::string str = "12345";

  // length and size are equal
  assert(str.length() == str.size());
  str.clear();
  assert(str.empty());

  // allocated storage
  // capacity is not always equal to size/length
  str.capacity();
}

int main() {
  initialize();
  string_operations();
  find();
  modify();

  return 0;
}