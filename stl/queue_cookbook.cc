#include <deque>
#include <list>
#include <queue>

int main(int argc, char **argv) {
  // init
  std::deque<int> d(3, 100);
  std::list<int> l(3, 100);
  std::queue<int> q1;
  std::queue<int> q2({1, 2, 3});
  std::queue<int> q3(d);
  std::queue<int, std::list<int> > q4(l);

  // add
  q1.push(1);
  q1.emplace(1);

  // pop
  q1.pop();

  // get
  q1.front();
  q1.back();

  // size
  q1.empty();
  q1.size();

  return 0;
}