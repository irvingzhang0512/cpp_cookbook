#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

void foo() {
  for (int i = 0; i < 5; i++) {
    std::cout << "foo " << i << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void bar(int n) {
  for (int i = 0; i < 5; i++) {
    std::cout << "bar " << n << " " << i << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void bar_ref(int& n) {
  for (int i = 0; i < 5; i++) {
    std::cout << "fn_with_args " << n << " " << i << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char** argv) {
  // init
  int n = 0;
  std::thread t1;  // t1 is not a thread
  std::thread t2(foo);
  std::thread t3(bar, n + 1);            // pass by value
  std::thread t4(bar_ref, std::ref(n));  // pass by reference
  std::thread t5(
      std::move(t3));  // t5 is now running f2(). t3 is no longer a thread
  std::thread t6([]() {
    for (int i = 0; i < 5; i++) {
      std::cout << "lambda " << i << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  // other member functions
  t2.get_id();
  t2.joinable();
  t2.detach();
  t4.join();
  t5.join();
  t6.join();

  std::cout << "Main Thread id " << std::this_thread::get_id() << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // sleep_until
  // http://www.cplusplus.com/reference/thread/this_thread/sleep_until/
  using std::chrono::system_clock;
  std::time_t tt = system_clock::to_time_t(system_clock::now());
  struct std::tm* ptm = std::localtime(&tt);
  std::cout << "Current time: " << std::put_time(ptm, "%X") << '\n';
  std::cout << "Waiting for the next minute to begin...\n";
  ++ptm->tm_min;
  ptm->tm_sec = 0;
  std::this_thread::sleep_until(system_clock::from_time_t(mktime(ptm)));
  std::cout << std::put_time(ptm, "%X") << " reached!\n";

  // https://www.zhihu.com/question/52892878
  std::this_thread::yield();

  return 0;
}