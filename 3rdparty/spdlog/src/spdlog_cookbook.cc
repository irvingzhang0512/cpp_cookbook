#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/spdlog.h"

int main() {
  // One sink one logger
  auto stdout_color_mt_logger =
      spdlog::stdout_color_mt("stdout_color_mt_logger");
  auto stdout_logger_mt = spdlog::stdout_logger_mt("stdout_logger_mt");
  auto rotating_logger_mt =
      spdlog::rotating_logger_mt("rotating_logger_mt",  // logger name
                                 "logs/rotating",       // log file path
                                 1048576 * 5,           // max file size
                                 3                      // max file
      );
  auto file_logger_mt =
      spdlog::basic_logger_mt("basic_file_logger_mt", "logs/file");
  auto daily_logger_mt =  // log filename basename.YYYY-MM-DD.ext
      spdlog::daily_logger_mt("daily_logger_mt",  // logger name
                              "logs/daily.txt",   // log file path
                              2,                  // hour
                              30                  // minute
      );  // create log file at given time point
  stdout_color_mt_logger->info("stdout_color_mt_logger info");
  stdout_logger_mt->info("stdout_logger_mt info");
  rotating_logger_mt->info("rotating_logger_mt info");
  file_logger_mt->info("file_logger_mt info");
  daily_logger_mt->info("daily_logger_mt info");

  // create different sinks for the same logger
  // sinks list could be found
  // [here](https://github.com/gabime/spdlog/tree/v1.x/include/spdlog/sinks/)

  // console sink
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::warn);
  console_sink->set_pattern("[multi_sink_example] [%^%l%$] %v");

  // file sink
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      "logs/multisink.txt", true);
  file_sink->set_level(spdlog::level::trace);

  // create logger based on console and file sink
  spdlog::logger logger("multi_sink", {console_sink, file_sink});
  logger.set_level(spdlog::level::debug);  // global level
  // level
  // spdlog::level::trace
  // spdlog::level::debug
  // spdlog::level::info
  // spdlog::level::warn
  // spdlog::level::err
  // spdlog::level::critical
  // spdlog::level::off

  // all kinds of logging
  logger.warn("this should appear in both console and file");
  logger.info(
      "this message should not appear in the console, only in the file");

  // python like formatting
  logger.info("Hello {} {} !!", "param1", 123.4);

  // register and get by logger name
  spdlog::register_logger(std::make_shared<spdlog::logger>(
      logger));  // register by its name 'multi_sink'
  auto another_logger = spdlog::get("multi_sink");
  another_logger->info("this logger is got by `spdlog::get(logger_name)`");
  spdlog::drop("multi_sink");
  // spdlog::drop_all();

  spdlog::warn("Easy padding in numbers like {:08d}", 12);
  spdlog::warn(
      "Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
  spdlog::info("Support for floats {:03.2f}", 1.23456);
  spdlog::info("Positional args are {1} {0}..", "too", "supported");
  spdlog::info("{:<30}", "left aligned");

  return 0;
}