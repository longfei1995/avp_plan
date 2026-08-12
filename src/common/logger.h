#pragma once
// #ifdef LOCAL_DEBUG

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/common.h>
#include <spdlog/logger.h>
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <iostream>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

class Logger {
public:
  static Logger& getInstance() {
    static Logger instance;
    return instance;
  }

  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  std::shared_ptr<spdlog::logger> getLogger() {
    return logger_;
  }

private:
  Logger() {
    try {
      // 创建log目录
      mkdir("log", 0755);

      // 创建两个sink
      std::vector<spdlog::sink_ptr> sinks;

      // 1. 控制台sink（带颜色）
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::debug);
      sinks.push_back(console_sink);

      // 2. 文件sink（轮转，每个文件20MB，最多保存5个文件）
      auto file_sink =
          std::make_shared<spdlog::sinks::rotating_file_sink_mt>("log/app.log",
                                                                 20 * 1024 * 1024,  
                                                                 5                  
          );
      file_sink->set_level(spdlog::level::debug);
      sinks.push_back(file_sink);

      // 创建logger
      logger_ = std::make_shared<spdlog::logger>("multi_sink", sinks.begin(), sinks.end());

      // 设置日志级别为Debug
      logger_->set_level(spdlog::level::debug);

      // 设置日志格式：[时间戳] [级别] [文件:行号] 消息
      logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v");

      // 注册为全局logger
      spdlog::register_logger(logger_);

      // 设置为默认logger
      spdlog::set_default_logger(logger_);

      // 自动刷新
      logger_->flush_on(spdlog::level::debug);

    } catch (const spdlog::spdlog_ex& ex) {
      std::cerr << "Log initialization failed: " << ex.what() << "\n";
    }
  }

  ~Logger() {
    if (logger_) {
      logger_->flush();
    }
  }
 
private:
  std::shared_ptr<spdlog::logger> logger_;
};

#define logDebug(...) SPDLOG_LOGGER_DEBUG(Logger::getInstance().getLogger(), __VA_ARGS__)
#define logInfo(...) SPDLOG_LOGGER_INFO(Logger::getInstance().getLogger(), __VA_ARGS__)
#define logWarn(...) SPDLOG_LOGGER_WARN(Logger::getInstance().getLogger(), __VA_ARGS__)
#define logError(...) SPDLOG_LOGGER_ERROR(Logger::getInstance().getLogger(), __VA_ARGS__)

// #else  // 使用gslog
// #include "gslog/logging.h"
// namespace gac::pnp::dp::interactive_decider::element_avp_apa {
// #define logDebug(...) HFLOGM_D("GPltDP", __VA_ARGS__)
// #define logInfo(...) HFLOGM_I("GPltDP", __VA_ARGS__)
// #define logWarn(...) HFLOGM_W("GPltDP", __VA_ARGS__)
// #define logError(...) HFLOGM_E("GPltDP", __VA_ARGS__)

// #endif