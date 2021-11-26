// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

namespace vpi {
  /**
   * Helper class for logging information
   *
   * See: http://dlecocq.github.io/blog/2011/02/24/wrapping-printf/
   */
  class Logger {
    public:
      typedef enum {
        DEBUG,
        INFO,
        WARN,
        ERROR
      } LogLevel;

      static void setLevel(LogLevel level) {
        m_currentLogLevel = level;
      }

      static void log(LogLevel level, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        if(level >= m_currentLogLevel) {
          if(level == LogLevel::DEBUG) {
            printf("DEBUG : ");
          } else if (level == LogLevel::INFO) {
            printf("INFO : ");
          } else if (level == LogLevel::WARN) {
            printf("WARN : ");
          } else if (level == LogLevel::ERROR) {
            printf("ERROR : ");
          }
          vprintf(fmt, args);
          printf("\n");
        }
        va_end(args);
      }

      static LogLevel m_currentLogLevel;
  };

  Logger::LogLevel Logger::m_currentLogLevel = Logger::LogLevel::WARN;
} // namespace vpi