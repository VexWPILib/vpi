// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

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

      Logger(LogLevel level=LogLevel::WARN) : m_currentLogLevel(level) {}

      void setLevel(LogLevel level) {
        m_currentLogLevel = level;
      }

      void log(LogLevel level, const char* fmt, ...) {
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

      private:
        LogLevel m_currentLogLevel;
  };

  extern Logger logger;
} // namespace vpi