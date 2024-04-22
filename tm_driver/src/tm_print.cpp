#include "tm_driver/tm_print.h"

#include <array>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <set>
#include <string>

#ifdef _WIN32
#define vsnprintf_s vsprintf_s
#else
#define vsnprintf_s vsnprintf
#endif

enum
{
  MAX_MSG_SIZE = 1024
};

void (*print_debug_function)(const std::string& fmt);
bool is_set_print_debug_function = false;
void (*print_info_function)(const std::string& fmt);
bool is_set_print_info_function = false;
void (*print_warn_function)(const std::string& fmt);
bool is_set_print_warn_function = false;
void (*print_error_function)(const std::string& fmt);
bool is_set_print_error_function = false;
void (*print_fatal_function)(const std::string& fmt);
bool is_set_print_fatal_function = false;
void (*print_once_function)(const std::string& fmt);
bool is_set_print_once_function = false;

std::set<std::string> printed_string;

bool is_print_debug_on_terminal = true;

void setup_print_debug(bool is_printing_debug)
{
  is_print_debug_on_terminal = is_printing_debug;
}

void default_debug_function_print(const std::string& msg)
{
  std::cerr << PRINT_CYAN << "[DEBUG] " << msg << "\n" << PRINT_RESET;
}

void default_print_info_function_print(const std::string& msg)
{
  std::cout << "[INFO] " << msg << "\n";
}

void default_print_warn_function_print(const std::string& msg)
{
  std::cerr << PRINT_YELLOW << "[WARN] " << msg << "\n" << PRINT_RESET;
}

void default_print_error_function_print(const std::string& msg)
{
  std::cerr << PRINT_RED << "[ERROR] " << msg << "\n" << PRINT_RESET;
}

void default_print_fatal_function_print(const std::string& msg)
{
  std::cerr << PRINT_GREEN << "[FATAL] " << msg << "\n" << PRINT_RESET;
}

void default_print_once_function_print(const std::string& msg)
{
  if (printed_string.count(msg) == 0)
  {
    std::cout << "[INFO_ONCE] " << msg << "\n";
    printed_string.insert(msg);
  }
}

int print_debug(const char* fmt, ...)
{
  std::array<char, MAX_MSG_SIZE> msg;
  va_list vl;
  va_start(vl, fmt);
  const int n = vsnprintf_s(msg.data(), MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
  if (!is_print_debug_on_terminal)
  {
    return n;
  }
  else if (is_set_print_debug_function)
  {
    print_debug_function(msg.data());
  }
  else
  {
    default_debug_function_print(msg.data());
  }
  return n;
}

int print_info(const char* fmt, ...)
{
  std::array<char, MAX_MSG_SIZE> msg;
  va_list vl;
  va_start(vl, fmt);
  const int n = vsnprintf_s(msg.data(), MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
  if (is_set_print_info_function)
  {
    print_info_function(msg.data());
  }
  else
  {
    default_print_info_function_print(msg.data());
  }
  return n;
}

int print_warn(const char* fmt, ...)
{
  std::array<char, MAX_MSG_SIZE> msg;
  va_list vl;
  va_start(vl, fmt);
  const int n = vsnprintf_s(msg.data(), MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
  if (is_set_print_warn_function)
  {
    print_warn_function(msg.data());
  }
  else
  {
    default_print_warn_function_print(msg.data());
  }
  return n;
}

int print_error(const char* fmt, ...)
{
  std::array<char, MAX_MSG_SIZE> msg;
  va_list vl;
  va_start(vl, fmt);
  const int n = vsnprintf_s(msg.data(), MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
  if (is_set_print_error_function)
  {
    print_error_function(msg.data());
  }
  else
  {
    default_print_error_function_print(msg.data());
  }
  return n;
}

int print_fatal(const char* fmt, ...)
{
  std::array<char, MAX_MSG_SIZE> msg;
  va_list vl;
  va_start(vl, fmt);
  const int n = vsnprintf_s(msg.data(), MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
  if (is_set_print_fatal_function)
  {
    print_fatal_function(msg.data());
  }
  else
  {
    default_print_fatal_function_print(msg.data());
  }
  return n;
}

int print_once(const char* fmt, ...)
{
  std::array<char, MAX_MSG_SIZE> msg;
  va_list vl;
  va_start(vl, fmt);
  const int n = vsnprintf_s(msg.data(), MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
  if (is_set_print_once_function)
  {
    print_once_function(msg.data());
  }
  else
  {
    default_print_once_function_print(msg.data());
  }
  return n;
}

void set_up_print_debug_function(void (*function_print)(const std::string& fmt))
{
  print_debug_function = function_print;
  is_set_print_debug_function = true;
}

void set_up_print_info_function(void (*function_print)(const std::string& fmt))
{
  print_info_function = function_print;
  is_set_print_info_function = true;
}

void set_up_print_warn_function(void (*function_print)(const std::string& fmt))
{
  print_warn_function = function_print;
  is_set_print_warn_function = true;
}

void set_up_print_error_function(void (*function_print)(const std::string& fmt))
{
  print_error_function = function_print;
  is_set_print_error_function = true;
}

void set_up_print_fatal_function(void (*function_print)(const std::string& fmt))
{
  print_fatal_function = function_print;
  is_set_print_fatal_function = true;
}

void set_up_print_once_function(void (*function_print)(const std::string& fmt))
{
  print_once_function = function_print;
  is_set_print_once_function = true;
}
