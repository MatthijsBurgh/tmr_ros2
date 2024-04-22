#ifndef TM_PRINT_H_
#define TM_PRINT_H_

#include <string>

void set_up_print_debug_function(void (*function_print)(const std::string& fmt));
void set_up_print_info_function(void (*function_print)(const std::string& fmt));
void set_up_print_warn_function(void (*function_print)(const std::string& fmt));
void set_up_print_error_function(void (*function_print)(const std::string& fmt));
void set_up_print_fatal_function(void (*function_print)(const std::string& fmt));
void set_up_print_once_function(void (*function_print)(const std::string& fmt));

void default_print_once_function_print(const std::string& msg);

static const std::string PRINT_RED("\033[0;31m");
static const std::string PRINT_GREEN("\033[1;32m");
static const std::string PRINT_YELLOW("\033[1;33m");
static const std::string PRINT_CYAN("\033[0;36m");
static const std::string PRINT_RESET("\033[0m");

/*int (*print_info_function)(const std::string& fmt, ...);
int (*print_warn_function)(const std::string& fmt, ...);
int (*print_error_function)(const std::string& fmt, ...);
int (*print_fatal_function)(const std::string& fmt, ...);
*/
int print_debug(const char* fmt, ...);
int print_info(const char* fmt, ...);
int print_warn(const char* fmt, ...);
int print_error(const char* fmt, ...);
int print_fatal(const char* fmt, ...);
int print_once(const char* fmt, ...);

void setup_print_debug(bool is_printing_debug);

#endif
