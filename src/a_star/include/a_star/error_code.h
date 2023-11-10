#ifndef ERROR_CODE_H_
#define ERROR_CODE_H_

#include <string>

enum ErrorCode {
  OK = 0,
  Error = 1,
  /*Planning*/
  // Astar
  GP_MOVE_COST_ERROR = 10001,
  GP_START_INVALID_ERROR,
  GP_GOAL_INVALID_ERROR,
  GP_PATH_SEARCH_ERROR
};

class ErrorInfo {
public:
  ErrorInfo() : error_code_(ErrorCode::OK), error_msg_(""){};
  ErrorInfo(ErrorCode error_code, const std::string &error_msg = "")
      : error_code_(error_code), error_msg_(error_msg){};
  ErrorInfo &operator=(const ErrorInfo &error_info) {
    if (&error_info != this) {
      error_code_ = error_info.error_code_;
      error_msg_ = error_info.error_msg_;
    }
    return *this;
  }
  static ErrorInfo OK() { return ErrorInfo(); }
  ErrorCode error_code() const { return error_code_; };
  const std::string &error_msg() const { return error_msg_; }
  bool operator==(const ErrorInfo &rhs) {
    return error_code_ == rhs.error_code();
  }
  bool IsOK() const { return (error_code_ == ErrorCode::OK); }

  ~ErrorInfo() = default;

private:
  ErrorCode error_code_;
  std::string error_msg_;
};

#endif