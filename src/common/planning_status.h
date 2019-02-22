// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_COMMON_PLANNING_STATUS_H_
#define SRC_PLANNING_SRC_COMMON_PLANNING_STATUS_H_

#include <string>

namespace planning {
    
    enum ErrorCode {
        OK = 0,
        CRASH
    };
    
    class PlanningStatus {
    public:
        explicit PlanningStatus(ErrorCode code = ErrorCode::OK)
        : code_(code), msg_("") {
        }
        
        PlanningStatus(ErrorCode code, const std::string& msg)
        : code_(code), msg_(msg) {
        }
        
        static PlanningStatus OK() {return PlanningStatus(); }
        
        ErrorCode code() const {return code_; }
        
        bool ok() const { return code_ == ErrorCode::OK; }
        
        std::string ToString() const {
            if (code_ == ErrorCode::OK) {
                return "OK";
            }
            return "ERROR: " + msg_;
        }
        
    private:
        ErrorCode code_;
        std::string msg_;
    };
}  // namespace planning

#endif  // SRC_PLANNING_SRC_COMMON_PLANNING_STATUS_H_
