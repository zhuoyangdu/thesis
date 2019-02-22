//
//  timer.h
//  rrt
//
//  Created by zhuoyang on 2018/11/13.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

#ifndef timer_h
#define timer_h
#include <chrono>

namespace planning {
class Timer {
public:
    Timer() {
        start_ = std::chrono::system_clock::now();
    }
    
    double duration() {
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start_;
        return elapsed_seconds.count();
    }

private:
    std::chrono::system_clock::time_point start_;
};
    
}
#endif /* timer_h */
