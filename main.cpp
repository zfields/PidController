/* Created and copyrighted by Zachary J. Fields. Offered as open source under the MIT License (MIT). */

#include <cstdint>
#include <iostream>

#include "pid_controller.h"

int main (int argc, char * argv[]) {
    PidController pid([](){
        static size_t i = 0;
        float process_variable[31] = { 35.7f, 35.8, 36.1, 36.5, 36.9, 37.5, 38.2, 39.0, 40.1, 40.9, 41.7, 42.5, 43.1, 43.6, 43.9, 44.0, 43.9, 43.7, 43.3, 42.8, 42.4, 42.1, 42.2, 42.4, 42.4, 42.3, 42.2, 42.3, 42.3, 42.3, 42.3 };
        std::cout << "\t" << process_variable[i] << " = sampleProcessVariable();" << std::endl;
        return process_variable[i++];
    }, 1, .25, 1);

    pid.setPoint(42.3);
    std::cout << pid.setPoint() << " = PID.setPoint();" << std::endl;
    for (size_t i = 0 ; i < 30 ; ++i ) {
        std::cout << pid.controlVariable() << " = PID.controlVariable();" << std::endl;
    }

    return 0;
}

/* Created and copyrighted by Zachary J. Fields. Offered as open source under the MIT License (MIT). */

