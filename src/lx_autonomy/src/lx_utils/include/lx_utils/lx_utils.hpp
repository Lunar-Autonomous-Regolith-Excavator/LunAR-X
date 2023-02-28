#ifndef LX_UTILS_H
#define LX_UTILS_H

struct lock_struct{
    bool mobility_lock;
    bool actuation_lock;
};

enum class JoyButtons : int {
                            START = 7, BACK = 6, GUIDE = 8, 
                            A = 0, B = 1, X = 2, Y = 3, 
                            LEFT_BUMP = 4, RIGHT_BUMP = 5,
                            LEFT_STICK_PRESS = 9, 
                            RIGHT_STICK_PRESS = 10
                        };

enum class JoyAxes : int {
                        D_V = 7, D_H = 6, 
                        LEFT_STICK_V = 1, LEFT_STICK_H = 0,  
                        RIGHT_STICK_V = 4, RIGHT_STICK_H = 3, 
                        RIGHT_TRIG = 5, LEFT_TRIG = 2
                     };

enum class OpModeEnum : int {
                            STANDBY = 0, 
                            TELEOP = 1, 
                            AUTONOMOUS = 2
                        };

enum class TaskModeEnum : int {
                            IDLE = 0, 
                            NAV = 1, 
                            EXC = 2, 
                            DMP = 3
                          };

void placeHolderFunction();

#endif