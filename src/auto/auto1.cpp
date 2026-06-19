#include "auto.h"


void auto1() {
    // 向前移动 10 英寸，超时 5 秒
    lemlib::MoveToPointParams params;
    lemlib::MoveToPointSettings settings;

    lemlib::moveToPoint({23.2_in, 10.3_in}, 2590_msec, params, settings);
    LiftUpDegree(-50, 340, 500);
    ClawClose();
}