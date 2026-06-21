#include "auto.h"


void auto1() {

    lemlib::MoveToPointParams params;
    lemlib::MoveToPointSettings settings;
    lemlib::moveToPoint({23.2_in, 10.3_in}, 2000_msec, params, settings);
    LiftUpDegree(-50, 340, 600);
    ClawOpen();
}