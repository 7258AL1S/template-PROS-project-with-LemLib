#ifdef __cplusplus
#include "main.h"
#include "motorcontrol.h"
#include "sensor.h"
#include "api.h"
#include "lemlib/lemlib.hpp"
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif


#include "auto_common.h"
/**
 * @brief 开局右转
 * @param StopFlag 1->1pin  2->2pin  3->3pin
 */
void auto1(int StopFlag);
/**
 * @brief 开局左转
 * @param StopFlag 1->1pin  2->2pin  3->3pin
 */
void auto2(int StopFlag);