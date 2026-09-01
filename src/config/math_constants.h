/**
 * @file math_constants.h
 * @brief 跨ARMCC和主机工具链共享的单精度数学常量。
 */

#ifndef AGV_MATH_CONSTANTS_H
#define AGV_MATH_CONSTANTS_H

/* ISO C math.h does not require M_PI; keep one typed project constant. */
static const float AGV_MATH_PI_F = 3.14159265358979323846f;

#endif
