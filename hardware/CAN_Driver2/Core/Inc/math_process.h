#ifndef __MATH_PROCESS_H
#define __MATH_PROCESS_H


#include "stdint.h"
#include "string.h"


//输出限幅
#define Output_Limit(output,max,min) \
        ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))

//绝对值
//#define ABS(x) ((x>0)? (x): (-x))


/*斜坡处理结构体类型*/
typedef struct _slope
{
    float target;
    float tmp_target;
		float add;
	
} slope_t;

float Map(float input_min ,float imput_max,float output_min ,float output_max,float value);
void abs_limit(float *a, float ABS_MAX,float offset);
void slope_process(slope_t *slope, float target, float scale, float inital);
float circle_error(float set ,float get ,float circle_para);
float ABS(float value);

#endif
