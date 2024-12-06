#include "math_process.h"



float ABS(float value)
{
	if(value>0)
		value = value;
	else if (value<0)
		value = -value;
	else
		value = 0;
	return value;
}

/**
  * @Bref   斜坡数据处理
  * @Param  slope  斜坡数据结构体
  * @Param  target 最终目标值
  * @Param  scale  分度
  */
float Map(float input_min ,float input_max,float output_min ,float output_max,float value)
{
	float out;
	if(value>input_max)
	{
		value = input_max;
	}
	if(value<input_min)
	{
		value = input_min;
	}
	
	out = ((ABS(value - input_min))/(ABS(input_max - input_min))) * (ABS(output_max - output_min)) + output_min;
	
	if (out<output_min)
		  out = output_min;
	if(out>output_max)
			out = output_max;
	return out ;
}

/**
  * @Bref   斜坡数据处理
  * @Param  slope  斜坡数据结构体
  * @Param  target 最终目标值
  * @Param  scale  分度
  * @Param  inital 当前值
  */
void slope_process(slope_t *slope, float target, float scale, float inital)
{
    slope->target = target;
		slope->tmp_target = inital;
		
        if(slope->tmp_target > slope->target)
            slope->add  -= scale;
        
        else if(slope->tmp_target < slope->target)
            slope->add  += scale;
				
				slope->tmp_target +=slope->add;
				
				if(ABS(slope->tmp_target - slope->target) < (scale*2))
				{
					slope->tmp_target = slope->target;
					slope->add = 0;
				}
				
				
     
    
}


/**
  *@Bref     环形数据偏差计算
  *@Param    set 设定值 get采样值 circle_para一圈数值
  *@Note     环形数据下，直接计算出PID中的偏差值 共四种情形 error 的方向是从get指向set的优弧
  */
float circle_error(float set ,float get ,float circle_para)
{
    float error;
    if(set > get)
    {
        if(set - get> circle_para/2)
            error = set - get - circle_para;
        else
            error = set - get;
    }
    else if(set < get)
    {
        if(set - get<-1*circle_para/2)
            error = set - get +circle_para;
        else
            error = set - get;
    }
    else	error = 0;

    return error;
}


/**
  * @Bref   兼容包含偏移量的限幅
  * @Param  数据变量指针，最大值，偏移量（初始值）
  */
void abs_limit(float *a, float ABS_MAX,float offset)
{
    if(*a > ABS_MAX+offset)
        *a = ABS_MAX+offset;
    if(*a < -ABS_MAX+offset)
        *a = -ABS_MAX+offset;
}



