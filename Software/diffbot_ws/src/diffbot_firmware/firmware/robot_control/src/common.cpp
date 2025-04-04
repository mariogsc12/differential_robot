
#include <common.h>

int saturate(int value,int min,int max){
    if(value>max)value=max;
    else if(value<min)value=min;
    return value;
}

