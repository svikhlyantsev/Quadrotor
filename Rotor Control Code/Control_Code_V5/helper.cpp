#include "helper.h"
#include <math.h>	

double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double toDeg(double rad){
 return rad*180/PI; 
}

double toRad(double deg){
	return deg*PI/180;
}

double lpfBasic(double data, double filterVal, double smoothVal){
	if (filterVal > 1){      // check to make sure param's are within range
	    filterVal = .999;
	}
	else if (filterVal <= 0){
		filterVal = 0;
	}
	smoothVal = data*(1.0 - filterVal) + smoothVal*filterVal; 
	return smoothVal; 
}