#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/*#include "platform.h"
#include "xil_printf.h"

#include "xil_io.h"
#include "xparameters.h"*/
//#include "xcounter.h"

//static float x_est[3];
//static float p_est[9];
//int isInitialized_kalmanfilter = 0;
//uint8_t isInitialized_kalmanfilter = 0;

/*
 * Arguments    : void
 * Return Type  : void
 */
//memset(&p_est[0], 0, 9U * sizeof(float));
float kalmanfilter(float z);
int main()
{
    //init_platform();



	float z[6] = {-0.4761, -0.4661, -0.4561, -0.4461, 0.4361, -0.4261};
	float y_true[6] = {0.0004, -0.0019, -0.0064, -0.0209, -0.0608, -0.1430};
	int error = 0;
	float y = 0;
	//xil_printf("s\n\r");
	y = kalmanfilter(z[0]);
	//xil_printf("s1\n\r");

	//int t1 = XCounter_Get_return(xcptr);

	for(int i=1; i<2;i++)
	{
		y = kalmanfilter(z[i]);
		if((y - y_true[i]) > 0.001 || (y - y_true[i]) < -0.001)
			error += 1;
	}


	printf("e: %d\n\r",error);
	printf("y: %f",y);



    //cleanup_platform();
    return 0;
}

