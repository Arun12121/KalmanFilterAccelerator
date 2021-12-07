#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xil_printf.h"

#include "xil_io.h"
#include "xparameters.h"
#include "xcounter.h"
#include "xkalmanfilter.h"

union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};


int main()
{
    init_platform();

    XCounter xc;
    XCounter* xcptr = &xc;

    XKalmanfilter kf;
    XKalmanfilter* kfptr = &kf;
    
    char buff[11];
    union conv32 xu;

	XCounter_Initialize(xcptr, 0);
	XCounter_EnableAutoRestart(xcptr);
	XCounter_Start(xcptr);

	XKalmanfilter_Initialize(kfptr, 0);

	float z[6] = {-0.4761, -0.4661, -0.4561, -0.4461, -0.4361, -0.4261};
	float y_true[6] = {-0.0004, -0.0019, -0.0064, -0.0209, -0.0608, -0.1430};
	int error = 0;
	float y = 10.0;
	xil_printf("s\n\r");
	xu.f32 = z[0];
	XKalmanfilter_Set_z(kfptr, xu.u32);
	XKalmanfilter_Start(kfptr);
	while(!XKalmanfilter_IsDone(kfptr));
	xu.u32 = XKalmanfilter_Get_return(kfptr);
	y = xu.f32;
	sprintf(buff,"%f ",y);
	xil_printf("%s\n\r",buff);

	int t1, t2, t3, t4;

	for(int i=1; i<2;i++)
	{
		t1 = XCounter_Get_return(xcptr);
		xu.f32 = z[i];
		XKalmanfilter_Set_z(kfptr, xu.u32);
		t2 = XCounter_Get_return(xcptr);
		XKalmanfilter_Start(kfptr);
		while(!XKalmanfilter_IsDone(kfptr));
		t3 = XCounter_Get_return(xcptr);
		xu.u32 = XKalmanfilter_Get_return(kfptr);
		y = xu.f32;
		t4 = XCounter_Get_return(xcptr);
		if((y - y_true[i]) > 0.001 || (y - y_true[i]) < -0.001)
		{
			sprintf(buff,"%f ",y);
			error += 1;
		}
	}
	xil_printf("e: %d     \n\r",error);
	xil_printf("wr: %d\n\r",t2-t1);
	xil_printf("cp: %d\n\r",t3-t2);
	xil_printf("rd: %d\n\r",t4-t3);
	xil_printf("%s\n\r",buff);

    cleanup_platform();
    return 0;
}
