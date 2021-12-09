#include <stdio.h>
#include <math.h>
#include <stdlib.h>

float kalmanfilter(float z);
int main()
{
	float z[6] = {-0.4761, -0.4661, -0.4561, -0.4461, -0.4361, -0.4261};
	float y_true[6] = {-0.0004, -0.0019, -0.0064, -0.0209, -0.0608, -0.1430};
	int error = 0;
	float y = 0;
	y = kalmanfilter(z[0]);

	for(int i=1; i<6;i++)
	{
		y = kalmanfilter(z[i]);
		if((y - y_true[i]) > 0.001 || (y - y_true[i]) < -0.001)
			error += 1;
	}


	printf("e: %d\n\r",error);
	printf("y: %f",y);

    return 0;
}
