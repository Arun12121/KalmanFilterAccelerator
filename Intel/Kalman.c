#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static double x_est[3];
static double p_est[9];
int isInitialized_kalmanfilter = 0;

/*
 * Arguments    : void
 * Return Type  : void
 */
void kalmanfilter_init(void)
{
  x_est[0] = 0.0;
  x_est[1] = 0.0;
  x_est[2] = 0.0;
  /*  x_est=[x,y,Vx,Vy,Ax,Ay]' */
  memset(&p_est[0], 0, 9U * sizeof(double));
}

void kalmanfilter_initialize(void)
{
  kalmanfilter_init();
  isInitialized_kalmanfilter = 1;
}

/*
 * Arguments    : const double z[2]
 *                double y[2]
 * Return Type  : void
 */
double kalmanfilter(double z)
{
  static const signed char b_a[9] = {1, 0, 0, 1, 1, 0, 0, 1, 1};
  static const signed char iv1[9] = {1, 1, 0, 0, 1, 1, 0, 0, 1};
  static const signed char d_a[3] = {1, 0, 0};
  static const signed char iv[3] = {1, 0, 0};
  double c_a[9];
  double p_prd[9];
  double c_B[3];
  double x_prd[3];
  double B;
  double B_tmp;
  double a;
  double b_B;
  double d;
  double d1;
  double d2;
  double d3;
  double y;
  int i;
  int k;
  int x_prd_tmp;
  signed char Q[9];
  if (!isInitialized_kalmanfilter) {
    kalmanfilter_initialize();
  }
  /*    Copyright 2010 The MathWorks, Inc. */
  /*  Initialize state transition matrix */
  /*      % [x  ] */
  /*      % [Vx] */
  /*  [Ax] */
  /*  Initialize measurement matrix */
  for (i = 0; i < 9; i++) {
    Q[i] = 0;
  }
  /*  Initial state conditions */
  /*  Predicted state and covariance */
  for (k = 0; k < 3; k++) {
    Q[k + 3 * k] = 1;
    d = 0.0;
    for (i = 0; i < 3; i++) {
      x_prd_tmp = k + 3 * i;
      d += (double)b_a[x_prd_tmp] * x_est[i];
      c_a[x_prd_tmp] = ((double)b_a[k] * p_est[3 * i] +
                        (double)b_a[k + 3] * p_est[3 * i + 1]) +
                       (double)b_a[k + 6] * p_est[3 * i + 2];
    }
    x_prd[k] = d;
  }
  /*  Estimation */
  B = 0.0;
  b_B = 0.0;
  /*  Estimated state and covariance */
  a = 0.0;
  for (i = 0; i < 3; i++) {
    d = c_a[i];
    d1 = c_a[i + 3];
    d2 = c_a[i + 6];
    B_tmp = 0.0;
    for (x_prd_tmp = 0; x_prd_tmp < 3; x_prd_tmp++) {
      k = i + 3 * x_prd_tmp;
      d3 = ((d * (double)iv1[3 * x_prd_tmp] +
             d1 * (double)iv1[3 * x_prd_tmp + 1]) +
            d2 * (double)iv1[3 * x_prd_tmp + 2]) +
           (double)Q[k];
      p_prd[k] = d3;
      B_tmp += (double)d_a[x_prd_tmp] * d3;
    }
    c_B[i] = B_tmp;
    B_tmp *= (double)iv[i];
    B += B_tmp;
    b_B += B_tmp;
    a += (double)d_a[i] * x_prd[i];
  }
  B_tmp = z - a;
  x_est[0] = x_prd[0] + c_B[0] / (b_B + 1000.0) * B_tmp;
  x_est[1] = x_prd[1] + c_B[1] / (b_B + 1000.0) * B_tmp;
  x_est[2] = x_prd[2] + c_B[2] / (b_B + 1000.0) * B_tmp;
  d = c_B[0] / (B + 1000.0);
  d1 = c_B[1] / (B + 1000.0);
  d2 = c_B[2] / (B + 1000.0);
  for (i = 0; i < 3; i++) {
    k = d_a[i];
    c_a[3 * i] = d * (double)k;
    c_a[3 * i + 1] = d1 * (double)k;
    c_a[3 * i + 2] = d2 * (double)k;
  }
  /*  Compute the estimated measurements */
  y = 0.0;
  for (i = 0; i < 3; i++) {
    d = c_a[i];
    d1 = c_a[i + 3];
    d2 = c_a[i + 6];
    for (x_prd_tmp = 0; x_prd_tmp < 3; x_prd_tmp++) {
      k = i + 3 * x_prd_tmp;
      p_est[k] = p_prd[k] -
                 ((d * p_prd[3 * x_prd_tmp] + d1 * p_prd[3 * x_prd_tmp + 1]) +
                  d2 * p_prd[3 * x_prd_tmp + 2]);
    }
    y += (double)d_a[i] * x_est[i];
  }
  return y;
}

int main()
{
	double z[6] = {-0.4761, -0.4661, -0.4561, -0.4461, -0.4361, -0.4261};
	double y_true[6] = {0.0004, -0.0019, -0.0064, -0.0209, -0.0608, -0.1430};
	int error = 0;
	double y = 0;
	y = kalmanfilter(z[0]);

  // Start measuring time
  struct timespec begin, end; 
  clock_gettime(CLOCK_REALTIME, &begin);

	for(int i=1; i<5;i++)
	{
		y = kalmanfilter(z[i]);
		if((y - y_true[i]) > 0.01 || (y - y_true[i]) < -0.01)
		{
			error += 1;
		}
	}
	
  clock_gettime(CLOCK_REALTIME, &end);
  long seconds = end.tv_sec - begin.tv_sec;
  long nanoseconds = end.tv_nsec - begin.tv_nsec;
  double elapsed = seconds + nanoseconds*1e-9;

	printf("e: %d\n",error);
    printf("Time measured: %ld nanoseconds.\n", nanoseconds);

    return 0;
}
