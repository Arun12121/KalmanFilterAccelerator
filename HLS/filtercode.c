#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/*#include "platform.h"
#include "xil_printf.h"

#include "xil_io.h"
#include "xparameters.h"*/
//#include "xcounter.h"

static float x_est[3];
static float p_est[9];
int isInitialized_kalmanfilter = 0;
//uint8_t isInitialized_kalmanfilter = 0;

/*
 * Arguments    : void
 * Return Type  : void
 */
//memset(&p_est[0], 0, 9U * sizeof(float));
void kalmanfilter_init()
{
  x_est[0] = 0.0;
  x_est[1] = 0.0;
  x_est[2] = 0.0;
  /*  x_est=[x,y,Vx,Vy,Ax,Ay]' */
  //memset(&p_est[0], 0, 9U * sizeof(float));
  for (int i = 0; i < 9; i++) {
    p_est[i] = 0.0;
  }

}

void kalmanfilter_initialize(void)
{
  kalmanfilter_init();
  isInitialized_kalmanfilter = 1;
}

/*
 * Arguments    : const float z[2]
 *                float y[2]
 * Return Type  : void
 */
float kalmanfilter(float z)
{
  #pragma HLS INTERFACE s_axilite port = return
  #pragma HLS INTERFACE s_axilite port = z




  static const signed char b_a[9] = {1, 0, 0, 1, 1, 0, 0, 1, 1};
  static const signed char iv1[9] = {1, 1, 0, 0, 1, 1, 0, 0, 1};
  static const signed char d_a[3] = {1, 0, 0};
  static const signed char iv[3] = {1, 0, 0};
  float c_a[9];
  float p_prd[9];
  float c_B[3];
  float x_prd[3];
  float B;
  float B_tmp;
  float a;
  float b_B;
  float d;
  float d1;
  float d2;
  float d3;
  float y;
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
      d += (float)b_a[x_prd_tmp] * x_est[i];
      c_a[x_prd_tmp] = ((float)b_a[k] * p_est[3 * i] +
                        (float)b_a[k + 3] * p_est[3 * i + 1]) +
                       (float)b_a[k + 6] * p_est[3 * i + 2];
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
      d3 = ((d * (float)iv1[3 * x_prd_tmp] +
             d1 * (float)iv1[3 * x_prd_tmp + 1]) +
            d2 * (float)iv1[3 * x_prd_tmp + 2]) +
           (float)Q[k];
      p_prd[k] = d3;
      B_tmp += (float)d_a[x_prd_tmp] * d3;
    }
    c_B[i] = B_tmp;
    B_tmp *= (float)iv[i];
    B += B_tmp;
    b_B += B_tmp;
    a += (float)d_a[i] * x_prd[i];
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
    c_a[3 * i] = d * (float)k;
    c_a[3 * i + 1] = d1 * (float)k;
    c_a[3 * i + 2] = d2 * (float)k;
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
    y += (float)d_a[i] * x_est[i];
  }
  return y;
}

