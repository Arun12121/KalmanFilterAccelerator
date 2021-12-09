#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ap_fixed.h>

/*
 * Arguments    : const float z[2]
 *                float y[2]
 * Return Type  : void
 */
float kalmanfilter(float zbuff)
{
  #pragma HLS INTERFACE s_axilite port = return
  #pragma HLS INTERFACE s_axilite port = zbuff



  static ap_fixed<32,16> x_est[3];
  static ap_fixed<32,16> p_est[9];

  static const ap_fixed<32,16> b_a[9] = {1, 0, 0, 1, 1, 0, 0, 1, 1};
  static const ap_fixed<32,16> iv1[9] = {1, 1, 0, 0, 1, 1, 0, 0, 1};
  static const ap_fixed<32,16> d_a[3] = {1, 0, 0};
  static const ap_fixed<32,16> iv[3] = {1, 0, 0};
  ap_fixed<32,16> c_a[9];
  ap_fixed<32,16> p_prd[9];
  ap_fixed<32,16> c_B[3];
  ap_fixed<32,16> x_prd[3];
  ap_fixed<32,16> B;
  ap_fixed<32,16> B_tmp;
  ap_fixed<32,16> a;
  ap_fixed<32,16> b_B;
  ap_fixed<32,16> d;
  ap_fixed<32,16> d1;
  ap_fixed<32,16> d2;
  ap_fixed<32,16> d3;
  ap_fixed<32,16> z = zbuff;
  ap_fixed<32,16> y;
  float ybuff;
  int i;
  int k;
  int x_prd_tmp;
  signed char Q[9];
  //if (!isInitialized_kalmanfilter) {
   // kalmanfilter_initialize();
  //}
  /*    Copyright 2010 The MathWorks, Inc. */
  /*  Initialize state transition matrix */
  /*      % [x  ] */
  /*      % [Vx] */
  /*  [Ax] */
  /*  Initialize measurement matrix */
  for (i = 0; i < 9; i++) {
#pragma HLS UNROLL
    Q[i] = 0;
  }
  /*  Initial state conditions */
  /*  Predicted state and covariance */
  for (k = 0; k < 3; k++) {
#pragma HLS UNROLL
    Q[k + 3 * k] = 1;
    d = 0.0;
    for (i = 0; i < 3; i++) {
#pragma HLS UNROLL
      x_prd_tmp = k + 3 * i;
      d += b_a[x_prd_tmp] * x_est[i];
      c_a[x_prd_tmp] = (b_a[k] * p_est[3 * i] +
                        b_a[k + 3] * p_est[3 * i + 1]) +
                       b_a[k + 6] * p_est[3 * i + 2];
    }
    x_prd[k] = d;
  }
  /*  Estimation */
  B = 0.0;
  b_B = 0.0;
  /*  Estimated state and covariance */
  a = 0.0;
  for (i = 0; i < 3; i++) {
#pragma HLS UNROLL
    d = c_a[i];
    d1 = c_a[i + 3];
    d2 = c_a[i + 6];
    B_tmp = 0.0;
    for (x_prd_tmp = 0; x_prd_tmp < 3; x_prd_tmp++) {
#pragma HLS UNROLL
      k = i + 3 * x_prd_tmp;
      d3 = ((d * iv1[3 * x_prd_tmp] +
             d1 * iv1[3 * x_prd_tmp + 1]) +
            d2 * iv1[3 * x_prd_tmp + 2]) +
           Q[k];
      p_prd[k] = d3;
      B_tmp += d_a[x_prd_tmp] * d3;
    }
    c_B[i] = B_tmp;
    B_tmp *= iv[i];
    B += B_tmp;
    b_B += B_tmp;
    a += d_a[i] * x_prd[i];
  }
  B_tmp = z - a;
  x_est[0] = x_prd[0] + c_B[0] / (b_B + 1000) * B_tmp;
  x_est[1] = x_prd[1] + c_B[1] / (b_B + 1000) * B_tmp;
  x_est[2] = x_prd[2] + c_B[2] / (b_B + 1000) * B_tmp;
  d = c_B[0] / (B + 1000);
  d1 = c_B[1] / (B + 1000);
  d2 = c_B[2] / (B + 1000);
  for (i = 0; i < 3; i++) {
#pragma HLS UNROLL
    k = d_a[i];
    c_a[3 * i] = d * k;
    c_a[3 * i + 1] = d1 * k;
    c_a[3 * i + 2] = d2 * k;
  }
  /*  Compute the estimated measurements */
  y = 0.0;
  for (i = 0; i < 3; i++) {
#pragma HLS UNROLL
    d = c_a[i];
    d1 = c_a[i + 3];
    d2 = c_a[i + 6];
    for (x_prd_tmp = 0; x_prd_tmp < 3; x_prd_tmp++) {
#pragma HLS UNROLL
      k = i + 3 * x_prd_tmp;
      p_est[k] = p_prd[k] -
                 ((d * p_prd[3 * x_prd_tmp] + d1 * p_prd[3 * x_prd_tmp + 1]) +
                  d2 * p_prd[3 * x_prd_tmp + 2]);
    }
    y += d_a[i] * x_est[i];
  }
  ybuff = y;
  return ybuff;
}

