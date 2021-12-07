# KalmanFilterAccelerator

### Performance
Current performance (no of cycles) for a single update step of a 3-dimensional state vector

| Software(Microblaze without floating point unit) | Software(Microblaze with floating point unit) | Software(Intel i5 9th Gen) |
| ------------- | ------------- | ------------- |
| 415750 @ 100MHz | 187581 @ 100MHz | 1761 ns (average of 10 runs) |

Hardware:
| | Write | Read | Computation | LUT | DSP | FF |
| ------------- | ------------- |
| --TODO-- | --TODO--  |


### Tasks

- Executing test C code in Microblaze :heavy_check_mark: (Arun)
- Adding and using a performance counter IP to MicroBlaze :heavy_check_mark: (Arun)
- Generate C code for Kalman filter using MATLAB :heavy_check_mark: (Arun)
- Execute Kalman Filter code in Microblaze with and without FPU :heavy_check_mark: (Arun)
- Design test ip with AXI in Vivado HLS and interface with Microblaze :heavy_check_mark: (Shakti)
- Mesaure performance of C code in Intel i5 processor :heavy_check_mark: (Arun)
- Design ip for kalman filter using floating point :heavy_check_mark: (Shakti)
- Write C code in microblaze for using the kalman filter ip generated by HLS (In Progress) (Arun)
- Design ip for kalman filter using fixed point (In Progress) (Shakti)
- Measure performance for hardware implementation
- Report (In Progress) (Shakti, Arun, Snehan)
- Video Content
- Video Editing


### Useful Links

- https://ieeexplore.ieee.org/document/7979508 
- https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=1784&context=etd

- https://www.xilinx.com/support/documentation/sw_manuals/xilinx2017_1/ug902-vivado-high-level-synthesis.pdf
- https://medium.com/@sapphire.sharma1996/behavioral-rtl-simulation-with-microblaze-131671e86f04
- https://gitlab.com/chandrachoodan/teach-fpga/-/tree/master/01-fft/vivado/fft-sdk
