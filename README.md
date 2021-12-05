# KalmanFilterAccelerator

Current performance (no of cycles) for a single update step of a 3-dimensional state vector

| Software(Microblaze without floating point unit) | Software(Microblaze with floating point unit) | Software(Intel i5 9th Gen) |
| ------------- | ------------- | ------------- |
| 415750 @ 100MHz | 187581 @ 100MHz | 1761 ns (average of 10 runs) |

| Hardware(Floating Point)  | Hardware(Fixed Point)  |
| ------------- | ------------- |
| --TODO-- | --TODO--  |


Useful Links:

- https://ieeexplore.ieee.org/document/7979508 
- https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=1784&context=etd

- https://www.xilinx.com/support/documentation/sw_manuals/xilinx2017_1/ug902-vivado-high-level-synthesis.pdf
- https://medium.com/@sapphire.sharma1996/behavioral-rtl-simulation-with-microblaze-131671e86f04
- https://gitlab.com/chandrachoodan/teach-fpga/-/tree/master/01-fft/vivado/fft-sdk
