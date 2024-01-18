### LF-PGVIO: A Visual-Inertial-Odometry Framework for Large Field-of-View Cameras using Points and Geodesic Segments [[PDF]](https://arxiv.org/abs/2306.06663)
Ze Wang, [Kailun Yang](https://yangkailun.com/), Hao Shi, Yufan Zhang, [Fei Gao](http://zju-fast.com/fei-gao/), [Kaiwei Wang](http://wangkaiwei.org/indexeg.html).


## Download PALVIO Dataset

### Indoor Dataset
ID01, ID06, ID10: [**Google Drive**](https://drive.google.com/drive/folders/1RdnUtMulDuhWBfAgq_CLp18EgDvTrZ89?usp=sharing)

ID01~ID10: [**Baidu Yun**](https://pan.baidu.com/s/1o6TgcDwfcDIFl6n9dzsysA), Code: d7wq 

ID01~ID01 parameters: [**LF-VIO**](https://github.com/flysoaryun/LF-VIO#readme)

### Outdoor Dataset
OD01~OD02: [**Baidu Yun**](https://pan.baidu.com/s/10suy_WEne2ExHogQtcC_IQ), Code: vbaq

OD01~OD02 parameters:
Pal_camera:
```
Fov: 360°x(40°~120°)

Resolution ratio: 1280x720

Lens: Designed by Hangzhou HuanJun Technology.

Sensor: mynteye module.

Frequency: 30Hz
```
Pal_camera:
```
model_type: scaramuzza
camera_name: pal
image_width: 1280
image_height: 960
poly_parameters:
   p0: -2.859221e+02 
   p1: 0.000000e+00 
   p2: 1.336620e-03 
   p3: -4.760020e-07 
   p4: 1.744120e-09 
inv_poly_parameters:
   p0: 441.554977 
   p1: 293.852005 
   p2: 37.695932 
   p3: 38.745264 
   p4: 21.601706 
   p5: 8.981299 
   p6: 4.295482 
   p7: 4.082164 
   p8: 3.517051 
   p9: 1.629742  
   p10: 0.304952
   p11: 0.0
   p12: 0.0
   p13: 0.0
   p14: 0.0
   p15: 0.0
   p16: 0.0
   p17: 0.0
   p18: 0.0 
   p19: 0.0 
affine_parameters:
   ac: 0.999977 
   ad: -0.000018 
   ae: 0.000018
   cx: 662.123354
   cy: 467.262942 
```


IMU(Kakute F7):
```
Frequency: 200Hz
acc_n: 0.02          # accelerometer measurement noise standard deviation.
gyr_n: 0.01         # gyroscope measurement noise standard deviation.    
acc_w: 0.04         # accelerometer bias random work noise standard deviation.  
gyr_w: 0.002      # gyroscope bias random work noise standard deviation.    
```

The extrinsic parameter between IMU and pal Camera
```
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0.9933825736079961, -0.006214058066792618, -0.1146841224158635,
         0.009561962263699604, 0.9995433275449167, 0.02866540141185659,
         0.1144536208672395, -0.02957231547880238, 0.9929883417380088]
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [-0.01746193243999558,0.04570631188028584,0.039410522812453984]
```

## Different FoVs and Images
<img src="figures\All_FoV.png" alt="All_FoV" width="500" />

## Our OCSD algorithm
<img src="figures\OCSD_benchmark.png" alt="OCSD_benchmark" style="zoom: 100%;" />
(a) OCSD (ours) evaluated on the SUN360 indoor dataset. (b) OCSD (ours) evaluated on the CVRG-Pano outdoor dataset. (c) ULSD evaluated
on the SUN360 indoor dataset. (d) ULSD evaluated on the CVRG-Pano outdoor dataset. The purple curve segments in the image denote the matched
pairs with dorth less than 5 pixels and overlap greater than 0.5. The green lines represent unmatched pairs.

## Our LF-PGVIO algorithm
<img src="figures\LF-PGVIO_flow.png" alt="LF-PGVIO_flow" style="zoom: 100%;" />

### Line feature residual
<img src="figures\Line_res.png" alt="LF-PGVIO_Line_res" width="500" />

Please refer to our paper for details.

## Outdoor experiment
<img src="figures\car_OD_exp.png" alt="LF-PGVIO_car" style="zoom: 100%;" />
(a) Our car experiment platform with a Panoramic Annular Lens (PAL) camera, a Livox-Mid-360 LiDAR, an IMU sensor, and an onboard computer. (b) Top view of trajectories of different algorithms and ground truth for the OD01 sequence in outdoor experiments. The car platform stacks images in a residual manner with a 0.5s interval on the first frame, and the trajectory aligns with the ground truth.


## Publication
If you use our code or dataset, please consider referencing the following paper:

**LF-PGVIO: A Visual-Inertial-Odometry Framework for Large Field-of-View Cameras using Points and Geodesic Segments.**
Z. Wang, K. Yang, H. Shi, Y. Zhang, F. Gao, K. Wang. 

```
@article{LF-PGVIO,
  title={LF-PGVIO: A Visual-Inertial-Odometry Framework for Large Field-of-View Cameras using Points and Geodesic Segments},
  author={Wang, Ze and Yang, Kailun and Shi, Hao and Zhang, Yufan and Gao, Fei and Wang, Kaiwei},
  journal={arXiv preprint arXiv:2306.06663},
  year={2023}
}
```
