# TrajectoryAlign 

这是一个用来估计两个轨迹之间的相对变换的程序。

## Install

### Dependencies

* Eigen3
* OpenCV
* Gflags
* Glog
* Sophus
* Ceres

### Install

```shell
git clone https://github.com/chenjianqu/TrajectoryAlign.git

cd TrajectoryAlign

mkdir -p build && cd build
cmake ..
make -j

```


## Demo

```shell
cd ${TrajectoryAlign_base}/bin

./trajectory_align \
-poses_b_path /mnt/done_data/22_GND_vslam/20230728_HS5001_2M/20230605_143344162_hq/pose_full/new_pose_imu.txt \
-poses_a_path /mnt/done_data/22_GND_vslam/20230728_HS5001_2M/20230605_143344162_hq/pose_opt/new_pose_2.txt

```





## 公式推导

### 估计旋转外参

已知一个lidar坐标系下的位姿 $\bold{R}_{\text{op}}$ 和IMU坐标系下的位姿 $\bold{R}_{\text{vins}}$ ，求旋转外参数 $\bold{R}_{\text{e}}$ 。由公式：
$$
\bold{R}_{\text{op}} =\bold{R}_{\text{e}} \bold{R}_{\text{vins}}
$$
误差：
$$
e = \ln (\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}
$$

导数：

$$
\frac{d \ln (\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee} }{d \bold{R}_e} 
= 
\lim_{\Delta \boldsymbol{ \theta} \to 0} { \frac{ 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{\text{e}}  \exp( \Delta \boldsymbol{ \theta}^{ \land } )
\bold{R}_{\text{vins}}
)^{\vee}} 
- \ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}} }{\Delta \boldsymbol{ \theta}} }
\\根据BCH的性质，有：
\\
=\lim_{\Delta \boldsymbol{ \theta} \to 0} { \frac{ 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{\text{e}} \bold{R}_{\text{vins}} \exp(( \bold{R}^{T}_{\text{vins}} \Delta \boldsymbol{ \theta})^{ \land } )
)^{\vee}} 
- 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}} }{\Delta \boldsymbol{ \theta}} }
\\
=\lim_{\Delta \boldsymbol{\theta} \to 0} { \frac{ 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{\text{e}} \bold{R}_{\text{vins}})^{\vee}} 
+
\bold{J}_r^{-1} \Delta \boldsymbol{\theta} 
- 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}} }{\Delta \boldsymbol{ \theta}} }
\\
=\bold{J}_r^{-1}
$$


其中 $\bold{J}_r^{-1}$ 的计算： $\bold{J}_r^{-1} = \bold{J}_r^{-1}( \ln (\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}  )$ 

和 $\bold{J}^{-1}_{r}$  为 SO(3) 上的右雅可比：
$$
\boldsymbol{J}_r^{-1} (\theta \boldsymbol{\omega}) = \frac{\theta}{2} \cot \frac{\theta}{2} \bold{I} + ( 1 - \frac{\theta}{2} \cot \frac{\theta}{2}) \boldsymbol{\omega} \boldsymbol{\omega} ^{T} + \frac{\theta}{2} \boldsymbol{\omega}^{\land}
$$



### 估计完整外参

已知一个lidar坐标系下的位姿 $\bold{T}_a$ 和IMU坐标系下的位姿 $\bold{T}_b$ ，求旋转外参数 $\bold{T}_{\text{e}}$ 。由公式：
$$
\bold{T}_a =\bold{T}_{\text{e}} \bold{T}_b
$$
误差：
$$
e = \ln (\bold{T}_a^{T} \bold{T}_{e} \bold{T}_b)^{\vee}
$$

下面将位姿写成 旋转和平移的形式。
$$
\bold{T}_a^{T} = 
\begin{bmatrix} 
\bold{R}_a & \bold{t}_a \\
\bold{0}^T & 1
\end{bmatrix}^{T} 
=
\begin{bmatrix} 
\bold{R}_a^T & - \bold{R}_a^T \bold{t}_a \\
\bold{0}^T & 1
\end{bmatrix}
$$
计算 $\bold{T}_a^{T} \bold{T}_{e}$ :
$$
\bold{T}_a^{T} \bold{T}_e =
\begin{bmatrix} 
\bold{R}_a^T & - \bold{R}_a^T \bold{t}_a \\
\bold{0}^T & 1
\end{bmatrix}
\begin{bmatrix} 
\bold{R}_e & \bold{t}_e \\
\bold{0}^T & 1
\end{bmatrix}
\\=
\begin{bmatrix} 
\bold{R}_a^T \bold{R}_e & \bold{R}_a^T \bold{t}_e -\bold{R}_a^T \bold{t}_a \\
\bold{0}^T & 1
\end{bmatrix}
$$
计算 $\bold{T}_a^{T} \bold{T}_{e} \bold{T}_{b}$ :
$$
\bold{T}_a^{T} \bold{T}_e \bold{T}_{b}
=
\begin{bmatrix} 
\bold{R}_a^T \bold{R}_e & \bold{R}_a^T \bold{t}_e -\bold{R}_a^T \bold{t}_a \\ \bold{0}^T & 1
\end{bmatrix} 
\begin{bmatrix} 
\bold{R}_b & \bold{t}_b \\ \bold{0}^T & 1
\end{bmatrix} \\
=
\begin{bmatrix} 
\bold{R}_a^T \bold{R}_e \bold{R}_b 
& \bold{R}_a^T \bold{R}_e \bold{t}_b + \bold{R}_a^T \bold{t}_e -\bold{R}_a^T \bold{t}_a \\ \bold{0}^T & 1
\end{bmatrix}
$$


误差方程重写成如下：
$$
\mathbf{e} \in \mathbb{R}^6 = 
\begin{bmatrix} 
\bold{R}_a^T \bold{R}_e \bold{t}_b + \bold{R}_a^T \bold{t}_e -\bold{R}_a^T \bold{t}_a \\
\ln (\bold{R}_a^{T} \bold{R}_e \bold{R}_b)^{\vee} 
\end{bmatrix}
$$


优化变量： $\mathbf{R}_e$ 和 $\mathbf{t}_e$ 

雅可比：



$$
\frac{d \ln (\bold{R}_a^{T} \bold{R}_e \bold{R}_b)^{\vee} }{d \bold{R}_e} 
=\bold{J}_r^{-1} \in \mathbb{R}^{3\times 3}
$$


其中 $\bold{J}_r^{-1}$ 的计算： $\bold{J}_r^{-1} = \bold{J}_r^{-1}( \ln (\bold{R}_a^{T} \bold{R}_e \bold{R}_b)^{\vee}  )$ 
$$
\frac{d \ln (\bold{R}_a^{T} \bold{R}_e \bold{R}_b)^{\vee} }{d \bold{t}_e} = \mathbf{0}  \in \mathbb{R}^{3\times 3}
$$

$$
\frac{d \bold{R}_a^T \bold{R}_e \bold{t}_b + \bold{R}_a^T \bold{t}_e -\bold{R}_a^T \bold{t}_a}{d \mathbf{R}_e}\\
=\frac{d \bold{R}_a^T \bold{R}_e \bold{t}_b }{d \mathbf{R}_e} \\
={\lim\limits_{\Delta \boldsymbol{ \theta}\rightarrow 0}
\frac{\bold{R}_a^T \bold{R}_e \exp( \Delta \boldsymbol{\theta}^{\land}) \bold{t}_b - 
\bold{R}_a^T \bold{R}_e \bold{t}_b
 }{\Delta \boldsymbol{\theta}}} \\
 ={\lim\limits_{\Delta \boldsymbol{ \theta}\rightarrow 0}
\frac{\bold{R}_a^T \bold{R}_e (\mathbf{I}+\Delta \boldsymbol{\theta}^{\land}) \bold{t}_b
 }{\Delta \boldsymbol{\theta}}} \\
  ={\lim\limits_{\Delta \boldsymbol{ \theta}\rightarrow 0}
\frac{\bold{R}_a^T \bold{R}_e \Delta \boldsymbol{\theta}^{\land} \bold{t}_b
 }{\Delta \boldsymbol{\theta}}} \\
   ={\lim\limits_{\Delta \boldsymbol{ \theta}\rightarrow 0}
\frac{ -\bold{R}_a^T \bold{R}_e \bold{t}_b^{\land} \Delta \boldsymbol{\theta}
 }{\Delta \boldsymbol{\theta}}} \\
 =  -\bold{R}_a^T \bold{R}_e \bold{t}_b^{\land} \in \mathbb{R}^{3\times 3}
$$



$$
\frac{d \bold{R}_a^T \bold{R}_e \bold{t}_b + \bold{R}_a^T \bold{t}_e -\bold{R}_a^T \bold{t}_a}{d \mathbf{t}_e} = \bold{R}_a^T \in \mathbb{R}^{3\times 3}
$$











