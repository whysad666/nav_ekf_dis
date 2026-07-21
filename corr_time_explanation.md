# `corr_time` 参数详细解读

`corr_time` 是 IMU 零偏一阶高斯-马尔可夫模型的相关时间，单位为秒。

当前配置为：

```yaml
corr_time: 360 # [s]
```

## 1. 数学含义

程序把陀螺和加速度计的残余零偏建模为：

$$
\dot{\boldsymbol b}_g
=
-\frac{1}{\tau}\boldsymbol b_g+\boldsymbol w_{bg}
$$

$$
\dot{\boldsymbol b}_a
=
-\frac{1}{\tau}\boldsymbol b_a+\boldsymbol w_{ba}
$$

其中：

- $\tau=\texttt{corr\_time}$
- $\boldsymbol b_g$：陀螺残余零偏
- $\boldsymbol b_a$：加速度计残余零偏
- $\boldsymbol w_{bg},\boldsymbol w_{ba}$：驱动零偏变化的白噪声

其自相关函数近似为：

$$
R_b(\Delta t)=\sigma_b^2e^{-|\Delta t|/\tau}
$$

因此 `corr_time: 360` 表示经过 360 秒后，当前零偏与初始零偏的相关性下降到：

$$
e^{-1}=0.3679
$$

对应的半相关时间为：

$$
t_{1/2}=\tau\ln 2=249.5\ \mathrm{s}
$$

## 2. 在代码中的位置

参数直接按秒读取，没有单位转换：

```cpp
corr_time = ParamCommon::getDouble(node["corr_time"], 36000.0);
```

如果配置文件没有该参数，默认值是 `36000 s`；当前明确配置成了 `360 s`。

它进入状态矩阵的位置是：

```cpp
F.block<3, 3>(BG_ID, BG_ID) =
    -1 / options_.prop_noise.corr_time * Eigen::Matrix3d::Identity();
F.block<3, 3>(BA_ID, BA_ID) =
    -1 / options_.prop_noise.corr_time * Eigen::Matrix3d::Identity();
```

离散传播近似为：

$$
\boldsymbol b_{k+1}
=
e^{-\Delta T/\tau}\boldsymbol b_k+\boldsymbol\eta_k
$$

程序每次处理两个 IMU 样本，因此代码使用 `2*dt`。200 Hz 下每次传播约为：

$$
\Delta T=0.01\ \mathrm{s}
$$

当前每次传播的衰减系数为：

$$
e^{-0.01/360}=0.9999722
$$

## 3. 它和其他零偏参数的区别

| 参数 | 作用 |
| --- | --- |
| `calib.b_g` | 启动前确定的固定陀螺零偏，直接从每帧角增量中扣除 |
| `initstd.imu_error.gyro_bias` | EKF 初始时对残余零偏的不确定度 |
| `bias_gyr_rw` | 允许陀螺残余零偏以多大强度随机变化 |
| `corr_time` | 这种残余零偏能够保持相关多长时间 |

因此，`calib.b_g` 负责扣除主要常值零偏，EKF 中的 $\boldsymbol b_g$ 负责估计剩余误差。

## 4. 参数大小的影响

### `corr_time` 较小

- 零偏误差相关性消失得快
- 状态矩阵对零偏误差的衰减更强
- 滤波器不愿长期保留同一个零偏误差
- 可能把实际慢变零偏错误归因到姿态、速度或位置

### `corr_time` 较大

- 零偏更接近常值或缓慢随机游走
- 零偏与姿态、速度误差的相关性保持更久
- 滤波器可以通过 GNSS 残差逐渐估计固定零偏
- 当 $\tau\rightarrow\infty$ 时，模型接近纯随机游走

$$
\dot{\boldsymbol b}=\boldsymbol w_b
$$

## 5. 当前 `360 s` 的具体含义

当前飞行约 1000 秒，`360 s` 意味着零偏相关性为：

$$
e^{-1000/360}\approx0.062
$$

即程序认为飞行后期零偏和起飞时零偏基本不相关。对于温度稳定、零偏缓慢变化的 IMU，这可能偏小。

另外，当前只有 GNSS 位置而没有速度观测，零偏可观性本来就比较弱。过小的 `corr_time` 可能进一步削弱滤波器持续估计同一零偏的能力。

## 6. 代码实现细节

当前程序并没有直接让已估计出的名义零偏随时间衰减。机械编排始终扣除：

```cpp
ret.dtheta = imu.dtheta - imuerror_.gyrbias * imu.dt;
```

`corr_time` 主要作用于误差状态和协方差传播，进而影响卡尔曼增益；名义零偏只在观测更新反馈时改变。因此它不是一个直接作用于陀螺数据的低通滤波器。

## 7. 结论

`corr_time` 具有以下特点：

- 不能降低陀螺白噪声
- 不能修正约两倍的比例因子
- 不能替代固定零偏扣除
- 只控制 EKF 对“残余零偏随时间变化速度”的假设

对于当前 156 数据，建议先修正零偏和陀螺比例因子，再对 `360 s`、`1800 s`、`3600 s`、`10000 s` 做对照。以当前约 1000 秒的飞行时长看，`360 s` 可能偏短。
