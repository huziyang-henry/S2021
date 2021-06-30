
# 作业二完成情况

## 完成内容

- [x] 正确地提交所有必须的文件，且代码能够编译运行。
- [x] 正确实现三角形栅格化算法。
- [x] 正确测试点是否在三角形内。
- [x] 正确实现 z-buffer 算法, 将三角形按顺序画在屏幕上。
- [x] 用 super-sampling 处理 Anti-aliasing。

## 函数功能说明

### 1. insideTriangle

采用点与3条边叉积的方向是否共向来检测点是否在三角形内。

### 2. normal_sampling，z-buffer算法

检测当前点的深度是比深度缓冲区的值小，如果小则写入深度缓冲区，并采用该点的颜色。

### 3. super_sampling， 提高项

使用std::vector<std::array<float, 4>> super_sampling_depth_buf来维护每个像素内的深度值，计算深度时需要对像素内的2x2的格子分别进行计算，然后根据权重计算出最后的颜色值。

## 其他功能说明

### 1. 直接运行./Rasterizer，点击s键可以切换normal_sampling和super_sampling

```cpp
if(key == 's')
{
    r.use_super_sampling = !r.use_super_sampling;
}
```

### 2. 输出图片时候，传入一个参数导出为normal_sampling，传入两个参数时候导出为super_sampling

```bash
./Rasterizer xxx.png # 一个参数，采用normal_sampling方式，xxx.png为图片名称
./Rasterizer xxx.png o # 两个参数，采用super_sampling，第一个参数是图片名称，第二个参数任意
```
