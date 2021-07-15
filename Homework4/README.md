# 作业四完成情况

## 完成内容

- [x] 提交格式正确，包括所有需要的文件。代码可以正常编译、执行。
- [x] De Casteljau 算法
- [x] 反走样

## 结果输出

- my_bezier_curve.png，实现De Casteljau算法的贝塞尔曲线
- my_bezier_curve_aa.png，对上面的贝塞尔曲线进行反走样处理

## 其他补充

- 函数bezier_anti_aliasing是贝塞尔曲线的反走样处理函数
- anti_aliasing是对曲线上的点进行反走样处理的具体步骤，采用针对每个点周围9个点进行颜色差值处理。可修改函数中的N值，来决定采用周围多少个点（如N=2，就是2x2个像素）
- 在main函数中添加了bezier_anti_aliasing的调用，并把结果输出到了my_bezier_curve_aa.png中。