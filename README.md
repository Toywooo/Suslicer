# suSlicer

suSlicer是一个针对stl文件的切片工具，其重要算法参考[Rodrigo Minetto](https://www.sciencedirect.com/science/article/abs/pii/S0010448517301215)等人的工作。

![](C:\Users\wuhua\Desktop\切片过程.png)

## How to start

### Compile:

Firstly, we use [CMake](https://cmake.org) to generate visual studio solution, then compiling and linking with visual C++.

### Coding

代码中包含两个工程：主程序suSlicer,以及单元测试程序 unitTest.
主要功能由定义在suSlicer.h和suSlicer.cpp中的四个函数实现：

1. binary_searching

   对所有的三角形实现二分查找，将三角形按照切分的层分组。

2. Build_triangle_list

   将所有的三角形按照切分的z坐标分组，在后续对三角形的处理中无需遍历所有的三角形，达到节省时间的目的。

3. ComputeIntersection

   求三角形与切平面的交点。

4. Slicing

   调用其他的函数实现切片过程。

5. contour_construction

   对每一层中的点进行哈希查找，以形成轮廓，将得到的轮廓存放到vector中，对于具有多个轮廓的层不对轮廓的先后顺序做区分。

### Question

每一次查找需要对交点的x,y,z三个值进行判断，如果可以将x,y,z三个值用哈希提取特征码作为该点的索引，则可以大大节省时间；对于部分模型该程序会报错而停止运行，输出轮廓完整但是在最底层存在一个多余的点；对于多个轮廓线的层没有对轮廓做区分，后续不便于形成连续路径。

