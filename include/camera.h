// camera.h -- 相机模型：数据类型与坐标变换接口
//
// 外参约定：世界 -> 相机，Pc = R * Pw + t
// 坐标单位：三维点与平移量使用相同长度单位
// 相机坐标系：X 向右、Y 向下、Z 向前（光轴指向 +Z）
// 像素坐标系：原点在图像左上角，u 向右增大，v 向下增大

#ifndef REPROJECT_CAMERA_H
#define REPROJECT_CAMERA_H

// 三维点（世界坐标或相机坐标）
struct Point3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 二维像素点（u 水平、v 垂直）
struct Point2
{
    double u = 0.0;
    double v = 0.0;
};

// 3x3 矩阵，行优先存储：m[行][列]
struct Matrix3
{
    double m[3][3] = {};
};

// 相机内参：fx, fy 为像素单位焦距；cx, cy 为主点
// width / height 为图像尺寸，填 0 表示不做"是否出画"检查
struct Intrinsics
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int    width  = 0;
    int    height = 0;
};

// 相机外参：R 为旋转，t 为平移
struct Extrinsics
{
    Matrix3 R;
    Point3  t;
};

// 一对匹配：世界坐标系下的三维点 + 对应的观测像素
struct Correspondence
{
    Point3 world;
    Point2 observed;
};

// 3x3 矩阵乘三维向量
Point3 multiply(const Matrix3& M, const Point3& v);

// 三维向量相加
Point3 add(const Point3& a, const Point3& b);

// 旋转向量（Rodrigues 形式）转旋转矩阵
Matrix3 rotationVectorToMatrix(const Point3& r);

// 世界坐标 -> 相机坐标：Pc = R * Pw + t
Point3 worldToCamera(const Extrinsics& ext, const Point3& Pw);

#endif  // REPROJECT_CAMERA_H
