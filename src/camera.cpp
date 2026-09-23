// camera.cpp -- 相机模型实现

#include "camera.h"

#include <cmath>

// 结果的第 i 个数 = 矩阵第 i 行与向量逐项相乘再相加
Point3 multiply(const Matrix3& M, const Point3& v)
{
    Point3 out;
    out.x = M.m[0][0] * v.x + M.m[0][1] * v.y + M.m[0][2] * v.z;
    out.y = M.m[1][0] * v.x + M.m[1][1] * v.y + M.m[1][2] * v.z;
    out.z = M.m[2][0] * v.x + M.m[2][1] * v.y + M.m[2][2] * v.z;
    return out;
}

Point3 add(const Point3& a, const Point3& b)
{
    return Point3{a.x + b.x, a.y + b.y, a.z + b.z};
}

// Rodrigues 公式：R = I + sin(theta) * [k]x + (1 - cos(theta)) * [k]x^2
// 其中 k 为旋转轴单位向量，theta 为旋转向量的模长
Matrix3 rotationVectorToMatrix(const Point3& r)
{
    const double theta = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);

    Matrix3 result;
    if (theta < 1e-12)
    {
        result.m[0][0] = result.m[1][1] = result.m[2][2] = 1.0;
        return result;
    }

    const double kx = r.x / theta;
    const double ky = r.y / theta;
    const double kz = r.z / theta;

    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double v = 1.0 - c;

    result.m[0][0] = c + kx * kx * v;
    result.m[0][1] = kx * ky * v - kz * s;
    result.m[0][2] = kx * kz * v + ky * s;

    result.m[1][0] = ky * kx * v + kz * s;
    result.m[1][1] = c + ky * ky * v;
    result.m[1][2] = ky * kz * v - kx * s;

    result.m[2][0] = kz * kx * v - ky * s;
    result.m[2][1] = kz * ky * v + kx * s;
    result.m[2][2] = c + kz * kz * v;

    return result;
}

// 先旋转再平移，顺序不可交换
Point3 worldToCamera(const Extrinsics& ext, const Point3& Pw)
{
    const Point3 rotated = multiply(ext.R, Pw);
    return add(rotated, ext.t);
}
