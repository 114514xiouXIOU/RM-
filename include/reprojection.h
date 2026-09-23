// reprojection.h -- 针孔模型重投影与误差评估接口
//
// 流程：
//   1) worldToCamera()      世界点 -> 相机点
//   2) projectCameraPoint() 深度检查、透视除法、内参映射 -> 像素
//   3) pixelDistance()      与观测点的像素欧氏距离

#ifndef REPROJECT_REPROJECTION_H
#define REPROJECT_REPROJECTION_H

#include "camera.h"

#include <string>

// 单点投影结果
// valid == false 时 pixel 无意义，reason 说明失败原因
struct ProjectionResult
{
    bool        valid = false;
    Point2      pixel;
    double      depth = 0.0;
    std::string reason;
};

// 单点完整评估结果
struct Evaluation
{
    bool        valid = false;
    Point2      reprojected;
    Point2      observed;
    double      error = 0.0;
    double      depth = 0.0;
    std::string reason;
};

// 相机点 -> 像素
// x_n = Xc / Zc, y_n = Yc / Zc
// u   = fx * x_n + cx
// v   = fy * y_n + cy
// Zc <= 0 时返回 valid == false
ProjectionResult projectCameraPoint(const Intrinsics& K, const Point3& Pc);

// 世界点 -> 像素（worldToCamera 与 projectCameraPoint 的组合）
ProjectionResult project(const Intrinsics& K, const Extrinsics& ext, const Point3& Pw);

// 像素平面欧氏距离 sqrt(du^2 + dv^2)，单位像素
double pixelDistance(const Point2& a, const Point2& b);

// 一条匹配的完整评估：重投影 + 像素误差
Evaluation evaluate(const Intrinsics& K, const Extrinsics& ext, const Correspondence& corr);

// 像素是否落在图像范围内（width/height <= 0 时不检查）
bool insideImage(const Intrinsics& K, const Point2& p);

#endif  // REPROJECT_REPROJECTION_H
