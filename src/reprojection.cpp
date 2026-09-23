// reprojection.cpp -- 针孔模型重投影与误差评估实现

#include "reprojection.h"

#include <cmath>

ProjectionResult projectCameraPoint(const Intrinsics& K, const Point3& Pc)
{
    ProjectionResult result;
    result.depth = Pc.z;

    // 非正深度处理：点在相机平面上或相机后方时无法成像
    // Zc < 0 直接做透视除法会把相机背后的点翻折到画面前方，
    // 得到一个看似正常但完全错误的像素坐标，因此必须在除法前拦截
    constexpr double kMinDepth = 1e-9;
    if (Pc.z <= kMinDepth)
    {
        result.valid  = false;
        result.reason = "point is not in front of the camera (Zc <= 0)";
        return result;
    }

    // 透视除法 + 内参映射
    const double xNormalized = Pc.x / Pc.z;
    const double yNormalized = Pc.y / Pc.z;

    result.pixel.u = K.fx * xNormalized + K.cx;
    result.pixel.v = K.fy * yNormalized + K.cy;
    result.valid   = true;
    return result;
}

ProjectionResult project(const Intrinsics& K, const Extrinsics& ext, const Point3& Pw)
{
    return projectCameraPoint(K, worldToCamera(ext, Pw));
}

double pixelDistance(const Point2& a, const Point2& b)
{
    const double du = a.u - b.u;
    const double dv = a.v - b.v;
    return std::sqrt(du * du + dv * dv);
}

Evaluation evaluate(const Intrinsics& K, const Extrinsics& ext, const Correspondence& corr)
{
    Evaluation eval;
    eval.observed = corr.observed;

    const ProjectionResult proj = project(K, ext, corr.world);
    eval.depth = proj.depth;

    if (!proj.valid)
    {
        eval.valid  = false;
        eval.reason = proj.reason;
        return eval;
    }

    eval.valid       = true;
    eval.reprojected = proj.pixel;
    eval.error       = pixelDistance(proj.pixel, corr.observed);
    return eval;
}

bool insideImage(const Intrinsics& K, const Point2& p)
{
    if (K.width <= 0 || K.height <= 0)
    {
        return true;
    }
    return p.u >= 0.0 && p.u < static_cast<double>(K.width)
        && p.v >= 0.0 && p.v < static_cast<double>(K.height);
}
