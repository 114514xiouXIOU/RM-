// main.cpp -- 读取数据、逐点计算并输出重投影结果
//
// 输入文件格式：
//   num_points
//   fx fy cx cy width height
//   r00 r01 r02
//   r10 r11 r12
//   r20 r21 r22
//   tx ty tz
//   Xw Yw Zw u_obs v_obs      (重复 num_points 行)
//
// 整行以 # 开头视为注释；width/height 填 0 表示不检查是否出画。
//
// 输出：每点的相机坐标、重投影像素、观测像素、深度、像素误差与是否出画，末行为统计。

#include "camera.h"
#include "reprojection.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// 过滤掉整行注释与空行，使后续的 >> 能直接解析数字
class CommentSkippingStream : public std::istringstream
{
public:
    explicit CommentSkippingStream(std::istream& in)
        : std::istringstream(buildFilteredText(in))
    {
    }

private:
    static std::string buildFilteredText(std::istream& in)
    {
        std::string filtered;
        std::string line;
        while (std::getline(in, line))
        {
            const std::size_t firstNonSpace = line.find_first_not_of(" \t\r\n");
            if (firstNonSpace == std::string::npos)
            {
                continue;
            }
            if (line[firstNonSpace] == '#')
            {
                continue;
            }
            filtered += line;
            filtered += '\n';
        }
        return filtered;
    }
};

bool readIntrinsicsAndExtrinsics(std::istream& in, Intrinsics& K, Extrinsics& ext)
{
    if (!(in >> K.fx >> K.fy >> K.cx >> K.cy >> K.width >> K.height))
    {
        std::cerr << "Error: failed to read intrinsics "
                     "(expected: fx fy cx cy width height)\n";
        return false;
    }
    if (K.fx <= 0.0 || K.fy <= 0.0)
    {
        std::cerr << "Error: fx / fy must be positive (unit: pixels)\n";
        return false;
    }

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            if (!(in >> ext.R.m[row][col]))
            {
                std::cerr << "Error: failed to read rotation matrix "
                             "(expected: 9 numbers, row-major)\n";
                return false;
            }
        }
    }

    if (!(in >> ext.t.x >> ext.t.y >> ext.t.z))
    {
        std::cerr << "Error: failed to read translation vector (expected: tx ty tz)\n";
        return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    std::ifstream file;
    if (argc > 1)
    {
        file.open(argv[1]);
        if (!file)
        {
            std::cerr << "Error: cannot open file " << argv[1] << "\n";
            return EXIT_FAILURE;
        }
    }
    std::istream& rawIn = (argc > 1) ? static_cast<std::istream&>(file) : std::cin;
    CommentSkippingStream input(rawIn);

    int numPoints = 0;
    if (!(input >> numPoints) || numPoints <= 0)
    {
        std::cerr << "Error: the first number must be a positive integer "
                     "(num_points)\n";
        return EXIT_FAILURE;
    }

    Intrinsics K;
    Extrinsics ext;
    if (!readIntrinsicsAndExtrinsics(input, K, ext))
    {
        return EXIT_FAILURE;
    }

    std::vector<Correspondence> data;
    data.reserve(static_cast<std::size_t>(numPoints));
    for (int i = 0; i < numPoints; ++i)
    {
        Correspondence corr;
        if (!(input >> corr.world.x >> corr.world.y >> corr.world.z
                     >> corr.observed.u >> corr.observed.v))
        {
            std::cerr << "Error: failed to read point #" << (i + 1)
                      << " (5 numbers required: Xw Yw Zw u_obs v_obs)\n";
            return EXIT_FAILURE;
        }
        data.push_back(corr);
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "================ Camera configuration ================\n";
    std::cout << "Intrinsics K = [ fx=" << K.fx << ", fy=" << K.fy
              << ", cx=" << K.cx << ", cy=" << K.cy << " ]\n";
    if (K.width > 0 && K.height > 0)
    {
        std::cout << "Image size   = " << K.width << " x " << K.height << "\n";
    }
    std::cout << "Extrinsics R (row-major) =\n";
    for (int row = 0; row < 3; ++row)
    {
        std::cout << "  [ " << ext.R.m[row][0] << "  " << ext.R.m[row][1]
                  << "  " << ext.R.m[row][2] << " ]\n";
    }
    std::cout << "Extrinsics t = [ " << ext.t.x << ", " << ext.t.y << ", "
              << ext.t.z << " ]\n";
    std::cout << "Convention   : Pc = R * Pw + t   (world -> camera)\n\n";

    std::cout << "================ Reprojection result ================\n";
    std::cout << "  # |       World point Pw        |       Camera point Pc       |"
                 "     Reprojected      |      Observed       |  Depth   |  Error   | In\n";
    std::cout << "    |    Xw        Yw        Zw   |    Xc        Yc        Zc   |"
                 "      u          v     |     u         v     |    Zc    |  (px)    | frame\n";
    std::cout << "----+-----------------------------+-----------------------------+"
                 "----------------------+---------------------+----------+----------+-------\n";

    double sumError        = 0.0;
    double sumSquaredError = 0.0;
    double maxError        = 0.0;
    int    validCount      = 0;
    int    insideCount     = 0;

    for (std::size_t i = 0; i < data.size(); ++i)
    {
        const Correspondence& corr = data[i];
        const Point3     Pc = worldToCamera(ext, corr.world);
        const Evaluation e  = evaluate(K, ext, corr);

        std::cout << std::setw(3) << (i + 1) << " | "
                  << std::setw(8) << corr.world.x << " "
                  << std::setw(8) << corr.world.y << " "
                  << std::setw(8) << corr.world.z << " | "
                  << std::setw(8) << Pc.x << " "
                  << std::setw(8) << Pc.y << " "
                  << std::setw(8) << Pc.z << " | ";

        if (!e.valid)
        {
            std::cout << std::setw(20) << "INVALID" << " | "
                      << std::setw(10) << "-" << " " << std::setw(8) << "-" << " | "
                      << std::setw(8) << "-" << " | "
                      << std::setw(8) << "-" << " | "
                      << "  -   " << "  <- " << e.reason << "\n";
            continue;
        }

        const bool inside = insideImage(K, e.reprojected);
        ++validCount;
        if (inside)
        {
            ++insideCount;
        }

        sumError        += e.error;
        sumSquaredError += e.error * e.error;
        maxError = std::max(maxError, e.error);

        std::cout << std::setw(9) << e.reprojected.u << " "
                  << std::setw(9) << e.reprojected.v << "  | "
                  << std::setw(9) << e.observed.u << " "
                  << std::setw(9) << e.observed.v << "  | "
                  << std::setw(8) << e.depth << " | "
                  << std::setw(8) << e.error << " | "
                  << (inside ? " yes " : " no  ") << "\n";
    }

    std::cout << "\n================ Summary ================\n";
    std::cout << "Total points      : " << data.size() << "\n";
    std::cout << "Valid projections : " << validCount << "\n";
    std::cout << "Inside the frame  : " << insideCount << "\n";
    if (validCount > 0)
    {
        const double meanError = sumError / static_cast<double>(validCount);
        const double rmse      = std::sqrt(sumSquaredError / static_cast<double>(validCount));
        std::cout << "Mean pixel error  : " << meanError << " px\n";
        std::cout << "RMSE              : " << rmse << " px\n";
        std::cout << "Max pixel error   : " << maxError << " px\n";
    }
    else
    {
        std::cout << "No valid projection at all. Check whether the extrinsic\n"
                     "convention is reversed (world -> camera vs camera -> world).\n";
    }

    return EXIT_SUCCESS;
}
