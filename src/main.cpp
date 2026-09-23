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

// -----------------------------------------------------------------------------
// 交互模式：逐步提示用户输入
// -----------------------------------------------------------------------------

// 读取一个 double，失败时提示重试。返回 false 表示用户要求结束（输入 q 或到达 EOF）
bool readDoubleInput(const std::string& prompt, double& value)
{
    for (;;)
    {
        std::cout << prompt << std::flush;
        std::string token;
        if (!(std::cin >> token))
        {
            std::cout << "\n";      // EOF（例如按了 Ctrl+Z），换行后正常退出
            return false;
        }
        if (token == "q" || token == "Q")
        {
            std::cout << "\n";
            return false;
        }
        try
        {
            value = std::stod(token);   // 能解析成数字就接受
            return true;
        }
        catch (const std::exception&)
        {
            // 输入的不是数字（例如输入 abc 或 u=5），提示后 continue 重新问一次
            std::cout << "  Invalid number, please try again "
                         "(or type 'q' to quit).\n";
        }
    }
}

// 按提示逐项读取内参与外参
void readInteractiveInput(Intrinsics& K, Extrinsics& ext)
{
    std::cout << "=== Reprojection (interactive mode) ===\n"
              << "Enter the camera parameters step by step.\n"
              << "fx/fy/cx/cy are in pixels; t and world points share the same length unit.\n"
              << "Image width/height are optional; enter 0 to skip the in-frame check.\n\n";

    std::cout << "[Intrinsics]\n";
    readDoubleInput("  fx = ", K.fx);
    readDoubleInput("  fy = ", K.fy);
    readDoubleInput("  cx = ", K.cx);
    readDoubleInput("  cy = ", K.cy);

    double width = 0.0;
    double height = 0.0;
    readDoubleInput("  image width  (0 = do not check) = ", width);
    readDoubleInput("  image height (0 = do not check) = ", height);
    K.width  = static_cast<int>(width);
    K.height = static_cast<int>(height);

    std::cout << "\n[Extrinsics] rotation matrix R, row-major (9 numbers)\n";
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            const std::string name =
                "  r" + std::to_string(row) + std::to_string(col) + " = ";
            readDoubleInput(name, ext.R.m[row][col]);
        }
    }

    std::cout << "[Extrinsics] translation t (3 numbers)\n";
    readDoubleInput("  tx = ", ext.t.x);
    readDoubleInput("  ty = ", ext.t.y);
    readDoubleInput("  tz = ", ext.t.z);
    std::cout << "\n";
}

// 打印相机配置摘要，便于在开始计算前核对参数
void printCameraConfig(const Intrinsics& K, const Extrinsics& ext)
{
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
}

int main(int argc, char* argv[])
{
    Intrinsics K;
    Extrinsics ext;
    std::vector<Correspondence> data;   // 待计算的匹配对（文件模式与交互模式共用）

    if (argc > 1)
    {
        // ---------- 文件模式 ----------
        std::ifstream file(argv[1]);
        if (!file)
        {
            std::cerr << "Error: cannot open file " << argv[1] << "\n";
            return EXIT_FAILURE;
        }
        CommentSkippingStream input(file);

        int numPoints = 0;
        if (!(input >> numPoints) || numPoints <= 0)
        {
            std::cerr << "Error: the first number must be a positive integer "
                         "(num_points)\n";
            return EXIT_FAILURE;
        }

        if (!readIntrinsicsAndExtrinsics(input, K, ext))
        {
            return EXIT_FAILURE;
        }

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
    }
    else
    {
        // ---------- 交互模式 ----------
        readInteractiveInput(K, ext);
        printCameraConfig(K, ext);          // 先让用户核对一遍参数
        std::cout << "=== Enter points one by one; type 'q' at Xw to finish ===\n";
        for (;;)
        {
            Correspondence corr;
            std::cout << "\n--- Point " << (data.size() + 1) << " ---\n";

            double x = 0.0;
            if (!readDoubleInput("  Xw = ", x))
            {
                break;                          // 输入 q 或 EOF，结束录入
            }
            corr.world.x = x;
            readDoubleInput("  Yw = ", corr.world.y);
            readDoubleInput("  Zw = ", corr.world.z);
            readDoubleInput("  observed u = ", corr.observed.u);
            readDoubleInput("  observed v = ", corr.observed.v);

            data.push_back(corr);

            // 每输入一个点就立刻给出结果，不用等到最后
            const Point3     Pc = worldToCamera(ext, corr.world);
            const Evaluation e  = evaluate(K, ext, corr);

            std::cout << std::fixed << std::setprecision(4);
            std::cout << "  Pc        = (" << Pc.x << ", " << Pc.y << ", " << Pc.z << ")\n";
            std::cout << "  depth Zc  = " << Pc.z << "\n";
            if (!e.valid)
            {
                std::cout << "  result    = INVALID (" << e.reason << ")\n";
            }
            else
            {
                std::cout << "  projected = (" << e.reprojected.u << ", "
                          << e.reprojected.v << ")\n";
                std::cout << "  observed  = (" << e.observed.u << ", "
                          << e.observed.v << ")\n";
                std::cout << "  error     = " << e.error << " px\n";
            }
        }
    }

    if (data.empty())
    {
        std::cerr << "No point was given.\n";
        return EXIT_FAILURE;
    }

    // 文件模式在这里打印配置；
    // 交互模式已在录入完成后立即打印过（见上面 else 分支），这里不能重复打印
    if (argc > 1)
    {
        printCameraConfig(K, ext);
    }

    // 交互模式下每个点的结果已经在输入时逐点打印过，这里就不再重复打表格
    const bool printTable = (argc > 1);

    if (printTable)
    {
        std::cout << "================ Reprojection result ================\n";
        std::cout << "  # |       World point Pw        |       Camera point Pc       |"
                     "     Reprojected      |      Observed       |  Depth   |  Error   | In\n";
        std::cout << "    |    Xw        Yw        Zw   |    Xc        Yc        Zc   |"
                     "      u          v     |     u         v     |    Zc    |  (px)    | frame\n";
        std::cout << "----+-----------------------------+-----------------------------+"
                     "----------------------+---------------------+----------+----------+-------\n";
    }

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

        if (printTable)
        {
            std::cout << std::setw(3) << (i + 1) << " | "
                      << std::setw(8) << corr.world.x << " "
                      << std::setw(8) << corr.world.y << " "
                      << std::setw(8) << corr.world.z << " | "
                      << std::setw(8) << Pc.x << " "
                      << std::setw(8) << Pc.y << " "
                      << std::setw(8) << Pc.z << " | ";
        }

        if (!e.valid)
        {
            if (printTable)
            {
                std::cout << std::setw(20) << "INVALID" << " | "
                          << std::setw(10) << "-" << " " << std::setw(8) << "-" << " | "
                          << std::setw(8) << "-" << " | "
                          << std::setw(8) << "-" << " | "
                          << "  -   " << "  <- " << e.reason << "\n";
            }
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

        if (printTable)
        {
            std::cout << std::setw(9) << e.reprojected.u << " "
                      << std::setw(9) << e.reprojected.v << "  | "
                      << std::setw(9) << e.observed.u << " "
                      << std::setw(9) << e.observed.v << "  | "
                      << std::setw(8) << e.depth << " | "
                      << std::setw(8) << e.error << " | "
                      << (inside ? " yes " : " no  ") << "\n";
        }
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
