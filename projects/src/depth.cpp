#include <iostream>
#include <memory>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "libobsensor/ObSensor.hpp"

namespace fs = std::filesystem;

int main() {
    
    std::string filePath = "/home/michall/AAAProjects/RGBD/projects/data/1280_800_30fps.bag";
    // Camera parameters (from user: Orbbec Gemini 336L)
    const double BASELINE_MM = 95.0;
    const double FX_PX       = 620.0;

    // std::string filePath = "/home/michall/AAAProjects/RGBD/projects/testproject/data/IR02.bag";
    // // Camera parameters (from user: Orbbec Gemini 336L)
    // const double BASELINE_MM = 95.0;
    // const double FX_PX       = 410.7;

    try {
        auto playback = std::make_shared<ob::PlaybackDevice>(filePath);
        auto pipe     = std::make_shared<ob::Pipeline>(playback);
        auto config   = std::make_shared<ob::Config>();

        bool playbackStop = false;
        playback->setPlaybackStatusChangeCallback([&](OBPlaybackStatus status) {
            if (status == OB_PLAYBACK_STOPPED) playbackStop = true;
        });

        auto sensorList = playback->getSensorList();
        for (uint32_t i = 0; i < sensorList->getCount(); i++) {
            auto sensorType = sensorList->getSensorType(i);
            config->enableStream(sensorType);
            std::cout << "Sensor type: " << sensorType << std::endl;
        }

        pipe->start(config);

        fs::create_directories("./output/depth");

        int frameIndex  = 0;
        int saveIndex   = 0;

        // Stereo SGBM parameters
        int numDisparities = 64;
        int blockSize      = 7;
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0, numDisparities, blockSize,
            8 * 1 * blockSize * blockSize,   // P1
            32 * 1 * blockSize * blockSize,  // P2
            1,    // disp12MaxDiff
            63,   // preFilterCap
            10,   // uniquenessRatio
            200,  // speckleWindowSize
            32,   // speckleRange
            cv::StereoSGBM::MODE_SGBM
        );

        while (!playbackStop) {
            auto frameSet = pipe->waitForFrames(1000);
            if (frameSet == nullptr) continue;

            frameIndex++;
            if (frameIndex % 8 != 0) continue;

            auto leftRaw  = frameSet->getFrame(OB_FRAME_IR_LEFT);
            auto rightRaw = frameSet->getFrame(OB_FRAME_IR_RIGHT);

            if (leftRaw && rightRaw) {
                auto left  = leftRaw->as<ob::IRFrame>();
                auto right = rightRaw->as<ob::IRFrame>();

                int width  = left->getWidth();
                int height = left->getHeight();

                cv::Mat leftImg(height, width, CV_8UC1, (void*)left->getData());
                cv::Mat rightImg(height, width, CV_8UC1, (void*)right->getData());

                cv::Mat disp, dispF32, depth;

                sgbm->compute(leftImg, rightImg, disp);

                disp.convertTo(dispF32, CV_32F);
                dispF32 /= 16.0f;

                // depth = (fx * baseline) / disparity
                // mask out invalid (<=0) disparities

                // dispF32 已经除以 16，无效值为 -1 或 <=0
                cv::Mat mask = (dispF32 > 0);   // 有效视差必须大于 0
                // 找到所有非零像素点的包围矩形
                cv::Rect roi = cv::boundingRect(mask);
                // roi.x, roi.y 是矩形左上角，roi.width, roi.height 是宽高

                cv::Mat depth32F(height, width, CV_32F);
                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        float d = dispF32.at<float>(y, x);
                        if (d > 0.0f) {
                            depth32F.at<float>(y, x) = static_cast<float>(FX_PX * BASELINE_MM / d);
                            if (depth32F.at<float>(y, x) > 20000.0f) {  // Cap depth at 20 meters for visualization
                                depth32F.at<float>(y, x) = 20000.0f;
                            }
                        } else {
                            depth32F.at<float>(y, x) = 0.0f;
                        }
                    }
                }
                // 裁剪深度图 (假设已经算出 depth32F)
                cv::Mat depth32F_cropped = depth32F(roi);

                // // Save raw 16-bit depth (mm, rounded)
                // cv::Mat depth16U;
                // depth32F.convertTo(depth16U, CV_16UC1);
                // std::string rawPath = "./depth_output/depth_" + std::to_string(saveIndex) + ".png";
                // cv::imwrite(rawPath, depth16U);

                // Save color visualization
                double minVal, maxVal;
                cv::minMaxLoc(depth32F_cropped, &minVal, &maxVal, nullptr, nullptr);
                cv::Mat depthNorm;
                if (maxVal > minVal) {
                    depth32F_cropped.convertTo(depthNorm, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
                } else {
                    depthNorm = cv::Mat::zeros(height, width, CV_8UC1);
                }
                cv::bitwise_not(depthNorm, depthNorm);  // Invert for better visualization (closer = brighter)
                cv::Mat depthColor;
                cv::applyColorMap(depthNorm, depthColor, cv::COLORMAP_JET);
                std::string vizPath = "./output/depth/" + std::to_string(saveIndex) + "_depth_viz.png";
                cv::imwrite(vizPath, depthColor);

                // origin image
                std::string leftPath  = "./output/depth/" + std::to_string(saveIndex) + "_origin_left.png";
                std::string rightPath = "./output/depth/" + std::to_string(saveIndex) + "_origin_right.png";
                cv::imwrite(leftPath, leftImg);
                cv::imwrite(rightPath, rightImg);

                std::cout << "Saved depth frame " << saveIndex << " (min=" << minVal << "mm, max=" << maxVal << "mm)" << std::endl;

                saveIndex++;
            }
        }

        pipe->stop();
    }
    catch (ob::Error &e) {
        std::cerr << "Orbbec error: " << e.getMessage() << std::endl;
        return 1;
    }

    return 0;
}
