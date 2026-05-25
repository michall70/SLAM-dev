#include <iostream>
#include <memory>
#include <chrono>
#include <vector>
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "libobsensor/ObSensor.hpp"

namespace fs = std::filesystem;

static void drawCameraArrow(cv::Mat &img, int x, int y, float angleDeg, int len) {
    int ex = static_cast<int>(x + len * cos(angleDeg * CV_PI / 180.0));
    int ey = static_cast<int>(y + len * sin(angleDeg * CV_PI / 180.0));
    cv::arrowedLine(img, cv::Point(x, y), cv::Point(ex, ey), cv::Scalar(0, 0, 200), 2, cv::LINE_AA, 0, 0.3);
}

static void saveMatchImage(const cv::Mat &img1, const cv::Mat &img2,
                           const std::vector<cv::KeyPoint> &kp1,
                           const std::vector<cv::KeyPoint> &kp2,
                           const std::vector<cv::DMatch> &matches,
                           const std::string &path) {
    // Only draw a subset of matches to avoid clutter
    int maxDraw = 200;
    std::vector<cv::DMatch> subset;
    for (size_t i = 0; i < matches.size() && i < maxDraw; i++) {
        subset.push_back(matches[i]);
    }
    cv::Mat out;
    cv::drawMatches(img1, kp1, img2, kp2, subset, out,
                    cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imwrite(path, out);
}

int main() {
    std::string filePath = "/home/michall/AAAProjects/RGBD/projects/data/parallel/03z_p.bag";

    auto playback = std::make_shared<ob::PlaybackDevice>(filePath);
    auto pipe     = std::make_shared<ob::Pipeline>(playback);
    auto config   = std::make_shared<ob::Config>();

    bool playbackStop = false;
    playback->setPlaybackStatusChangeCallback([&](OBPlaybackStatus status) {
        if (status == OB_PLAYBACK_STOPPED) playbackStop = true;
    });

    auto sensorList = playback->getSensorList();
    for (uint32_t i = 0; i < sensorList->getCount(); i++) {
        config->enableStream(sensorList->getSensorType(i));
    }

    // Camera intrinsics
    auto profileList   = pipe->getStreamProfileList(OB_SENSOR_IR_LEFT);
    auto streamProfile = profileList->getVideoStreamProfile(
        OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, OB_FPS_ANY);
    auto intrinsic = streamProfile->getIntrinsic();

    std::cout << "IR Left intrinsics: fx=" << intrinsic.fx
              << " fy=" << intrinsic.fy
              << " cx=" << intrinsic.cx
              << " cy=" << intrinsic.cy << std::endl;

    cv::Matx33d K(intrinsic.fx, 0, intrinsic.cx,
                  0, intrinsic.fy, intrinsic.cy,
                  0, 0, 1);

    // ORB
    auto orb = cv::ORB::create(2000, 1.5f, 2, 31, 0, 2,
                               cv::ORB::HARRIS_SCORE, 31, 20);

    // Warm-up
    {
        cv::Mat dummy(cv::Size(1280, 800), CV_8UC1);
        cv::randu(dummy, 0, 256);
        std::vector<cv::KeyPoint> wk;
        cv::Mat                   wd;
        orb->detectAndCompute(dummy, cv::noArray(), wk, wd);
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);

    pipe->start(config);

    // ─── VO state ───────────────────────────────────────────────────────────
    std::vector<cv::KeyPoint> prevKp;
    cv::Mat                   prevDesc;
    cv::Mat                   prevImg;   // for match visualization

    cv::Mat R_accum = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_accum = cv::Mat::zeros(3, 1, CV_64F);

    // For final plot: positions + rotation matrices
    std::vector<cv::Point3f> traj;
    std::vector<cv::Matx33d> trajR;     // camera orientation (world→cam)
    traj.push_back(cv::Point3f(0, 0, 0));
    trajR.push_back(cv::Matx33d::eye());

    int frameIndex  = 0;
    int validMotion = 0;

    fs::create_directories("./output/traj_output");

    // Main loop
    while (!playbackStop) {
        auto frameSet = pipe->waitForFrames(1000);
        if (frameSet == nullptr) continue;
        if (frameIndex % 2 != 0) {  // Process every 2nd frame to reduce load
            frameIndex++;
            continue;
        }

        auto leftRaw = frameSet->getFrame(OB_FRAME_IR_LEFT);
        if (!leftRaw) continue;

        auto left   = leftRaw->as<ob::IRFrame>();
        int  width  = left->getWidth();
        int  height = left->getHeight();

        cv::Mat leftImg(height, width, CV_8UC1, (void *)left->getData());

        std::vector<cv::KeyPoint> kp;
        cv::Mat                   desc;
        orb->detectAndCompute(leftImg, cv::noArray(), kp, desc);

        if (desc.empty()) {
            frameIndex++;
            continue;
        }

        if (prevDesc.empty()) {
            prevKp   = kp;
            prevDesc = desc.clone();
            leftImg.copyTo(prevImg);
            frameIndex++;
            continue;
        }

        // ─── Match ──────────────────────────────────────────────────────────
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(desc, prevDesc, knn, 2);

        std::vector<cv::Point2f> ptsCur, ptsPrev;
        std::vector<cv::DMatch>  goodMatches;
        for (auto &m : knn) {
            if (m.size() >= 2 
                && m[0].distance < 0.75f * m[1].distance
                && m[0].distance < 50) {
                ptsCur.push_back(kp[m[0].queryIdx].pt);
                ptsPrev.push_back(prevKp[m[0].trainIdx].pt);
                goodMatches.push_back(m[0]);
            }
        }

        if (ptsCur.size() < 8) {
            prevKp   = kp;
            prevDesc = desc.clone();
            leftImg.copyTo(prevImg);
            frameIndex++;
            continue;
        }

        // ─── Motion estimation ──────────────────────────────────────────────
        cv::Mat inlierMask;
        cv::Mat E = cv::findEssentialMat(ptsCur, ptsPrev, K, cv::RANSAC, 0.999, 1.0, inlierMask);

        if (E.empty()) {
            prevKp   = kp;
            prevDesc = desc.clone();
            leftImg.copyTo(prevImg);
            frameIndex++;
            continue;
        }

        cv::Mat R_rel, t_rel;
        int inlierCount = cv::recoverPose(E, ptsCur, ptsPrev, K, R_rel, t_rel, inlierMask);

        if (inlierCount < 6) {
            prevKp   = kp;
            prevDesc = desc.clone();
            leftImg.copyTo(prevImg);
            frameIndex++;
            continue;
        }

        // ─── Accumulate ─────────────────────────────────────────────────────
        // recoverPose returns R_rel,t_rel such that X_prev = R_rel * X_cur + t_rel.
        // Solving for current pose: R_cur = R_rel^T * R_prev, t_cur = R_rel^T * t_prev - R_rel^T * t_rel
        cv::Mat R_rel_t = R_rel.t();
        R_accum         = R_rel_t * R_accum;
        t_accum         = R_rel_t * t_accum - R_rel_t * t_rel;

        cv::Mat C = -R_accum.t() * t_accum;
        traj.push_back(cv::Point3f(C.at<double>(0), C.at<double>(1), C.at<double>(2)));
        trajR.push_back(cv::Matx33d(R_accum.at<double>(0, 0), R_accum.at<double>(0, 1), R_accum.at<double>(0, 2),
                                    R_accum.at<double>(1, 0), R_accum.at<double>(1, 1), R_accum.at<double>(1, 2),
                                    R_accum.at<double>(2, 0), R_accum.at<double>(2, 1), R_accum.at<double>(2, 2)));

        validMotion++;

        if (frameIndex % 20 == 0) {
            std::cout << "Frame " << frameIndex << ": " << inlierCount << "/" << ptsCur.size()
                      << " inliers, pos=(" << std::fixed << std::setprecision(2)
                      << C.at<double>(0) << ", " << C.at<double>(1) << ", "
                      << C.at<double>(2) << ")" << std::endl;
        }

        // ─── Save match visualization every 40 frames ───────────────────────
        if (frameIndex % 1 == 0 && !goodMatches.empty()) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%04d", frameIndex);
            std::string mPath = "./output/traj_output/match_" + std::string(buf) + ".png";
            saveMatchImage(leftImg, prevImg, kp, prevKp, goodMatches, mPath);
        }

        prevKp   = kp;
        prevDesc = desc.clone();
        leftImg.copyTo(prevImg);
        frameIndex++;
    }

    pipe->stop();

    std::cout << "\nProcessed " << frameIndex << " frames, "
              << validMotion << " motion estimates." << std::endl;

    // ─── Save trajectory data for 3D viewer ─────────────────────────────────
    std::ofstream dataFile("./output/traj_output/trajectory.txt");
    dataFile << "# frame tx ty tz r00 r01 r02 r10 r11 r12 r20 r21 r22\n";
    for (size_t i = 0; i < traj.size(); i++) {
        auto &R = trajR[i];
        dataFile << i << " "
                 << traj[i].x << " " << traj[i].y << " " << traj[i].z << " "
                 << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " "
                 << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
                 << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n";
    }
    std::cout << "Saved trajectory data to ./output/traj_output/trajectory.txt" << std::endl;

    // ─── Draw 2D trajectory with orientation markers ────────────────────────
    if (traj.size() < 2) {
        std::cout << "Not enough trajectory points." << std::endl;
        return 0;
    }

    float minX = 1e9, maxX = -1e9, minZ = 1e9, maxZ = -1e9;
    for (auto &p : traj) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.z < minZ) minZ = p.z;
        if (p.z > maxZ) maxZ = p.z;
    }

    float rangeX = maxX - minX;
    float rangeZ = maxZ - minZ;
    if (rangeX < 1) rangeX = 1;
    if (rangeZ < 1) rangeZ = 1;

    int imgSize = 800;
    float scale  = (imgSize - 80) / std::max(rangeX, rangeZ);
    float cxPlot = imgSize / 2.0f - (minX + maxX) / 2.0f * scale;
    float cyPlot = imgSize / 2.0f - (minZ + maxZ) / 2.0f * scale;

    cv::Mat trajImg(imgSize, imgSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // Grid
    for (float g = -100; g <= 100; g += 1.0f) {
        int px = static_cast<int>(g * scale + cxPlot);
        if (px >= 0 && px < imgSize)
            cv::line(trajImg, cv::Point(px, 0), cv::Point(px, imgSize), cv::Scalar(230, 230, 230));
        int pz = static_cast<int>(g * scale + cyPlot);
        if (pz >= 0 && pz < imgSize)
            cv::line(trajImg, cv::Point(0, pz), cv::Point(imgSize, pz), cv::Scalar(230, 230, 230));
    }

    // Trajectory line
    for (size_t i = 1; i < traj.size(); i++) {
        int x1 = static_cast<int>(traj[i - 1].x * scale + cxPlot);
        int y1 = static_cast<int>(traj[i - 1].z * scale + cyPlot);
        int x2 = static_cast<int>(traj[i].x * scale + cxPlot);
        int y2 = static_cast<int>(traj[i].z * scale + cyPlot);

        float t    = static_cast<float>(i) / traj.size();
        cv::Scalar color(static_cast<int>(255 * t), 0, static_cast<int>(255 * (1 - t)));
        cv::line(trajImg, cv::Point(x1, y1), cv::Point(x2, y2), color, 2, cv::LINE_AA);
    }

    // Camera orientation markers every N points
    int markerStep = std::max(1, static_cast<int>(traj.size()) / 20);
    for (size_t i = 0; i < traj.size(); i += markerStep) {
        int px = static_cast<int>(traj[i].x * scale + cxPlot);
        int py = static_cast<int>(traj[i].z * scale + cyPlot);

        // Project camera Z-axis (viewing direction) from 3D to XZ plane
        cv::Matx33d R = trajR[i];
        // Camera looks along +Z in cam space; in world: R * (0,0,1)
        double dx = R(0, 0) * 0 + R(0, 1) * 0 + R(0, 2) * 1;
        double dy = R(2, 0) * 0 + R(2, 1) * 0 + R(2, 2) * 1;  // Z component in world XZ plane
        float angle = atan2(static_cast<float>(dy), static_cast<float>(dx)) * 180.0f / CV_PI;
        drawCameraArrow(trajImg, px, py, angle, 30);
    }

    // Start / end
    int sx = static_cast<int>(traj.front().x * scale + cxPlot);
    int sy = static_cast<int>(traj.front().z * scale + cyPlot);
    cv::circle(trajImg, cv::Point(sx, sy), 6, cv::Scalar(0, 200, 0), -1);
    int ex = static_cast<int>(traj.back().x * scale + cxPlot);
    int ey = static_cast<int>(traj.back().z * scale + cyPlot);
    cv::circle(trajImg, cv::Point(ex, ey), 6, cv::Scalar(0, 0, 200), -1);
    cv::putText(trajImg, "start", cv::Point(sx + 8, sy + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 150, 0), 1);
    cv::putText(trajImg, "end", cv::Point(ex + 8, ey + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 150), 1);
    cv::putText(trajImg, "Arrows = camera viewing direction",
                cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 50, 50), 1);

    cv::imwrite("./output/traj_output/trajectory.png", trajImg);
    std::cout << "Saved 2D trajectory to ./output/traj_output/trajectory.png" << std::endl;

    return 0;
}

