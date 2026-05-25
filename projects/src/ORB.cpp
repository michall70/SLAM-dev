#include <iostream>
#include <memory>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "libobsensor/ObSensor.hpp"

namespace fs = std::filesystem;

int main() {
    std::string filePath = "/home/michall/AAAProjects/RGBD/projects/data/IR02.bag";

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

    fs::create_directories("./output/orb");

    // ─── ORB detector ──────────────────────────────────────────────────────
    // cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold,
    //                  firstLevel, WTA_K, scoreType, patchSize, fastThreshold)
    //
    // nfeatures       = 500  → max number of keypoints to keep (strongest ones survive)
    // scaleFactor     = 1.2  → image pyramid scale ratio (1.2 = each level is 1/1.2 smaller)
    // nlevels         = 8    → how many pyramid levels (more = finds features at more sizes)
    // edgeThreshold   = 31   → ignore pixels this close to the border (default 31)
    // firstLevel      = 0    → starting pyramid level (0 = original full-size image)
    // WTA_K           = 2    → # of pixels used per descriptor bit (2=ORIG, 3=PROVIDED)
    // scoreType       = cv::ORB::HARRIS_SCORE  → or FAST_SCORE (faster but less stable)
    // patchSize       = 31   → size of the patch used for descriptor extraction
    // fastThreshold   = 20   → higher = fewer corners, faster; lower = more corners, slower
    //
    auto orb = cv::ORB::create(
        500,       // nfeatures:           keep top 500 keypoints
        1.2f,      // scaleFactor:         pyramid shrink by 1.2× each level
        8,         // nlevels:             8 pyramid levels
        31,        // edgeThreshold:       skip 31px border
        0,         // firstLevel:          start at original size
        2,         // WTA_K:               2 pixels per descriptor bit
        cv::ORB::HARRIS_SCORE,  // scoreType:  Harris corner quality (more stable)
        31,        // patchSize:           31×31 pixel patch for descriptor
        20         // fastThreshold:       FAST corner threshold (lower=more corners)
    );

    // Warm-up: ORB lazily allocates internal buffers on the first call (~2.7s).
    // Use random noise so FAST finds keypoints and triggers full allocation.
    {
        cv::Mat dummy(cv::Size(1280, 800), CV_8UC1);
        cv::randu(dummy, 0, 256);
        std::vector<cv::KeyPoint> wk;
        cv::Mat                   wd;
        orb->detectAndCompute(dummy, cv::noArray(), wk, wd);
    }

    pipe->start(config);

    int frameIndex = 0;
    int saveIndex  = 0;

    while (!playbackStop) {
        auto frameSet = pipe->waitForFrames(1000);
        if (frameSet == nullptr) continue;

        frameIndex++;
        if (frameIndex % 8 != 0) continue;   // process every 8th frame

        auto leftRaw = frameSet->getFrame(OB_FRAME_IR_LEFT);
        if (!leftRaw) continue;

        auto left   = leftRaw->as<ob::IRFrame>();
        int  width  = left->getWidth();
        int  height = left->getHeight();

        // Wrap the SDK's raw buffer into an OpenCV Mat (no copy!)
        cv::Mat leftImg(height, width, CV_8UC1, (void *)left->getData());

        // ─── Detect ORB keypoints and compute descriptors ──────────────────
        // cv::noArray()  = no mask (use whole image)
        // keypoints      ← output: list of (x, y, size, angle, response, octave)
        // descriptors    ← output: CV_8U matrix, rows=keypoints, cols=32 bytes
        //
        auto t0 = std::chrono::steady_clock::now();

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat                   descriptors;
        orb->detectAndCompute(leftImg, cv::noArray(), keypoints, descriptors);

        auto t1 = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

        // ─── Draw keypoints on the image ────────────────────────────────────
        // cv::Scalar::all(-1)  → use random color per keypoint
        // DRAW_RICH_KEYPOINTS  → draw circle + orientation + size
        //
        cv::Mat orbImg;
        cv::drawKeypoints(leftImg, keypoints, orbImg, cv::Scalar::all(-1),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        char buf[32];
        snprintf(buf, sizeof(buf), "%04d", saveIndex);

        std::string rawPath = "./output/orb/" + std::string(buf) + ".png";
        cv::imwrite(rawPath, leftImg);

        std::string orbPath = "./output/orb/" + std::string(buf) + "_orb.png";
        cv::imwrite(orbPath, orbImg);

        std::cout << buf << ": " << keypoints.size()
                  << " keypoints (" << elapsed << "ms)" << std::endl;

        saveIndex++;
    }

    pipe->stop();
    return 0;
}
