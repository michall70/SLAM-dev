#include <iostream>
#include "libobsensor/ObSensor.hpp"

int main() {
    try {
        // Get valid .bag
        std::string   filePath = "/home/michall/AAAProjects/RGBD/projects/testproject/data/IR01.bag";

        // 1.Create a playback device with a Rosbag file
        std::shared_ptr<ob::PlaybackDevice> playback = std::make_shared<ob::PlaybackDevice>(filePath);
        // 2.Create a pipeline with the playback device
        std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(playback);
        // 3.Enable all recording streams from the playback device
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

        // 4.Set playback status change callback, when the playback stops, start the pipeline again with the same config
        bool playbackStop = false;
        playback->setPlaybackStatusChangeCallback([&](OBPlaybackStatus status) {
            if(status == OB_PLAYBACK_STOPPED) {
                playbackStop = true;
            }
        });

        // 5.Get the list of playback sensors
        auto sensorList = playback->getSensorList();
        // 6.enable the data streams for playback
        for(uint32_t i = 0; i < sensorList->getCount(); i++) {
            auto sensorType = sensorList->getSensorType(i);
            config->enableStream(sensorType);
            std::cout << "Supported Sensor type: " << sensorType << std::endl;
        }

        // 7.Start the pipeline with the config
        pipe->start(config);

        while(!playbackStop) {
            auto frameSet = pipe->waitForFrames(1000);
            if(frameSet == nullptr) {
                continue;
            }
            // Get depth frame
            auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
        }

        // 9.Stop the pipeline.
        pipe->stop();
    }
    catch (ob::Error &e) {
        std::cerr << e.getMessage() << std::endl;
    }

    return 0;
}