//
// Created by Andrey Pahomov on 09.02.21.
//

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Eye.h"

using namespace libcamera;

Eye eye;
std::vector<const Stream *> streams;
std::map<libcamera::FrameBuffer *, MappedBuffer> mappedBuffers;
Rectangle sensor_area;

cv::Mat dst, dst_bayer, dst_rgb;

double zoom = 1;

#define CHECK(ret) assert((ret) == 0)

std::map<int, uint64_t> lastBufferTime;

void processInfo(int idStream, FrameBuffer *buffer)
{
    try {

        if(!lastBufferTime.count(idStream))
            lastBufferTime.insert({idStream, 0});

        const FrameMetadata &metadata = buffer->metadata();

        double fps = metadata.timestamp - lastBufferTime[idStream];
        fps = lastBufferTime[idStream] && fps ? 1000000000.0 / fps : 0.0;
        lastBufferTime[idStream] = metadata.timestamp;

        std::cout
                << "seq: " << metadata.sequence
                << "\tbytesused:" << metadata.planes[0].bytesused
                << "\ttimestamp:" << metadata.timestamp
                << "\tfps:" << fps << "\n";

    } catch (...) {
        std::cerr << "processInfo error\n";
    }

}

static void processRequest(Request *request) {
    if (request->status() == Request::RequestCancelled)
        return;

    Rectangle crop;

    const Request::BufferMap &buffers = request->buffers();



    /* Process buffers. */
    if (request->buffers().count(streams.at(0))) {

        FrameBuffer *buffer = request->buffers().at(streams.at(0));
        std::memcpy(dst.data, (uchar *) mappedBuffers[buffer].memory, mappedBuffers[buffer].size);
        cv::imshow("test", dst);

        int x0 = sensor_area.center().x;
        int y0 = sensor_area.center().y;

        int w = 2 * x0 / zoom;
        int h = 2 * y0 / zoom;
        int x = x0 - w / 2;
        int y = y0 - h / 2;

        crop.x = x;
        crop.y = y;
        crop.width = w;
        crop.height = h;

        double real_zoom = sqrt((sensor_area.width * sensor_area.height) / (w * h));

        zoom += 0.1;
        if (zoom > 20) zoom = 1.0;

        processInfo(0, buffer);
    }


    if (streams.size() > 1)
        if (request->buffers().count(streams.at(1))) {
            FrameBuffer *buffer = request->buffers().at(streams.at(1));

//            dst_bayer.data = (uchar *) mappedBuffers[buffer].memory;
            std::memcpy(dst_bayer.data, (uchar *) mappedBuffers[buffer].memory, mappedBuffers[buffer].size);
            cv::cvtColor(dst_bayer, dst_rgb, cv::COLOR_BayerBG2BGR);
            cv::imshow("test1", dst_rgb);

            crop.x = 0;
            crop.y = 0;
            crop.width = sensor_area.width;
            crop.height = sensor_area.height;

            processInfo(1, buffer);
        }

    request->reuse(Request::ReuseBuffers);

    crop.translateBy(sensor_area.topLeft());
    request->controls().set(libcamera::controls::ScalerCrop, crop);
    request->controls().set(libcamera::controls::ExposureTime, 50000);
    request->controls().set(libcamera::controls::AnalogueGain, 10.0);
    request->controls().set(libcamera::controls::Sharpness, 0);

    eye.AddRequest(request);

    cv::waitKey(1);
}

int main() {
   std::string idSensor;

    CHECK(eye.Init());
    eye.ShowCamers();
    CHECK(eye.SetCameraId(0, idSensor));
    CHECK(eye.CameraConfiguration({ StreamRole::Raw, StreamRole::Viewfinder }));
//    CHECK(eye.CameraConfiguration({ StreamRole::Viewfinder}));

    CHECK(eye.SetConfiguration(0, {800, 600}, PixelFormat::fromString("RGB888")));
    CHECK(eye.SetConfiguration(1, {800, 600}));
    CHECK(eye.StreamConfiguration());
    CHECK(eye.BufferAlloc());
    CHECK(eye.CreateRequests());
    eye.SetCallBack(processRequest);

    sensor_area = eye.GetSensorArea();
    streams = eye.GetStreams();
    mappedBuffers = eye.GetMappedBuffers();
    libcamera::StreamConfiguration &streamConfig = eye.GetStreamConfiguration(0);

    dst = cv::Mat(
            eye.GetStreamConfiguration(0).size.height,
            eye.GetStreamConfiguration(0).size.width, CV_8UC3
            );
    if (streams.size() > 1)
        dst_bayer = cv::Mat(
                eye.GetStreamConfiguration(1).size.height,
                eye.GetStreamConfiguration(1).size.width, CV_8UC1
                );

    ControlList controlList;
    controlList.set(controls::ScalerCrop, Rectangle(0, 0, sensor_area.width, sensor_area.height));

    CHECK(eye.Start(&controlList));
    CHECK(eye.Join());

    return 0;
}
