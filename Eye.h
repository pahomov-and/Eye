//
// Created by Andrey Pahomov on 09.02.21.
//

#ifndef EYE_EYE_H
#define EYE_EYE_H

#include <map>
#include <vector>
#include <queue>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/ipa/ipa_controls.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/camera.h>
#include <libcamera/buffer.h>
#include <libcamera/controls.h>
#include <libcamera/property_ids.h>
#include <libcamera/control_ids.h>
#include <libcamera/object.h>
#include <libcamera/bound_method.h>

#include "EventLoop.h"

struct MappedBuffer {
    void *memory;
    size_t size;
};



/**
 * Stream
 *
 * Each Camera supports a variable number of Stream. A Stream is
 * produced by processing data produced by an image source, usually
 * by an ISP.
 *
 *   +-------------------------------------------------------+
 *   | Camera                                                |
 *   |                +-----------+                          |
 *   | +--------+     |           |------> [  Main output  ] |
 *   | | Image  |     |           |                          |
 *   | |        |---->|    ISP    |------> [   Viewfinder  ] |
 *   | | Source |     |           |                          |
 *   | +--------+     |           |------> [ Still Capture ] |
 *   |                +-----------+                          |
 *   +-------------------------------------------------------+
 *
 * The number and capabilities of the Stream in a Camera are
 * a platform dependent property, and it's the pipeline handler
 * implementation that has the responsibility of correctly
 * report them.
 */

class Eye {
public:
    Eye();
    ~Eye();

    int Init();
    int SetCameraId(int id, std::string &idSensor);

    /**
     * --------------------------------------------------------------------
     * Camera Configuration.
     *
     * Camera configuration is tricky! It boils down to assign resources
     * of the system (such as DMA engines, scalers, format converters) to
     * the different image streams an application has requested.
     *
     * Depending on the system characteristics, some combinations of
     * sizes, formats and stream usages might or might not be possible.
     *
     * A Camera produces a CameraConfigration based on a set of intended
     * roles for each Stream the application requires.
     */
    int CameraConfiguration(const libcamera::StreamRoles &roles = {});
    int SetConfiguration(int idStream, libcamera::Size size, libcamera::PixelFormat pixelFormat);
    int SetConfiguration(int idStream, libcamera::Size size);
    libcamera::StreamConfiguration &GetStreamConfiguration(int idStream);
    int StreamConfiguration();

    int BufferAlloc();


    /**
     *  --------------------------------------------------------------------
     *  Frame Capture
     *  Libcamera frames capture model is based on the 'Request' concept.
     *  For each frame a Request has to be queued to the Camera.
     *
     *  A Request refers to (at least one) Stream for which a Buffer that
     *  will be filled with image data shall be added to the Request.
     *
     *  A Request is associated with a list of Controls, which are tunable
     *  parameters (similar to v4l2_controls) that have to be applied to
     *  the image.
     *
     *  Once a request completes, all its buffers will contain image data
     *  that applications can access and for each of them a list of metadata
     *  properties that reports the capture parameters applied to the image.
     */
    int CreateRequests();

    void SetCallBack(void (*call)(libcamera::Request *));
    void AddRequest(libcamera::Request *request);
    libcamera::Rectangle GetSensorArea();
    std::map<libcamera::FrameBuffer *, MappedBuffer> GetMappedBuffers();
    std::vector<const libcamera::Stream *> GetStreams();

    int Start(libcamera::ControlList *controlList);
    int Join();


    void ShowCamers();
    void ShowPropertis(int id);
    void ShowControls(int id);
    void ShowStreams();

    static void (*processRequest)(libcamera::Request *);

private:
    std::shared_ptr<libcamera::Camera> camera_;
    static EventLoop eventLoop_;

    libcamera::FrameBufferAllocator *allocator_;
    std::map<libcamera::FrameBuffer *, MappedBuffer> mappedBuffers_;

    std::map<const libcamera::Stream *, std::queue<libcamera::FrameBuffer *>> freeBuffers_;

    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::queue<libcamera::Request *> doneQueue_;
    std::queue<libcamera::Request *> freeQueue_;


    bool isStart_;
    libcamera::CameraManager *cm_;
    int cameraId_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;

    double zoom_;
    int countStreams_;

    static void requestComplete(libcamera::Request *request);

};

#endif //EYE_EYE_H
