//
// Created by Andrey Pahomov on 09.02.21.
//

#include <iostream>
#include <cstdlib>

#include <memory>
#include <sys/mman.h>

#include "Eye.h"



using namespace libcamera;

static void emptyCall(Request *request) {
    std::cerr << "Not set call\n";
    exit(EXIT_FAILURE);
}

void (*Eye::processRequest)(libcamera::Request *) = emptyCall;

/*
 * ----------------------------------------------------------------------------
 * Camera Naming.
 *
 * Applications are responsible for deciding how to name cameras, and present
 * that information to the users. Every camera has a unique identifier, though
 * this string is not designed to be friendly for a human reader.
 *
 * To support human consumable names, libcamera provides camera properties
 * that allow an application to determine a naming scheme based on its needs.
 *
 * In this example, we focus on the location property, but also detail the
 * model string for external cameras, as this is more likely to be visible
 * information to the user of an externally connected device.
 *
 * The unique camera ID is appended for informative purposes.
 */
std::string cameraName(Camera *camera)
{
    const ControlList &props = camera->properties();
    std::string name;

    switch (props.get(properties::Location)) {
        case properties::CameraLocationFront:
            name = "Internal front camera";
            break;
        case properties::CameraLocationBack:
            name = "Internal back camera";
            break;
        case properties::CameraLocationExternal:
            name = "External camera";
            if (props.contains(properties::Model))
                name += " '" + props.get(properties::Model) + "'";
            break;
    }

    name += " (" + camera->id() + ")";

    return name;
}


Eye::Eye():
    allocator_(nullptr),
    countStreams_(0),
    zoom_(1),
    cm_(nullptr),
    isStart_(false),
    cameraId_(-1) {

}

Eye::~Eye() {
//    eventLoop_.exit(EXIT_SUCCESS);
    camera_->stop();
    delete allocator_;
    camera_->release();
    camera_.reset();
    cm_->stop();

    eventLoop_.exit(EXIT_SUCCESS);
}

int Eye::Init() {
    cm_ = new CameraManager();
    int ret = cm_->start();
    if (ret) {
        std::cout  << "Failed to start camera manager:"
                   << ret << std::endl;
        return EXIT_FAILURE;
    }

    isStart_ = true;

    return EXIT_SUCCESS;
}

int  Eye::SetCameraId(int id, std::string &idSensor) {
    if (id>=0) {
        std::string cameraName = cm_->cameras()[id]->id();
        camera_ = cm_->get(cameraName);
        if (camera_) {
            cameraId_ = id;
            idSensor = cameraName;

            if (camera_->acquire()) {
                std::cout << "Failed to acquire camera\n";
                camera_.reset();
                idSensor = "";
                return EXIT_FAILURE;
            }

            return EXIT_SUCCESS;
        }

    }

    return EXIT_FAILURE;
}

int Eye::CameraConfiguration(const StreamRoles &roles) {
    /*
     * The CameraConfiguration contains a StreamConfiguration instance
     * for each StreamRole requested by the application, provided
     * the Camera can support all of them.
     *
     * Each StreamConfiguration has default size and format, assigned
     * by the Camera depending on the Role the application has requested.
     */
    config_ = camera_->generateConfiguration(roles);

    countStreams_ = config_->size();

    return EXIT_SUCCESS;
}

int Eye::SetConfiguration(int idStream,
        libcamera::Size size, libcamera::PixelFormat pixelFormat) {

    if (idStream >= countStreams_)
        return EXIT_FAILURE;

    config_->at(idStream).size = size;
    config_->at(idStream).pixelFormat = pixelFormat;

    return EXIT_SUCCESS;
}

int Eye::SetConfiguration(int idStream, libcamera::Size size) {

    if (idStream >= countStreams_)
        return EXIT_FAILURE;

    config_->at(idStream).size = size;

    return EXIT_SUCCESS;
}

libcamera::StreamConfiguration &Eye::GetStreamConfiguration(int idStream) {
    return config_->at(idStream);
}

int Eye::StreamConfiguration() {

    config_->validate();
    if (camera_->configure(config_.get())) {
        std::cout << "CONFIGURATION FAILED!" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int Eye::BufferAlloc() {

    /* Allocate and map buffers. */
    allocator_ = new FrameBufferAllocator(camera_);
    for (struct StreamConfiguration &config : *config_) {
        Stream *stream = config.stream();

        int ret = allocator_->allocate(stream);
        if (ret < 0) {
            std::cerr << "Failed to allocate capture buffers\n";
            return  ret;
        }

        for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream)) {
            /* Map memory buffers and cache the mappings. */
            const FrameBuffer::Plane &plane = buffer->planes().front();
            void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
                                plane.fd.fd(), 0);
            mappedBuffers_[buffer.get()] = {memory, plane.length};

            /* Store buffers on the free list. */
            freeBuffers_[stream].push(buffer.get());
        }
    }

    return  EXIT_SUCCESS;
}

int Eye::CreateRequests() {
    /* Create requests and fill them with buffers. */

    for (struct StreamConfiguration &config:  *config_) {
        Stream *stream = config.stream();

        while (!freeBuffers_[stream].empty()) {
            FrameBuffer *buffer = freeBuffers_[stream].front();//.dequeue();

            std::unique_ptr<Request> request = camera_->createRequest();
            if (!request) {
                std::cerr << "Can't create request\n";
                return -ENOMEM;

            }

            int ret = request->addBuffer(stream, buffer);
            if (ret < 0) {
                std::cerr << "Can't set buffer for request";
                return -ENOMEM;
            }

            requests_.push_back(std::move(request));

            freeBuffers_[stream].pop();
        }
    }

    return EXIT_SUCCESS;
}

void Eye::RequestCompleted(void (*processRequest)(libcamera::Request *), Request *request) {
    if (request->status() == Request::RequestCancelled)
        return;

    Eye::eventLoop_.callLater(std::bind(processRequest, request));
}

void Eye::SetCallBack(void (*call)(libcamera::Request *)) {
    processRequest = call;
    camera_->requestCompleted.connect(processRequest);
}

void Eye::AddRequest(libcamera::Request *request) {
    camera_->queueRequest(request);
}

Rectangle Eye::GetSensorArea() {
    return camera_->properties().get(libcamera::properties::ScalerCropMaximum);
}

std::map<FrameBuffer *, MappedBuffer> Eye::GetMappedBuffers() {
    return mappedBuffers_;
}

std::vector<const Stream *> Eye::GetStreams() {
    std::vector<const Stream *> ret;

    for (auto &conf: *config_) {
        ret.push_back(conf.stream());
    }
    return std::move(ret);
}

int Eye::Start(libcamera::ControlList *controlList) {
    camera_->start(controlList);

    for (std::unique_ptr<Request> &request : requests_)
        camera_->queueRequest(request.get());

    if (!cm_->cameras().size()) {
        std::cout << "No cameras were identified on the system."
                  << std::endl;
        cm_->stop();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


int Eye::Join() {
    eventLoop_.timeout(-1);
    eventLoop_.exec();
    camera_->stop();
    delete allocator_;
    camera_->release();
    camera_.reset();
    cm_->stop();

    return EXIT_SUCCESS;
}

void Eye::ShowCamers() {
    if (isStart_) {
        for (auto const &camera : cm_->cameras())
            std::cout << " - " << cameraName(camera.get()) << std::endl;
    }
}


void Eye::ShowPropertis(int id) {
    std::cout << "\nPropertis:\n";
    for(auto prop: cm_->cameras()[id]->properties()) {
        std::cout
        <<  "\tid: 0x" << std::hex << prop.first << std::dec
        <<  "\t "<< prop.second.toString() << "\n";
    }

}

void Eye::ShowControls(int id) {
    std::cout << "\nControls:\n";
    for(auto cont: cm_->cameras()[id]->controls()) {
        std::cout
        <<  "\tid: 0x" << std::hex << cont.first->id() << std::dec
        <<"\tname: "  << cont.first->name()
        <<  "\t "<< cont.second.toString() << "\n";
    }
}

void Eye::ShowStreams() {
    for (auto camConfig = config_->begin();
         camConfig != config_->end(); ++camConfig) {
        std::cout << "Configuration is: "
                  << camConfig->toString() << "\n";

        if(camConfig->stream())
            for(auto pixelformat: camConfig->formats().pixelformats()) {
                std::cout << "\t" << pixelformat.toString() << "\n";
            }
    }
}
