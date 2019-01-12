#include "perception/vision/camera/CombinedCamera.hpp"
#include "perception/vision/camera/NaoCamera.hpp"

Camera *CombinedCamera::top_camera_ = NULL;
Camera *CombinedCamera::bot_camera_ = NULL;

CombinedCamera::CombinedCamera(
    bool dumpframes,
    int dumprate,
    string dumpfile
){
    if (dumpframes) {
        // Setting recording on either camera starts it for both
        top_camera_->startRecording(dumpfile.c_str(), dumprate);
    }
}

Camera* CombinedCamera::getCamera() {
    return top_camera_;
}

CombinedCamera::~CombinedCamera(){
    // Stopping recording on either camera starts it for both
    top_camera_->stopRecording();
}

const uint8_t* CombinedCamera::getFrameTop() {
    return top_camera_->get();
}

const uint8_t* CombinedCamera::getFrameBottom() {
    return bot_camera_->get();
}

Camera* CombinedCamera::getCameraTop() {
    return top_camera_;
}
Camera* CombinedCamera::getCameraBot() {
    return bot_camera_;
}
void CombinedCamera::setCameraTop(Camera* camera) {
    top_camera_ = camera;
}
void CombinedCamera::setCameraBot(Camera* camera) {
    bot_camera_ = camera;
}

CombinedCameraSettings CombinedCamera::getCameraSettings(){
    NaoCamera* top = (NaoCamera*)(top_camera_);
    NaoCamera* bot = (NaoCamera*)(bot_camera_);
    
    CombinedCameraSettings settings;
    settings.top_camera_settings = top->topCameraSettings;
    settings.bot_camera_settings = bot->botCameraSettings;
    return settings;
}

