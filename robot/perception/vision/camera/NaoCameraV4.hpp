#pragma once

#include <string>

#include "perception/vision/camera/NaoCamera.hpp"

/**
 * NaoCameraV4 provides methods for the vision module to interact with
 * the nao camera
 */
class NaoCameraV4 : public NaoCamera {
  public:
   /**
    * Constructor
    * opens the device, calibrates it, and sets it up for streaming
    *
    * @param filename the device or file to get images from
    * @param method the io method to use
    * @see IOMethod
    * @param format the format of the image
    * @see AL::kVGA
    * @param cameraChoice - 'camera.top' or 'camera.bottom'
    */
   NaoCameraV4(Blackboard *blackboard,
               const char *filename,
               const std::string cameraChoice,
               const IOMethod method = IO_METHOD_MMAP,
               const int format = AL::k960p);

   /**
    * Destructor
    * closes the device
    */
   virtual ~NaoCameraV4();
};
