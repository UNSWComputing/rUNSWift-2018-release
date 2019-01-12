#pragma once

struct CameraSettings {
    unsigned int hflip;
    unsigned int vflip;
    unsigned int brightness;
    unsigned int contrast;
    unsigned int saturation;
    unsigned int hue;
    unsigned int sharpness;
    unsigned int backlightCompensation;
    unsigned int exposure;
    unsigned int gain;
    unsigned int whiteBalance;
    unsigned int exposureAuto;
    unsigned int autoWhiteBalance;
    unsigned int exposureAlgorithm;
    unsigned int aeTargetAvgLuma;
    unsigned int aeTargetAvgLumaDark;
    unsigned int aeTargetGain;
    unsigned int aeMinVirtGain;
    unsigned int aeMaxVirtGain;
    unsigned int aeMinVirtAGain;
    unsigned int aeMaxVirtAGain;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
      ar & hflip;
      ar & vflip;
      ar & brightness;
      ar & contrast;
      ar & saturation;
      ar & hue;
      ar & sharpness;
      ar & backlightCompensation;
      ar & exposure;
      ar & gain;

      if (file_version <= 19){
        int tmp;
        ar & tmp;
        whiteBalance = (unsigned int) tmp;
      } else {
        ar & whiteBalance;
      }

      if (file_version >= 20){        
        ar & exposureAuto;
        ar & autoWhiteBalance;
        ar & exposureAlgorithm;
      }
      if (file_version >=21){
        ar & aeTargetAvgLuma;
        ar & aeTargetAvgLumaDark;
        ar & aeTargetGain;
        ar & aeMinVirtGain;
        ar & aeMaxVirtGain;
      }
      if (file_version >=22){
        ar & aeMinVirtAGain;
        ar & aeMaxVirtAGain;
      }

    }
};
