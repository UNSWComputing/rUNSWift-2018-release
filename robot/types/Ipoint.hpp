

#pragma once

#include <vector>
#include <math.h>
#include <limits>
#include <iostream>

#include "perception/vision/VisionDefinitions.hpp"


//-------------------------------------------------------

struct Ipoint {

  //! Constructor
  Ipoint() {  };

  //! Destructor
  virtual ~Ipoint() {};

  //! Gets the distance in descriptor space between Ipoints
  float operator-(const Ipoint &rhs)
  {
		if(this->laplacian-rhs.laplacian !=0) return std::numeric_limits<float>::max(); // can't be match
    float sum=0.f;
    for(int i=0; i < SURF_DESCRIPTOR_LENGTH; ++i){
      sum += (this->descriptor[i] - rhs.descriptor[i])*(this->descriptor[i] - rhs.descriptor[i]);
		}
    return sqrt(sum);
  };

	//! Enables Ipoints to be sorted by location on x axis
  bool operator<(const Ipoint &rhs) const
  {
		if (this->x < rhs.x) return true;
		else return false;
  };

  //! Coordinates of the detected interest point
  float x,y;

  //! Detected scale
  float scale;

  //! Sign of laplacian for fast matching purposes
  int laplacian;

  //! If we think this point is on a robot
  int isRobot;

  //! Vector of descriptor components
  float descriptor[SURF_DESCRIPTOR_LENGTH];

	template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & x & y & scale;
      ar & laplacian & isRobot;
      ar & descriptor;
   }

};

inline std::ostream& operator<<(std::ostream& os, const Ipoint& Ip) {
  os << "Coordinates: [" << Ip.x << "," << Ip.y << "] Scale: " << Ip.scale << " laplacian: " << Ip.laplacian << " isRobot: " << Ip.isRobot << std::endl;
  return os;
}
