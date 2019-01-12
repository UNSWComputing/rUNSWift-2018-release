#pragma once

#include "types/RansacTypes.hpp"

/* 2010 code required two field edges to be written, one robot relative,
 * and one image relative for debugging. This is just a wrapper. Feel
 * free to change
 */
struct FieldBoundaryInfo
{
   FieldBoundaryInfo () {}
   FieldBoundaryInfo (RANSACLine rrBoundary, RANSACLine imageBoundary) :
      rrBoundary (rrBoundary),
      imageBoundary (imageBoundary) {}

   virtual ~FieldBoundaryInfo () {}

   RANSACLine rrBoundary;
   RANSACLine imageBoundary;

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & rrBoundary;
      ar & imageBoundary;
   }
};

inline std::ostream& operator<<(std::ostream& os, const FieldBoundaryInfo& fieldBoundary) {
   os << fieldBoundary.rrBoundary;
   os << fieldBoundary.imageBoundary;
   return os;
}

inline std::istream& operator>>(std::istream& is, FieldBoundaryInfo& fieldBoundary) {
   is >> fieldBoundary.rrBoundary;
   is >> fieldBoundary.imageBoundary;
   return is;
}
