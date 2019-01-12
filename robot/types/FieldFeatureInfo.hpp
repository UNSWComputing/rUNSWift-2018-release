#ifndef FIELD_FEATURE_INFO_HPP
#define FIELD_FEATURE_INFO_HPP

#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp>

#include "types/Point.hpp"
#include "types/RRCoord.hpp"
#include "types/BBox.hpp"
#include "utils/basic_maths.hpp"

struct FieldLinePointInfo {
   Point p, rrp;

   FieldLinePointInfo () {
   }
   FieldLinePointInfo (Point p, Point rrp)
      : p (p), rrp (rrp) {
   }


   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 4) {
         ar & p & rrp;
      }
   }
};

inline std::ostream& operator<<(std::ostream& os, const FieldLinePointInfo& fieldLinePointInfo) {
   os << fieldLinePointInfo.p;
   os << fieldLinePointInfo.rrp;
   return os;
}

inline std::istream& operator>>(std::istream& is, FieldLinePointInfo& fieldLinePointInfo) {
   is >> fieldLinePointInfo.p;
   is >> fieldLinePointInfo.rrp;
   return is;
}

#ifndef SWIG
BOOST_CLASS_VERSION(FieldLinePointInfo, 4);
#endif

/* Line structure, given in terms of parameters
 * t1, t2 and t3, that satisfy the equation:
 * t1x + t2y + t3 = 0, each of quich are integral.
 **/
struct LineInfo {
   Point p1, p2;
   int t1, t2, t3;
   RRCoord rr;

   LineInfo () {
      p1 = Point(0,0);
      p2 = Point(0,0);
      t1 = 0;
      t2 = 0;
      t3 = 0;
      rr = RRCoord();
   }
   LineInfo (Point p1, Point p2, RRCoord rr = RRCoord(0,0))
      : p1(p1), p2(p2), rr(rr) {
      t1 = p1.y() - p2.y();
      t2 = p2.x() - p1.x();
      t3 = p1.x() * p2.y() - p2.x() * p1.y();
   }


   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & p1 & p2;
      if (file_version >= 1) {
         ar & t1 & t2 & t3;
      }
      if (file_version >= 4) {
         ar & rr;
      }
   }
};

inline std::ostream& operator<<(std::ostream& os, const LineInfo& lineInfo) {
   os << lineInfo.p1;
   os << lineInfo.p2;
   os.write((char*) &(lineInfo.t1), sizeof(int));
   os.write((char*) &(lineInfo.t2), sizeof(int));
   os.write((char*) &(lineInfo.t3), sizeof(int));
   os << lineInfo.rr;
   return os;
}

inline std::istream& operator>>(std::istream& is, LineInfo& lineInfo) {
   is >> lineInfo.p1;
   is >> lineInfo.p2;
   is.read((char*) &(lineInfo.t1), sizeof(int));
   is.read((char*) &(lineInfo.t2), sizeof(int));
   is.read((char*) &(lineInfo.t3), sizeof(int));
   is >> lineInfo.rr;
   return is;
}

#ifndef SWIG
BOOST_CLASS_VERSION(LineInfo, 4);
#endif

struct CornerInfo
{
   // The main corner (kink) point.
   Point p;

   // The points that form the ends of the corner.
   // e1
   // |
   // |
   // p --- e2
   Point e1;
   Point e2;

   CornerInfo () {}
   CornerInfo (Point p, Point e1, Point e2) : p (p), e1 (e1), e2 (e2) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & p;
      ar & e1;
      ar & e2;
   }
};

inline std::ostream& operator<<(std::ostream& os, const CornerInfo& cornerInfo) {
   os << cornerInfo.p;
   os << cornerInfo.e1;
   os << cornerInfo.e2;
   return os;
}

inline std::istream& operator>>(std::istream& is, CornerInfo& cornerInfo) {
   is >> cornerInfo.p;
   is >> cornerInfo.e1;
   is >> cornerInfo.e2;
   return is;
}

struct TJunctionInfo
{
   Point p;

   TJunctionInfo() {}
   TJunctionInfo(Point p) : p (p) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 3) {
         ar & p;
      }
   }
};

inline std::ostream& operator<<(std::ostream& os, const TJunctionInfo& tJunctionInfo) {
   os << tJunctionInfo.p;
   return os;
}

inline std::istream& operator>>(std::istream& is, TJunctionInfo& tJunctionInfo) {
   is >> tJunctionInfo.p;
   return is;
}

#ifndef SWIG
BOOST_CLASS_VERSION(TJunctionInfo, 3);
#endif

struct GoalBoxCornerInfo
{
   Point p;
   /* Left is from the orientation of the STRIKER */
   bool left_corner;

   GoalBoxCornerInfo() {}
   GoalBoxCornerInfo(Point p, bool left_corner) : p (p), left_corner (left_corner) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 5) {
         ar & p;
         ar & left_corner;
      }
   }
};

inline std::ostream& operator<<(std::ostream& os, const GoalBoxCornerInfo& goal_box_corner_info) {
   os << goal_box_corner_info.p;
   os << goal_box_corner_info.left_corner;
   return os;
}

inline std::istream& operator>>(std::istream& is, GoalBoxCornerInfo& goal_box_corner_info) {
   is >> goal_box_corner_info.p;
   is >> goal_box_corner_info.left_corner;
   return is;
}

#ifndef SWIG
BOOST_CLASS_VERSION(GoalBoxCornerInfo, 5);
#endif

struct PenaltySpotInfo
{
   Point p;
   int w, h;

   PenaltySpotInfo() {}
   PenaltySpotInfo(Point p) : p (p) {}
   PenaltySpotInfo(Point p, int w_, int h_) : p (p), w(w_), h(h_) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 3) {
         ar & p;
      }
   }
};

inline std::ostream& operator<<(std::ostream& os, const PenaltySpotInfo& penaltySpotInfo) {
   os << penaltySpotInfo.p;
   return os;
}

inline std::istream& operator>>(std::istream& is, PenaltySpotInfo& penaltySpotInfo) {
   is >> penaltySpotInfo.p;
   return is;
}

#ifndef SWIG
BOOST_CLASS_VERSION(PenaltySpotInfo, 3);
#endif

struct XJunctionInfo
{
   Point p;

   XJunctionInfo() {}
   XJunctionInfo(Point p) : p (p) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 3) {
         ar & p;
      }
   }
};

inline std::ostream& operator<<(std::ostream& os, const XJunctionInfo& xJunctionInfo) {
   os << xJunctionInfo.p;
   return os;
}

inline std::istream& operator>>(std::istream& is, XJunctionInfo& xJunctionInfo) {
   is >> xJunctionInfo.p;
   return is;
}

#ifndef SWIG
BOOST_CLASS_VERSION(XJunctionInfo, 3);
#endif
struct CentreCircleInfo
{
   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
   }
};

struct ParallelLinesInfo
{
   LineInfo l1, l2;

   ParallelLinesInfo() {}
   ParallelLinesInfo(LineInfo l1, LineInfo l2) : l1 (l1), l2 (l2) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 3) {
         ar & l1 & l2;
      }
   }
};
inline std::ostream& operator<<(std::ostream& os, const ParallelLinesInfo& parallelLinesInfo) {
   os << parallelLinesInfo.l1;
   os << parallelLinesInfo.l2;
   return os;
}

inline std::istream& operator>>(std::istream& is, ParallelLinesInfo& parallelLinesInfo) {
   is >> parallelLinesInfo.l1;
   is >> parallelLinesInfo.l2;
   return is;
}


#ifndef SWIG
BOOST_CLASS_VERSION(ParallelLinesInfo, 3);
#endif


struct FieldFeatureInfo {

   enum Type
   {
      fNone           = 0x00,
      fLine           = 0x01,
      fCorner         = 0x02,
      fTJunction      = 0x03,
      fPenaltySpot    = 0x04,
      fCentreCircle   = 0x05,
      fFieldLinePoint = 0x06,
      fXJunction      = 0x07,
      fParallelLines  = 0x08,
      fGoalBoxCorner  = 0x09
   };

   /* Names of corresponding enums from above */
   static const char *const TypeName[];

   FieldFeatureInfo (RRCoord rr, Type type) :
      rr(rr),
      type(type) {
   }

   FieldFeatureInfo (RRCoord rr, LineInfo line) :
      rr (rr),
      type (fLine),
      lineUsed (false),
      line (line)
   {
   }

   FieldFeatureInfo (RRCoord rr, CornerInfo corner) :
      rr (rr),
      type (fCorner),
      corner (corner)
   {
   }

   FieldFeatureInfo (RRCoord rr, TJunctionInfo tjunction) :
      rr (rr),
      type (fTJunction),
      tjunction (tjunction)
   {
   }

   FieldFeatureInfo (RRCoord rr, GoalBoxCornerInfo gbc) :
      rr (rr),
      type (fGoalBoxCorner),
      goal_box_corner (gbc)
   {
   }

   FieldFeatureInfo (RRCoord rr, PenaltySpotInfo penaltyspot) :
      rr (rr),
      type (fPenaltySpot),
      penaltyspot (penaltyspot)
   {
   }

   FieldFeatureInfo (RRCoord rr, XJunctionInfo xjunction) :
      rr (rr),
      type (fXJunction),
      xjunction (xjunction)
   {
   }

   FieldFeatureInfo (RRCoord rr, CentreCircleInfo centrecircle) :
      rr (rr),
      type (fCentreCircle),
      centrecircle (centrecircle)
   {
   }

   FieldFeatureInfo (RRCoord rr, std::vector<FieldLinePointInfo> fieldlinepoints) :
      rr (rr),
      type (fFieldLinePoint),
      fieldlinepoints (fieldlinepoints)
   {
   }

   FieldFeatureInfo (RRCoord rr, ParallelLinesInfo parallellines) :
      rr (rr),
      type (fParallelLines),
      parallellines (parallellines)
   {
   }

   FieldFeatureInfo () {
   }

   FieldFeatureInfo(const FieldFeatureInfo &other) {
      this->rr = other.rr;
      this->type = other.type;

      if (type == fLine) {
         this->lineUsed = other.lineUsed;
         this->line = other.line;
      } else if (type == fCorner) {
         this->corner = other.corner;
      } else if (type == fTJunction) {
         this->tjunction = other.tjunction;
      } else if (type == fPenaltySpot) {
         this->penaltyspot = other.penaltyspot;
      } else if (type == fCentreCircle) {
         this->centrecircle = other.centrecircle;
      } else if (type == fFieldLinePoint) {
         this->fieldlinepoints = other.fieldlinepoints;
      } else if (type == fXJunction) {
         this->xjunction = other.xjunction;
      } else if (type == fParallelLines) {
         this->parallellines = other.parallellines;
      } else if (type == fGoalBoxCorner) {
         this->goal_box_corner = other.goal_box_corner;
      }
   }

   virtual ~FieldFeatureInfo () {
   }

   RRCoord rr;
   Type type;
   bool lineUsed;

   LineInfo           line;
   CornerInfo         corner;
   TJunctionInfo      tjunction;
   PenaltySpotInfo    penaltyspot;
   XJunctionInfo      xjunction;
   CentreCircleInfo   centrecircle;
   GoalBoxCornerInfo  goal_box_corner;
   std::vector<FieldLinePointInfo> fieldlinepoints;
   ParallelLinesInfo  parallellines;

   double distance(void) const {
      if (type == fLine) {
         return pointSegmentDist(Point(0, 0), line.p1, line.p2);
      } else if (type == fCorner) {
         return rr.distance();
      } else if (type == fTJunction) {
         return rr.distance();
      } else if (type == fPenaltySpot) {
         return rr.distance();
      } else if (type == fCentreCircle) {
         return rr.distance();
      } else if (type == fFieldLinePoint) {
         double distanceSum = 0.0;
         for (unsigned i = 0; i < fieldlinepoints.size(); i++) {
            double distance = sqrt(fieldlinepoints[i].rrp.x()*fieldlinepoints[i].rrp.x() +
                  fieldlinepoints[i].rrp.y()*fieldlinepoints[i].rrp.y());
            distanceSum += distance;
         }
         return fieldlinepoints.size() == 0 ? 0.0 : distanceSum/fieldlinepoints.size();
      } else if (type == fXJunction) {
         return rr.distance();
      } else if (type == fParallelLines) {
         double d1 = pointSegmentDist(Point(0, 0), parallellines.l1.p1, parallellines.l1.p2);
         double d2 = pointSegmentDist(Point(0, 0), parallellines.l2.p1, parallellines.l2.p2);
         double result = (d1+d2)/2.0;
         return result;
      } else if (type == fGoalBoxCorner) {
         return rr.distance();
      } else {
         return 0.0;
      }
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & rr;
      ar & type;

      if (type == fLine) {
         ar & line;
         ar & lineUsed;
      } else if (type == fCorner) {
         ar & corner;
      } else if (type == fTJunction) {
         ar & tjunction;
      } else if (type == fPenaltySpot) {
         ar & penaltyspot;
      } else if (type == fCentreCircle) {
         ar & centrecircle;
      } else if (type == fFieldLinePoint) {
         ar & fieldlinepoints;
      } else if (type == fXJunction) {
         ar & xjunction;
      } else if (type == fParallelLines) {
         ar & parallellines;
      } else if (type == fGoalBoxCorner){
         ar & goal_box_corner;
      }
   }
};
inline std::ostream& operator<<(std::ostream& os, const FieldFeatureInfo& fieldFeatureInfo) {
   os << fieldFeatureInfo.rr;
   os.write((char*) &(fieldFeatureInfo.type), sizeof(FieldFeatureInfo::Type));
   os.write((char*) &(fieldFeatureInfo.lineUsed), sizeof(bool));

   os << fieldFeatureInfo.line;
   os << fieldFeatureInfo.corner;
   os << fieldFeatureInfo.tjunction;
   os << fieldFeatureInfo.penaltyspot;
   os << fieldFeatureInfo.xjunction;
   os << fieldFeatureInfo.parallellines;
   os << fieldFeatureInfo.goal_box_corner;

   unsigned numFieldlinepoints = fieldFeatureInfo.fieldlinepoints.size();
   os.write((char*) &numFieldlinepoints, sizeof(unsigned));
   for (unsigned i = 0; i < numFieldlinepoints; i++) {
      os << fieldFeatureInfo.fieldlinepoints[i];
   }

   return os;
}

inline std::istream& operator>>(std::istream& is, FieldFeatureInfo& fieldFeatureInfo) {
   is >> fieldFeatureInfo.rr;
   is.read((char*) &(fieldFeatureInfo.type), sizeof(FieldFeatureInfo::Type));
   is.read((char*) &(fieldFeatureInfo.lineUsed), sizeof(bool));

   is >> fieldFeatureInfo.line;
   is >> fieldFeatureInfo.corner;
   is >> fieldFeatureInfo.tjunction;
   is >> fieldFeatureInfo.penaltyspot;
   is >> fieldFeatureInfo.xjunction;
   is >> fieldFeatureInfo.parallellines;
   is >> fieldFeatureInfo.goal_box_corner;

   unsigned numFieldlinepoints;
   is.read((char*) &numFieldlinepoints, sizeof(unsigned));
   for (unsigned i = 0; i < numFieldlinepoints; i++) {
      FieldLinePointInfo fieldLinePointInfo;
      is >> fieldLinePointInfo;
      fieldFeatureInfo.fieldlinepoints.push_back(fieldLinePointInfo);
   }

   return is;
}

#endif
