#pragma once

#include "types/Point.hpp"
#include <iostream>

struct BBox
{
   BBox () {}
   BBox (Point a, Point b) : a(a), b(b) {};
   virtual ~BBox () {}

   /*
   Expands a bounding box by the factor provided, scaling it around its centre
   point.
   */
   BBox expand(float multiplier)
   {
       // The expanded bounding box.
       BBox newBox;

       // The amount by which the bounding box should be expanded on each side.
       int xExpand = (width()*(multiplier-1.0f))/2.0f;
       int yExpand = (height()*(multiplier-1.0f))/2.0f;

       // Initialise the new bounding box with the old one.
       newBox.a = a;
       newBox.b = b;

       // Expand the new bounding box by multiplier.
       newBox.a.x() -= xExpand;
       newBox.b.x() += xExpand;
       newBox.a.y() -= yExpand;
       newBox.b.y() += yExpand;

       // Return the expanded bounding box.
       return(newBox);
   }

   int width() const
   {
      return b.x() - a.x();
   }

   int height() const
   {
      return b.y() - a.y();
   }

   bool within(const Point p) const
   {
      return (a.x() <= p.x() && p.x() <= b.x())
         && (a.y() <= p.y() && p.y() <= b.y());
   }

   bool validIndex(const Point p) const
   {
      return (0 <= p.x() && p.x() <= width())
         && (0 <= p.y() && p.y() <= height());
   }

   Point a, b;

   bool operator== (const BBox &other) const
   {
      return a == other.a && b == other.b;
   }

   bool operator!= (const BBox &other) const
   {
      return (a != other.a || b != other.b);
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & a;
      ar & b;
   }
};

inline std::ostream& operator<<(std::ostream& os, const BBox& bbox) {
   os << bbox.a;
   os << bbox.b;
   return os;
}

inline std::istream& operator>>(std::istream& is, BBox& bbox) {
   is >> bbox.a;
   is >> bbox.b;
   return is;
}
