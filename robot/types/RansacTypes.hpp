#pragma once

#include <math.h>
#include <boost/serialization/version.hpp>
#include <limits>

#include "types/Point.hpp"

struct RANSACLine
{
   Point p1, p2;
   /**
    * Line defined in terms of
    * t1 * x + t2 * y + t3 = 0
    */
   long long t1, t2, t3;
   float var;
   long long sqrtt1t2;

   RANSACLine(Point p1, Point p2, float var = 0) : p1(p1), p2(p2), var(var)
   {
      t1 = p1.y() - p2.y();
      t2 = p2.x() - p1.x();
      t3 = p1.x() * p2.y() - p2.x() * p1.y();
      sqrtt1t2 = sqrt(pow(t1, 2) + pow(t2, 2));
   }

   RANSACLine() {};

   RANSACLine(const RANSACLine &other) {
      this->t1 = other.t1;
      this->t2 = other.t2;
      this->t3 = other.t3;
      this->var = other.var;
      this->sqrtt1t2 = other.sqrtt1t2;

      this->p1.x() = other.p1.x();
      this->p1.y() = other.p1.y();
      this->p2.x() = other.p2.x();
      this->p2.y() = other.p2.y();
   }

    /*
    Calculates the distance between this line and the given point.
    */
    inline int distance(const Point& other)
    {
        if(sqrtt1t2 > 0)
            return(abs(t1 * other.x() + t2 * other.y() + t3)/sqrtt1t2);
        // TODO: A better way to deal with cases where sqrtt1t2 == 0.
        else
            return(0);
    }

    /*
    Calculates the distance between this line and the given point.
    */
    inline int distanceDirected(const Point& other)
    {
        if(sqrtt1t2 > 0)
            return(abs(t1 * other.x() + t2 * other.y() + t3)/sqrtt1t2);
        // TODO: A better way to deal with cases where sqrtt1t2 == 0.
        else
            return(0);
    }

    /*
    Calculates the intersection between this line and another. If the lines are
    parallel (int_min, int_max) is returned.
    */
    inline Point getIntersect(const RANSACLine& other)
    {
        /*
        x = (t3*other.t2 - t2*other.t3) / (t2*other.t1 - t1*other.t2);
        y = (t1*other.t3 - t3*other.t1) / (t2*other.t1 - t1*other.t2);
        */

        // The point of intersection.
        Point intersect;

        // Calculate the divisor.
        long long divisor = t2*other.t1 - t1*other.t2;

        // Check for parallel lines.
        if(divisor == 0)
        {
            intersect.x() = std::numeric_limits<int>::min();
            intersect.y() = std::numeric_limits<int>::max();
        }
        else
        {
            // Lines are not parallel, calculate the intersection.
            intersect.x() = (int)((t3*other.t2 - t2*other.t3)/divisor);
            intersect.y() = (int)((t1*other.t3 - t3*other.t1)/divisor);
        }

        // Return the calculated intersection point.
        return(intersect);
    }

    /*
    Gets the angle of this line in the absolute coordinate space.
    */
    inline float getAngle()
    {
        return(atan2(t1, t2));
    }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & t1 & t2 & t3;
      ar & var;
      if (file_version >= 1) {
        ar & p1 & p2;
      } else {
        p1 = Point (0, 0);
        p2 = Point (0, 0);
      }
   }
};

inline std::ostream& operator<<(std::ostream& os, const RANSACLine& line) {
   os << line.p1;
   os << line.p2;
   os.write((char*) &(line.t1), sizeof(int));
   os.write((char*) &(line.t2), sizeof(int));
   os.write((char*) &(line.t3), sizeof(int));
   os.write((char*) &(line.var), sizeof(float));

   return os;
}

inline std::istream& operator>>(std::istream& is, RANSACLine& line) {
   is >> line.p1;
   is >> line.p2;
   is.read((char*) &(line.t1), sizeof(int));
   is.read((char*) &(line.t2), sizeof(int));
   is.read((char*) &(line.t3), sizeof(int));
   is.read((char*) &(line.var), sizeof(float));

   return is;
}

BOOST_CLASS_VERSION(RANSACLine, 1);

struct RANSACCircle
{
   PointF centre;
   PointF secondaryCentre;
   float radius;
   float var;

   RANSACCircle(PointF centre, float radius, float var = 0)
      : centre(centre), secondaryCentre(centre), radius(radius), var(var)
   {
   }

   RANSACCircle(const Point p1, const Point p2, const Point p3, float var = 0)
      : var(var)
   {
      float bx = p1.x(); float by = p1.y();
      float cx = p2.x(); float cy = p2.y();
      float dx = p3.x(); float dy = p3.y();

      float temp = cx*cx+cy*cy;
      float bc   = (bx*bx + by*by - temp)/2.0;
      float cd   = (temp - dx*dx - dy*dy)/2.0;
      float det  = (bx-cx)*(cy-dy)-(cx-dx)*(by-cy);

      centre = PointF();
      secondaryCentre = PointF();
      radius = 0;

      if (fabs(det) < 1.0e-6) {
         this->radius = std::numeric_limits<float>::quiet_NaN();
         return;
      }

      det = 1 / det;
      centre.x() = (bc*(cy-dy)-cd*(by-cy))*det;
      centre.y() = ((bx-cx)*cd-(cx-dx)*bc)*det;
      cx = centre.x(); cy = centre.y();
      radius = sqrt((cx-bx)*(cx-bx)+(cy-by)*(cy-by));
      secondaryCentre = centre;
   }

    RANSACCircle(const Point p1, const Point p2, float radius, float var = 0)
      : radius(radius), var(var)
    {
        // Check that the points aren't right on top of each other.
        centre = PointF();
        secondaryCentre = PointF();
        if (p1 == p2) {
            this->radius = std::numeric_limits<float>::quiet_NaN();
            return;
        };

        // Half the distance to the points, also the length of the edge of the
        // right angled triangle formed by splitting the triangle p1, p2,
        // centre.
        float centreEdge = sqrt(pow(p1.x()-p2.x(), 2) + pow(p1.y()-p2.y(), 2))
                                                                            / 2;

        // The "height" of the triangle p1, p2, centre.
        float height = sqrt(pow(radius, 2) - pow(centreEdge, 2));

        // The unit vector in the direction from p1 to p2.
        PointF betweenVector = (p1-p2).cast<float>().normalized();

        // The vector perpendicular to betweenPoints in the counterclockwise
        // direction, of the length required to reach centre from half way
        // between p1 and p2.
        PointF perpendicularVector;

        // The point half way between p1 and p2.
        PointF betweenPoint = p1.cast<float>() + (betweenVector*centreEdge);

        // Create the perpendicular vector.
        perpendicularVector.x() = betweenVector.y()*height;
        perpendicularVector.y() = -betweenVector.x()*height;

        // Find the two potential circles.
        this->centre = betweenPoint + perpendicularVector;
        this->secondaryCentre = betweenPoint - perpendicularVector;
    }

   RANSACCircle() {};

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & centre;
      ar & secondaryCentre;
      ar & radius;
      ar & var;
   }

};

BOOST_CLASS_VERSION(RANSACCircle, 0);
