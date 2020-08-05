// Polygon manipulation library for the Proficio.
#include <math.h>
#include <stdio.h>      // printf
#include <stdlib.h>     // abs

class TargetZone
{ 
  //========================== STRUCTS ==============================
    /**
     * 3D point
     */
    struct Point
    {
      double x;
      double y;
      double z;
      Point() {}
      Point(double x, double y, double z) : x(x), y(y), z(z) {}

      bool operator<(const Point &o) const
      {
        if (x != o.x) {
          return x < o.x;
        }
        if (y != o.y) {
          return y < o.y;
        }   
        return z < o.z;
      }
      
      Point operator+(const Point &d) const
      {
        double newX = x + d.x;
        double newY = y + d.y;
        double newZ = z + d.z;
        return Point(newX, newY, newZ);        
      }
      
      Point operator-(const Point &d) const
      {
        double newX = x - d.x;
        double newY = y - d.y;
        double newZ = z - d.z;
        return Point(newX, newY, newZ);        
      }
    };
    
    /**
     * Rectangle
     */
    struct Rectangle
    {
      Point tl; // top left
      Point bl; // bottom left
      Point br; // bottom right
      Point tr; // top right
      Rectangle() {}
      Rectangle(Point tl, Point bl, Point br, Point tr) : tl(tl), bl(bl), br(br), tr(tr) {}
    };

  // ======================= PROPERTIES =============================
  private:
    Rectangle _zone;
    Rectangle _target;
    Rectangle _targetLim;
    Point _center; // System center
    double _zDepth;
    double _angle; //radians
    double _width;
    
  public:
    Rectangle zoneR;
    Rectangle targetR;
    Rectangle targetLimR;
    double errorLim;
    const int &angle;  
  
  //========================== METHODS ==============================
  /**
   * Manage the logic of determining where imaginary target region is
   * and determining if the cursor is still within acceptable range
   */
  public:
    /**
     * Constructor
     */
    TargetZone(double centerX, double centerY, double centerZ,
               double width, double length, double errLim = 0, double depth = 0) : angle(_angle)
    {
      _center = Point(centerX, centerY, centerZ);
      double dx = length/2;
      double dy = width/2;
      
      Point tl = _center + Point(dx, dy, 0);
      Point bl = _center + Point(dx, -dy, 0);
      Point br = _center + Point(-dx, -dy, 0);
      Point tr = _center + Point(-dx, dy, 0);
      
      // set base zone and target region
      _zone = Rectangle(tl, bl, br, tr);
      _target = Rectangle(tl, bl, br, tr);
      
      // set accessible zone and target region
      zoneR = Rectangle(tl, bl, br, tr);
      targetR = Rectangle(tl, bl, br, tr);
      
      // set acceptable depth. Probably not used...
      _zDepth = depth;
      
      _width = width;
      
      // set Limit you can be outside of target
      errorLim = errLim;
    }
  
    /**
     * Distance from system center, and the width of target region
     */
    void setTarget(double distance, double targetWidth)
    {
      // delta for center
      double dx = distance;
      Point targetCenter = _center + Point(-dx, 0, 0); //Negative dx b/c reasons.
      
      // delta for width
      dx = targetWidth/2;
      double dy = _width/2;
      Point tl = targetCenter + Point( dx, dy, 0);
      Point bl = targetCenter + Point( dx,-dy, 0);
      Point br = targetCenter + Point(-dx,-dy, 0);
      Point tr = targetCenter + Point(-dx, dy, 0);
      
      // set target region
      _target = Rectangle(tl, bl, br, tr);
      targetR = rotateRectangle(_target, _angle);
      
      // set Target error boundary for center--------------------------------------------
      dx = (targetWidth/2) + errorLim;
            
      tl = targetCenter + Point( dx, dy, 0);
      bl = targetCenter + Point( dx,-dy, 0);
      br = targetCenter + Point(-dx,-dy, 0);
      tr = targetCenter + Point(-dx, dy, 0);
      
      // set target region with error boundary
      _targetLim = Rectangle(tl, bl, br, tr);
      targetLimR = rotateRectangle(_targetLim, _angle);
    }
     
    /**
     * Rotate public rectangle and target, theta is in radians
     */
    void rotate(double theta)
    {
      // set angle
      this->_angle = theta;
      
      // Rotate zone
      zoneR = rotateRectangle(_zone, theta);
      
      // Rotate target
      targetR = rotateRectangle(_target, theta);
    }
    

    /**
     * Determine if a point is within a rectangle (should be private)
     */
    bool inRegion(Rectangle rectangle, Point p)
    {
      Point V[4] {rectangle.tl, rectangle.bl, rectangle.br, rectangle.tr};
      return cn_PnPoly(p, V, 4) == 1; // 4 points in rectangle
    }
    
    /**
     * See if point is still within acceptable zone for pursuit
     */
    bool inZone(double x, double y, double z = 0)
    {
      Point p = Point(x, y, z);
      return inRegion(zoneR, p);      
    }
    
    /**
     * See if point is within the target with error boundary
     */
    bool inTargetLim(double x, double y, double z = 0)
    {
      Point p = Point(x, y, z);
      return inRegion(targetLimR, p);
    }
    
    
    /**
     * See if point is within the target
     */
    bool inTarget(double x, double y, double z = 0)
    {
      Point p = Point(x, y, z);
      return inRegion(targetR, p);
    }
    
    /**
     * insure vector in the correct direction
     */
    bool inDirection(double x1, double x2, double y1, double y2, double tolerance)
    {
      double dot = x1*x2 + y1*y2;
      double det = x1*y2 - y1*x2;
      double angle = atan2(det, dot); // TODO, is this radians or degrees?
      return abs(angle) < tolerance;
    }
    
    /**
     * Return magnitude of vector in 2d
     */
    double magnitude(double dx, double dy)
    {
      return sqrt(dx*dx + dy*dy);
    }
    
    /**
     *
     */
    bool adequateForce(double fx, double fy, double minMag, double tolerance)
    {
      // rotate unit vector about 0,0
      bool dir = inDirection(fx, cos(angle), fy, cos(angle), tolerance);
      bool mag = magnitude(fx,fy) >= minMag;
      return dir && mag;
    }
  
  private:    
    /** a Point is defined by its coordinates {int x, y;}
     * isLeft(): tests if a point is Left|On|Right of an infinite line.
     *   Input:  three points P0, P1, and P2
     *    Return: >0 for P2 left of the line through P0 and P1
     *            =0 for P2  on the line
     *            <0 for P2  right of the line
     *    See: Algorithm 1 "Area of Triangles and Polygons"
     */
    inline int isLeft(Point P0, Point P1, Point P2)
    {
      return ( (P1.x - P0.x) * (P2.y - P0.y) - (P2.x -  P0.x) * (P1.y - P0.y) );
    }
    

    /** cn_PnPoly(): crossing number test for a point in a polygon
     *      Input:   P = a point,
     *               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
     *      Return:  0 = outside, 1 = inside
     * This code is patterned after [Franklin, 2000]
     */
    int cn_PnPoly(Point P, Point* V, int n)
    {
      int cn = 0; // the  crossing number counter

      // loop through all edges of the polygon
      for (int i=0; i<n; i++)
      {    // edge from V[i]  to V[i+1]
        if (((V[i].y <= P.y) && (V[(i+1) % n].y > P.y))     // an upward crossing
          || ((V[i].y > P.y) && (V[(i+1) % n].y <=  P.y))) 
        { // a downward crossing
          // compute  the actual edge-ray intersect x-coordinate
          float vt = (float)(P.y  - V[i].y) / (V[(i+1) % n].y - V[i].y);
          if (P.x <  V[i].x + vt * (V[(i+1) % n].x - V[i].x)) // P.x < intersect
            ++cn;   // a valid crossing of y=P.y right of P.x
        }
      }
      return (cn&1);    // 0 if even (out), and 1 if  odd (in)
      //return true;
    }
    
    /**
     * Rotate rectangle about system center in the XY plane theta in radians
     */
    Rectangle rotateRectangle(Rectangle rect, double theta)
    {
      Point tl = rotatePoint(rect.tl, theta);
      Point bl = rotatePoint(rect.bl, theta);
      Point br = rotatePoint(rect.br, theta);
      Point tr = rotatePoint(rect.tr, theta);
      
      return Rectangle(tl, bl, br, tr);
    }
    
    /**
     * Rotate point about system center in the XY plane, theta is in radians
     */
    Point rotatePoint(Point p, double theta)
    {
      double ox = _center.x;
      double oy = _center.y;
      
      double qx = ox + (cos(theta) * (p.x - ox)) - (sin(theta) * (p.y - oy));
      double qy = oy + (sin(theta) * (p.x - ox)) + (cos(theta) * (p.y - oy));
      
      return Point(qx, qy, p.z);
    }
};
