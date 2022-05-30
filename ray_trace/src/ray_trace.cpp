#include "ros/ros.h"
#include "ray_trace/ray_trace.h"
#include "geometry_msgs/Point.h"
#include "ray_trace/Triangle.h"
#include <iostream>
#include <vector>
#include <locale.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>


float kEpsilon = 0.000001;

struct V3f{
  float x;
  float y;
  float z;
};

V3f crossProduct(V3f point1, V3f point2){

  V3f vector; 

  vector.x = point1.y * point2.z - point2.y * point1.z; 
  vector.y = point2.x * point1.z - point1.x * point2.z; 
  vector.z = point1.x * point2.y - point1.y * point2.x; 

  return vector;
}

float dotProduct(V3f dot1, V3f dot2){

  float dot = dot1.x * dot2.x + dot1.y * dot2.y + dot1.z * dot2.z; 

  return dot;
}

//orig: ray origin, dir: ray direction, Triangle vertices: p0, p1, p2.  
V3f rayTriangleIntersect(V3f orig, V3f dir, V3f p0, V3f p1, V3f p2){ 
  V3f P;
  P.x = -1;
  P.y = -1;
  P.z = -1;

// compute plane's normal

  V3f p0p1, p0p2;

  p0p1.x = p1.x - p0.x; 
  p0p1.y = p1.y - p0.y; 
  p0p1.z = p1.z - p0.z; 

  p0p2.x = p2.x - p0.x;
  p0p2.y = p2.y - p0.y; 
  p0p2.z = p2.z - p0.z;

  // no need to normalize
  V3f N = crossProduct(p0p1, p0p2); // N 

  // Step 1: finding P

  // check if ray and plane are parallel ?
  float NdotRayDirection = dotProduct(N, dir); // if the result is 0, the function will return the value false (no intersection).

  if (fabs(NdotRayDirection) < kEpsilon){ // almost 0 

      return P; // they are parallel so they don't intersect ! 
  }

  // compute d parameter using equation 2
  float d = dotProduct(N, p0); 

  // compute t (equation P=O+tR P intersection point ray origin O and its direction R)

  float t =-((dotProduct(N, orig) - d) / NdotRayDirection);

  // check if the triangle is in behind the ray
  //if (t < 0){ return false; } // the triangle is behind 

  // compute the intersection point using equation

  //this part should do the work, but it does not work.
  P.x = orig.x + t * dir.x; 
  P.y = orig.y + t * dir.y; 
  P.z = orig.z + t * dir.z; 


  // Step 2: inside-outside test
  V3f C; // vector perpendicular to triangle's plane 

  // edge 0
  V3f edge0; 

  edge0.x = p1.x - p0.x;
  edge0.y = p1.y - p0.y;
  edge0.z = p1.z - p0.z;

  V3f vp0; 

  vp0.x = P.x - p0.x;
  vp0.y = P.y - p0.y; 
  vp0.z = P.z - p0.z; 

  C = crossProduct(edge0, vp0); 

  if (dotProduct(N, C) < 0) { return P; }// P is on the right side 

  // edge 1
  V3f edge1;

  edge1.x = p2.x - p1.x;
  edge1.y = p2.y - p1.y;
  edge1.z = p2.z - p1.z;

  V3f vp1; 

  vp1.x = P.x - p1.x; 
  vp1.y = P.y - p1.y; 
  vp1.z = P.z - p1.z; 

  C = crossProduct(edge1, vp1); 

  if (dotProduct(N, C) < 0) { return P; } // P is on the right side 

  // edge 2
  V3f edge2;

  edge2.x = p0.x - p2.x;    
  edge2.y = p0.y - p2.y;
  edge2.z = p0.z - p2.z;

  V3f vp2; 

  vp2.x = P.x - p2.x;
  vp2.y = P.y - p2.y;
  vp2.z = P.z - p2.z;

  C = crossProduct(edge2, vp2);

  if (dotProduct(N, C) < 0) { return P; } // P is on the right side; 

  return P; // this ray hits the triangle 
}

V3f vector_from_point(geometry_msgs::Point p)
{
  V3f v;
  v.x = p.x;
  v.y = p.y;
  v.z = p.z;
  return v;
}

geometry_msgs::Point point_from_vector(V3f v)
{
  geometry_msgs::Point p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

bool intersect(ray_trace::ray_trace::Request  &req,
         ray_trace::ray_trace::Response &res)
{
    std::vector <geometry_msgs::Point> intersections;
    std::vector <geometry_msgs::Point> rays = req.rays;
    std::vector <geometry_msgs::Point> origins = req.origins;
    std::vector <ray_trace::Triangle> triangles = req.triangles;

    int k = 0;
    for (int i=0; i<rays.size(); i++)
    {
      for (ray_trace::Triangle triangle : triangles)
      {
        V3f orig = vector_from_point(origins[i]);
        V3f dir = vector_from_point(rays[i]);
        V3f p1 = vector_from_point(triangle.p1);
        V3f p2 = vector_from_point(triangle.p2);
        V3f p3 = vector_from_point(triangle.p3);
        V3f intersection_point = rayTriangleIntersect(orig, dir, p1, p2, p3);
      }
    }
    res.intersections = intersections;
    return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ray_tracing_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ray_tracing_server", intersect);
  ROS_INFO("Ray tracing server ready");
  ros::spin();

  return 0;
}