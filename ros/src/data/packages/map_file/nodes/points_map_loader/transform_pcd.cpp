#include <iostream>
#include <ros/ros.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <pcl/common/transforms.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#define PI 3.141592652
double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f  tf_btol;
static Eigen::Matrix4f  tf_atob;


typedef struct
{
  double Lat;
  double Lon;
  double High;
}llh_InitTypeDef;

typedef struct
{
  double xx;
  double yy;
  double zz;
}xyz_InitTypeDef;

typedef struct
{
  double ex;
  double ey;
  double ez;
}enu_InitTypeDef;

typedef struct
{
  double n;
  double e;
  double d;
}ned_InitTypeDef;


llh_InitTypeDef llh0;
xyz_InitTypeDef xyz0;
xyz_InitTypeDef xyzA;
enu_InitTypeDef enuA;

xyz_InitTypeDef llh2xyz(double , double, double);
enu_InitTypeDef xyz2enu(xyz_InitTypeDef, xyz_InitTypeDef);

double _latitude, _longitude, _altitude;
std::string _input_filename;
std::string _output_filename;
std::string _path;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "transform_pcd");
  ros::NodeHandle n;
  ros::NodeHandle private_n("~");

  //set parameters
  private_n.getParam("latitude",_latitude);
  private_n.getParam("longitude",_longitude);
  private_n.getParam("altitude",_altitude);    
  private_n.getParam("roll", _tf_roll);
  private_n.getParam("pitch", _tf_pitch);
  private_n.getParam("yaw", _tf_yaw);

  private_n.getParam("input_filename",_input_filename);
  private_n.param("output_filename",_output_filename,std::string("/transformed_")+_input_filename);

  private_n.getParam("path",_path);
  llh0.Lat = 22.99665875;
  llh0.Lon = 120.222584889;
  llh0.High = 98.211;
  xyz0 = llh2xyz(22.99665875, 120.222584889, 98.211);//EE RTK base station
  xyzA = llh2xyz(_latitude, _longitude, _altitude);
 fprintf(stderr, "Input position: (%f, %f, %f)\n",_latitude,_longitude,_altitude);  
  enuA = xyz2enu(xyzA, xyz0);
 fprintf(stderr, "Base station: (22.99665875, 120.222584889, 98.211)\n");

 fprintf(stderr, "East: %f, North: %f, Up: %f\n",enuA.ex,enuA.ey,enuA.ez );

  _tf_roll = 0;
  _tf_pitch = 0;
  _tf_yaw = PI/2 -_tf_yaw;;

  _tf_x= enuA.ex;
  _tf_y = enuA.ey;
  _tf_z = enuA.ez;

  pcl::PointXYZI p;
  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

//load pcd file
  if (pcl::io::loadPCDFile (_path+_input_filename, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

//   for (size_t i = 0; i < cloud->points.size (); ++i)
//     std::cout << "    " << cloud->points[i].x
//               << " "    << cloud->points[i].y
//               << " "    << cloud->points[i].z
// << " "    << cloud->points[i].intensity << std::endl;


  for (size_t i = 0; i < cloud->points.size (); ++i)  {
    p.x = cloud->points[i].x;
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    p.intensity = cloud->points[i].intensity;
    scan.push_back(p);
    
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);//WEN20180421    

//teansform point cloud
  // pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
//save the new file
   pcl::io::savePCDFileASCII(_path + _output_filename, *transformed_scan_ptr);
   std::cout << "Saved " << transformed_scan_ptr->points.size() << " data points to " << _output_filename << "." << std::endl;

  return (0);
}



xyz_InitTypeDef llh2xyz(double L1, double L2, double H1)
{
   double a, b, e;
   double sinphi, cosphi, coslam, sinlam, tan2phi;
   double tmp, tmpden, tmp2;
   xyz_InitTypeDef xyzC;

   L1=(L1*PI)/180;
   L2=(L2*PI)/180;

   a = 6378137.0000;
   b = 6356752.3142;
   e = sqrt(1-(b/a)*(b/a));  

   sinphi = sin(L1);
   cosphi = cos(L1);
   coslam = cos(L2);
   sinlam = sin(L2);
   tan2phi = (tan(L1))*(tan(L1));
   tmp = 1 - e*e;
   tmpden = sqrt( 1 + tmp*tan2phi );

   xyzC.xx = (a*coslam)/tmpden + H1*coslam*cosphi;

   xyzC.yy = (a*sinlam)/tmpden + H1*sinlam*cosphi;

   tmp2 = sqrt(1 - e*e*sinphi*sinphi);
   xyzC.zz = (a*tmp*sinphi)/tmp2 + H1*sinphi;

   return (xyzC);
}


enu_InitTypeDef xyz2enu(xyz_InitTypeDef xe1, xyz_InitTypeDef xe2)
{
   enu_InitTypeDef enuC;
   double a, b, c;
   double phi, lam, sinphi, cosphi, sinlam, coslam;

   a=xe1.xx-xe2.xx;
   b=xe1.yy-xe2.yy;
   c=xe1.zz-xe2.zz;

   phi=(llh0.Lat*PI)/180;
   lam=(llh0.Lon*PI)/180;
   sinphi=sin(phi);
   cosphi=cos(phi);
   sinlam=sin(lam);
   coslam=cos(lam);

   enuC.ex=(-sinlam)*a+(coslam)*b+(0)*c;
   enuC.ey=(-sinphi*coslam)*a+(-sinphi*sinlam)*b+(cosphi)*c;
   enuC.ez=(cosphi*coslam)*a+(cosphi*sinlam)*b+(sinphi)*c;
   return (enuC);
}