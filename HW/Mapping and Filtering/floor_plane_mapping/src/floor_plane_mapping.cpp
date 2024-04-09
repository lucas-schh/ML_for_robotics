#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <Eigen/Core>
#include <map>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>

typedef std::pair<int,int> Coord;
typedef std::list<pcl::PointXYZ> PointList;
typedef std::map<Coord, PointList> ListMatrix;

class FloorPlaneMap {

    protected:

      ros::NodeHandle nh_;
      tf::TransformListener listener_;
      std::string base_frame_;
      double max_range_;
      pcl::PointCloud<pcl::PointXYZ> pc;
      pcl::PointCloud<pcl::PointXYZ> pc_robot;
      double width_map,height_map;
      int width,height;
      double resolution;
      ListMatrix M;
      double alpha;

      cv::Mat_<double> D_table;

      // OccupancyGrid
      nav_msgs::OccupancyGrid og;

      ros::Subscriber scan_sub_;
      ros::Publisher og_pub;

    protected:

      void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg){
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::fromROSMsg(*msg, temp);
        // Make sure the point cloud is in the base-frame
        listener_.waitForTransform("bubbleRob",msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
        pcl_ros::transformPointCloud("bubbleRob",msg->header.stamp, temp, msg->header.frame_id, pc_robot, listener_);


        unsigned int n = temp.size();
        std::vector<size_t> pidx;
        // First count the useful points
        for (unsigned int i=0;i<n;i++) {
            float x = temp[i].x;
            float y = temp[i].y;
            float d = hypot(x,y);

            if (d < 1e-2) {
                // Bogus point, ignore
                continue;
            }
            x = pc_robot[i].x;
            y = pc_robot[i].y;
            d = hypot(x,y);
            if (d > max_range_) {
                // too far, ignore
                continue;
            }
            pidx.push_back(i);
        }


        listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
        pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, pc, listener_);

        n = pidx.size();

        og.header.stamp = ros::Time::now();
        og.header.frame_id = "world";
        og.info.resolution = resolution;
        og.info.width = width;
        og.info.height = height;
        og.info.origin.position.x = -width_map/2;
        og.info.origin.position.y = -height_map/2;
        og.info.origin.position.z = 0;
        og.info.origin.orientation.w = 1;
        og.info.origin.orientation.x = 0;
        og.info.origin.orientation.y = 0;
        og.info.origin.orientation.z = 0;

        for (unsigned int i=0;i<n;i++) {
            float x = pc[pidx[i]].x;
            float y = pc[pidx[i]].y;
            Coord coord(int((x + width_map/2)*(width/10)), int((y + height_map/2)*(height/10)));
            M[coord].push_back(pc[pidx[i]]);
        }

        tf::StampedTransform transform;
        listener_.waitForTransform("world",msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
        listener_.lookupTransform("/world", "/bubbleRob",ros::Time(0), transform);

        for (ListMatrix::const_iterator it=M.begin();it!=M.end();it++) {
          // for each point and its corresponding pointlist,
          Coord coord = it->first;
          const PointList & pl = it->second; // pl : pointlist;

          // ROS_INFO("pos robot = %f, %f",transform.getOrigin().x(), transform.getOrigin().y());
          // ROS_INFO("pos coord = %d, %d",coord.first, coord.second);
          // ROS_INFO("diff coord = %f, %f",transform.getOrigin().x()-coord.first, transform.getOrigin().y()-coord.second);
          double d=sqrt((transform.getOrigin().x()-pl.begin()->x)*(transform.getOrigin().x()-pl.begin()->x) + (transform.getOrigin().y()-pl.begin()->y)*(transform.getOrigin().y()-pl.begin()->y));
          D_table(0,0)=0.5+0.3*exp(-d/alpha);
          D_table(1,1)=0.5+0.3*exp(-d/alpha);
          D_table(0,1)=0.5-0.3*exp(-d/alpha);
          D_table(1,0)=0.5-0.3*exp(-d/alpha);




          Eigen::MatrixXf A(pl.size(),3);
          Eigen::MatrixXf B(pl.size(),1);
          int count=0;
          int D = 1;

          for (PointList::const_iterator jt=pl.begin();jt!=pl.end();jt++) {
              // for each pointlist of point (coord)
              float x = jt->x;
              float y = jt->y;
              float z = jt->z;

              A(count,0) = x;
              A(count,1) = y;
              A(count,2) = 1;

              B(count,0) = z;
              count++;
          }
          Eigen::MatrixXf X = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
          Eigen::Vector3f NP; NP << X(0),X(1),-1;
          Eigen::Vector3f NQ; NQ << 0,0,-1; //floor_plane
          if(NP.norm()<=0) return;
          double angle = acos(abs(NP.dot(NQ))/(NP.norm()*NQ.norm()))*180/M_PI;
          if(abs(angle)>30) D=0;

          int i = coord.first;
          int j = coord.second;
          if ((i < 0) || (j<0) || (i>=width) || (j>=height)) continue;

          double num=(D_table(D,1)*(double(og.data[i+j*height])/100.0));
          double den=(D_table(D,1)*(double(og.data[i+j*height])/100.0)+D_table(D,0)*(1-(double(og.data[i+j*height])/100.0)));
          double P_T_D=num/den;
          // ROS_INFO("P_T_D = %f", P_T_D);

          og.data[i+j*height] = P_T_D*100;


        }
        og_pub.publish(og);
      }

    public:
      FloorPlaneMap() : nh_("~") {
      nh_.param("width",width,10);
      nh_.param("height",height,10);
      nh_.param("width_map",width_map,10.0);
      nh_.param("height_map",height_map,10.0);
      nh_.param("resolution",resolution,1.0);
      nh_.param("base_frame",base_frame_,std::string("/body"));
      nh_.param("max_range",max_range_,5.0);
      nh_.param("alpha",alpha,1.0);
      og.data.assign(width*height,50);

      D_table = cv::Mat_<double>(2,2);


      ros::Duration(0.5).sleep();

      og_pub = nh_.advertise<nav_msgs::OccupancyGrid>("image",1);
      scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneMap::pc_callback,this);

      }
};

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"floor_plane_Map");
    FloorPlaneMap fp;
    ros::spin();
    return 0;
}
