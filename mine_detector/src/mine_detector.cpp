#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>

class DetectMine {
    protected:
        ros::Publisher marker_pub_;
        ros::Subscriber metal_sub_;
        tf::TransformListener listener_;
        ros::NodeHandle nh_;
        std::string base_frame_;
        float r_cyl;

        visualization_msgs::MarkerArray marker_array;
        int marker_id;

    protected:
        void metal_callback(const std_msgs::Float32::ConstPtr& msg){
            tf::StampedTransform transform;
            listener_.waitForTransform(base_frame_,"/VSV/Tool",ros::Time(0),ros::Duration(1.0));
            listener_.lookupTransform(base_frame_, "/VSV/Tool",ros::Time(0), transform);

            if(msg->data==1){
              visualization_msgs::Marker m;
              m.header.stamp = ros::Time::now();
              m.header.frame_id = "world";
              m.ns = "mine_detector";
              m.id = marker_id;
              m.type = visualization_msgs::Marker::CYLINDER;
              m.action = visualization_msgs::Marker::ADD;
              m.pose.position.x = transform.getOrigin().x();
              m.pose.position.y = transform.getOrigin().y();
              m.pose.position.z = 0;
              //tf::quaternionTFToMsg(Q,m.pose.orientation);
              m.scale.x = 2*r_cyl; //diameter in x direction
              m.scale.y = 2*r_cyl; //diameter in y direction
              m.scale.z = 2.0;
              m.color.a = 0.5;
              m.color.r = 1.0;
              m.color.g = 0.0;
              m.color.b = 1.0;

              bool add=true;
              visualization_msgs::Marker m_temp;

              for (int i=0;i<marker_array.markers.size();i++){
                  m_temp=marker_array.markers[i];
                  if(abs(m_temp.pose.position.x-m.pose.position.x)<(2*r_cyl) && abs(m_temp.pose.position.y-m.pose.position.y)<(2*r_cyl)) {
                      marker_array.markers[i].pose.position.x=(m_temp.pose.position.x+m.pose.position.x)/2;
                      m_temp.pose.position.y=(m_temp.pose.position.y+m.pose.position.y)/2;
                      add=false;
                  }
              }
              if(add) {
                  marker_array.markers.push_back(m);
                  marker_id++;
              }
            }
            marker_pub_.publish(marker_array);
        }

    public:
        DetectMine() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            ROS_INFO("Searching for mines");

            marker_id=1;
            r_cyl=1;

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            metal_sub_ = nh_.subscribe("/vrep/metalDetector",1,&DetectMine::metal_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("mine_array",1);
        }
};

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"mine_detector");
    DetectMine fp;
    ros::spin();
    return 0;
}
