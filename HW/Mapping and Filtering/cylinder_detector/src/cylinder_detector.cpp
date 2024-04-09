#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>

class DetectCyl {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;
        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        double tolerance;
        double tolerance_floor;
        int n_samples;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        pcl::PointCloud<pcl::PointXYZ> pc;

        double min_r_cylinder;
        double max_r_cylinder;

        visualization_msgs::MarkerArray marker_array;
        int marker_id;

    protected:
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, pc, listener_);

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
                x = pc[i].x;
                y = pc[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }

            // Here RANSAC on the floor
            //delete points which are on the ground plane
            n = pidx.size();
            std::vector<size_t> pidx_notfloor;
            size_t best_floor = 0;
            double X_floor[3] = {0,0,0};
            double a,b,c=0;
            double x_floor,y_floor,z_floor,M_floor;
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {

                //generate 3 numbers in [0,n-1]
                size_t p1 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                size_t p2 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                size_t p3 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);

                Eigen::Vector3f P1; P1 << pc[pidx[p1]].x, pc[pidx[p1]].y, pc[pidx[p1]].z;
                Eigen::Vector3f P2; P2 << pc[pidx[p2]].x, pc[pidx[p2]].y, pc[pidx[p2]].z; 
                Eigen::Vector3f P3; P3 << pc[pidx[p3]].x, pc[pidx[p3]].y, pc[pidx[p3]].z;

                int S_floor=0;
                Eigen::Vector3f ABC;
                ABC=P1.cross(P2).cross(P3);
                ABC=1/(sqrt(ABC(0)*ABC(0)+ABC(1)*ABC(1)+ABC(2)*ABC(2)))*ABC;

                for(unsigned int k=0;k<n;k++) {
                    x_floor = pc[pidx[k]].x;
                    y_floor = pc[pidx[k]].y;
                    z_floor = pc[pidx[k]].z;
                    M_floor=abs(z_floor-ABC(0)*x_floor-ABC(1)*y_floor-ABC(2));
                    if(M_floor<tolerance_floor) {
                        S_floor++;
                    }
                }

                if(S_floor>best_floor) {
                    best_floor=S_floor;

                    ABC=(P2-P1).cross(P3-P1);
                    double a0,b0,c0,d0;
                    a0=ABC(0);
                    b0=ABC(1);
                    c0=ABC(2);
                    d0=a0*P1(0)+b0*P1(1)+c0*P1(2);
                    a=-a0/c0;
                    b=-b0/c0;
                    c=d0/c0;
                }

            }
            // At the end, make sure to store the best plane estimate in X
            // X = {a,b,c}. This will be used for display
            X_floor[0]=a;
            X_floor[1]=b;
            X_floor[2]=c;

            for(unsigned int k=0;k<n;k++) {
                x_floor = pc[pidx[k]].x;
                y_floor = pc[pidx[k]].y;
                z_floor = pc[pidx[k]].z;
                M_floor=abs(z_floor-X_floor[0]*x_floor-X_floor[1]*y_floor-X_floor[2]);
                if(M_floor>tolerance_floor) {
                    pidx_notfloor.push_back(pidx[k]);
                }
            }

            listener_.waitForTransform("world",msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud("world",msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            n=pidx_notfloor.size();
            if(n==0) return;

            // //RANSAC for cylinder
            size_t best = 0;
            double X[3] = {0,0,0};
            a=0;
            b=0;
            double r_b=0;
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {
                // Implement RANSAC here.
                size_t p1 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                size_t p2 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                size_t p3 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);

                Eigen::Vector2f P1; P1 << lastpc_[pidx_notfloor[p1]].x, lastpc_[pidx_notfloor[p1]].y;
                Eigen::Vector2f P2; P2 << lastpc_[pidx_notfloor[p2]].x, lastpc_[pidx_notfloor[p2]].y;
                Eigen::Vector2f P3; P3 << lastpc_[pidx_notfloor[p3]].x, lastpc_[pidx_notfloor[p3]].y;

                //if one of the point is outside the map we generate another one
                if(abs(P1(0))>5.0 || abs(P1(1))>5.0 || abs(P2(0))>5.0 || abs(P2(1))>5.0 || abs(P3(0))>5.0 || abs(P3(1))>5.0) continue;
                // ROS_INFO("boucle");
                // ROS_INFO("P1.x = %f",P1(0));
                // ROS_INFO("P1.y = %f",P1(1));
                // ROS_INFO("P2.x = %f",P2(0));
                // ROS_INFO("P2.y = %f",P2(1));

                Eigen::Vector2f P12; P12 << lastpc_[pidx_notfloor[p2]].x - lastpc_[pidx_notfloor[p1]].x, lastpc_[pidx_notfloor[p2]].y - lastpc_[pidx_notfloor[p1]].y;

                double l = P12.norm();

                // ROS_INFO("l = %f",l);

                Eigen::Vector2f P3_uvframe; P3_uvframe << lastpc_[pidx_notfloor[p3]].x-lastpc_[pidx_notfloor[p1]].x, lastpc_[pidx_notfloor[p3]].y-lastpc_[pidx_notfloor[p1]].y;

                double xc,yc,r=0;
                xc = l/2;
                // ROS_INFO("xc = %f",xc);
                yc = (P3_uvframe(0)*P3_uvframe(0)-l*P3_uvframe(0)+P3_uvframe(1)*P3_uvframe(1))/(2*P3_uvframe(1));
                // ROS_INFO("yc = %f",yc);
                r = sqrt(xc*xc + yc*yc);
                // ROS_INFO("r = %f",r);

                double xc_world, yc_world;
                xc_world = P1(0)+xc;
                yc_world = P1(1)+yc;

                int S=0;
                double x,y,M;
                for(unsigned int k=0;k<n;k++) {
                    x = lastpc_[pidx_notfloor[k]].x;
                    y = lastpc_[pidx_notfloor[k]].y;
                    M = abs(r-sqrt((x-xc_world)*(x-xc_world)+(y-yc_world)*(y-yc_world)));
                    if(M<tolerance) {
                        S++;
                    }
                }
                //ROS_INFO("r = %f",r);
                if((S>best) && (S>900) && (r>min_r_cylinder) && (r<max_r_cylinder)) {
                    best=S;
                    a = xc_world;
                    b = yc_world;
                    r_b=r;
                }
            }
            X[0]=a ;
            X[1]=b;
            X[2]=r_b;

            // ROS_INFO("r_b = %f",r_b);

            // END OF TODO
            // ROS_INFO("Extracted floor plane: a = %.2f , b= %.2f",
            //         X[0],X[1]);

            visualization_msgs::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = "world";
            m.ns = "floor_plane";
            m.id = marker_id;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = X[0];
            m.pose.position.y = X[1];
            m.pose.position.z = 0;
            //tf::quaternionTFToMsg(Q,m.pose.orientation);
            m.scale.x = 2*r_b; //diameter in x direction
            m.scale.y = 2*r_b; //diameter in y direction
            m.scale.z = 2.0;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;

            bool add=true;
            visualization_msgs::Marker m_temp;

            for (int i=0;i<marker_array.markers.size();i++){
                m_temp=marker_array.markers[i];
                if(abs(m_temp.pose.position.x-X[0])<0.5 && abs(m_temp.pose.position.y-X[1])<0.5) {
                    marker_array.markers[i].pose.position.x=(m_temp.pose.position.x+X[0])/2;
                    m_temp.pose.position.y=(m_temp.pose.position.y+X[1])/2;
                    m_temp.scale.x=(m_temp.scale.x+2*r_b)/2;
                    m_temp.scale.y=(m_temp.scale.y+2*r_b)/2;
                    add=false;
                }
            }
            if(add) {
                marker_array.markers.push_back(m);
                marker_id++;
            }

            marker_pub_.publish(marker_array);
        }

    public:
        DetectCyl() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);
            nh_.param("tolerance_floor",tolerance_floor,1.0);

            nh_.param("min_r_cylinder",min_r_cylinder,0.04);
            nh_.param("max_r_cylinder",max_r_cylinder,0.2);

            ROS_INFO("Searching for cylinders");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            marker_id=1;

            // Make sure TF is ready
            ros::Duration(0.5).sleep();
            scan_sub_ = nh_.subscribe("scans",1,&DetectCyl::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cylinder_arr",1);
        }
};

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"cylinder_detector");
    DetectCyl fp;
    ros::spin();
    return 0;
}
