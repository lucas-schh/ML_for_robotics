#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>


class FloorPlaneRansac {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        double tolerance;
        int n_samples; 

        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            //
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
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }
            
            //
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c
            // Remember to use the n_samples and the tolerance variable
            n = pidx.size();
            size_t best = 0;
            double X[3] = {0,0,0};
            double a,b,c=0;
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {
                // Implement RANSAC here. Useful commands:
                // Select a random number in in [0,i-1]
                // size_t a = std::min((rand() / (double)RAND_MAX) * i,(double)i-1);
                //generate 3 numbers in [0,n-1]
                size_t p1 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                size_t p2 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                size_t p3 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                // // Create a 3D point:
                // // Eigen::Vector3f P; P << x,y,z;
                Eigen::Vector3f P1; P1 << lastpc_[pidx[p1]].x, lastpc_[pidx[p1]].y, lastpc_[pidx[p1]].z;
                Eigen::Vector3f P2; P2 << lastpc_[pidx[p2]].x, lastpc_[pidx[p2]].y, lastpc_[pidx[p2]].z; 
                Eigen::Vector3f P3; P3 << lastpc_[pidx[p3]].x, lastpc_[pidx[p3]].y, lastpc_[pidx[p3]].z;

                int S=0;
                Eigen::Vector3f P1_temp;
                Eigen::Vector3f P2_temp;
                Eigen::Vector3f P3_temp;
                Eigen::Vector3f ABC;
                double x,y,z,M;

                for(unsigned int k=0;k<n;k++) {
                    x = lastpc_[pidx[k]].x;
                    y = lastpc_[pidx[k]].y;
                    z = lastpc_[pidx[k]].z;
                    P1_temp << P1(0),P1(1),P1(2);
                    P2_temp << P2(0),P2(1),P2(2);
                    P3_temp << P3(0),P3(1),P3(2);
                    ABC=P1_temp.cross(P2_temp).cross(P3_temp);
                    ABC=1/(sqrt(ABC(0)*ABC(0)+ABC(1)*ABC(1)+ABC(2)*ABC(2)))*ABC;
                    M=abs(z-ABC(0)*x-ABC(1)*y-ABC(2));
                    if(M<tolerance) {
                        S++;
                    }
                }

                if(S>best) {
                    best=S;
                    // ROS_INFO("%d = best",best);

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
                // Dot product
                // double x = P.dot(P);
                // Cross product
                // Eigen::Vector3f Q = P.cross(P);
                // Vector norm
                // double norm = P.norm();

            }
            // At the end, make sure to store the best plane estimate in X
            // X = {a,b,c}. This will be used for display
            X[0]=a;
            X[1]=b;
            X[2]=c;

            // END OF TODO
            ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X[0],X[1],X[2]);

            Eigen::Vector3f O,u,v,w;
            w << X[0], X[1], -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf::Quaternion Q;
            R.getRotation(Q);
            
            visualization_msgs::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = base_frame_;
            m.ns = "floor_plane";
            m.id = 1;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = O(0);
            m.pose.position.y = O(1);
            m.pose.position.z = O(2);
            tf::quaternionTFToMsg(Q,m.pose.orientation);
            m.scale.x = 1.0;
            m.scale.y = 1.0;
            m.scale.z = 0.01;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;

            marker_pub_.publish(m);
            
        }

    public:
        FloorPlaneRansac() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRansac::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Ransac");
    FloorPlaneRansac fp;

    ros::spin();
    return 0;
}


