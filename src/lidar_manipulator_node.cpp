#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h> // pcl_ros 추가

class LidarManipulator {
public:
    LidarManipulator() {
        accumulated_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("accumulated_pointcloud", 1);
        visualization_pub = nh.advertise<visualization_msgs::MarkerArray>("joint_visualization", 1);
        scan_sub = nh.subscribe("scan", 1, &LidarManipulator::scanCallback, this);
        joint_state_sub = nh.subscribe("/joint_states", 1, &LidarManipulator::jointStateCallback, this);

        dh_parameters <<    0.0,    0.0,    0.0,     // 1.5078 rad = 90도
                            0.0,  1.507,    0.0,     //a, alpha, d 순서 !!!
                           0.13,    0.0,    0.0,     //잘못 입력되었던 DH 파라미터 수정 (우선 그리퍼값 부여)
                          0.126,    0.0,    0.0,
                          0.124, -1.507,    0.0; 

        joint_positions.resize(5); // 4개의 관절에 대한 현재 위치 벡터
        joint_positions.setZero();

        accumulated_cloud->width = 0;
        accumulated_cloud->height = 1;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (!std::isinf(range)) {
                float angle = scan->angle_min + i * scan->angle_increment;
                pcl::PointXYZ point;
                point.x = -range * cos(angle);
                point.y = -range * sin(angle);
                point.z = 0;
                cloud_lidar->points.push_back(point);
            }
        }

        calculateTransforms(); // 변환 행렬 계산

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_lidar, *cloud_world, end_effector_T); // pcl_transform으로 변환행렬 적용

        *accumulated_cloud += *cloud_world;   

        sensor_msgs::PointCloud2 accumulated_cloud_msg;
        pcl::toROSMsg(*accumulated_cloud, accumulated_cloud_msg);
        accumulated_cloud_msg.header.frame_id = "map";
        accumulated_cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(accumulated_cloud_msg);
    }


    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {    
        for (int i = 0; i < joint_state->position.size(); ++i) {
            if (joint_state->name[i] == "joint1")
                joint_positions(0) = joint_state->position[i];
            else if (joint_state->name[i] == "joint2")
                joint_positions(1) = -joint_state->position[i] + 1.379; //theta2 (-) 부호 적용 & +79도 보정
            else if (joint_state->name[i] == "joint3")
                joint_positions(2) = -joint_state->position[i] - 1.379; //theta3 (-) 부호 적용 & -79도 보정
            else if (joint_state->name[i] == "joint4")
                joint_positions(3) = -joint_state->position[i];         //theta4 (-) 부호 적용

            joint_positions(4) = 0;
        }
    
        // 조인트값을 잘 받았는지 확인용 출력문 추가
        std::cout << "Joint positions: " << joint_positions.transpose() << std::endl;
    }

    void calculateTransforms() {     
        ROS_INFO("calculateTransforms() method called");  // 디버깅용 출력문      
        Eigen::Matrix4f T01, T02, T03, T04, T12, T23, T34, T45;
        T01 = T02 = T03 = T04 = T12 = T23 = T34 = T45 = Eigen::Matrix4f::Identity(); // 각 관절의 변환 행렬 초기화

        for (int i = 0; i < 5; ++i) {
                       
            float a = dh_parameters(i, 0);
            float alpha = dh_parameters(i, 1);
            float d = dh_parameters(i, 2);
            float theta = joint_positions(i);
          
            Eigen::Matrix4f T;
            T <<             cos(theta),              -sin(theta),           0,               a,
                sin(theta) * cos(alpha),  cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d,
                sin(theta) * sin(alpha),  cos(theta) * sin(alpha),  cos(alpha),  cos(alpha) * d,
                                      0,                        0,           0,              1;

            if (i == 0) {
                T01 = T;
            } else if (i == 1) {
                T12 = T;
            } else if (i == 2) {
                T23 = T;
            } else if (i == 3) {
                T34 = T;
            } else if (i == 4) {
                T45 = T;
            }            
        }
        
        T02 = T01 * T12;             // Joint 2까지의 변환 행렬
        T03 = T01 * T12 * T23;       // Joint 3까지의 변환 행렬
        T04 = T01 * T12 * T23 * T34; // Joint 4가지의 변환 행렬
        end_effector_T = T01 * T12 * T23 * T34 * T45; // 모든 관절의 변환 행렬을 곱하여 최종 엔드 이펙터의 변환 행렬을 얻음
        end_effector_T(2, 3) = end_effector_T(2, 3) + (0.077 + 0.09); //DH parameter로 구현되지 못한 z축 0.077 보정 + 라이다 높이 보정 (0.09)

        ROS_INFO_STREAM("T01 transformation matrix:\n" << T01); // 디버깅용 출력문 T01
        ROS_INFO_STREAM("T02 transformation matrix:\n" << T02); // 디버깅용 출력문 T02
        ROS_INFO_STREAM("T03 transformation matrix:\n" << T03); // 디버깅용 출력문 T03
        ROS_INFO_STREAM("End effector transformation matrix:\n" << end_effector_T); // 디버깅용 출력문 end_effector_T
        
    }

    void visualizeTransforms() {
        visualization_msgs::MarkerArray marker_array;
       
        publishTransformMarker(T01, "T01", 0.0, 1.0, 0.0, marker_array);//초록색      
        publishTransformMarker(T02, "T02", 0.0, 0.0, 1.0, marker_array);//파란색       
        publishTransformMarker(T03, "T03", 1.0, 0.0, 1.0, marker_array);//보라색        
        publishTransformMarker(end_effector_T, "end_effector_T", 1.0, 1.0, 1.0, marker_array);//흰색

        visualization_pub.publish(marker_array);
    }

        void publishTransformMarker(const Eigen::Matrix4f& transform, const std::string& marker_ns, float r, float g, float b, visualization_msgs::MarkerArray& marker_array) {
        Eigen::Vector3f position = transform.block<3, 1>(0, 3); // 병진성분t 추출 
        Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0); // 회전성분R 추출
        Eigen::Quaternionf quaternion(rotation); // 회전성분을 쿼터니언으로 변환하기 위한 행렬 선언

        // Normalize the quaternion to ensure valid values
        quaternion.normalize(); // 정규화

        tf2::Quaternion tf_quaternion;
        tf_quaternion.setValue(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()); // 쿼터니언 변환

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = marker_ns;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = position(0);
        marker.pose.position.y = position(1); //Translation값 적용
        marker.pose.position.z = position(2);
        tf2::convert(tf_quaternion, marker.pose.orientation);  // Rotation값 적용 (쿼터니언)
        marker.scale.x = 1.0;  
        marker.scale.y = 0.1;   //화살표 크기 세팅
        marker.scale.z = 0.1;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker_array.markers.push_back(marker);
    }



private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber joint_state_sub;
    ros::Publisher cloud_pub;
    ros::Publisher visualization_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
    Eigen::Matrix<float, 5, 3> dh_parameters;
    Eigen::VectorXf joint_positions;
    Eigen::Matrix4f end_effector_T;
    Eigen::Matrix4f T01;
    Eigen::Matrix4f T02;
    Eigen::Matrix4f T03;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_manipulator_node");

    LidarManipulator manipulator;

    ros::Rate loop_rate(10); // 속도 조절
    while (ros::ok()) {
        manipulator.visualizeTransforms();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
