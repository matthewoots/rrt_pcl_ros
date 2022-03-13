/*
 * rrt.cpp
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */
#include "rrt_standalone.h"

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include "rrt_pcl_ros/point_array.h"
#include <std_msgs/Float32MultiArray.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

#define check_size 3
#define param_size 10

using namespace std;
// using namespace rrt_helper;

std::vector<Vector3d> rrt_path;
double _step_size, _obs_threshold, _xybuffer, _zbuffer, _min_height, _max_height, _passage_size, _timeout, _scale_z;
int _max_tries, _number_of_runs;

bool message_check[check_size] = {false, false, false};
bool _is_using_formation, _is_circle_formation;
double _circle_radius, _runtime_error;
vector<double> formation_param, solo_param;

double yaw;
Vector3d translation;
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc1(new pcl::PointCloud<pcl::PointXYZ>);

std::vector<Vector3d> bspline;

vector<bool> runtime_success;
vector<double> runtime_average;
double final_run_time;

class simple_node
{
private:

    ros::NodeHandle _nh;
    ros::Publisher rrt_pub;
    ros::Subscriber pcl_sub, formation_msg_sub, solo_msg_sub, pcl2_msg_sub;

    // For debug
    ros::Publisher altered_pcl_pub, bs_pub;
    

public:
    sensor_msgs::PointCloud2 pcl_pc2;


    simple_node(ros::NodeHandle &nodeHandle)
    {
        /* ------------ Subscribe from swarm param ------------ */
        /** 
        * @brief Handles formation parameters from float64 array
        */
        formation_msg_sub = _nh.subscribe<std_msgs::Float32MultiArray>(
            "/param/formation_settings", 1, &simple_node::formationParamMsgCallBack, this);
        /** 
        * @brief Handles solo parameters from float64 array
        */
        solo_msg_sub = _nh.subscribe<std_msgs::Float32MultiArray>(
            "/param/solo_settings", 1, &simple_node::soloParamMsgCallBack, this);
        pcl2_msg_sub = _nh.subscribe<sensor_msgs::PointCloud2>(
            "/param/pcl", 1,  boost::bind(&simple_node::pcl2MsgCallBack, this, _1));

        /** 
        * @brief Publisher of rrt points
        */
        rrt_pub = _nh.advertise<rrt_pcl_ros::point_array>("/rrt", 10);

        /** For debug */
        altered_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/query_pcl", 10);
        bs_pub = _nh.advertise<rrt_pcl_ros::point_array>("/bs", 10);

        printf("%s[rrt.cpp] Constructor Setup Ready! \n", KGRN);
    }
    ~simple_node(){};

    void path_message_wrapper_publisher()
    {
        rrt_pcl_ros::point_array msg; geometry_msgs::Point data;

        int v_size = rrt_path.size();
        for (int i = 0; i < v_size; i++)
        {
            data.x = rrt_path[i].x(); data.y = rrt_path[i].y(); data.z = rrt_path[i].z();
            msg.array.push_back(data);
        }

        rrt_pub.publish(msg);
    }

    void pcl2MsgCallBack(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl_pc2 = *msg;
        message_check[0] = true;
        printf("%s[rrt.cpp] PCL Param Msg received! \n", KGRN);
    }
    /** @brief Handles formation parameters from float64 array */
    void formationParamMsgCallBack(const  std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        std_msgs::Float32MultiArray multi_array = *msg;
        
        formation_param.clear();
        for (int i = 0; i < multi_array.data.size(); i++)
        {
            formation_param.push_back((double)multi_array.data[i]);
        }
        message_check[1] = true;
        printf("%s[rrt.cpp] RRT Formation Param Msg received! \n", KGRN);
    }

    /** @brief Handles solo parameters from float64 array */
    void soloParamMsgCallBack(const  std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        std_msgs::Float32MultiArray multi_array = *msg;
        
        solo_param.clear();
        for (int i = 0; i < multi_array.data.size(); i++)
        {
            solo_param.push_back((double)multi_array.data[i]);
        }
        message_check[2] = true;
        printf("%s[rrt.cpp] RRT Solo Param Msg received! \n", KGRN);
    }

    void query_pcl_publisher(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc,
        double _yaw, Vector3d _translation)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*transformed_pc, msg);
        sensor_msgs::PointCloud2 tmp_pc = transform_sensor_cloud(msg,
            - Vector3d(0, 0, - _yaw), Vector3d(0, 0, 0));
        msg = transform_sensor_cloud(tmp_pc,
            - Vector3d(0, 0, 0), _translation);

        altered_pcl_pub.publish(msg);
    }

    void bspline_publisher()
    {
        rrt_pcl_ros::point_array msg; 

        for (int i = 0; i < bspline.size(); i++)
        {
            geometry_msgs::Point data;
            data.x = bspline[i].x(); data.y = bspline[i].y(); data.z = bspline[i].z();
            msg.array.push_back(data);
        }

        printf("%s[main.cpp] Bspline message converted %s\n", KGRN, KNRM);

        bs_pub.publish(msg);
    }
};

bool initialize_rrt_params(vector<double> params)
{
    if (params.size() != param_size)
        return false;

    _step_size = params[0];
    _obs_threshold = params[1];
    _xybuffer = params[2];
    _zbuffer = params[3];
    _passage_size = params[4];

    _min_height = params[5];
    _max_height = params[6];
    _max_tries = params[7];
    _timeout = params[8];
    _scale_z = params[9];

    return true;
}


bool run_rrt(sensor_msgs::PointCloud2 pcl_pc, 
    Vector3d start, Vector3d end, vector<VectorXd> no_fly_zone)
{
    final_run_time = -1.0;
    double start_time = ros::Time::now().toSec();
    rrt_path.clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_pcl_pc = 
        pcl2_converter(pcl_pc);

    printf("%s[rrt.cpp] Start (%lf %lf %lf) End (%lf %lf %lf) \n", 
        KBLU, start.x(), start.y(), start.z(),
        end.x(), end.y(), end.z());

    // Find the origin of the transformed frame
    Vector3d _origin;
    _origin.x() = (start.x() + end.x()) / 2;
    _origin.y() = (start.y() + end.y()) / 2;
    _origin.z() = (start.z() + end.z()) / 2;
    
    // Do the preparation for transformation
    // Find the translation vector and the yaw angle
    Vector3d tmp_vect = end - start;
    yaw = atan2(tmp_vect.y(), tmp_vect.x()) / 3.1415926535 * 180;
    Vector3d rotation = Vector3d(0,0,yaw);

    translation = Vector3d(_origin.x(), _origin.y(), 0);       
    
    printf("%s[rrt.cpp] translation vector [%lf %lf %lf] yaw %lf\n", KBLU, 
        translation.x(), translation.y(), translation.z(), yaw);

    Vector3d transformed_translation = rotate_vector(rotation, translation);

    // We align everthing to the rotated x axis
    // So now we are playing in X and Z axis
    // Translate then rotate to temporary frame for start and end points
    // geometry_msgs::Point transformed_start = transform_point(
    //     vector_to_point(start), -rotation, transformed_translation);
    // geometry_msgs::Point transformed_end = transform_point(
    //     vector_to_point(end), -rotation, transformed_translation);

    geometry_msgs::Point transformed_start = forward_transform_point(
        vector_to_point(start), rotation, translation);
    geometry_msgs::Point transformed_end = forward_transform_point(
        vector_to_point(end), rotation, translation);

    Vector3d t_start = point_to_vector(transformed_start);
    Vector3d t_end = point_to_vector(transformed_end);

    printf("%s[rrt.cpp] transformed start (%lf %lf %lf) transformed end (%lf %lf %lf) \n", 
        KBLU, t_start.x(), t_start.y(), t_start.z(),
        t_end.x(), t_end.y(), t_end.z());

    // We find the original pcl in the transformed frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl_pc =
        base_to_transform_pcl(original_pcl_pc, 
        Vector3d(0, 0, yaw), translation);

    // Map size and origin should be determined and isolated 
    // Find a way to rotate the boundary so that we can minimize the space
    Vector3d _map_size;
    _map_size.x() = abs(t_start.x() - t_end.x()) + _xybuffer;
    _map_size.y() = abs(t_start.y() - t_end.y()) + _xybuffer + _passage_size;
    _map_size.z() = abs(t_start.z() - t_end.z()) + _zbuffer;

    printf("%s[rrt.cpp] map_size start (%lf %lf %lf)\n", 
        KBLU, _map_size.x(), _map_size.y(), _map_size.z());

    // We can crop the pointcloud to the dimensions that we are using
    // Origin will already to (0,0,0)
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped_pc1 = pcl2_filter(
        transformed_pcl_pc, Vector3d(0,0,_origin.z()), _map_size);

    // *** For DEBUG ***
    transformed_pc1->points.clear();
    transformed_pc1 = transformed_cropped_pc1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped_pc = 
        pcl_z_scale(transformed_cropped_pc1, _scale_z);

    // map size is affected too
    // _min_height and _max_height is different too

    // *** Added in transform ***
    Vector3d map_size = Vector3d(_map_size.x(), _map_size.y(), _map_size.z() * _scale_z);
    
    double min_height = _min_height * _scale_z;
    double max_height = _max_height * _scale_z;

    double t_z = _origin.z() * _scale_z;

    double step_size = _step_size + _scale_z / 2.0;

    Vector3d t_t_start = Vector3d(t_start.x(), t_start.y(), t_start.z() *_scale_z);
    Vector3d t_t_end = Vector3d(t_end.x(), t_end.y(), t_end.z() *_scale_z);
    // *** End of adding in transform ***
    
    // Set line division to floor of step_size
    // Size must be 3 or more for linspace to work
    int line_search_division = max(4,(int)floor(step_size)); 

    size_t num_points = original_pcl_pc->size();
    int total = static_cast<int>(num_points);
    printf("%s[rrt.cpp] Actual obstacle size %d! \n", KGRN, total);

    // Run RRT 
    rrt_node rrt;
    rrt.error = true;
    int rrt_tries = 0;

    double prev = ros::Time::now().toSec();
    
    while (ros::Time::now().toSec() - prev < _runtime_error && rrt.process_status())
    {
        // Use for transformed start and end in transformed frame
        rrt.initialize(t_t_start, t_t_end, transformed_cropped_pc,
            map_size, Vector3d(0,0,t_z),
            step_size, _obs_threshold,
            min_height, max_height,
            line_search_division, _timeout, no_fly_zone,
            rotation, translation);
        rrt.run();
        rrt_tries++;
    }

    if (rrt_tries > _max_tries)
    {
        printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
        return false;
    }

    if (rrt.process_status())
    {
        printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
        return false;
    }

    printf("%s[rrt.cpp] rrt_size %d! %s\n", KGRN, (rrt.path_extraction()).size(), KNRM);


    // Extract path from transformed frame into normal frame
    // Remember to factor in z scale back
    std::vector<Vector3d> transformed_path1 = rrt.path_extraction();
    std::vector<Vector3d> transformed_path;

    // Transformed path needs to scale back the z
    for (int j = 0; j < transformed_path1.size(); j++)
    {
        Vector3d tmp_path = Vector3d(transformed_path1[j].x(), 
            transformed_path1[j].y(), transformed_path1[j].z() / _scale_z);

        transformed_path.push_back(tmp_path);
    }

    std::vector<Vector3d> path;

    // Vector3d transformed_translation_to_original = rotate_vector(
    //     -rotation, -translation);
    for (int j = 0; j < transformed_path.size(); j++)
    {
        Vector3d tmp_path = transformed_path[j];
        
        // geometry_msgs::Point n_tmp_path = transform_point(
        //     vector_to_point(tmp_path),
        //     -rotation, transformed_translation_to_original);
        geometry_msgs::Point n_tmp_path = backward_transform_point(
            vector_to_point(tmp_path), rotation, translation);


        path.push_back(point_to_vector(n_tmp_path));
    }

    // Somehow the algorithm will miss the start data
    path.push_back(start);

    // Now the whole path is flipped, hence we need to flip it back in bspline

    rrt_path = path;

    final_run_time = ros::Time::now().toSec() - start_time;
    
    // create_bspline(_bs_order, end, start);
    return true;
}

void rrt_bspline(int _order)
{
    bspline.clear();

    int v_size = rrt_path.size();
    MatrixXd wp = MatrixXd::Zero(3,v_size);
    
    // We need to flip it back since RRT path is inverted 
    for (int i = rrt_path.size()-1; i >= 0; i--)
    {
        wp(0,rrt_path.size()-1 - i) = rrt_path[i].x(); 
        wp(1,rrt_path.size()-1 - i) = rrt_path[i].y(); 
        wp(2,rrt_path.size()-1 - i) = rrt_path[i].z();
    }

    // Somehow the algorithm will miss the start data
    // wp(0,v_size-1) = s.x(); 
    // wp(1,v_size-1) = s.y(); 
    // wp(2,v_size-1) = s.z();

    // // Somehow the algorithm will miss the start data
    // // Since the nodes are counting backwards, hence the start point is the end point
    // Vector3d end_pose = e;

    // Start position will be at rrt_path.size()-1
    MatrixXd global_cp = setClampedPath(wp, 
    2, 4, _order, rrt_path[rrt_path.size()-1]);
    VectorXd knots = setKnotsPath(global_cp, 1, _order);
    std::vector<Vector3d> bs_tmp = updateFullPath(global_cp, 
        1, _order, knots);

    // Since the nodes are flipped we have to flip them back first
    // Change back the order
    for (int i = 0; i < bs_tmp.size(); i++)
        bspline.push_back(bs_tmp[i]);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_simple_node");
    ros::NodeHandle _nh("~"); 

    simple_node simple_node(_nh);
    
    // ROS Params
    _nh.param<bool>("is_using_formation", _is_using_formation, false); 
    _nh.param<bool>("is_circle_formation", _is_circle_formation, false); 
    _nh.param<double>("runtime_error", _runtime_error, 10.0); 
    _nh.param<double>("circle_radius", _circle_radius, 1.0); 
    _nh.param<int>("number_of_runs", _number_of_runs, 10); 

    std::vector<double> start_list, end_list;
    Vector3d start, end;
    double sum = 0;
    _nh.getParam("start_position", start_list);
    _nh.getParam("end_position", end_list);
    for(unsigned i = 0; i < start_list.size(); i++) 
    {
      start((int)i) = start_list[(int)i];
      end((int)i) = end_list[(int)i];
    }

    printf("%s[rrt.cpp] start and end positions [%lf %lf %lf] [%lf %lf %lf]\n", KGRN, 
        start.x(), start.y(), start.z(), end.x(), end.y(), end.z());
    
    int rrt_run_count = 0;
    
    while (ros::ok())
    {
        int status_pass = 0;
        for (int i = 0; i < check_size; i++)
        {
            if (message_check[i])
                status_pass++;
        }
        
        if (status_pass == check_size)
        {
            // x_min, x_max, y_min, y_max
            vector<VectorXd> no_fly_zone;
            

            vector<double> rrt_params;
            if (_is_using_formation)
                rrt_params = formation_param;
            else
                rrt_params = solo_param;
            initialize_rrt_params(rrt_params);

            if (run_rrt(simple_node.pcl_pc2, start, end, no_fly_zone))
            {
                rrt_bspline(3);
                printf("%s[rrt.cpp] RRT Succeeded\n", KGRN);

                // For Debug
                simple_node.path_message_wrapper_publisher();
                simple_node.query_pcl_publisher(transformed_pc1, yaw, translation);
                simple_node.bspline_publisher();

                // Log down time if successful
                runtime_success.push_back(true);
            }
            else
                runtime_success.push_back(false);

            runtime_average.push_back(final_run_time);

            rrt_run_count++;
        }
        ros::spinOnce();

        if (rrt_run_count == _number_of_runs)
        {
            int passes = 0;
            double avg_runtime = 0.0;
            for (int i = 0; i < _number_of_runs; i++)
            {
                if (runtime_success[i])
                {
                    passes++;
                    avg_runtime += runtime_average[i];
                }
            }
            avg_runtime = avg_runtime / passes;

            printf("%s------------- Results ------------- \n", KYEL);
            printf("%s  %d runs [success %d / %d]\n", KCYN, _number_of_runs, passes, _number_of_runs);
            printf("%s  Average runtime %lf \n", KCYN, avg_runtime);

            return 0;
        }
        // ros::Duration(1.5).sleep();
    }

    return 0;
}

/** @brief Competition waypoints */
// x_min, x_max, y_min, y_max
// vector<VectorXd> no_fly_zone;

// Wall
// Vector3d start = Vector3d(0,0,1.5);
// Vector3d end = Vector3d(13,0,1.5);

// Pole
// Vector3d start = Vector3d(13,-2.5,1.5);
// Vector3d end = Vector3d(27.5,-2.5,1.5);

// VectorXd a(4); 
// a(0) = 15.0; a(1) =  25.0; a(2) = -5.0; a(3) = -1.6;
// no_fly_zone.push_back(a);
// a(0) = 15.0; a(1) =  25.0; a(2) = 1.6; a(3) = 5.0;
// no_fly_zone.push_back(a);


// Square Hoops
// Vector3d start = Vector3d(31,0,1.5);
// Vector3d end = Vector3d(30,-8,1.5);

// Circle Hoops
// Vector3d start = Vector3d(22.5,-10,1.5);
// Vector3d end = Vector3d(17.5,-10,1.5);

// Triangle Hoops
// Vector3d start = Vector3d(12.5,-10,1.5);
// Vector3d end = Vector3d(7.5,-12,1.5);