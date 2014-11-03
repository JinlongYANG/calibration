#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include "calibration/calibration_node.hpp"
#include "calibration/handkp_leap_msg_calibration.h"
#include "calibration/handkp_leap_msg.h"
#include "calibration/Ransac.h"
#include "calibration/Hand_XYZRGB.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include "calibration/pixelransac.hpp"

#define PI 3.14159265


using namespace image_transport;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace cv;
using namespace image_geometry;
using namespace pcl;
using namespace Eigen;
using namespace std;

//using namespace tf2;

float max_depth = 1.5;
float min_depth = 0.3;
float x_halflength = 0.4;
float y_halflength = 0.3;
float manually_cali_z = -0.00;

Eigen::Matrix4f eigenTransform_visual2leap, eigenTransform_leap2visual;

Calibration_Node::Calibration_Node(ros::NodeHandle& nh):
    imageTransport_(nh),
    timeSynchronizer_(10),
    reconfigureServer_(ros::NodeHandle(nh,"calibration")),
    //transformListener_(buffer_, true)
    reconfigureCallback_(boost::bind(&Calibration_Node::updateConfig, this, _1, _2))
{

    pointCloud2_.subscribe(nh, "/camera/depth_registered/points", 1);
    leapMotion_.subscribe(nh, "/leap_data",30);


    timeSynchronizer_.connectInput(pointCloud2_, leapMotion_);

    hand_cld_pub_ = nh.advertise<sensor_msgs::PointCloud2>("Hand_pcl",1);
    tool_cld_pub_ = nh.advertise<sensor_msgs::PointCloud2>("tool_pcl",1);
    hkp_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("Hand_kp_cl",1);

    timeSynchronizer_.registerCallback(boost::bind(&Calibration_Node::syncedCallback, this, _1, _2));
    reconfigureServer_.setCallback(reconfigureCallback_);

    rl_=180;
    gl_=0;
    bl_=0;

    rh_=255;
    gh_=50;
    bh_=50;

    calibration_points_= 30;

    calibration_done_ = false;


}

void  Calibration_Node::updateConfig(calibration::CalibrationConfig &config, uint32_t level){

    rl_=config.rl;
    gl_=config.gl;
    bl_=config.bl;

    rh_=config.rh;
    gh_=config.gh;
    bh_=config.bh;

    calibration_points_ = config.number_of_calibration_points;
}


void Calibration_Node::syncedCallback(const PointCloud2ConstPtr& pclpointer_pointCloud2, const leap_msgs::Leap::ConstPtr& ptr_leap){



    pcl::PointCloud<pcl::PointXYZRGB> msg_pcl;
    pcl::PointCloud<pcl::PointXYZRGB> tooltipcloud;
    pcl::PointCloud<pcl::PointXYZRGB> handcloud, hand1_kpt;

    ToolPosition tool_position;
    HandKeyPoints hand_kpt;
    Hand_XYZRGB hand1_XYZRGB, hand2_XYZRGB;

    ros::Time time_stamp = ros::Time::now();

    //ROS_INFO("Callback begins");

    try
    {
        if(calibration_done_ == false){
            int seq = pclpointer_pointCloud2->header.seq;

            /*******************   get color image and depth image    *******************/

            //ROS_INFO("current image seq: %d ",pclpointer_pointCloud2->header.seq);


            /*******************   read in the leapmotion data   *******************/

            tool_position.set_Leap_Msg(ptr_leap);

            Point3d Lm_keypoint;

            if(tool_position.fingers_count != 0)
            {
                //std::cout<<tool_position.fingertip_position.size()<<std::endl;
                Lm_keypoint.x = tool_position.fingertip_position.at(0).x/1000.0;
                Lm_keypoint.y = tool_position.fingertip_position.at(0).y/1000.0;
                Lm_keypoint.z = tool_position.fingertip_position.at(0).z/1000.0;

                /*******************   get point cloud    *******************/

                fromROSMsg(*pclpointer_pointCloud2, msg_pcl);



                /*******************   get tool position in kinect   *******************/

                for (size_t i = 0; i < msg_pcl.points.size (); ++i){

                    if(msg_pcl.points[i].z < max_depth && msg_pcl.points[i].z > min_depth
                            && msg_pcl.points[i].x < x_halflength
                            && msg_pcl.points[i].x > -x_halflength
                            && msg_pcl.points[i].y < y_halflength
                            && msg_pcl.points[i].y > -y_halflength){
                        handcloud.push_back(msg_pcl.points[i]);
                        uint32_t rgb = *reinterpret_cast<int*>(&msg_pcl.points[i].rgb);
                        uint8_t r = (rgb >> 16) & 0x0000ff;
                        uint8_t g = (rgb >> 8) & 0x0000ff;
                        uint8_t b = (rgb) & 0x0000ff;
                        if(rl_ < r && r < rh_
                                && gl_ < g && g < gh_
                                && bl_ < b && b < bh_){
                            tooltipcloud.push_back(msg_pcl.points[i]);
                        }
                    }
                }

                //std::cout<<"size of tool tip cloud"<<tooltipcloud.size()<<std::endl;

                Point3d tool_center;
                if(tooltipcloud.size()>=10){
                    /*******************   publish pointCloud   *******************/
                    sensor_msgs::PointCloud2 cloud_msg;
                    toROSMsg(tooltipcloud,cloud_msg);
                    cloud_msg.header.frame_id=pclpointer_pointCloud2->header.frame_id;
                    cloud_msg.header.stamp = time_stamp;
                    tool_cld_pub_.publish(cloud_msg);

                    /******************  ransac to get the visual center   ***********/
                    Ransac(tooltipcloud, tool_center, 10, 0.05);


                    /*******************   choose record the data or not    ***********/
                    //cout<<"data size: "<<leap_motion_points_.rows<<endl;
                    //std::cout<<"Visual center: "<<tool_center.x<<" "<<tool_center.y<<" "<<tool_center.z<<std::endl;
                    //cout<<"Leap motion: "<<Lm_keypoint.x<<" "<< Lm_keypoint.y << " " << Lm_keypoint.z << endl;

//                    cout<<"Do you want to save this data? (y / n)"<<endl;
//                    char c;
//                    cin.get(c);
//                    while(c != 'y' && c != 'Y' && c != 'n' && c != 'N')
//                        cin.get(c);
                    if(seq%2 == 0){
                        Mat l;
                        l = Mat::zeros(1, 3, CV_32F);
                        l.at<float>(0,0) = Lm_keypoint.x;
                        l.at<float>(0,1) = Lm_keypoint.y;
                        l.at<float>(0,2) = Lm_keypoint.z;

                        leap_motion_points_.push_back(l);

                        l.at<float>(0,0) = tool_center.x;
                        l.at<float>(0,1) = tool_center.y;
                        l.at<float>(0,2) = tool_center.z;

                        Xtion_points_.push_back(l);


                        //cout<< "data saved"<< endl;
                    }
//                    else if (c == 'n' || c == 'N'){
//                        cout<< "data abandoned"<<endl;
//                    }

                    cout<<"data size: "<<leap_motion_points_.rows<<" / " << calibration_points_<<endl;
                }
                    /*******************   if data is many enough, calculate the transform   **********/
                    if(leap_motion_points_.rows >= calibration_points_ ){

                        Mat R,t;
                        Mat estimateMat;
                        bool RT_flag;
                        PixelRansac::pixelransac::compute(leap_motion_points_,Xtion_points_,2*calibration_points_,R,t,RT_flag);
                        if(RT_flag == true){
                            estimateMat=Mat::zeros(4,4,CV_32F);
                            R.copyTo(estimateMat(cv::Rect(0,0,3,3)));
                            t.copyTo(estimateMat(cv::Rect(3,0,1,3)));
                            estimateMat.at<float>(3,3)=1;
                            cv2eigen(estimateMat,eigenTransform_visual2leap);
                            eigenTransform_leap2visual = eigenTransform_visual2leap.inverse();
                            calibration_done_ = true;
                            cout<<"Estimate transform from visual to leap: "<< endl;
                            cout << eigenTransform_visual2leap <<endl;
                            cout<<"Estimate transform from leap to visual: "<< endl;
                            cout << eigenTransform_leap2visual <<endl;
                        }
                        else{
                            leap_motion_points_ = Scalar(0,0,0);
                            Xtion_points_ = Scalar(0,0,0);
                            cout<< "Transformation calculation failed, please try again!"<<endl;

                        }
                        sensor_msgs::PointCloud2 tool_pcl_clear;
                        toROSMsg(tooltipcloud,tool_pcl_clear);
                        tool_pcl_clear.header.frame_id=pclpointer_pointCloud2->header.frame_id;
                        tool_pcl_clear.header.stamp = time_stamp;
                        tool_cld_pub_.publish(tool_pcl_clear);
                    }

            }


            /*******************   clear data   *******************/
            tool_position.Clear();


            //ROS_INFO("One callback done");

        }
        else{
            //cout<<"Estimate transform from visual to leap: "<< endl;
            //cout << eigenTransform_visual2leap <<endl;
            //cout<<"Estimate transform from leap to visual: "<< endl;
            //cout << eigenTransform_leap2visual <<endl;

            int seq = pclpointer_pointCloud2->header.seq;


            /*******************   read in the leapmotion data   *******************/
            hand_kpt.set_Leap_Msg(ptr_leap);
            if(hand_kpt.hands_count != 0){
                hand1_XYZRGB.setValues(hand_kpt);

                if(hand_kpt.hands_count > 1){
                    hand2_XYZRGB.palm_center.x = hand_kpt.hand_position.at(1).x/1000.0;
                    hand2_XYZRGB.palm_center.y = hand_kpt.hand_position.at(1).y/1000.0;
                    hand2_XYZRGB.palm_center.z = hand_kpt.hand_position.at(1).z/1000.0;
                    uint8_t r2 = 0, g2 = 255, b2 = 0;
                    uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
                    hand2_XYZRGB.palm_center.rgb = *reinterpret_cast<float*>(&rgb2);
                    //std::cout<<"hand2: palm center: "<<hand2_XYZRGB.palm_center.x<<" "<<hand2_XYZRGB.palm_center.y << " " << hand2_XYZRGB.palm_center.z<<std::endl;
                }


                /*******************   Leap motion to Xtion coordinate transform   *******************/


                PointXYZRGB after_transform;
                Mat transform;
                transform = Mat::zeros(4,4, CV_32F);
                eigen2cv(eigenTransform_leap2visual, transform);
                after_transform.x = hand1_XYZRGB.palm_center.x*transform.at<float>(0,0)+hand1_XYZRGB.palm_center.y*transform.at<float>(0,1)+hand1_XYZRGB.palm_center.z*transform.at<float>(0,2)+transform.at<float>(0,3);
                after_transform.y = hand1_XYZRGB.palm_center.x*transform.at<float>(1,0)+hand1_XYZRGB.palm_center.y*transform.at<float>(1,1)+hand1_XYZRGB.palm_center.z*transform.at<float>(1,2)+transform.at<float>(1,3);
                //after_transform.z = hand1_XYZRGB.palm_center.x*transform.at<float>(2,0)+hand1_XYZRGB.palm_center.y*transform.at<float>(2,1)+hand1_XYZRGB.palm_center.z*transform.at<float>(2,2)+transform.at<float>(2,3);
                //This is manually calibration//
                after_transform.z = hand1_XYZRGB.palm_center.x*transform.at<float>(2,0)+hand1_XYZRGB.palm_center.y*transform.at<float>(2,1)+hand1_XYZRGB.palm_center.z*transform.at<float>(2,2)+transform.at<float>(2,3)+manually_cali_z;

                after_transform.rgb = hand1_XYZRGB.palm_center.rgb;
                hand1_kpt.push_back(after_transform);

                //std::cout<<"Palm center: "<<hand1_kpt.at(0).x<<" "<<hand1_kpt.at(0).y << " " << hand1_kpt.at(0).z<<std::endl;
                for(size_t i = 0; i < hand1_XYZRGB.fingertip_position.size(); ++i){
                    PointXYZRGB finger_after_transform;
                    finger_after_transform.x = hand1_XYZRGB.fingertip_position.at(i).x*transform.at<float>(0,0)+hand1_XYZRGB.fingertip_position.at(i).y*transform.at<float>(0,1)+hand1_XYZRGB.fingertip_position.at(i).z*transform.at<float>(0,2)+transform.at<float>(0,3);
                    finger_after_transform.y = hand1_XYZRGB.fingertip_position.at(i).x*transform.at<float>(1,0)+hand1_XYZRGB.fingertip_position.at(i).y*transform.at<float>(1,1)+hand1_XYZRGB.fingertip_position.at(i).z*transform.at<float>(1,2)+transform.at<float>(1,3);
                    //finger_after_transform.z = hand1_XYZRGB.fingertip_position.at(i).x*transform.at<float>(2,0)+hand1_XYZRGB.fingertip_position.at(i).y*transform.at<float>(2,1)+hand1_XYZRGB.fingertip_position.at(i).z*transform.at<float>(2,2)+transform.at<float>(2,3) - hand1_kpt.at(0).z;
                    //This is manually calibration//
                    finger_after_transform.z = hand1_XYZRGB.fingertip_position.at(i).x*transform.at<float>(2,0)+hand1_XYZRGB.fingertip_position.at(i).y*transform.at<float>(2,1)+hand1_XYZRGB.fingertip_position.at(i).z*transform.at<float>(2,2)+transform.at<float>(2,3)+manually_cali_z;
                    finger_after_transform.rgb = hand1_XYZRGB.fingertip_position.at(i).rgb;
                    hand1_kpt.push_back(finger_after_transform);
                    //joints position:
                    for(int j = 0; j<=4;j++){
                        PointXYZRGB joints_after_transform;
                        joints_after_transform.x = hand1_XYZRGB.bone_position.at(i).at(j).x*transform.at<float>(0,0)+hand1_XYZRGB.bone_position.at(i).at(j).y*transform.at<float>(0,1)+hand1_XYZRGB.bone_position.at(i).at(j).z*transform.at<float>(0,2)+transform.at<float>(0,3) ;
                        joints_after_transform.y = hand1_XYZRGB.bone_position.at(i).at(j).x*transform.at<float>(1,0)+hand1_XYZRGB.bone_position.at(i).at(j).y*transform.at<float>(1,1)+hand1_XYZRGB.bone_position.at(i).at(j).z*transform.at<float>(1,2)+transform.at<float>(1,3) ;
                        joints_after_transform.z = hand1_XYZRGB.bone_position.at(i).at(j).x*transform.at<float>(2,0)+hand1_XYZRGB.bone_position.at(i).at(j).y*transform.at<float>(2,1)+hand1_XYZRGB.bone_position.at(i).at(j).z*transform.at<float>(2,2)+transform.at<float>(2,3) + manually_cali_z;
                        joints_after_transform.rgb = hand1_XYZRGB.bone_position.at(i).at(j).rgb;
                        hand1_kpt.push_back(joints_after_transform);
                    }
                    //std::cout<<"finger "<<i<<": "<<hand1_kpt.at(i).x<<" "<<hand1_kpt.at(i).y << " " << hand1_kpt.at(i).z<<std::endl;
                }


                /*******************   get point cloud    *******************/
                fromROSMsg(*pclpointer_pointCloud2, msg_pcl);

                //std::cout<<msg_pcl.points.size()<<std::endl;}

                /*******************   segment hand from the point cloud   *******************/
                pcl::PointXYZRGB p;

                for (size_t i = 0; i < msg_pcl.points.size (); ++i){
                    if(abs(msg_pcl.points[i].x - hand1_kpt.at(0).x)< 0.15 &&
                        abs(msg_pcl.points[i].y - hand1_kpt.at(0).y)< 0.15 &&
                        abs(msg_pcl.points[i].z - hand1_kpt.at(0).z)< 0.15/* &&
                       /msg_pcl.points[i].g < 240 &&
                        msg_pcl.points[i].b < 240*/){
                        p.rgb = msg_pcl.points[i].rgb;
                        p.x = msg_pcl.points[i].x - hand1_kpt.at(0).x;
                        p.y = msg_pcl.points[i].y - hand1_kpt.at(0).y;
                        p.z = msg_pcl.points[i].z - hand1_kpt.at(0).z;
                        handcloud.push_back(p);
                    }
                }
                hand1_kpt.push_back(hand1_kpt.at(0));
                hand1_kpt.at(31).rgb = 16777215;
                hand1_kpt.at(0).x = 0;
                hand1_kpt.at(0).y = 0;
                hand1_kpt.at(0).z = 0;                
                std::cout<<"Size: "<<hand1_kpt.size()<<std::endl;


                /*******************   publish pointCloud   *******************/
                sensor_msgs::PointCloud2 hand1_kpt_msg;
                toROSMsg(hand1_kpt,hand1_kpt_msg);
                hand1_kpt_msg.header.frame_id=pclpointer_pointCloud2->header.frame_id;
                hand1_kpt_msg.header.stamp = time_stamp;
                hkp_cloud_pub_.publish(hand1_kpt_msg);

                /*******************   clear data   *******************/
                hand_kpt.Clear();
            }

            cout<<"Estimate transform from visual to leap: "<< endl;
            cout << eigenTransform_visual2leap <<endl;
            cout<<"Estimate transform from leap to visual: "<< endl;
            cout << eigenTransform_leap2visual <<endl;
        }

        /*******************   publish pointCloud (small area for calibration, segmented hand for finger tracking*******************/
        sensor_msgs::PointCloud2 cloud_msg;
        toROSMsg(handcloud,cloud_msg);
        cloud_msg.header.frame_id=pclpointer_pointCloud2->header.frame_id;
        cloud_msg.header.stamp = time_stamp;
        hand_cld_pub_.publish(cloud_msg);

        ROS_INFO("One callback done");

    }
    catch (std::exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}


