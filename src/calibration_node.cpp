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

float max_depth = 1.0;
float min_depth = 0.2;
float x_halflength = 0.5;
float y_halflength = 0.4;
Eigen::Matrix4f eigenTransform_visual2leap, eigenTransform_leap2visual;

Calibration_Node::Calibration_Node(ros::NodeHandle& nh):
    imageTransport_(nh),
    timeSynchronizer_(30),
    reconfigureServer_(ros::NodeHandle(nh,"calibration")),
    //transformListener_(buffer_, true)
    reconfigureCallback_(boost::bind(&Calibration_Node::updateConfig, this, _1, _2))
{

    rgbCameraSubscriber_.subscribe(nh, "/camera/rgb/image_rect_color", 30);
    rgbCameraInfoSubscriber_.subscribe(nh, "/camera/rgb/camera_info", 30);
    depthCameraSubscriber_.subscribe(nh, "/camera/depth_registered/image_raw", 30);
    depthCameraInfoSubscriber_.subscribe(nh, "/camera/depth/camera_info", 30);
    //pointCloud2_.subscribe(nh, "/camera/depth/points", 5);
    pointCloud2_.subscribe(nh, "/camera/depth_registered/points", 30);
    leapMotion_.subscribe(nh, "/leap_data",30);


    timeSynchronizer_.connectInput(rgbCameraSubscriber_, depthCameraSubscriber_,rgbCameraInfoSubscriber_,depthCameraInfoSubscriber_, pointCloud2_, leapMotion_);

    bgrImagePublisher_ = imageTransport_.advertise("BGR_Image", 1);
    depthImagePublisher_ = imageTransport_.advertise("Depth_Image", 1);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("Hand_pcl",1);
    hkp_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("Hand_kp_cl",1);

    timeSynchronizer_.registerCallback(boost::bind(&Calibration_Node::syncedCallback, this, _1, _2, _3, _4, _5, _6));
    reconfigureServer_.setCallback(reconfigureCallback_);

    rl_=120;
    gl_=0;
    bl_=0;

    rh_=255;
    gh_=50;
    bh_=50;

    calibration_done_ = false;


}

void  Calibration_Node::updateConfig(calibration::CalibrationConfig &config, uint32_t level){

    rl_=config.rl;
    gl_=config.gl;
    bl_=config.bl;

    rh_=config.rh;
    gh_=config.gh;
    bh_=config.bh;
}


void Calibration_Node::syncedCallback(const ImageConstPtr& cvpointer_rgbImage,const ImageConstPtr& cvpointer_depthImage, const CameraInfoConstPtr& cvpointer_rgbInfo, const CameraInfoConstPtr& cvpointer_depthInfo, const PointCloud2ConstPtr& pclpointer_pointCloud2, const leap_msgs::Leap::ConstPtr& ptr_leap){


    cv_bridge::CvImagePtr cvpointer_rgbFrame, cvpointer_depthFrame;
    Mat BGRImage,DepthImage, DepthMat;

    pcl::PointCloud<pcl::PointXYZRGB> msg_pcl;
    pcl::PointCloud<pcl::PointXYZRGB> tooltipcloud;
    pcl::PointCloud<pcl::PointXYZRGB> handcloud, hand1_kpt, hand2_kpt;

    ToolPosition tool_position;
    HandKeyPoints hand_kpt;
    Hand_XYZRGB hand1_XYZRGB, hand2_XYZRGB;

    ROS_INFO("Callback begins");

    try
    {
        if(calibration_done_ == false){
            int seq = cvpointer_rgbInfo->header.seq;

            /*******************   get color image and depth image    *******************/
            cvpointer_rgbFrame = cv_bridge::toCvCopy(cvpointer_rgbImage);
            cvpointer_depthFrame = cv_bridge::toCvCopy(cvpointer_depthImage);

            ROS_INFO("current image seq: %d ",cvpointer_rgbInfo->header.seq);
            BGRImage=cvpointer_rgbFrame->image;
            cvtColor( BGRImage, BGRImage, CV_RGB2BGR);
            DepthMat=cvpointer_depthFrame->image;
            /*maximum depth: 3m*/
            DepthImage = DepthMat*0.33;

            /*******************   read in the leapmotion data   *******************/

            tool_position.set_Leap_Msg(ptr_leap);

            Point3d Lm_keypoint;

            if(tool_position.fingers_count == 1)
            {
                std::cout<<tool_position.fingertip_position.size()<<std::endl;
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

                std::cout<<"size of tool tip cloud"<<tooltipcloud.size()<<std::endl;

                Point3d tool_center;
                if(tooltipcloud.size()>=10){
                    /*******************   publish pointCloud   *******************/
                    sensor_msgs::PointCloud2 cloud_msg;
                    toROSMsg(tooltipcloud,cloud_msg);
                    cloud_msg.header.frame_id=cvpointer_depthInfo->header.frame_id;
                    cloud_pub_.publish(cloud_msg);
                    /******************  ransac to get the visual center   ***********/
                    Ransac(tooltipcloud, tool_center, 10, 0.05);


                    /*******************   choose record the data or not    ***********/
                    cout<<"data size: "<<leap_motion_points_.rows<<endl;
                    std::cout<<"Visual center: "<<tool_center.x<<" "<<tool_center.y<<" "<<tool_center.z<<std::endl;
                    cout<<"Leap motion: "<<Lm_keypoint.x<<" "<< Lm_keypoint.y << " " << Lm_keypoint.z << endl;

                    cout<<"Do you want to save this data? (y / n)"<<endl;
                    char c;
                    cin.get(c);
                    while(c != 'y' && c != 'Y' && c != 'n' && c != 'N')
                        cin.get(c);
                    if(c == 'y'||c == 'Y'){
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


                        cout<< "data saved"<< endl;
                    }
                    else if (c == 'n' || c == 'N'){
                        cout<< "data abandoned"<<endl;
                    }

                    cout<<"data size: "<<leap_motion_points_.rows<<endl;
                    /*******************   if data is many enough, calculate the transform   **********/
                    if(leap_motion_points_.rows == 10 ){

                        Mat R,t;
                        Mat estimateMat;
                        bool RT_flag;
                        PixelRansac::pixelransac::compute(leap_motion_points_,Xtion_points_,20,R,t,RT_flag);
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
                    }
                }
            }

            /*******************   Convert the CvImage to a ROS image message and publish it to topics.   *******************/
            cv_bridge::CvImage bgrImage_msg;
            bgrImage_msg.encoding = sensor_msgs::image_encodings::BGR8;
            bgrImage_msg.image    = BGRImage;
            bgrImage_msg.header.seq = seq;
            bgrImage_msg.header.frame_id = seq;
            bgrImage_msg.header.stamp = ros::Time::now();
            bgrImagePublisher_.publish(bgrImage_msg.toImageMsg());

            cv_bridge::CvImage depthImage_msg;
            depthImage_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            depthImage_msg.image    = DepthImage;
            depthImage_msg.header.seq = seq;
            depthImage_msg.header.frame_id = seq;
            depthImage_msg.header.stamp = ros::Time::now();
            depthImagePublisher_.publish(depthImage_msg.toImageMsg());

            /*******************   clear data   *******************/
            tool_position.Clear();


            ROS_INFO("One callback done");

        }
        else{
            cout<<"Estimate transform from visual to leap: "<< endl;
            cout << eigenTransform_visual2leap <<endl;
            cout<<"Estimate transform from leap to visual: "<< endl;
            cout << eigenTransform_leap2visual <<endl;

            int seq = cvpointer_rgbInfo->header.seq;

            /*******************   get color image and depth image    *******************/
            cvpointer_rgbFrame = cv_bridge::toCvCopy(cvpointer_rgbImage);
            cvpointer_depthFrame = cv_bridge::toCvCopy(cvpointer_depthImage);

            ROS_INFO("current image seq: %d ",cvpointer_rgbInfo->header.seq);
            BGRImage=cvpointer_rgbFrame->image;
            cvtColor( BGRImage, BGRImage, CV_RGB2BGR);
            DepthMat=cvpointer_depthFrame->image;
            /*maximum depth: 3m*/
            DepthImage = DepthMat*0.33;

            /*******************   read in the leapmotion data   *******************/
            hand_kpt.set_Leap_Msg(ptr_leap);
            if(hand_kpt.hands_count != 0){
                std::cout<< hand_kpt.hands_count<<std::endl;
                hand1_XYZRGB.palm_center.x = hand_kpt.hand_position.at(0).x/1000.0;
                hand1_XYZRGB.palm_center.y = hand_kpt.hand_position.at(0).y/1000.0;
                hand1_XYZRGB.palm_center.z = hand_kpt.hand_position.at(0).z/1000.0;
                uint8_t r1 = 0, g1 = 255, b1 = 0;
                uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
                hand1_XYZRGB.palm_center.rgb = *reinterpret_cast<float*>(&rgb1);
                std::cout<<"hand1: palm center: "<<hand1_XYZRGB.palm_center.x<<" "<<hand1_XYZRGB.palm_center.y << " " << hand1_XYZRGB.palm_center.z<<std::endl;

                pcl::PointXYZRGB h1_fingertips;
                for(size_t i = 0; i < hand_kpt.fingertip_position.size ()/* && i < 5*/; ++i){
                    h1_fingertips.x = hand_kpt.fingertip_position.at(i).x/1000.0;
                    h1_fingertips.y = hand_kpt.fingertip_position.at(i).y/1000.0;
                    h1_fingertips.z = hand_kpt.fingertip_position.at(i).z/1000.0;
                    uint8_t rf = 0, gf = 150, bf = 150;
                    uint32_t rgbf = ((uint32_t)rf << 16 | (uint32_t)gf << 8 | (uint32_t)bf);
                    h1_fingertips.rgb = *reinterpret_cast<float*>(&rgbf);
                    hand1_XYZRGB.fingertip_position.push_back(h1_fingertips);

                }

                if(hand_kpt.hands_count > 1){
                    hand2_XYZRGB.palm_center.x = hand_kpt.hand_position.at(1).x/1000.0;
                    hand2_XYZRGB.palm_center.y = hand_kpt.hand_position.at(1).y/1000.0;
                    hand2_XYZRGB.palm_center.z = hand_kpt.hand_position.at(1).z/1000.0;
                    uint8_t r2 = 0, g2 = 255, b2 = 0;
                    uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
                    hand2_XYZRGB.palm_center.rgb = *reinterpret_cast<float*>(&rgb2);
                    std::cout<<"hand2: palm center: "<<hand2_XYZRGB.palm_center.x<<" "<<hand2_XYZRGB.palm_center.y << " " << hand2_XYZRGB.palm_center.z<<std::endl;
                }
            }

            /*******************   Leap motion to Xtion coordinate transform   *******************/

            if(hand_kpt.hands_count != 0){
                PointXYZRGB after_transform;
                Mat transform;
                transform = Mat::zeros(4,4, CV_32F);
                eigen2cv(eigenTransform_leap2visual, transform);
                after_transform.x = hand1_XYZRGB.palm_center.x*transform.at<float>(0,0)+hand1_XYZRGB.palm_center.y*transform.at<float>(0,1)+hand1_XYZRGB.palm_center.z*transform.at<float>(0,2)+transform.at<float>(0,3);
                after_transform.y = hand1_XYZRGB.palm_center.x*transform.at<float>(1,0)+hand1_XYZRGB.palm_center.y*transform.at<float>(1,1)+hand1_XYZRGB.palm_center.z*transform.at<float>(1,2)+transform.at<float>(1,3);
                after_transform.z = hand1_XYZRGB.palm_center.x*transform.at<float>(2,0)+hand1_XYZRGB.palm_center.y*transform.at<float>(2,1)+hand1_XYZRGB.palm_center.z*transform.at<float>(2,2)+transform.at<float>(2,3);
                hand1_kpt.push_back(after_transform);

                std::cout<<"Palm center: "<<hand1_kpt.at(0).x<<" "<<hand1_kpt.at(0).y << " " << hand1_kpt.at(0).z<<std::endl;
                for(size_t i = 0; i < hand1_XYZRGB.fingertip_position.size(); ++i){
                    PointXYZRGB finger_after_transform;
                    finger_after_transform.x = hand1_XYZRGB.fingertip_position.at(i).x*transform.at<float>(0,0)+hand1_XYZRGB.fingertip_position.at(i).y*transform.at<float>(0,1)+hand1_XYZRGB.fingertip_position.at(i).z*transform.at<float>(0,2)+transform.at<float>(0,3);
                    finger_after_transform.y = hand1_XYZRGB.fingertip_position.at(i).x*transform.at<float>(1,0)+hand1_XYZRGB.fingertip_position.at(i).y*transform.at<float>(1,1)+hand1_XYZRGB.fingertip_position.at(i).z*transform.at<float>(1,2)+transform.at<float>(1,3);
                    finger_after_transform.z = hand1_XYZRGB.fingertip_position.at(i).x*transform.at<float>(2,0)+hand1_XYZRGB.fingertip_position.at(i).y*transform.at<float>(2,1)+hand1_XYZRGB.fingertip_position.at(i).z*transform.at<float>(2,2)+transform.at<float>(2,3);
                    finger_after_transform.rgb = hand1_XYZRGB.fingertip_position.at(i).rgb;
                    hand1_kpt.push_back(finger_after_transform);
                    std::cout<<"finger "<<i<<": "<<hand1_kpt.at(i).x<<" "<<hand1_kpt.at(i).y << " " << hand1_kpt.at(i).z<<std::endl;
                }
                std::cout<<"Size: "<<hand1_kpt.size()<<std::endl;

                /*******************   get point cloud    *******************/
                fromROSMsg(*pclpointer_pointCloud2, msg_pcl);

                std::cout<<msg_pcl.points.size()<<std::endl;}

            /*******************   segment hand from the point cloud   *******************/
            pcl::PointXYZRGB p;
            uint8_t r = 255, g = r, b = r;
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

            for (size_t i = 0; i < msg_pcl.points.size (); ++i){
                if(msg_pcl.points[i].z < max_depth && msg_pcl.points[i].z > min_depth){
                    p.rgb = *reinterpret_cast<float*>(&rgb);
                    p.x = msg_pcl.points[i].x;
                    p.y = msg_pcl.points[i].y;
                    p.z = msg_pcl.points[i].z;
                    handcloud.push_back(p);
                }
            }
            /*******************   Convert the CvImage to a ROS image message and publish it to topics.   *******************/
            cv_bridge::CvImage bgrImage_msg;
            bgrImage_msg.encoding = sensor_msgs::image_encodings::BGR8;
            bgrImage_msg.image    = BGRImage;
            bgrImage_msg.header.seq = seq;
            bgrImage_msg.header.frame_id = seq;
            bgrImage_msg.header.stamp = ros::Time::now();
            bgrImagePublisher_.publish(bgrImage_msg.toImageMsg());

            cv_bridge::CvImage depthImage_msg;
            depthImage_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            depthImage_msg.image    = DepthImage;
            depthImage_msg.header.seq = seq;
            depthImage_msg.header.frame_id = seq;
            depthImage_msg.header.stamp = ros::Time::now();
            depthImagePublisher_.publish(depthImage_msg.toImageMsg());

            /*******************   publish pointCloud   *******************/
            sensor_msgs::PointCloud2 cloud_msg;
            toROSMsg(handcloud,cloud_msg);
            cloud_msg.header.frame_id=cvpointer_depthInfo->header.frame_id;
            cloud_pub_.publish(cloud_msg);

            sensor_msgs::PointCloud2 hand1_kpt_msg;
            toROSMsg(hand1_kpt,hand1_kpt_msg);
            hand1_kpt_msg.header.frame_id=cvpointer_depthInfo->header.frame_id;
            hkp_cloud_pub_.publish(hand1_kpt_msg);

            /*******************   clear data   *******************/
            hand_kpt.Clear();


            ROS_INFO("One callback done");
        }
    }
    catch (std::exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}


