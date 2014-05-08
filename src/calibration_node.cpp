#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include "calibration/calibration_node.hpp"
#include "calibration/handkp_leap_msg.h"
#include "calibration/Hand_XYZRGB.h"
#include "calibration/Ransac.h"
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
float min_depth = 0.3;
float x_halflength = 0.4;
float y_halflength = 0.3;

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

    rl_=100;
    gl_=0;
    bl_=0;

    rh_=255;
    gh_=50;
    bh_=50;


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

    ToolPosition tool_position;

    ROS_INFO("Callback begins");

    try
    {
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
                std::cout<<"Visual center: "<<tool_center.x<<" "<<tool_center.y<<" "<<tool_center.z<<std::endl;
                cout<<"Leap motion: "<<Lm_keypoint.x<<" "<< Lm_keypoint.y << " " << Lm_keypoint.z << endl;

                cout<<"Do you want to save this data? (y / n)"<<endl;
                char c;
                cin.get(c);
                while(c != 'y' && c != 'Y' && c != 'n' && c != 'N')
                    cin.get(c);
                if(c == 'y'||c == 'Y'){
                    cout<< "data saved"<< endl;
                }
                else if (c == 'n' || c == 'N'){
                    cout<< "data abandoned"<<endl;
                }
            }
            /*******************   if data is many enough, calculate the transform   **********/



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
    catch (std::exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}


