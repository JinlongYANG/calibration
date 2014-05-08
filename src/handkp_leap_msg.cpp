#include "calibration/handkp_leap_msg.h"

ToolPosition::ToolPosition()
{
    frame_id=0;
    time_stamp=0;
    hands_count=0;
    fingers_count=0;

}



void ToolPosition::set_Leap_Msg(const leap_msgs::Leap::ConstPtr& msg)       //2nd & 3rd argument are ids from previous frame
{
    frame_id = msg->leap_frame_id;
    time_stamp = msg->leap_time_stamp;
    hands_count = msg->hands.size();
    fingers_count = msg->fingers.size();

    for(int j=0;j< msg->fingers.size(); j++)
    {
        Point3d pt3d;
        pt3d.x = msg->fingers.at(j).pose.position.x;
        pt3d.y = msg->fingers.at(j).pose.position.y;
        pt3d.z = msg->fingers.at(j).pose.position.z;
        fingertip_position.push_back(pt3d);

    }

}


void ToolPosition::Clear()
{

}

void ToolPosition::put_into_cloud()
{
    ;
}
