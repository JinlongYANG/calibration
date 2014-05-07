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

    for(int i=0;i< msg->hands.size(); i++)
    {

        Point3d handpt3d;
        handpt3d.x = msg->hands.at(i).pose.position.x;
        handpt3d.y = msg->hands.at(i).pose.position.y;
        handpt3d.z = msg->hands.at(i).pose.position.z;
        hand_position.push_back(handpt3d);

        hand_orientation.push_back(msg->hands.at(i).pose.orientation.x);
        hand_orientation.push_back(msg->hands.at(i).pose.orientation.y);
        hand_orientation.push_back(msg->hands.at(i).pose.orientation.z);
        hand_orientation.push_back(msg->hands.at(i).pose.orientation.w);

        for(int j=0;j< msg->fingers.size(); j++)
        {
            Point3d pt3d;
            pt3d.x = msg->fingers.at(j).pose.position.x;
            pt3d.y = msg->fingers.at(j).pose.position.y;
            pt3d.z = msg->fingers.at(j).pose.position.z;
            fingertip_position.push_back(pt3d);

            pt3d.x = msg->fingers.at(j).direction.x;
            pt3d.y = msg->fingers.at(j).direction.y;
            pt3d.z = msg->fingers.at(j).direction.z;
            finger_direction.push_back(pt3d);

            pt3d.x = msg->fingers.at(j).velocity.x;
            pt3d.y = msg->fingers.at(j).velocity.y;
            pt3d.z = msg->fingers.at(j).velocity.z;
            fingertip_velocity.push_back(pt3d);

            Point2d pt2d;
            pt2d.x = msg->fingers.at(j).length;
            pt2d.y = msg->fingers.at(j).width;
            finger_shape.push_back(pt2d);

        }

    }
}


void ToolPosition::Clear()
{

}

void ToolPosition::put_into_cloud()
{
    ;
}
