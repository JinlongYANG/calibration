#include "calibration/handkp_leap_msg.h"

HandKeyPoints::HandKeyPoints()
{
    frame_id=0;
    time_stamp=0;
    hands_count=0;
    fingers_count=0;

}



void HandKeyPoints::set_Leap_Msg(const leap_msgs::Leap::ConstPtr& msg)       //2nd & 3rd argument are ids from previous frame
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

        for(int finger_index = 0; finger_index < 5; finger_index++){
            for(int j=0;j< msg->fingers.size(); j++)
            {

                if ( finger_index == msg->fingers.at(j).name)
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

                    finger_names.push_back(msg->fingers.at(j).name);

                    std::vector<Point3d> one_finger;
                    for(int k = 0; k<4; k++){
                        Point3d bone_pt3d;
                        bone_pt3d.x = msg->fingers.at(j).bones[k].start.x;
                        bone_pt3d.y = msg->fingers.at(j).bones[k].start.y;
                        bone_pt3d.z = msg->fingers.at(j).bones[k].start.z;

                        one_finger.push_back(bone_pt3d);
                    }
                    Point3d bone_pt3d;
                    bone_pt3d.x = msg->fingers.at(j).bones[3].end.x;
                    bone_pt3d.y = msg->fingers.at(j).bones[3].end.y;
                    bone_pt3d.z = msg->fingers.at(j).bones[3].end.z;

                    one_finger.push_back(bone_pt3d);
                    bone.push_back(one_finger);
                }
            }
        }

    }
}


void HandKeyPoints::Clear()
{
    hands_count=0;
    fingers_count=0;
    hand_position.clear();
    hand_orientation.clear();
    fingertip_position.clear();
    finger_direction.clear();
    fingertip_velocity.clear();
    finger_shape.clear();
}

void HandKeyPoints::put_into_cloud()
{
    ;
}
