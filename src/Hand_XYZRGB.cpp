#include "calibration/Hand_XYZRGB.h"

Hand_XYZRGB::Hand_XYZRGB()
{


}

void Hand_XYZRGB::setValues(const HandKeyPoints &hand_kpt){
    palm_center.x = hand_kpt.hand_position.at(0).x/1000.0;
    palm_center.y = hand_kpt.hand_position.at(0).y/1000.0;
    palm_center.z = hand_kpt.hand_position.at(0).z/1000.0;
    //set palm center color
    uint8_t r1 = 0, g1 = 255, b1 = 0;
    uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
    palm_center.rgb = *reinterpret_cast<float*>(&rgb1);
    //std::cout<<"hand1: palm center: "<<hand1_XYZRGB.palm_center.x<<" "<<hand1_XYZRGB.palm_center.y << " " << hand1_XYZRGB.palm_center.z<<std::endl;

    pcl::PointXYZRGB h1_fingertips;
    for(size_t i = 0; i < hand_kpt.fingertip_position.size ()/* && i < 5*/; ++i){
        h1_fingertips.x = hand_kpt.fingertip_position.at(i).x/1000.0;
        h1_fingertips.y = hand_kpt.fingertip_position.at(i).y/1000.0;
        h1_fingertips.z = hand_kpt.fingertip_position.at(i).z/1000.0;
        //set fingertip color
        uint8_t rf = 0, gf = 150, bf = 150;
        uint32_t rgbf = ((uint32_t)rf << 16 | (uint32_t)gf << 8 | (uint32_t)bf);
        h1_fingertips.rgb = *reinterpret_cast<float*>(&rgbf);
        fingertip_position.push_back(h1_fingertips);
        finger_name.push_back(hand_kpt.finger_names.at(i));

        //bones:
        std::vector<pcl::PointXYZRGB> one_finger;
        pcl::PointXYZRGB joints;
        //set joints color
        rf = 63*hand_kpt.finger_names.at(i);
        bf = 255-rf;
        for(int j = 0; j<=4;j++){
            joints.x = hand_kpt.bone.at(i).at(j).x/1000.0;
            joints.y = hand_kpt.bone.at(i).at(j).y/1000.0;
            joints.z = hand_kpt.bone.at(i).at(j).z/1000.0;
            gf = j*50;
            rgbf = ((uint32_t)rf << 16 | (uint32_t)gf << 8 | (uint32_t)bf);
            joints.rgb = *reinterpret_cast<float*>(&rgbf);
            one_finger.push_back(joints);
        }
        bone_position.push_back(one_finger);
    }

}
