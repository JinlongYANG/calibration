#include <opencv2/core/core.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

void Ransac(pcl::PointCloud<pcl::PointXYZRGB> tooltip_pcl, Point3d& center, int maximum_round, float range){

    Point3d chosen_center;
    srand((unsigned)time(0));
    float max_ratio = 0;
    int best_index = 0;
    /*****************  Ransac to find chosen center  *********************/
    for(int i = 0; i < maximum_round; i++){
        float random_index = rand() % tooltip_pcl.size();
//        cout<<"random idex:"<<random_index<<endl;

        chosen_center.x = tooltip_pcl.at(random_index).x;
        chosen_center.y = tooltip_pcl.at(random_index).y;
        chosen_center.z = tooltip_pcl.at(random_index).z;

        int inlier = 0;

        for(int j = 0; j < tooltip_pcl.size(); j++){
            if(abs(tooltip_pcl.at(j).x - chosen_center.x) < range &&
                    abs(tooltip_pcl.at(j).y - chosen_center.y) < range &&
                    abs(tooltip_pcl.at(j).z - chosen_center.z) < range){
                inlier++;
            }
        }
        float random_ration = inlier*1.0/tooltip_pcl.size();
        if(random_ration > max_ratio){
            max_ratio = random_ration;
            best_index = random_index;
        }

        if(max_ratio>0.9)
            break;
    }
    /***************   Calculate true center using all inliers **********/
    int inlier = 0;
    for(int j = 0; j < tooltip_pcl.size(); j++){
        if(abs(tooltip_pcl.at(j).x - tooltip_pcl.at(best_index).x) < range &&
                abs(tooltip_pcl.at(j).y - tooltip_pcl.at(best_index).y) < range &&
                abs(tooltip_pcl.at(j).z - tooltip_pcl.at(best_index).z) < range){
            center.x += tooltip_pcl.at(j).x;
            center.y += tooltip_pcl.at(j).y;
            center.z += tooltip_pcl.at(j).z;
            inlier++;
        }
    }
    center.x = center.x/inlier;
    center.y = center.y/inlier;
    center.z = center.z/inlier;

}

