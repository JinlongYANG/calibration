#ifndef PIXELRANSAC_HPP_INCLUDED
#define PIXELRANSAC_HPP_INCLUDED

#include <vector>
#include "poseestimate.hpp"
#include <opencv2/opencv.hpp>
#include <ctime>
#include <cstdlib>
#include <algorithm>
using namespace std;
using namespace cv;



namespace PixelRansac
{


/**********************************************
            Class definition
***********************************************/


vector<int> shuffle(int number){
    std::srand( unsigned (std::time(0)));
    std::vector<int> myvector;

    //set some values
    for(int i = 0; i<number; ++i){
        myvector.push_back(i);       
    }

    std::random_shuffle(myvector.begin(),myvector.end());
    return myvector;
}

void checkModel(Mat point_p,Mat point_c,Mat R, Mat t, float& error){
    Mat err;
    err = point_c - ( ( R*point_p ) + t ) ;
    error =err.dot(err) ;
    error =sqrt(error);
}

class pixelransac
{
public:

    //Mat A and B have size [nx3];

    static void compute(Mat A, Mat B, int n, Mat& Rotation, Mat& Translation, bool& RT_flag){
        vector<int> newvec;
        vector<float> Ratio, inliers_num;
        vector<Mat> R_models , t_models;
        int rows =A.rows;
        float threshold=0.4;
        RT_flag = true;


        //generate n models
        for(int Round = 0; Round<n; Round++){
            int inliers_count=0;
            //randomly select 4 pairs of points and estimate the model
            newvec=shuffle(int(A.rows));
            //cout<<"NWVEC  "<<newvec[0]<<" "<<newvec[1]<<" " <<newvec[2] << " " <<newvec[3]<<endl;
            int select = 4;
//            if (rows > 4)
//                select = 2*rows /3;

            Mat points_curr,points_prev;
            for (int j=0; j<select; ++j){
                int index = newvec[j];
                //cout<<"index" << index<<endl;
                points_prev.push_back(B.row(index));
                points_curr.push_back(A.row(index));
            }
            //cout<<"points_prev  \n"<<points_prev<<endl;
            //cout<<"points_curr  \n"<<points_curr<<endl;


            //model estimation
            Mat R,t;
            poseEstimate::poseestimate::compute(points_curr.t(),points_prev.t(),R,t);
            //model generation done
            //use the generated model to test all the samples to count outliers and inliers
            for (int i=0 ; i<rows; ++i){
                Mat point_p = B.row(i);
                Mat point_c = A.row(i);
                float error;

                checkModel(point_p.t(),point_c.t(),R,t,error);
                //cout<<" ---ERROR--> "<<error;

                // if error is less than threshold inliers_count++
                if (error < threshold){
                    inliers_count +=1;

                }
            }
            //save Ratio, R and t
            float ratio=inliers_count*1.0/rows;
            Ratio.push_back(ratio);
            inliers_num.push_back(inliers_count);
            R_models.push_back(R);
            t_models.push_back(t);
        }
        //ROS_INFO("%d many models created, select best one!",n);
        int index=0;
        float max=0.0;
        for (int i=0; i<n; i++){
            if (max < Ratio[i]){
                max = Ratio[i];
                index = i;
            }
        }
        cout<<"best fit model has a ratio = "<<max<<endl;
        //best model
        float t_limit = 6.0;
        float R_limit = 0.1;

        if (max == 0){
            RT_flag = false;
            Rotation = R_models[index];
            Translation = t_models[index];
            cout<< "estimated t_limit "<<Translation.dot(Translation)<<endl;
            cout<< "estimated R_limit "<<determinant(Rotation)<<endl;
            cout<<"MAX = 0!!!"<<endl;

        } else if (inliers_num[index] >= 4) {
            //Select inlier points according to the best model and then generate an accurate model using all inlier points
            Mat inlierA, inlierB;
            for (int i=0 ; i<rows; ++i){
                Mat point_p = B.row(i);
                Mat point_c = A.row(i);
                float error;

                checkModel(point_p.t(),point_c.t(),R_models[index],t_models[index],error);
                //cout<<" ---ERROR--> "<<error;

                // if error is less than threshold inliers_count++
                if (error < threshold){
                    inlierA.push_back(point_c);
                    inlierB.push_back(point_p);
                }
            }
            poseEstimate::poseestimate::compute(inlierA.t(),inlierB.t(),Rotation,Translation);
        } else{
            Rotation = R_models[index];
            Translation = t_models[index];
        }

        if(Translation.dot(Translation) > t_limit || abs( determinant(Rotation) ) < R_limit){
            RT_flag = false;
            cout<< "estimated t_limit "<<Translation.dot(Translation)<<endl;
            cout<< "estimated R_limit "<<determinant(Rotation)<<endl;

        }
    }




private:

protected:
};
}



#endif // PIXELRANSAC_HPP_INCLUDED
