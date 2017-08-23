#include "ros/ros.h"
#include <ARToolKitPlus/TrackerSingleMarker.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <iostream>
#include <stdio.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define IMG_WIDTH 640
#define IMG_HEIGHT 360

using namespace cv;
using namespace std;
using ARToolKitPlus::TrackerSingleMarker;

double pattern_width = 100.0;// /mm
bool useBCH = true;

int number_saved[40];
int saved_total = 0;

char fileName[256];
int num_Done=0;
ARFloat Area_mark,Area_img;
ARFloat Fitness,Percentage;
ARFloat Area_best;
ARFloat corners_best[4][2];

double threshold_F=0.75,threshold_P=0.1;
double threshold_F_low=0.4,threshold_P_low=0.01;

Point2f srcTri[3], dstTri[3];

Mat img;

struct passwd* pw;    
const char* home_dir;

std_msgs::Int32 rec_num;
ros::Publisher num_pub;

TrackerSingleMarker tracker(IMG_WIDTH, IMG_HEIGHT, 8, 6, 6, 6, 0);

void cvMatToRawData(const cv::Mat& img, std::vector<unsigned char>& rawData){
    /* convert the image into a gray level image*/
    cv::Mat gray;
    if( img.channels() > 1)
        cv::cvtColor(img, gray, CV_RGB2GRAY);
    else
        gray = img;
    rawData.resize(gray.rows*gray.cols);
    for( size_t i = 0; i < gray.rows; i++){
        for( size_t j = 0; j < gray.cols; j++)
            rawData[i*gray.cols+j] = gray.at<unsigned char>(i,j);
    }
}

void affine(Mat & src, Mat &affined_img)
{
    /* Affine transform matrix */
    int max_p_id = -1;
    int min_p_id = -1;
    int small_x_id = -1;
    int max = -1;
    int min = IMG_HEIGHT + IMG_WIDTH;
    int small_x = IMG_WIDTH;

    for(int i=0; i<4; i++) //find right down corner
    {
        if(corners_best[i][0] + corners_best[i][1] > max)
        {
            max = corners_best[i][0] + corners_best[i][1];
            max_p_id = i;
        }
    }

    for(int i=0; i<4; i++) //find left up corner
    {
        if(corners_best[i][0] + corners_best[i][1] < min)
        {
            min = corners_best[i][0] + corners_best[i][1];
            min_p_id = i;
        }
    }

    for(int i=0; i<4; i++) //find left down corner
    {
        if(i!=max_p_id && i!=min_p_id)
        {
            if(corners_best[i][0] < small_x)
            {
                small_x = corners_best[i][0];
                small_x_id = i;
            }
        }
    }

    srcTri[0].x = corners_best[max_p_id][0];
    srcTri[0].y = corners_best[max_p_id][1];
    srcTri[1].x = corners_best[small_x_id][0];
    srcTri[1].y = corners_best[small_x_id][1];
    srcTri[2].x = corners_best[min_p_id][0];
    srcTri[2].y = corners_best[min_p_id][1];


    //cout<<"corners_best "<<corners_best[0][0]<<" "<<corners_best[0][1]<<" "<<corners_best[1][0]<<" "<<corners_best[1][1]<<" "<<corners_best[2][0]<<" "<<corners_best[2][1]<<endl;

    int new_hight = IMG_HEIGHT / 4;

    dstTri[0].x = IMG_WIDTH/2 + new_hight;
    dstTri[0].y = IMG_HEIGHT/2 + new_hight;
    dstTri[1].x = IMG_WIDTH/2 - new_hight;
    dstTri[1].y = IMG_HEIGHT/2 + new_hight;
    dstTri[2].x = IMG_WIDTH/2 - new_hight;
    dstTri[2].y = IMG_HEIGHT/2 - new_hight;

    Mat affine_matrix = cvCreateMat(2, 3, CV_32FC1);
    affined_img = Mat::zeros( src.rows, src.cols, src.type() );
    affine_matrix = getAffineTransform(srcTri, dstTri);

    warpAffine(src, affined_img, affine_matrix, affined_img.size());

    Mat kernel(5,5,CV_32F,Scalar(0));
    kernel.at<float>(2,2)=10.0;
    kernel.at<float>(0,0)=-1.0;
    kernel.at<float>(4,4)=-1.0;
    kernel.at<float>(0,4)=-1.0;
    kernel.at<float>(4,0)=-1.0;
    kernel.at<float>(2,0)=-1.0;
    kernel.at<float>(0,2)=-1.0;
    kernel.at<float>(2,4)=-1.0;
    kernel.at<float>(4,2)=-1.0;
    filter2D(affined_img,affined_img,affined_img.depth(),kernel);
    GaussianBlur( affined_img, affined_img, Size(9,9),0,0);
    /*******************/
}

int find_and_judge(Mat &image)
{
    if(image.data!=0)
    {
        /* convert the opencv matrix into a raw camera buffer*/
        std::vector<unsigned char> cameraBuffer;
        cvMatToRawData(image, cameraBuffer);
        
        Area_img=image.cols*image.rows;
        tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
           
        std::vector<int> markerId_tmp;
        int id_best = 20000;
        float conf_best = 0;
        ARFloat T_best[16];
        int thresholds[12] = {20,40,60,80,100,120,140,160,180,200,220,240};
        
        for( size_t i = 0; i < 12; i++){
            tracker.setThreshold(thresholds[i]);
            
            // let's use lookup-table undistortion for high-speed
            // note: LUT only works with images up to 1024x1024
            tracker.setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
            
            // switch to simple ID based markers
            // use the tool in tools/IdPatGen to generate markers
            tracker.setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
            
            ARToolKitPlus::ARMarkerInfo* marker_info = 0;
            markerId_tmp = tracker.calc(&cameraBuffer[0], &marker_info);
            if( markerId_tmp.size() > 0){
                id_best = tracker.selectBestMarkerByCf(); /*choose the marker with highest confidence*/
                float conf = tracker.getConfidence();
                
                if( conf > conf_best){
                    conf_best = conf;
                    std::copy(tracker.getModelViewMatrix(),tracker.getModelViewMatrix()+16,T_best);
                    const ARToolKitPlus::ARMarkerInfo* marker_info = tracker.getMarkerInfoById(id_best);
                    for( size_t s = 0; s < 4; s++){
                        for( size_t t = 0; t < 2; t++){
                            corners_best[s][t] = marker_info->vertex[s][t];
                        }
                    }
                }
            }
        }

        
        if(id_best > -1)std::cout << "Found marker: " << id_best << endl;
                    
        cv::Mat todraw;
        cv::cvtColor(image,todraw,CV_GRAY2RGB);
        
        if( id_best >= 0 && id_best < 10000)
        {
            /*visualize those markers*/
            double R[9],t[3];
            const ARFloat* T = T_best;
            
            ARFloat Xmax_corners_best=corners_best[0][0];
            ARFloat Ymax_corners_best=corners_best[0][1];
            ARFloat Xmin_corners_best=corners_best[0][0];
            ARFloat Ymin_corners_best=corners_best[0][1];
            ARFloat X_Differ,Y_Differ;
            ARFloat X_centre_best,Y_centre_best;// centre of the markers
            ARFloat corner_square[1][2];
            
            /**1.draw the detected corners*/
            for( size_t i = 0; i < 4; i++){
                cv::circle(todraw,cv::Point2f(corners_best[i][0],
                           corners_best[i][1]),3,cv::Scalar(0,255,255),1,CV_AA);
                
                if(corners_best[i][0]>Xmax_corners_best)
                    Xmax_corners_best=corners_best[i][0];
                
                if(corners_best[i][0]<Xmin_corners_best)
                    Xmin_corners_best=corners_best[i][0];
                
                if(corners_best[i][1]>Ymax_corners_best)
                    Ymax_corners_best=corners_best[i][1];
                
                if(corners_best[i][1]<Ymin_corners_best)
                    Ymin_corners_best=corners_best[i][1];
            }
            
            /*compute the Aera*/
            X_Differ=(Xmax_corners_best-Xmin_corners_best);
            Y_Differ=(Ymax_corners_best-Ymin_corners_best);
            X_centre_best=0.5*X_Differ+Xmin_corners_best;
            Y_centre_best=0.5*Y_Differ+Ymin_corners_best;
            if(X_Differ>Y_Differ)
            {
                Area_best=X_Differ*X_Differ;
                corner_square[0][0]=X_centre_best-0.5*X_Differ;
                corner_square[0][1]=Y_centre_best-0.5*X_Differ;
                rectangle(todraw, Point(corner_square[0][0],corner_square[0][1]),
                        Point(corner_square[0][0]+X_Differ, corner_square[0][1] + X_Differ),
                        Scalar(0, 0, 255), 2, 8);
            }
            else
            {
                Area_best=Y_Differ*Y_Differ;
                corner_square[0][0]=X_centre_best-0.5*Y_Differ;
                corner_square[0][1]=Y_centre_best-0.5*Y_Differ;
                rectangle(todraw, Point(corner_square[0][0],corner_square[0][1]),
                        Point(corner_square[0][0]+Y_Differ, corner_square[0][1] + Y_Differ),
                        Scalar(0, 0, 255), 2, 8);
            }
            
            ARFloat corners_best_temp[4][2];
            
            for (int i=1;i<4;i++)
            {
                corners_best_temp[0][0]=0;corners_best_temp[0][1]=0;
                corners_best_temp[i][0]=corners_best[0][0]-corners_best[i][0];
                corners_best_temp[i][1]=corners_best[0][1]-corners_best[i][1];
            }
            /*compute the Aera by an algebraic formula:Aera=0.5*(|x0y1-x1y0|+|x1y2-x2y1|+|x3y2-x2y3|)*/
            Area_mark=fabs(corners_best_temp[1][0]*corners_best_temp[2][1]
                    -corners_best_temp[2][0]*corners_best_temp[1][1])+
                    fabs(corners_best_temp[2][0]*corners_best_temp[3][1]
                    -corners_best_temp[3][0]*corners_best_temp[2][1]);
            Fitness=(0.5*Area_mark)/(Area_best);
            Percentage= Area_mark/Area_img;

            if(Fitness>threshold_F && Percentage >threshold_P)
            {
                return id_best;
            }
            else return -3;
        }
        else return -2;

        //cv::imshow("todraw", todraw);
        //cv::waitKey(20);
    }
    else return -1;

    
}

void imageCallback(const sensor_msgs::Image &msg)
{
    //printf("callback established\n");
    cout<<"Hit"<<endl;
    cout << msg.data.size() << endl;
    /*for (int i=0;i<numPixels;i++){
        cameraBuffer[i] = 0.2989*msg.data[3*i]+0.5870*msg.data[3*i+1]+0.1140*msg.data[3*i+2];
        //printf("%s\n", );B,G,R
    }*/
    //printf("successful callback\n");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:%s",e.what());
    }
    img = cv_ptr->image;
    cvtColor(img, img, CV_BGR2GRAY);

    Fitness = 0.0;
    Percentage = 0.0;

    bool if_recged = false;
    int id_best = find_and_judge(img);

    if(id_best >= 0 || id_best == -3)
    {
        for(int i=0; i<saved_total; i++)
        {
            if(number_saved[i] == id_best)
            {
                if_recged = true;
                cout << "Already saved!" << std::endl;
                break;
            }
        }
    }
    
    if(!if_recged)
    {
        if(id_best >= 0){   //good threshhold
            cout << " - Result is correct and good!" << std::endl;
            char resultName[256];
            sprintf(resultName,"%s/Result/%d.png", home_dir, id_best);
            imwrite(resultName,img);
            number_saved[saved_total] = id_best;
            saved_total ++;
        }

        else if(Fitness>threshold_F_low && Percentage>threshold_P_low && id_best == -3)
        {
            Mat affined_img;
            affine(img, affined_img);
            id_best = find_and_judge(affined_img);
            if(id_best >= 0)
            {
                cout << " - Use Processed Result!" << std::endl;
                char resultName[256];
                sprintf(resultName,"%s/Result/%d.png", home_dir, id_best);
                imwrite(resultName,affined_img);
                number_saved[saved_total] = id_best;
                saved_total ++;

                //imshow("affined_img", affined_img);
                //waitKey(2);
            }
            else cout << " - Processed Result Can not be Used!" << std::endl;
        }
        else{
            cout << " No Result" << std::endl; 
        }
    }
    

    rec_num.data = id_best;
                            
    num_pub.publish(rec_num);
}



int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "qr_detection_single");
    ros::NodeHandle node;

    pw = getpwuid(getuid());    
    home_dir = pw->pw_dir;

    tracker.setPatternWidth(pattern_width);
    tracker.setBorderWidth(useBCH ? 0.125 : 0.25);
    // load a camera calibration file.
    if (!tracker.initWithoutCameraFile())
    {
        std::cerr << "ERROR: init() failed" << std::endl;
        return -1;
    } 

    ros::Subscriber image_sub;
    image_sub = node.subscribe("/camera/image_raw",5,imageCallback);
    //ros::Publisher _pub = n.advertise<std_msgs::Bool>("/image_loss",1) ;
    num_pub = node.advertise<std_msgs::Int32>("/rec_num",1) ;
    while(node.ok())
    {
        ros::Rate loop_rate(10);
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}