#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>
#include<std_msgs/Float32.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include <opencv2/aruco.hpp>
#include<pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <vector>
using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;


cv_bridge::CvImagePtr rawImagePtr,LidarImgPtr;
Mat rawImage,InRanImg,GrayImg,MaterialImg,Road_MaterialImg,hsvImgWhite,hsvImgYellow,hsvImgBlue,hsvImg,BilateralImg,EqualizeImg;
Mat L_HSVImg,R_HSVImg;
Mat rstImg,X;
//Mat warpMat(2,4,CV_32FC1);
Mat warpMat;
Mat LidarImg,LidarImg_warp;
int offset =80; // middle of lane
vector<Vec4i> lines;


int Whigh_H=180,Whigh_S=60,Whigh_V=255;
int Wlow_H=0,Wlow_S=0,Wlow_V=200;//v=200
int Yhigh_H=27,Yhigh_S=255,Yhigh_V=255; //Yhigh_H=50,Yhigh_S=255,Yhigh_V=255;
int Ylow_H=17,Ylow_S=90,Ylow_V=171;//Ylow_H=15,Ylow_S=90,Ylow_V=171
//int Yhigh_H= 40,Yhigh_S= 255,Yhigh_V= 255;
//int Ylow_H=  20, Ylow_S= 0, Ylow_V=  171;//h=25
int Bhigh_H=135,Bhigh_S=255,Bhigh_V=255;
int Blow_H=90,Blow_S=90,Blow_V=171;//h=25

int _a=0,_b=0,_c=0,__a=0,__b=0,__c=0;
int _a2=0,_b2=0,_c2=0,__a2=0,__b2=0,__c2=0;

int flag =1;
int size;
Vec4i temp;
Rect roi;
Point lineForm[1][4];
Point Road_erase[1][18];
const Point* ppt[1]={lineForm[0]};
const Point* Road_ppt[1]={Road_erase[0]};
int npt[]={4};
int R_E_npt[]={18};

int offset_x_cam=500;
int offset_y_cam=300;
int past_offset_x;
Point2f laneConerInput[4] = {Point2f(0,0),Point2f(479,0),Point2f(479,639),Point2f(0,639)};
Point2f laneConerOutput[4] = {Point2f(18,0),Point2f(101,0),Point2f(123,73),Point2f(0,73)};
vector<Point2f> homoInput;
vector<Point2f> homoOutput;
//ros::NodeHandle nh;
//ros::Publisher waypoint_pub = nh.advertise<std_msgs::Float32>("/Lidar/waypoint_misvalue",10);
std_msgs::Float32 waypoint;

sensor_msgs::PointCloud2 way_output;
sensor_msgs::PointCloud2 lane_output;

ros::Publisher lane_waypoint;
ros::Publisher lane_tracking;

int start_X1 ,start_X2,start_X1_ag,start_X2_ag ,start_flag=1,start_Y1=0,start_Y2=0;
int detectFlag =1;
//std::vector laneDetected<vector<int>>;
int laneDetected[100][6]={};

int numOfLanePoint_R = 0;
int numOfLanePoint_L = 0;

int LRflag=0;
double lane_calcal_R =0,lane_calcal_L =0;
double pastTheta=0;

int reliability=0; // left:-1 right:1 both:2 non:0
int reliability_Cnt_Num=0;
int reliability_Cnt_All=0;
int reliability_Cnt_L =0;
int reliability_Cnt_R=0;
int reliability_Cnt_Non =0;
Mat Road_Of_Interest;

int left_ran_pt[480]={0};
int right_ran_pt[480]={0};

//variable to get more lane(to check reliability)
Mat warp_erase;
int erase_range = 10;
double avgtime;
double sumtime=0;
int cnttime=0;
int past_car_position = 340;
int car_position = 340 ;
int tempConsensus;
int _x1,_y1,_x2,_y2;
typedef struct _pt{
    int x;
    int y;
    _pt( )
    {
        x = 0;
        y = 0;
    }
    _pt( int _x, int _y)
    {
        x = _x;
        y = _y;
    }
} pt;

typedef struct _hough{
    double rho;
    double theta;

    _hough( )
    {
        rho = 0;
        theta = 0;
    }
    _hough( double _rho, double _theta)
    {
        rho = _rho;
        theta = _theta;
    }
} Hough;
typedef struct _ransac{

    /*a0X^2 + a1X + a2*/
    double consensus;
    double a0;
    double a1;
    double a2;

    _ransac( )
    {
        consensus = 0;
        a0 = 0;
        a1 = 0;
        a2 = 0;

    }

} Ransac;
Ransac pastL,pastR;
Ransac pastwindow;

void computeC2MC1(const Mat &R1, const Mat &tvec1, const Mat &R2, const Mat &tvec2,
                  Mat &R_1to2, Mat &tvec_1to2)
{
    //c2Mc1 = c2Mo * oMc1 = c2Mo * c1Mo.inv()
    R_1to2 = R2 * R1.t();
    tvec_1to2 = R2 * (-R1.t()*tvec1) + tvec2;
}
inline pt drawLine(Mat &targetMat,Mat &output,double &rho,double &theta,int &offset_x, int &offset_y,int &window_size){
    int next_Y,next_X;
    double realRho = rho *cos(theta/57.296);
    double tempRho;

    //cout<<offset_x<<endl;


    next_Y = offset_y + (window_size *480/640);

    tempRho = window_size *3/8/cos(theta/57.296);
    next_X = offset_x + tempRho*sin(theta/57.296)/cos(theta/57.296);
    // cout<<"draw"<<endl;
    if(offset_y==0){


        //  line(output,Point(offset_x, output.rows -offset_y),Point(next_X,output.rows-next_Y),Scalar(255),2);
        rectangle(output,Point(offset_x-window_size/2,output.rows-offset_y),Point(offset_x+window_size/2,output.rows-next_Y),Scalar(255),1);
        if(LRflag == 1){
            start_X2 = next_X;
        }
        else{
            start_X1 =  next_X;
        }

    }

    else{
        double variableTheta=0;
        if(pastTheta>theta){
            variableTheta= pastTheta- theta;

        }
        else{
            variableTheta=theta - pastTheta;
        }
        if(variableTheta>50){
            //cout<<"too big variance"<<endl;
            theta =pastTheta;
            tempRho = window_size *3/8/cos(theta/57.296);
            next_X = offset_x + tempRho*sin(theta/57.296)/cos(theta/57.296);
        }
        line(output,Point(offset_x, output.rows -offset_y- 1),Point(next_X,output.rows-next_Y),Scalar(255),2);
        rectangle(output,Point(offset_x-window_size/2,output.rows-offset_y),Point(offset_x+window_size/2,output.rows-next_Y),Scalar(255),1);

    }

    pt ans(next_X,next_Y);
    if(LRflag == 1){
        laneDetected[numOfLanePoint_R][0] = next_X-targetMat.cols/2;
        laneDetected[numOfLanePoint_R][1] = next_Y;
        laneDetected[numOfLanePoint_R++][2] = theta;
        // cout<<"Yoff"<<next_Y<<"Rtheta"<<theta<<endl;
    }
    else{
        laneDetected[numOfLanePoint_L][3] = next_X-targetMat.cols/2;
        laneDetected[numOfLanePoint_L][4] = next_Y;
        laneDetected[numOfLanePoint_L++][5] = theta;


    }
    pastTheta = theta;
    return ans;
}

inline void getLine(Mat &targetMat ,Mat&output,int &offset_x, int &offset_y,int &window_size) // get line with Hough TF && LSM
{
    //made by LEE IN HA 2019.07
    //cout<<"x: "<<offset_x<<"y: "<<offset_y<<endl;
    int MatRows = targetMat.rows;
    int halfOfWindow = window_size /2;

    if((offset_y >= MatRows- window_size*3/4)||(offset_x<=halfOfWindow)||(offset_x>=targetMat.cols-window_size))
        return; //end of Y or X
    int numPT=0;
    double AvgRho=0,SumRho=0;
    double AvgTheta=0, SumTheta=0;
    double tempTheta, tempRho;
    double dataArrTheta[50000]={};
    double dataArrRho[50000]={};

    uchar *data_output;
    uchar *Lidar_data_output;
    uchar *data_checking;
    double tempX,tempY;

    int Act_start_ptX,Act_start_ptY;
    Act_start_ptX = offset_x - halfOfWindow;
    Act_start_ptY = MatRows - offset_y;

    int wind_size_Y = window_size * 480/640;

    Mat window_ROI = targetMat(Rect(Act_start_ptX,Act_start_ptY-wind_size_Y,window_size,wind_size_Y));
    //    Mat targetMat_ROI= targetMat.clone();
    //  rectangle(targetMat_ROI,Point(Act_start_ptX,Act_start_ptY),Point(Act_start_ptX+window_size,Act_start_ptY+wind_size_Y),Scalar(255),1);
    //cout<<"next"<<endl;

    //imshow("targetMat",window_ROI);
    // imshow("targetMat with Roi",targetMat_ROI);
    int testcnt=0;
    vector<Point> TrueData;

    for(int i=window_ROI.rows-1;i>=0;i--)
    {
        data_output = window_ROI.ptr<uchar>(i);

        for(int j=0;j<window_ROI.cols;j++)
        {
            if(data_output[j]==0||j==0||i==0){continue; // because y factor is inversed in Mat
            }
            else{
                //                cout<<"data"<<testcnt<<"%:"<<data_output[j]<<",("<<j<<","<<i<<")"<<endl;
                TrueData.push_back(Point(j,wind_size_Y-i)); //slope can be infinite so do inverse
                testcnt++;
            }
        }
    }
    //  cout<<"find"<<testcnt<<endl;

    int numPoint = TrueData.size();//number of data
    int randIdx;

    int Xconsensus =0;
    int Yconsensus =0;

    int widMaxConsensus=0;
    double tempLSM =0;
    double widMinLSM=999999999;
    Ransac fittest_pt;
    Mat testing_line = window_ROI.clone();
    testing_line = Scalar(0);
    int clock1 = clock();

    if(numPoint>30){
        for(int i = 0; i<300;i++){ //ransac
            tempConsensus=0;
            //cout<<"npt"<<numPoint<<endl;
            if(i!=0){
                randIdx = rand()%numPoint;
                _x1 = TrueData[randIdx].x;
                _y1 = TrueData[randIdx].y;
                //cout<<"("<<_x1<<","<<_y1<<")"<<endl;
                randIdx = rand()%numPoint;
                _x2 = TrueData[randIdx].x;
                _y2 = TrueData[randIdx].y;
            }
            // cout<<"("<<_x2<<","<<_y2<<")"<<endl;
            if(_x1==_x2&&_y1==_y2)continue;
            else if(_x1==_x2){
                double a0;
                double a1;
                a0 = 0;
                a1 = _x1;
                int startPointX;
                for(int checkY = 0;checkY<wind_size_Y;checkY++){ //
                    startPointX = _x1;
                    data_checking = window_ROI.ptr<uchar>(testing_line.rows-checkY);
                    for(int checkX=startPointX-5;checkX<startPointX+6;checkX++){
                        // cause slope is calculated in inverted img, chage x and y again
                        //printf("(%d, %d)\n",checkX,checkY);
                        if(data_checking[checkX]){
                            tempConsensus++;

                            double sumTemp = (6-(checkX-startPointX))*(6-(checkX-startPointX));
                            tempLSM += sqrt(sumTemp);
                        }
                    }
                }

                //                     printf("!!!!!Xsame!!!!!!!");
                if(tempConsensus>=widMaxConsensus){
                    fittest_pt.a0 = a0;
                    fittest_pt.a1 = a1;
                    widMaxConsensus=tempConsensus;
                    widMinLSM =tempLSM;
                }
                if(tempLSM<=widMinLSM && (tempConsensus*100/numPoint)>40){
                    fittest_pt.a0 = a0;
                    fittest_pt.a1 = a1;
                    widMaxConsensus=tempConsensus;
                    widMinLSM =tempLSM;
                }
            }
            else if(_y1==_y2){
                //    printf("!!!!!Ysame!!!!!!!");
            }
            else{
                line(testing_line,Point(_x1, testing_line.rows - _y1),Point(_x2,testing_line.rows-_y2),Scalar(255),1);

                double a0;
                double a1; //y= a0*x+a1;
                a0 = (double)(_y2-_y1)/(double)(_x2-_x1);
                a1 = (double)(_y2-_y1)/(double)(_x2-_x1)*(double)(-_x1)+(double)_y1;
                int checkY; // from this X, counting the inline data
                int startPointX; // to avoid checkX is less than thershold
                if(a0>0.5||a0<-0.5){
                    for(checkY = 0;checkY<wind_size_Y;checkY++){ //

                        startPointX = ((double)checkY - (double)a1)/(double)a0;

                        if(startPointX <0)
                            continue;
                        else if(startPointX<5)
                            startPointX = 0;
                        else if(startPointX>window_size)continue;
                        else if(startPointX>= window_size-6)
                            startPointX = window_size-6;
                        data_checking = window_ROI.ptr<uchar>(testing_line.rows-checkY); // cause slope is calculated in inverted img, chage x and y again
                        for(int checkX=startPointX-5;checkX<startPointX+6;checkX++){

                            //printf("(%d, %d)\n",checkX,checkY);
                            if(data_checking[checkX]){
                                tempConsensus++;

                                double sumTemp = (6-(checkX-startPointX))*(6-(checkX-startPointX));
                                tempLSM += sqrt(sumTemp);
                            }
                        }
                        //cout<<checkX<<endl;
                    }
                    //                cout<<"asd"<<endl;
                }
                if(tempConsensus>=widMaxConsensus){
                    fittest_pt.a0 = a0;
                    fittest_pt.a1 = a1;
                    widMaxConsensus=tempConsensus;
                    widMinLSM =tempLSM;
                }

                //imshow("testing_Line",testing_line);
            }

        }
        //        printf("y=%lfX+%lf\n",fittest_pt.a0,fittest_pt.a1);
        //        printf("%d percent is inliner",widMaxConsensus*100/numPoint);
        //        cout<<endl;
    }
    else{
        // printf("low data\n");
    }
    int clock2 = clock();
    sumtime +=double(clock2-clock1)/CLOCKS_PER_SEC;
    cnttime++;
    avgtime = sumtime/(double)cnttime;
    //printf("%lf sec\n",avgtime);
    //cout<<endl;
    Mat smallran = window_ROI.clone();
    smallran = Scalar(0);
    int ranP1,ranP2,ranP22;
    //y= a0*x+a1;
    past_offset_x =offset_x;

    if(numPoint>30){
        pastwindow.a0=fittest_pt.a0;
        pastwindow.a1=fittest_pt.a1;
        if(fittest_pt.a0!=0){

            ranP1 = (-fittest_pt.a1)/fittest_pt.a0;
            ranP2 =  (testing_line.rows-fittest_pt.a1)/fittest_pt.a0;
            ranP22 =  ((testing_line.rows/2)-fittest_pt.a1)/fittest_pt.a0;

            if(offset_y!=0){
                int tempabs= abs(past_offset_x-(ranP1 +offset_x-window_size/2));
                if(tempabs<15){

                    line(smallran,Point(ranP1+offset_x, smallran.rows+offset_y),Point(ranP2+offset_x,0+offset_y),Scalar(255),1);
                    line(output,Point(ranP1+offset_x-window_size/2,targetMat.rows - offset_y),Point(ranP2+offset_x-window_size/2,targetMat.rows - (smallran.rows+offset_y)),Scalar(255),2);
                }
            }
            else{

                line(smallran,Point(ranP1+offset_x, smallran.rows+offset_y),Point(ranP2+offset_x,0+offset_y),Scalar(255),1);
                line(output,Point(ranP1+offset_x-window_size/2,targetMat.rows - offset_y),Point(ranP2+offset_x-window_size/2,targetMat.rows - (smallran.rows+offset_y)),Scalar(255),2);

            }
        }
        else{
            ranP2 = fittest_pt.a1;
            int tempabs= abs(past_offset_x-(ranP1 +offset_x-window_size/2));
            if(offset_y!=0){
                if(tempabs<15){
                    line(smallran,Point(fittest_pt.a1, smallran.rows),Point(fittest_pt.a1,0),Scalar(255),1);
                    line(output,Point(fittest_pt.a1+offset_x-window_size/2,targetMat.rows - offset_y),Point(fittest_pt.a1+offset_x-window_size/2,targetMat.rows - (smallran.rows+offset_y)),Scalar(255),2);
                    //        line(output,Point(fittest_pt.a1+offset_x-window_size/2, smallran.rows+offset_y),Point(fittest_pt.a1+offset_xoffset_x,0+offset_y),Scalar(255),2);
                }

            }
            else{

                line(smallran,Point(fittest_pt.a1, smallran.rows),Point(fittest_pt.a1,0),Scalar(255),1);
                line(output,Point(fittest_pt.a1+offset_x-window_size/2,targetMat.rows - offset_y),Point(fittest_pt.a1+offset_x-window_size/2,targetMat.rows - (smallran.rows+offset_y)),Scalar(255),2);


            }
        }
    }
    else{
        fittest_pt.a0=pastwindow.a0;
        fittest_pt.a1=pastwindow.a1;
        if(fittest_pt.a0!=0){
            ranP1 = (-fittest_pt.a1)/fittest_pt.a0;
            ranP2 =  (testing_line.rows-fittest_pt.a1)/fittest_pt.a0;
            ranP22 =  ((testing_line.rows/2)-fittest_pt.a1)/fittest_pt.a0;

            if(offset_y!=0){
                int tempabs= abs(past_offset_x-(ranP1 +offset_x-window_size/2));
                line(smallran,Point(ranP1+offset_x, smallran.rows+offset_y),Point(ranP2+offset_x,0+offset_y),Scalar(255),1);
                line(output,Point(ranP1+offset_x-window_size/2,targetMat.rows - offset_y),Point(ranP2+offset_x-window_size/2,targetMat.rows - (smallran.rows+offset_y)),Scalar(255),2);
            }
            else{

            }



        }
        else{
            ranP2 = fittest_pt.a1;
            int tempabs= abs(past_offset_x-(ranP1 +offset_x-window_size/2));
            if(offset_y!=0){
                line(smallran,Point(fittest_pt.a1, smallran.rows),Point(fittest_pt.a1,0),Scalar(255),1);
                line(output,Point(fittest_pt.a1+offset_x-window_size/2,targetMat.rows - offset_y),Point(fittest_pt.a1+offset_x-window_size/2,targetMat.rows - (smallran.rows+offset_y)),Scalar(255),2);
                //        line(output,Point(fittest_pt.a1+offset_x-window_size/2, smallran.rows+offset_y),Point(fittest_pt.a1+offset_xoffset_x,0+offset_y),Scalar(255),2)
            }
            else{

            }

        }
    }
    //    imshow("window ransac",smallran);
    //cout <<"NumPT"<<numPT<<endl;
    past_offset_x =offset_x;
    offset_x = ranP2 +offset_x-window_size/2;
    if(numPoint>30){

        if(offset_y!=0){

            int tempabs= abs(past_offset_x-(ranP1 +offset_x-window_size/2));
            if(tempabs<25){
                rectangle(output,Point(offset_x-window_size/2,output.rows-offset_y),Point(offset_x+window_size/2,output.rows-(offset_y+window_size*3/4)),Scalar(255),1);
                offset_y += window_size * 3/4 ;
                if(LRflag == 1){
                    laneDetected[numOfLanePoint_R][0] = offset_x-targetMat.cols/2;
                    laneDetected[numOfLanePoint_R++][1] = offset_y;
                    laneDetected[numOfLanePoint_R][0] = ranP22 +past_offset_x-window_size/2-targetMat.cols/2;
                    laneDetected[numOfLanePoint_R++][1] = offset_y-window_size * 3/8;
                    // cout<<"Yoff"<<next_Y<<"Rtheta"<<theta<<endl;
                }
                else{
                    laneDetected[numOfLanePoint_L][3] = offset_x-targetMat.cols/2;
                    laneDetected[numOfLanePoint_L++][4] = offset_y;
                    laneDetected[numOfLanePoint_L][3] = ranP22 +past_offset_x-window_size/2-targetMat.cols/2;
                    laneDetected[numOfLanePoint_L++][4] = offset_y-window_size * 3/8;


                }
                getLine(targetMat,output,offset_x,offset_y,window_size);
            }
        }
        else{
            //cout<<"offset_Y:"<<offset_y<<endl;

            //cout<<"next PT: "<<"x: "<<offset_x<<"y: "<<offset_y<<"wid"<<window_size<<endl;
            rectangle(output,Point(offset_x-window_size/2,output.rows-offset_y),Point(offset_x+window_size/2,output.rows-(offset_y+window_size*3/4)),Scalar(255),1);
            offset_y += window_size * 3/4 ;
            if(LRflag == 1){
                laneDetected[numOfLanePoint_R][0] = offset_x-targetMat.cols/2;
                laneDetected[numOfLanePoint_R++][1] = offset_y;
                laneDetected[numOfLanePoint_R][0] = ranP22 +past_offset_x-window_size/2-targetMat.cols/2;
                laneDetected[numOfLanePoint_R++][1] = offset_y-window_size * 3/8;
                // cout<<"Yoff"<<next_Y<<"Rtheta"<<theta<<endl;
            }
            else{
                laneDetected[numOfLanePoint_L][3] = offset_x-targetMat.cols/2;
                laneDetected[numOfLanePoint_L++][4] = offset_y;
                laneDetected[numOfLanePoint_L][3] = ranP22 +past_offset_x-window_size/2-targetMat.cols/2;
                laneDetected[numOfLanePoint_L++][4] = offset_y-window_size * 3/8;



            }
            getLine(targetMat,output,offset_x,offset_y,window_size);
            //fillPoly(Road_MaterialImg,Road_ppt,R_E_npt,1,Scalar(255),8);
        }
    }
    else{

    }
    //left

    //right
}
inline int checkReliability(){
    if(numOfLanePoint_L>2){
        if(numOfLanePoint_R>2)
            return 2;

        else
            return -1;
    }
    else{
        if(numOfLanePoint_R>2)
            return 1;
        else
            return 0;

    }
}
void check_con(Mat &src){
    //    Mat srtInv;
    //    bitwise_not(src,srcInv);
    double tempX=0.0625*2,tempY=0.08333*2;
    double Ycon[9] = {tempY,tempY,tempY,0,0,0,tempY,tempY,tempY};
    Mat Xdst,Ydst;
    Mat Xkernel(1,9,CV_32F);
    Mat Ykernel(3,3,CV_32F);
    //    cout<<Ykernel<<endl;
    float *data_temp;
    data_temp = Xkernel.ptr<float>(0);

    for(int i=0; i<9;i++){
        if(i ==4)continue;
        data_temp[i] = tempX;
    }
    data_temp[4]=0;
    data_temp = Ykernel.ptr<float>(0);
    for(int i=0; i<3;i++){
        data_temp[i] = tempY;
    }
    data_temp = Ykernel.ptr<float>(1);
    for(int i=0; i<3;i++){
        data_temp[i] = 0;
    }
    data_temp = Ykernel.ptr<float>(2);
    for(int i=0; i<3;i++){
        data_temp[i] = tempY;
    }
    //cout<<Xkernel<<endl;
    Mat ttttttt;

    filter2D(src, Xdst, -1, Xkernel, Point(-1, -1), 0, BORDER_DEFAULT);
    filter2D(src, Ydst, -1, Ykernel, Point(-1, -1), 0, BORDER_DEFAULT);
    ttttttt = Xdst+Ydst;
    threshold(ttttttt,ttttttt,240,255,THRESH_BINARY);

    //    imshow("befofofofofofofo32ffo",Xdst);
    //imshow("befofofofofofof23offo",Ydst);

    //imshow("befofofofofofof1offo",ttttttt);
}

void LanefilterConv(Mat &src,Mat &dst){
    Mat Lkernel,Rkernel;

    Lkernel = Mat::zeros(1, 430, CV_32F);
    Rkernel = Mat::zeros(1, 430, CV_32F);// normalized box filter의 커널 만들기.
    //    Lkernel *=-0.5;
    //    Rkernel *=-0.5;
    Mat Llane,Rlane, Ltemp,Rtemp;

    float *data_temp;
    data_temp = Lkernel.ptr<float>(0);
    for(int i = 0;i<4;i++){
        //        data_temp[i] = (float)(i+1)/5.0/10.0;
        //        data_temp[i+211] = data_temp[i];

        //        data_temp[8-i] = (float)(i+1)/5.0/10.0;
        //        data_temp[8-i+211] = data_temp[8-i];

        data_temp[i] = 1.0/10.0;
        data_temp[i+211] = data_temp[i];

        data_temp[8-i] = 1.0/10.0;
        data_temp[8-i+211] = data_temp[8-i];
    }

    data_temp[4] = 1.0/10.0;
    data_temp[215] = 1.0/10.0;

    data_temp = Rkernel.ptr<float>(0);
    for(int i = 0;i<4;i++){
        //        data_temp[i+211] = (float)(i+1)/5.0/10.0;
        //        data_temp[i+421] = (float)(i+1)/5.0/10.0;

        //        data_temp[8-i+211] = (float)(i+1)/5.0/10.0;
        //        data_temp[8-i+421] = (float)(i+1)/5.0/10.0;
        data_temp[i+211] = 1.0/10.0;
        data_temp[i+421] = 1.0/10.0;

        data_temp[8-i+211] = 1.0/10.0;
        data_temp[8-i+421] = 1.0/10.0;
    }
    //    int srcCols = src.Cols;
    //    int srcRaws = src.Raws;
    //    for(int y=0;y<srcRaws;y++){
    //        data_temp = Lkernel.src<uint>(0);
    //        for(int x=0;x<srcCols;x++){

    //        }
    //    }

    data_temp[215] = 1.0/10.0;
    data_temp[425] = 1.0/10.0;

    /// Apply filter
    // cout <<kernel<<endl;
    //    filter2D(src, dst, -1, Lkernel, Point(-1, -1), 0, BORDER_DEFAULT);
    filter2D(src, Llane, -1, Lkernel, Point(-1, -1), 0, BORDER_DEFAULT);
    filter2D(src, Rlane, -1, Rkernel, Point(-1, -1), 0, BORDER_DEFAULT);

    //namedWindow("Before filtering", CV_WINDOW_AUTOSIZE);

    //imshow("Before filtering", src);


    threshold(Llane,Llane,240,255,THRESH_BINARY);
    threshold(Rlane,Rlane,240,255,THRESH_BINARY);

    //namedWindow("filter2D Demo", CV_WINDOW_AUTOSIZE);

    //imshow("after Lfiltering", Llane);
    imshow("after Rfiltering", Rlane);
    imshow("after filtering", Llane+Rlane);

    dst = Llane+Rlane;
}

void Cam_LidarImg(const  sensor_msgs::Image &subImgMsgs)   //bag test
{
    //cout<<"start"<<endl;

    if(subImgMsgs.data.size())
    {

        //        cout<<"start"<<endl;
        LidarImgPtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        LidarImg = LidarImgPtr->image;

        LidarImgPtr->image = LidarImg;
        flip(LidarImg,LidarImg,1);

        //("oh yeah",LidarImg);



        start_flag=1;

    }
}
void subImgCallback(const  sensor_msgs::CompressedImageConstPtr &subImgMsgs)   //bag test
{
    if(subImgMsgs->data.size())
    {
        if(start_flag==1){
            //cout<<"start"<<endl;
            start_flag=0;
            rawImage = imdecode(Mat(subImgMsgs->data),1);



            flip(rawImage,rawImage,0);

            cvtColor(rawImage,GrayImg,COLOR_BGR2GRAY);


            Mat rawWarpImg,warpImg;
            Size warpSize(640,360);

            //       namedWindow("White binary img");
            //       createTrackbar("WHigh Hue","White binary img",&Whigh_H,180);
            //       createTrackbar("WHigh Sat","White binary img",&Whigh_S,255);
            //       createTrackbar("WHigh Val","White binary img",&Whigh_V,255);
            //       createTrackbar("Wlow Hue","White binary img",&Wlow_H,180);
            //       createTrackbar("Wlow Sat","White binary img",&Wlow_S,255);
            //       createTrackbar("Wlow Val","White binary img",&Wlow_V,255);

            //            namedWindow("Yellow binary img");
            //            createTrackbar("YHigh Hue","Yellow binary img",&Yhigh_H,180);
            //            createTrackbar("YHigh Sat","Yellow binary img",&Yhigh_S,255);
            //            createTrackbar("YHigh Val","Yellow binary img",&Yhigh_V,255);
            //            createTrackbar("Ylow Hue","Yellow binary img",&Ylow_H,180);
            //            createTrackbar("Ylow Sat","Yellow binary img",&Ylow_S,255);
            //            createTrackbar("Ylow Val","Yellow binary img",&Ylow_V,255);

            //            namedWindow("Blue binary img");
            //            createTrackbar("BHigh Hue","Blue binary img",&Bhigh_H,180);
            //            createTrackbar("BHigh Sat","Blue binary img",&Bhigh_S,255);
            //            createTrackbar("BHigh Val","Blue binary img",&Bhigh_V,255);
            //            createTrackbar("Blow Hue","Blue binary img",&Blow_H,180);
            //            createTrackbar("Blow Sat","Blue binary img",&Blow_S,255);
            //            createTrackbar("Blow Val","Blue binary img",&Blow_V,255);

            //            namedWindow("rawImage");
            //            createTrackbar("raw","rawImage",&_a,500);
            //            createTrackbar("raw2","rawImage",&_b,500);
            //            createTrackbar("raw3","rawImage",&_c,500);
            //            createTrackbar("raw---","rawImage",&__a,500);
            //            createTrackbar("raw2---","rawImage",&__b,500);
            //            createTrackbar("raw3--","rawImage",&__c,500);

            //            createTrackbar("raw4","rawImage",&_a2,5000);
            //            createTrackbar("raw5","rawImage",&_b2,5000);
            //            createTrackbar("raw6","rawImage",&_c2,5000);
            //            createTrackbar("raw7---","rawImage",&__a2,5000);
            //            createTrackbar("raw8---","rawImage",&__b2,5000);
            //            createTrackbar("raw9--","rawImage",&__c2,5000);


            double c[9] = {297.491371, 0, 278.022229,
                           0, 295.658647, 295.511134,
                           0, 0, 1};
            Mat cameraMatrix(3, 3, CV_64FC1, c);
            double d[4] = {0.005902 ,-0.015082, 0.003146, -0.000331};
            Mat distCoeffs(4, 1, CV_64FC1, d);

            double r[3] = {-0.68+(double)(_a-__a)/(double)2000,
                           -0.67+(double)(_b-__b)/(double)2000,
                           -1.476+(double)(_c-__c)/(double)2000};

            Mat rvec(3, 1, CV_64FC1, r);
            double t[3] = {-4.7+(double)(_a2-__a2)/(double)10,
                           -100.4+(double)(_b2-__b2)/(double)10,
                           162+(double)(_c2-__c2)/(double)10};
            Mat tvec(3, 1, CV_64FC1, t);


            Mat R_desired = (Mat_<double>(3,3) <<
                             0, 1, 0,
                             -1, 0, 0,
                             0, 0, 1);
            Mat R;

            Rodrigues(rvec, R);
            Mat normal = (Mat_<double>(3,1) << 0, 0, 1);
            Mat normal1 = R*normal;

            Mat origin(3, 1, CV_64F, Scalar(0));


            Mat origin1 = R*origin + tvec;

            double d_inv1 = 1.0 / normal1.dot(origin1);
            Mat R_1to2, tvec_1to2;
            Mat tvec_desired = tvec.clone();
            computeC2MC1(R, tvec, R_desired, tvec_desired, R_1to2, tvec_1to2);
            Mat H = R_1to2 + d_inv1 * tvec_1to2*normal1.t();
            H = cameraMatrix * H * cameraMatrix.inv();
            H = H/H.at<double>(2,2);
            //std::cout << "H:\n" << H << std::endl;

            warpPerspective(rawImage, rawWarpImg, H, warpSize);
            warpPerspective(LidarImg, LidarImg_warp, H, warpSize);
            flip(rawWarpImg,rawWarpImg,1);
            flip(LidarImg_warp,LidarImg_warp,1);

            //hconcat(rawImage, rawWarpImg, compare);
            //imshow("Bird eye view", compare);

            cvtColor(rawWarpImg,rstImg,COLOR_BGR2HSV);
            imshow("assadasd",rawWarpImg);
            imshow("assadasd",rawImage);

            flip(rstImg,rstImg,1);

            //imshow("213124",LidarImg_warp);
            inRange(rstImg,Scalar(Wlow_H,Wlow_S,Wlow_V),Scalar(Whigh_H,Whigh_S,Whigh_V),hsvImgWhite);
            inRange(rstImg,Scalar(Ylow_H,Ylow_S,Ylow_V),Scalar(Yhigh_H,Yhigh_S,Yhigh_V),hsvImgYellow);
            inRange(rstImg,Scalar(Blow_H,Blow_S,Blow_V),Scalar(Bhigh_H,Bhigh_S,Bhigh_V),hsvImgBlue);

            //            fillPoly(LMaterialImg,Lppt,Lnpt,1,Scalar(255),8);
            //            bitwise_and(hsvImgWhite,RMaterialImg,hsvImgWhite);


            add(hsvImgWhite,hsvImgYellow,hsvImg);
            add(hsvImg,hsvImgBlue,hsvImg);
            flip(hsvImgBlue,hsvImgBlue,1);
            flip(hsvImgYellow,hsvImgYellow,1);

            flip(rstImg,rstImg,1);

            //imshow("hhssvv img",rstImg);
            //imshow("Blue binary img",hsvImgBlue);
            //imshow("Yellow binary img",hsvImgYellow);
            //imshow("sadsadhsv img",rawImage);
            //            imshow("Rhsv img",hsvImgWhite);

            //            laneConerOutput[0] = Point2f(0,0);
            //            laneConerOutput[1] = Point2f(rawImage.cols-1,0);
            //            laneConerOutput[2] = Point2f(rawImage.cols-1,rawImage.rows-1);
            //            laneConerOutput[3] = Point2f(0,rawImage.rows-1);
            Mat homoWarp;
            Mat sobelX;
            homoWarp = findHomography(homoInput,homoOutput);
            //warpPerspective(rawImage,rawWarpImg,homoWarp,warpSize);

            warpMat = getPerspectiveTransform(laneConerInput,laneConerOutput);
            warpPerspective(hsvImg,warpImg,warpMat,warpImg.size());

            aruco::drawAxis(rawImage, cameraMatrix, distCoeffs, rvec, tvec, 50);
            MaterialImg = warpImg.clone();// img for set ROI
            MaterialImg = Scalar(0);
            fillPoly(MaterialImg,ppt,npt,1,Scalar(255),8);

            //imshow("rawImage",rawImage);

            //imshow("warp",hsvImg);
            cvtColor(rawWarpImg,GrayImg,COLOR_BGR2GRAY);

            warpImg = hsvImg;
            flip(warpImg,warpImg,1);

            //            Sobel(GrayImg,sobelX,CV_8U,1,0,3,1,3);
            //            threshold(sobelX,sobelX,100,255,THRESH_BINARY);
            //            imshow("As",sobelX);
            Mat convolLaneFilter= warpImg.clone();
            //            convolLaneFilter = Scalar(0);
            //LanefilterConv(warpImg,convolLaneFilter);
            // LanefilterConv(LidarImg_warp,LidarImg_warp);
            //            warpImg = convolLaneFilter;

            cvtColor(LidarImg_warp,LidarImg_warp,COLOR_BGR2GRAY);
            //            bitwise_not(Road_MaterialImg,Road_Of_Interest);
            //            bitwise_and(Road_Of_Interest,warpImg,convolLaneFilter);
            //            imshow("road of interest",Road_Of_Interest);
            imshow("please",warpImg);
            // check_con(warpImg);
            //            Sobel(GrayImg,sobelX,CV_8U,1,0,3,1,0.7);
            //            imshow("As",sobelX);
            //warpImg += sobelX;
            //cout<<"old fin"<<endl;


            //imshow("binary",hsvImgYellow+hsvImgWhite);
            //  if(start_flag == 0){

            int init_MAX_Y = 200;

            dilate(warpImg,warpImg,Mat());
            erode(warpImg,warpImg,Mat());
            //            warpImg += sobelX;
            //imshow("erase it",warpImg);
            //cout<<"closed fin"<<endl;

            Mat Lane_init_ROI = convolLaneFilter(Rect(0,convolLaneFilter.rows - init_MAX_Y,convolLaneFilter.cols,init_MAX_Y));

            threshold( Lane_init_ROI, Lane_init_ROI,100,255,THRESH_BINARY );
            circle(Lane_init_ROI,Point(car_position,194),3,Scalar(255));

            imshow("Roi",Lane_init_ROI);

            Mat sum_show = Mat::zeros(warpImg.rows,warpImg.cols,warpImg.type());
            Mat leftLanedetect = Mat::zeros(warpImg.rows,warpImg.cols,warpImg.type());
            Mat rightLanedetect = Mat::zeros(warpImg.rows,warpImg.cols,warpImg.type());

            uchar sum_data_arr[1000]={};
            uchar *data_output;
            uchar *data_temp;
            data_output = sum_show.ptr<uchar>(0);
            for(int i = 0;i<Lane_init_ROI.rows-10;i++){
                data_temp = Lane_init_ROI.ptr<uchar>(i);
                for(int j = 0; j<Lane_init_ROI.cols;j++){
                    if(data_temp[j]!=0){

                        sum_data_arr[j]++;
                        data_output = sum_show.ptr<uchar>(sum_data_arr[j]);
                        data_output[j] =255;

                    }
                }
            }
            imshow("sum",sum_show);


            data_temp = sum_show.ptr<uchar>(0);
            int max1 =0,max2=0;
            int sumtemp=0,sumtemp2=0;
            int varianceTemp=0;
            if(detectFlag ==1){
                if(car_position !=320){
                    for(int i=car_position-50;i>car_position-150&&i>10;i--)
                    {
                        sumtemp2 =0;
                        sumtemp =0;
                        for(int j=i-3;j<i+3;j++){
                            sumtemp +=sum_data_arr[j];
                        }

                        if(max1 <sumtemp && sumtemp>500){
                            max1 = sumtemp;
                            for(int j=i-10;j<i+10;j++){
                                if(sum_data_arr[j]<5)break;
                                varianceTemp +=sqrt((i-j)*(i-j))*sum_data_arr[j];
                                sumtemp2+=sum_data_arr[j];
                            }
                            if(sumtemp2){
                               // cout<<varianceTemp/sumtemp2<<endl;
                            }

                            start_X1 = i;

                        }
                        if(sumtemp){
                            cout<<"X:"<<i<<" sumtemp"<<sumtemp<<endl;
                        }
                        if(max1>500)
                            break;
                        //cout<<"working"<<i<<endl;
                    }
                    for(int i=car_position+50;i<sum_show.cols && i<car_position+139;i++)
                    {
                        sumtemp =0;
                        for(int j=i-3;j<i+3;j++){
                            sumtemp +=sum_data_arr[j];
                        }
                        if(max2 <sumtemp&& sumtemp>700){
                            max2 = sumtemp;
                            start_X2 = i;
                        }
                        if(max2>700)
                            break;

                    }
                }
                else{
                    for(int i=car_position-10;i>car_position-210&&i>3;i--)
                    {
                        sumtemp =0;
                        for(int j=i-3;j<i+3;j++){
                            sumtemp +=sum_data_arr[j];
                        }
                        if(max1 <sumtemp){
                            max1 = sumtemp;
                            start_X1 = i;
                        }
                        if(max1>700)
                            break;
                        //cout<<"working"<<i<<endl;
                    }
                    for(int i=car_position+10;i<sum_show.cols && i<car_position+207;i++)
                    {
                        sumtemp =0;
                        for(int j=i-3;j<i+3;j++){
                            sumtemp +=sum_data_arr[j];
                        }
                        if(max2 <sumtemp){
                            max2 = sumtemp;
                            start_X2 = i;
                        }
                        if(max2>700)
                            break;

                    }
                }
                detectFlag =0;
                // cout<<"max1:"<<max1<<" max2:"<<max2<<endl;

            }
            // start_X2 = start_X1 + 195;
            //cout << "start_X1:"<<start_X1<<" start_X2:"<<start_X2<<start_X2-start_X1<<endl;

            //}
            int window_SZ =108;
            start_Y1=0;
            start_Y2=0;
            numOfLanePoint_L =0;
            LRflag = -1;
            pastTheta = 0;
            getLine(warpImg,leftLanedetect,start_X1,start_Y1,window_SZ);


            start_Y1=0;
            start_Y2=0;
            numOfLanePoint_R =0;
            LRflag = 1;
            pastTheta = 0;
            getLine(warpImg,rightLanedetect,start_X2,start_Y2,window_SZ);
            //cout<<"endendend"<<endl;
            //imshow("wwewewe",rightLanedetect+leftLanedetect);
            int cloudNum=0;

            Mat X = Mat::zeros(3,3,CV_32F);
            Mat coef = Mat::zeros(1,3,CV_32F);
            Mat X_inverse = Mat::zeros(3,3,CV_32F);
            Mat Y = Mat::zeros(3,1,CV_32F);
            float a0,a1,a2;
            float *data_output_ransac_R;
            float *data_output_ransac_L;


            int distance=0;
            int consensus=0;
            reliability = checkReliability();
            Ransac L_fittest,R_fittest;
            if(reliability >0){ //it means right(1) or both(2)
                for(int i = 0;i<numOfLanePoint_R-2;i++){
                    for(int j=i+1;j<numOfLanePoint_R-1;j++){
                        for(int k=j+1;k<numOfLanePoint_R;k++){
                            consensus=0;
                            if(i>=numOfLanePoint_R||k>=numOfLanePoint_R||j>=numOfLanePoint_R)
                                continue;
                            //cout<<"i:"<<i<<" j:"<<j<<" k"<<k<<endl;
                            data_output_ransac_R = X.ptr<float>(0);

                            data_output_ransac_R[0] = laneDetected[i][1]*laneDetected[i][1];
                            data_output_ransac_R[1] = laneDetected[i][1];
                            data_output_ransac_R[2] = 1;

                            data_output_ransac_R = X.ptr<float>(1);
                            data_output_ransac_R[0] = laneDetected[j][1]*laneDetected[j][1];
                            data_output_ransac_R[1] = laneDetected[j][1];
                            data_output_ransac_R[2] = 1;

                            data_output_ransac_R = X.ptr<float>(2);
                            data_output_ransac_R[0] = laneDetected[k][1]*laneDetected[k][1];
                            data_output_ransac_R[1] = laneDetected[k][1];
                            data_output_ransac_R[2] = 1;


                            data_output_ransac_R = Y.ptr<float>(0);
                            data_output_ransac_R[0] = laneDetected[i][0];

                            data_output_ransac_R = Y.ptr<float>(1);
                            data_output_ransac_R[0] = laneDetected[j][0];

                            data_output_ransac_R = Y.ptr<float>(2);
                            data_output_ransac_R[0] = laneDetected[k][0];

                            X_inverse = X.inv();
                            coef = X_inverse * Y;

                            //                    cout<<"x1:"<<laneDetected[i][0]<<" y1: "<<laneDetected[i][1];
                            //                    cout<<" x2:"<<laneDetected[j][0]<<" y2: "<<laneDetected[j][1];
                            //                    cout<<" x3:"<<laneDetected[k][0]<<" y3: "<<laneDetected[k][1]<<endl;
                            //                    cout<<" coef "<<coef<<endl;

                            data_output_ransac_R = coef.ptr<float>(0);
                            a0 = data_output_ransac_R[0];

                            data_output_ransac_R = coef.ptr<float>(1);
                            a1 = data_output_ransac_R[0];

                            data_output_ransac_R = coef.ptr<float>(2);
                            a2 = data_output_ransac_R[0];


                            int  quadraticX , quadraticY,slope;

                            for(int idx = 0; idx<numOfLanePoint_R;idx++){

                                quadraticX = laneDetected[idx][1];//because mat is transposed
                                quadraticY = a0*quadraticX*quadraticX + a1*quadraticX +a2;


                                distance = abs((int)(laneDetected[idx][0] - quadraticY));
                                //cout<<"dis:"<<distance<<endl;

                                if(distance <10){
                                    //cout<<"c+";
                                    consensus++;
                                }

                            }
                            if(consensus>R_fittest.consensus){
                                R_fittest.consensus = consensus;
                                R_fittest.a0 = a0;
                                R_fittest.a1 = a1;
                                R_fittest.a2 = a2;

                            }
                        }
                    }
                }
            }
            // cout<<"Rconsensus"<<consensus<<endl;
            if(reliability == -1 || reliability == 2){
                for(int i = 0;i<numOfLanePoint_L-2;i++){
                    for(int j=i+1;j<numOfLanePoint_L-1;j++){
                        for(int k=j+1;k<numOfLanePoint_L;k++){
                            if(i>=numOfLanePoint_L||k>=numOfLanePoint_L||j>=numOfLanePoint_L)
                                continue;
                            consensus=0;
                            //cout<<"i:"<<i<<" j:"<<j<<" k"<<k<<endl;
                            data_output_ransac_L = X.ptr<float>(0);

                            data_output_ransac_L[0] = laneDetected[i][4]*laneDetected[i][4];
                            data_output_ransac_L[1] = laneDetected[i][4];
                            data_output_ransac_L[2] = 1;

                            data_output_ransac_L = X.ptr<float>(1);
                            data_output_ransac_L[0] = laneDetected[j][4]*laneDetected[j][4];
                            data_output_ransac_L[1] = laneDetected[j][4];
                            data_output_ransac_L[2] = 1;

                            data_output_ransac_L = X.ptr<float>(2);
                            data_output_ransac_L[0] = laneDetected[k][4]*laneDetected[k][4];
                            data_output_ransac_L[1] = laneDetected[k][4];
                            data_output_ransac_L[2] = 1;



                            data_output_ransac_L = Y.ptr<float>(0);
                            data_output_ransac_L[0] = laneDetected[i][3];

                            data_output_ransac_L = Y.ptr<float>(1);
                            data_output_ransac_L[0] = laneDetected[j][3];

                            data_output_ransac_L = Y.ptr<float>(2);
                            data_output_ransac_L[0] = laneDetected[k][3];

                            X_inverse = X.inv();
                            coef = X_inverse * Y;


                            data_output_ransac_L = coef.ptr<float>(0);
                            a0 = data_output_ransac_L[0];

                            data_output_ransac_L = coef.ptr<float>(1);
                            a1 = data_output_ransac_L[0];

                            data_output_ransac_L = coef.ptr<float>(2);
                            a2 = data_output_ransac_L[0];


                            int  quadraticX , quadraticY,slope;

                            for(int idx = 0; idx<numOfLanePoint_L;idx++){

                                quadraticX = laneDetected[idx][4];//because mat is transposed
                                quadraticY = a0*quadraticX*quadraticX + a1*quadraticX +a2;

                                // cout<<quadraticY<<" "<<laneDetected[idx][3]<<endl;
                                distance = abs((int)(laneDetected[idx][3] - quadraticY));
                                //                        cout<<"dis:"<<distance<<endl;

                                if(distance <10){
                                    //                            cout<<"c+";

                                    consensus++;
                                }

                            }
                            if(consensus>L_fittest.consensus){
                                L_fittest.consensus = consensus;
                                L_fittest.a0 = a0;
                                L_fittest.a1 = a1;
                                L_fittest.a2 = a2;

                            }


                        }
                    }
                }
            }
            // cout<<"calcul ransac fin"<<endl;
            //cout<<"Lconsensus"<<consensus<<endl;
            //cout << "L_fittest: a0:"<<R_fittest.a0<<" a1:"<<R_fittest.a1<<" a2:"<<R_fittest.a2<<endl;
            Mat ransacDetected = Mat::zeros(warpImg.rows,warpImg.cols,warpImg.type());
            //cout<<reliability<<endl;
            reliability_Cnt_Num++;
            if(L_fittest.a0 >0.001 || L_fittest.a0<-0.001 || L_fittest.a0==0)
            {
                if(R_fittest.a0 >0.001 ||R_fittest.a0<-0.001 || R_fittest.a0==0){
                    reliability =0;
                }
                else{
                    reliability =1;
                }
            }
            else{
                if(R_fittest.a0 >0.001 ||R_fittest.a0<-0.001 || R_fittest.a0==0){
                    reliability =-1;
                }

            }
            if(reliability == 1){
                // cout<<"L is fake"<<endl;

                L_fittest.a0 = R_fittest.a0;
                L_fittest.a1 = R_fittest.a1;
                L_fittest.a2 = R_fittest.a2-215;
                pastL.a0 = L_fittest.a0;
                pastL.a1 = L_fittest.a1;
                pastL.a2 = L_fittest.a2;
                pastR.a0 = R_fittest.a0;
                pastR.a1 = R_fittest.a1;
                pastR.a2 = R_fittest.a2;
                reliability_Cnt_L++;
                detectFlag =1;

            }
            else if(reliability == -1){
                // cout<<"R is fake"<<endl;

                R_fittest.a0 = L_fittest.a0;
                R_fittest.a1 = L_fittest.a1;
                R_fittest.a2 = L_fittest.a2+215;
                pastL.a0 = L_fittest.a0;
                pastL.a1 = L_fittest.a1;
                pastL.a2 = L_fittest.a2;
                pastR.a0 = R_fittest.a0;
                pastR.a1 = R_fittest.a1;
                pastR.a2 = R_fittest.a2;
                reliability_Cnt_R++;
                detectFlag =1;

            }
            else if(reliability == 0){
//                L_fittest.a0 = pastL.a0;
//                L_fittest.a1 = pastL.a1;
//                L_fittest.a2 = pastL.a2;
//                R_fittest.a0 = pastR.a0;
//                R_fittest.a1 = pastR.a1;
//                R_fittest.a2 = pastR.a2;
                reliability_Cnt_Non++;
                detectFlag =1;
                // cout<<"Error!!!!!!!"<<endl;
            }
            else{
                pastL.a0 = L_fittest.a0;
                pastL.a1 = L_fittest.a1;
                pastL.a2 = L_fittest.a2;
                pastR.a0 = R_fittest.a0;
                pastR.a1 = R_fittest.a1;
                pastR.a2 = R_fittest.a2;
                reliability_Cnt_All++;
            }
            //cout<<"reliablity: "<<reliability_Cnt_All*100/reliability_Cnt_Num<<endl;
            //cout<<"convert ransac fin"<<endl;
            int CurveX,CurveY;

            static pcl::PointCloud<pcl::PointXYZ> Lane;
            static pcl::PointCloud<pcl::PointXYZ> waypoint;
            int Road_temp=0;
            //cout<<"slope:"<<L_fittest.a0<<endl;
            if(reliability!=0){
                for(int i = 1;i<ransacDetected.rows;i++){
                    int temp;
                    CurveX = i;
                    //Draw Right

                    a0 = R_fittest.a0;
                    a1 = R_fittest.a1;
                    a2 = R_fittest.a2;
                    //cout<<a0<<endl;
                    CurveY = a0 *CurveX*CurveX + a1 * CurveX + a2+ ransacDetected.cols /2;
                    temp = CurveY;

                    if(CurveY<ransacDetected.cols && CurveY>0 ){
                        pcl::PointXYZ PTtemp;

                        data_temp = ransacDetected.ptr<uchar>(ransacDetected.rows-i);
                        //cout<<"CurveY"<<ransacDetected.rows-i<<endl;
                        data_temp[CurveY] =125;
                        PTtemp.x = (CurveY-ransacDetected.cols/2)*3.0/190.0;
                        PTtemp.y = CurveX*5.0/300.0;
                        PTtemp.z = 0;
                        Lane.push_back(PTtemp);
                        cloudNum++;
                        if(i%40==39){

                            Road_erase[0][Road_temp]=Point(CurveY-40,i);

                            Road_erase[0][17-Road_temp++]=Point(CurveY-175,i);
                        }
                        if(reliability == 2){
                            right_ran_pt[ransacDetected.rows - i] = CurveY;
                        }
                    }
                    //Draw Left
                    a0 = L_fittest.a0;
                    a1 = L_fittest.a1;
                    a2 = L_fittest.a2;

                    CurveY = a0 *CurveX*CurveX + a1 * CurveX + a2+ ransacDetected.cols /2;
                    if(CurveY<ransacDetected.cols && CurveY>0){
                        pcl::PointXYZ PTtemp1;
                        pcl::PointXYZ PTtemp2;

                        data_temp = ransacDetected.ptr<uchar>(ransacDetected.rows-i);
                        data_temp[CurveY] =125;
                        PTtemp1.x = (CurveY-ransacDetected.cols/2)*3.0/190.0;
                        PTtemp1.y = CurveX*5.0/300.0;
                        PTtemp1.z = 0;
                        cloudNum++;

                        Lane.push_back(PTtemp1);
                        PTtemp2.x = (Lane.points[cloudNum-2].x+Lane.points[cloudNum-1].x)/2;
                        PTtemp2.y =CurveX*5.0/300.0;
                        PTtemp2.z =0;
                        waypoint.push_back(PTtemp2);
                        if(CurveX == 10){
                            int tempA=Lane.points[cloudNum-2].x*190/3+320;
                            int tempB=Lane.points[cloudNum-1].x*190/3+320;
                            past_car_position = car_position;
                            car_position = (tempA+tempB)/2;
                            if(past_car_position<car_position){
                                car_position = past_car_position+1;
                            }
                            else if(past_car_position>car_position){
                                car_position = past_car_position-1;
                            }
                            // cout<<car_position<<endl;
                        }
                        if(reliability == 2){
                            double artan;
                            double slope = 2*a0 + a1;
                            if(slope < 0){
                                slope *= -1;
                            }
                            // cout<< "y:"<<ransacDetected.rows - i << " slope:"<<slope<<endl;
                            artan = atan(slope);
                            left_ran_pt[ransacDetected.rows - i] = CurveY * cos(artan);

                        }
                        //                        cout<<"diff"<<right_ran_pt[ransacDetected.rows -i] -left_ran_pt[ransacDetected.rows -i]<<endl;
                    }

                }

                dilate(ransacDetected,ransacDetected,Mat());
                dilate(ransacDetected,ransacDetected,Mat());
                dilate(ransacDetected,ransacDetected,Mat());

            }
            else{
                car_position =320;
            }

            //            for(int i = 1;i<ransacDetected.rows;i++){
            //                int temp;
            //                CurveX = i;
            //                //Draw Right
            //                a0 = R_fittest.a0;
            //                a1 = R_fittest.a1;
            //                a2 = R_fittest.a2;
            //                CurveY = a0 *CurveX*CurveX + a1 * CurveX + a2+ ransacDetected.cols /2;
            //                temp = CurveY;

            //                if(CurveY<ransacDetected.cols && CurveY>0 ){
            //                    pcl::PointXYZ PTtemp;

            //                    data_temp = ransacDetected.ptr<uchar>(ransacDetected.rows-i);
            //                    //cout<<"CurveY"<<ransacDetected.rows-i<<endl;
            //                    data_temp[CurveY] =125;
            //                    PTtemp.x = (CurveY-ransacDetected.cols/2)*3.0/215.0;
            //                    PTtemp.y = CurveX*3.0/190.0;
            //                    PTtemp.z = 0;
            //                    Lane.push_back(PTtemp);
            //                    cloudNum++;
            //                    if(i%40==1){

            //                        Road_erase[0][Road_temp]=Point(CurveY-40,i);

            //                        Road_erase[0][17-Road_temp++]=Point(CurveY-175,i);
            //                    }
            //                }
            //                //Draw Left
            //                a0 = L_fittest.a0;
            //                a1 = L_fittest.a1;
            //                a2 = L_fittest.a2;

            //                CurveY = a0 *CurveX*CurveX + a1 * CurveX + a2+ ransacDetected.cols /2;
            //                if(CurveY<ransacDetected.cols && CurveY>0){
            //                    pcl::PointXYZ PTtemp1;
            //                    pcl::PointXYZ PTtemp2;

            //                    data_temp = ransacDetected.ptr<uchar>(ransacDetected.rows-i);
            //                    data_temp[CurveY] =125;
            //                    PTtemp1.x = (CurveY-ransacDetected.cols/2)*3.0/190.0;
            //                    PTtemp1.y = CurveX*3.0/190.0;
            //                    PTtemp1.z = 0;
            //                    cloudNum++;;
            //                    Lane.push_back(PTtemp1);
            //                    PTtemp2.x = (Lane.points[cloudNum-2].x+Lane.points[cloudNum-1].x)/2;
            //                    PTtemp2.y =CurveX*3.0/190.0;
            //                    PTtemp2.z =0;
            //                    waypoint.push_back(PTtemp2);

            //                }

            //            }








            //            Road_MaterialImg =Scalar(0);
            //            fillPoly(Road_MaterialImg,Road_ppt,R_E_npt,1,Scalar(255),8);

            imshow("as213d",rightLanedetect+leftLanedetect+warpImg);
            imshow("asd",ransacDetected);

            pcl::toROSMsg(Lane,lane_output);
            pcl::toROSMsg(waypoint,way_output);
            //cout<<"pcltoRos fin"<<endl;

            lane_output.header.frame_id = "point_cloud"; //
            way_output.header.frame_id = "point_cloud"; // waypoint
            lane_tracking.publish(lane_output);
            lane_waypoint.publish(way_output);
            // cout<<"publish fin"<<endl;
            waypoint.clear();
            Lane.clear();
            waitKey(10);

            //cout<<"end"<<endl;
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Camera");
    srand(clock());
    Mat tmp1(640,480,CV_8UC3);
    LidarImg = tmp1;

    Road_Of_Interest = Mat::zeros(360,640,CV_8U);
    Road_MaterialImg = Mat::zeros(360,640,CV_8U);

    //640, 480
    lineForm[0][0]=Point(140,0);
    lineForm[0][1]=Point(140,359);
    lineForm[0][2]=Point(500,359);
    lineForm[0][3]=Point(500,0);



    homoOutput.push_back(Point2f(0,0));
    homoOutput.push_back(Point2f(639,0));
    homoOutput.push_back(Point2f(639,479));
    homoOutput.push_back(Point2f(0,479));

    homoInput.push_back(Point2f(18,0));
    homoInput.push_back(Point2f(101,0));
    homoInput.push_back(Point2f(123,73));
    homoInput.push_back(Point2f(0,73));


    ros::NodeHandle nh1;
    ros::Subscriber subImage1 = nh1.subscribe("camera2/usb_cam2/image_raw/compressed", 1, subImgCallback);
    ros::NodeHandle nh2;

    ros::Subscriber subImage2 = nh2.subscribe("camera2/usb_cam2/Lidar_cam_calibration", 1, Cam_LidarImg);

    ros::NodeHandle laneNh;
    lane_tracking = laneNh.advertise<sensor_msgs::PointCloud2> ("lane_tracking",480);
    lane_waypoint = laneNh.advertise<sensor_msgs::PointCloud2> ("lane_waypoint",480);

    waypoint.data=0;

    // namedWindow("test");

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

