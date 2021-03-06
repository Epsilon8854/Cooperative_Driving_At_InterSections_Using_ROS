#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/aruco.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <vector>
#include <string.h>
using namespace cv;
using namespace std;
using namespace ros;

#define MAX_VELO 50

int carSpeed = 20;
int end_flag = 0;

typedef unsigned short u16;
typedef unsigned int u32;

cv_bridge::CvImagePtr rawImagePtr, LidarImgPtr;
Mat rawImage, InRanImg, GrayImg, MaterialImg, Road_MaterialImg, hsvImgWhite, hsvImgYellow, hsvImgBlue, hsvImg, BilateralImg, EqualizeImg;
Mat L_HSVImg, R_HSVImg;
Mat rstImg, X;
//Mat warpMat(2,4,CV_32FC1);
Mat warpMat;
Mat LidarImg, LidarImg_warp;
int offset = 80; // middle of lane
vector<Vec4i> lines;
int traffic_st_flag = 0;
int traffic_r_flag = 1;
int Whigh_H = 180, Whigh_S = 60, Whigh_V = 255;
int Wlow_H = 0, Wlow_S = 0, Wlow_V = 200; //v=200
int Yhigh_H = 60, Yhigh_S = 255, Yhigh_V = 255;
int Ylow_H = 0, Ylow_S = 80, Ylow_V = 71;
int Bhigh_H = 180, Bhigh_S = 255, Bhigh_V = 255;
int Blow_H = 151, Blow_S = 50, Blow_V = 45; //h=25

// int Whigh_H = 180, Whigh_S = 60, Whigh_V = 255;
// int Wlow_H = 0, Wlow_S = 0, Wlow_V = 200;       //v=200
// int Yhigh_H = 27, Yhigh_S = 255, Yhigh_V = 255; //Yhigh_H=50,Yhigh_S=255,Yhigh_V=255;
// int Ylow_H = 1, Ylow_S = 90, Ylow_V = 171;     //Ylow_H=15,Ylow_S=90,Ylow_V=171
// //int Yhigh_H= 40,Yhigh_S= 255,Yhigh_V= 255;
// //int Ylow_H=  20, Ylow_S= 0, Ylow_V=  171;//h=25
// int Bhigh_H = 135, Bhigh_S = 255, Bhigh_V = 255;
// int Blow_H = 90, Blow_S = 90, Blow_V = 171; //h=25

int _a = 0, _b = 0, _c = 0, __a = 0, __b = 0, __c = 0;
int _a2 = 0, _b2 = 0, _c2 = 0, __a2 = 0, __b2 = 0, __c2 = 0;
int errorR = 0, errorL;

int flag = 1;
int size;
Vec4i temp;
Rect roi;
Point lineForm[1][4];
Point Road_erase[1][18];
const Point *ppt[1] = {lineForm[0]};
const Point *Road_ppt[1] = {Road_erase[0]};
int npt[] = {4};
int R_E_npt[] = {18};

int past_offset_x;
Point2f laneConerInput[4] = {Point2f(0, 0), Point2f(479, 0), Point2f(479, 639), Point2f(0, 639)};
Point2f laneConerOutput[4] = {Point2f(18, 0), Point2f(101, 0), Point2f(123, 73), Point2f(0, 73)};
vector<Point2f> homoInput;
vector<Point2f> homoOutput;
//ros::NodeHandle nh;
//ros::Publisher waypoint_pub = nh.advertise<std_msgs::Float32>("/Lidar/waypoint_misvalue",10);
std_msgs::Float32 waypoint;

sensor_msgs::PointCloud2 way_output;
sensor_msgs::PointCloud2 lane_output;

ros::Publisher lane_waypoint;
ros::Publisher lane_tracking;
ros::Publisher twist_data;
int start_X1, start_X2, start_X1_ag, start_X2_ag, start_flag = 1, start_Y1 = 0, start_Y2 = 0;
int detectFlag = 1;
//std::vector laneDetected<vector<int>>;
int laneDetected[100][6] = {{260, 0, 0 - 260, 0, 0}};

int numOfLanePoint_R = 0;
int numOfLanePoint_L = 0;

int LRflag = 0;
double lane_calcal_R = 0, lane_calcal_L = 0;
double pastTheta = 0;

int reliability = 0; // left:-1 right:1 both:2 non:0
int reliability_Cnt_Num = 0;
int reliability_Cnt_All = 0;
int reliability_Cnt_L = 0;
int reliability_Cnt_R = 0;
int reliability_Cnt_Non = 0;
Mat Road_Of_Interest;

int left_ran_pt[480] = {0};
int right_ran_pt[480] = {0};

//variable to get more lane(to check reliability)
Mat warp_erase;
int erase_range = 10;
double avgtime;
double sumtime = 0;
int cnttime = 0;
int car_position = 800;
int tempConsensus;
int _x1, _y1, _x2, _y2;
int trffic_flag[4] = {};
typedef struct _pt
{
    int x;
    int y;
    _pt()
    {
        x = 0;
        y = 0;
    }
    _pt(int _x, int _y)
    {
        x = _x;
        y = _y;
    }
} pt;

typedef struct _hough
{
    double rho;
    double theta;

    _hough()
    {
        rho = 0;
        theta = 0;
    }
    _hough(double _rho, double _theta)
    {
        rho = _rho;
        theta = _theta;
    }
} Hough;
typedef struct _ransac
{

    /*a0X^2 + a1X + a2*/
    double consensus;
    double a0;
    double a1;
    double a2;

    _ransac()
    {
        consensus = 0;
        a0 = 0;
        a1 = 80;
        a2 = 0;
    }

} Ransac;
Ransac pastL, pastR;
Ransac pastwindow;

void computeC2MC1(const Mat &R1, const Mat &tvec1, const Mat &R2, const Mat &tvec2,
                  Mat &R_1to2, Mat &tvec_1to2)
{
    //c2Mc1 = c2Mo * oMc1 = c2Mo * c1Mo.inv()
    R_1to2 = R2 * R1.t();
    tvec_1to2 = R2 * (-R1.t() * tvec1) + tvec2;
}
inline void getLine(Mat &targetMat, Mat &output, int &offset_x, int &offset_y, int &window_size) // get line with Hough TF && LSM
{
    //made by LEE IN HA 2019.07
    //cout<<"x: "<<offset_x<<"y: "<<offset_y<<endl;
    int MatRows = targetMat.rows;
    int halfOfWindow = window_size / 2;

    if ((offset_y >= MatRows - window_size * 1 / 2) || (offset_x <= halfOfWindow) || (offset_x >= targetMat.cols - window_size))
        return; //end of Y or X
    int numPT = 0;
    double AvgRho = 0, SumRho = 0;
    double AvgTheta = 0, SumTheta = 0;
    double tempTheta, tempRho;
    double dataArrTheta[50000] = {};
    double dataArrRho[50000] = {};

    uchar *data_output;
    uchar *Lidar_data_output;
    uchar *data_checking;
    double tempX, tempY;

    int Act_start_ptX, Act_start_ptY;
    Act_start_ptX = offset_x - halfOfWindow;
    Act_start_ptY = MatRows - offset_y;

    int wind_size_Y = window_size * 800 / 1600;

    Mat window_ROI = targetMat(Rect(Act_start_ptX, Act_start_ptY - wind_size_Y, window_size, wind_size_Y));
    //    Mat targetMat_ROI= targetMat.clone();
    //  rectangle(targetMat_ROI,Point(Act_start_ptX,Act_start_ptY),Point(Act_start_ptX+window_size,Act_start_ptY+wind_size_Y),Scalar(255),1);
    //cout<<"next"<<endl;

    // imshow("targetMat", window_ROI);
    // imshow("targetMat with Roi",targetMat_ROI);
    int testcnt = 0;
    vector<Point> TrueData;

    // 거꾸로된 mat 좌표계 뒤집기
    for (int i = window_ROI.rows - 1; i >= 0; i--)
    {
        data_output = window_ROI.ptr<uchar>(i);

        for (int j = 0; j < window_ROI.cols; j++)
        {
            if (data_output[j] == 0 || j == 0 || i == 0)
            {
                continue; // because y factor is inversed in Mat
            }
            else
            {
                //                cout<<"data"<<testcnt<<"%:"<<data_output[j]<<",("<<j<<","<<i<<")"<<endl;
                TrueData.push_back(Point(j, wind_size_Y - i)); //slope can be infinite so do inverse
                testcnt++;
            }
        }
    }
    //  cout<<"find"<<testcnt<<endl;

    int numPoint = TrueData.size(); //number of dataye
    int randIdx;

    int Xconsensus = 0;
    int Yconsensus = 0;

    int widMaxConsensus = 0;
    double tempLSM = 0;
    double widMinLSM = 999999999;
    Ransac fittest_pt;
    Mat testing_line = window_ROI.clone();
    testing_line = Scalar(0);
    int clock1 = clock();

    if (numPoint > 10)
    {
        for (int i = 1; i < 300; i++)
        { //ransac
            tempConsensus = 0;
            //cout<<"npt"<<numPoint<<endl;
            randIdx = rand() % numPoint;
            _x1 = TrueData[randIdx].x;
            _y1 = TrueData[randIdx].y;
            //cout<<"("<<_x1<<","<<_y1<<")"<<endl;
            randIdx = rand() % numPoint;
            _x2 = TrueData[randIdx].x;
            _y2 = TrueData[randIdx].y;

            // cout<<"("<<_x2<<","<<_y2<<")"<<endl;
            if (_x1 == _x2 || _y1 == _y2)
                continue;
            else
            {
                // line(testing_line, Point(_x1, testing_line.rows - _y1), Point(_x2, testing_line.rows - _y2), Scalar(255), 1);

                double a0;
                double a1; //y= a0*x+a1;
                a0 = (double)(_y2 - _y1) / (double)(_x2 - _x1);
                a1 = (double)(_y2 - _y1) / (double)(_x2 - _x1) * (double)(-_x1) + (double)_y1;
                int checkY;      // from this X, counting the inline data
                int startPointX; // to avoid checkX is less than thershold

                for (checkY = 0; checkY < wind_size_Y; checkY++)
                { //

                    startPointX = ((double)checkY - (double)a1) / (double)a0;

                    if (startPointX < 0)
                        continue;
                    else if (startPointX < 5)
                        startPointX = 0;
                    else if (startPointX > window_size)
                        continue;
                    else if (startPointX >= window_size - 6)
                        startPointX = window_size - 6;
                    data_checking = window_ROI.ptr<uchar>(testing_line.rows - checkY); // cause slope is calculated in inverted img, chage x and y again
                    for (int checkX = startPointX - 5; checkX < startPointX + 6; checkX++)
                    {

                        //printf("(%d, %d)\n",checkX,checkY);
                        if (data_checking[checkX])
                        {
                            tempConsensus++;

                            double sumTemp = (6 - (checkX - startPointX)) * (6 - (checkX - startPointX));
                            tempLSM += sqrt(sumTemp);
                        }
                    }
                    //cout<<checkX<<endl;
                }
                //                cout<<"asd"<<endl;

                if (tempConsensus >= widMaxConsensus)
                {
                    fittest_pt.a0 = a0;
                    fittest_pt.a1 = a1;
                    widMaxConsensus = tempConsensus;
                    widMinLSM = tempLSM;
                }

                // imshow("testing_Line", testing_line);
            }
        }
        //        printf("y=%lfX+%lf\n",fittest_pt.a0,fittest_pt.a1);
        //        printf("%d percent is inliner",widMaxConsensus*100/numPoint);
        //        cout<<endl;
    }
    else
    {
        // printf("low data\n");
    }
    int clock2 = clock();
    sumtime += double(clock2 - clock1) / CLOCKS_PER_SEC;
    cnttime++;
    avgtime = sumtime / (double)cnttime;
    //printf("%lf sec\n",avgtime);
    Mat smallran = window_ROI.clone();
    smallran = Scalar(0);
    int ranP1, ranP2, ranP22;
    //y= a0*x+a1;
    past_offset_x = offset_x;

    if (numPoint > 10)
    {
        pastwindow.a0 = fittest_pt.a0;
        pastwindow.a1 = fittest_pt.a1;
        if (fittest_pt.a0 != 0)
        {

            ranP1 = (-fittest_pt.a1) / fittest_pt.a0;
            ranP2 = (testing_line.rows - fittest_pt.a1) / fittest_pt.a0;
            ranP22 = ((testing_line.rows / 2) - fittest_pt.a1) / fittest_pt.a0;

            if (offset_y != 0)
            {
                int tempabs = abs(past_offset_x - (ranP1 + offset_x - window_size / 2));
                if (tempabs < 80)
                {

                    line(smallran, Point(ranP1 + offset_x, smallran.rows + offset_y), Point(ranP2 + offset_x, 0 + offset_y), Scalar(255), 1);
                    line(output, Point(ranP1 + offset_x - window_size / 2, targetMat.rows - offset_y), Point(ranP2 + offset_x - window_size / 2, targetMat.rows - (smallran.rows + offset_y)), Scalar(255), 2);
                }
            }
            else
            {
                line(smallran, Point(ranP1 + offset_x, smallran.rows + offset_y), Point(ranP2 + offset_x, 0 + offset_y), Scalar(255), 1);
                line(output, Point(ranP1 + offset_x - window_size / 2, targetMat.rows - offset_y), Point(ranP2 + offset_x - window_size / 2, targetMat.rows - (smallran.rows + offset_y)), Scalar(255), 2);
            }
        }
        else
        {
            ranP2 = fittest_pt.a1;
            int tempabs = abs(past_offset_x - (ranP1 + offset_x - window_size / 2));
            if (offset_y != 0)
            {
                if (tempabs < 80)
                {
                    line(smallran, Point(fittest_pt.a1, smallran.rows), Point(fittest_pt.a1, 0), Scalar(255), 1);
                    line(output, Point(fittest_pt.a1 + offset_x - window_size / 2, targetMat.rows - offset_y), Point(fittest_pt.a1 + offset_x - window_size / 2, targetMat.rows - (smallran.rows + offset_y)), Scalar(255), 2);
                    //        line(output,Point(fittest_pt.a1+offset_x-window_size/2, smallran.rows+offset_y),Point(fittest_pt.a1+offset_xoffset_x,0+offset_y),Scalar(255),2);
                }
            }
            else
            {

                line(smallran, Point(ranP1 + offset_x, smallran.rows + offset_y), Point(ranP2 + offset_x, 0 + offset_y), Scalar(255), 1);
                line(output, Point(ranP1 + offset_x - window_size / 2, targetMat.rows - offset_y), Point(ranP2 + offset_x - window_size / 2, targetMat.rows - (smallran.rows + offset_y)), Scalar(255), 2);
            }
        }
    }
    else
    {
        fittest_pt.a0 = pastwindow.a0;
        fittest_pt.a1 = pastwindow.a1;
        if (fittest_pt.a0 != 0)
        {
            ranP1 = (-fittest_pt.a1) / fittest_pt.a0;
            ranP2 = (testing_line.rows - fittest_pt.a1) / fittest_pt.a0;
            ranP22 = ((testing_line.rows / 2) - fittest_pt.a1) / fittest_pt.a0;

            if (offset_y != 0)
            {
                line(smallran, Point(ranP1 + offset_x, smallran.rows + offset_y), Point(ranP2 + offset_x, 0 + offset_y), Scalar(255), 1);
                line(output, Point(ranP1 + offset_x - window_size / 2, targetMat.rows - offset_y), Point(ranP2 + offset_x - window_size / 2, targetMat.rows - (smallran.rows + offset_y)), Scalar(255), 2);
            }
            else
            {
            }
        }
        else
        {
            ranP2 = fittest_pt.a1;
            if (offset_y != 0)
            {
                line(smallran, Point(fittest_pt.a1, smallran.rows), Point(fittest_pt.a1, 0), Scalar(255), 1);
                line(output, Point(fittest_pt.a1 + offset_x - window_size / 2, targetMat.rows - offset_y), Point(fittest_pt.a1 + offset_x - window_size / 2, targetMat.rows - (smallran.rows + offset_y)), Scalar(255), 2);
                //        line(output,Point(fittest_pt.a1+offset_x-window_size/2, smallran.rows+offset_y),Point(fittest_pt.a1+offset_xoffset_x,0+offset_y),Scalar(255),2)
            }
            else
            {
            }
        }
    }
    // imshow("window ransac", smallran);
    //cout <<"NumPT"<<numPT<<endl;
    past_offset_x = offset_x;
    offset_x = ranP2 + offset_x - window_size / 2;
    if (numPoint > 10)
    {

        if (offset_y != 0)
        {

            int tempabs = abs(past_offset_x - (ranP1 + offset_x - window_size / 2));
            if (tempabs < 80)
            {
                rectangle(output, Point(offset_x - window_size / 2, output.rows - offset_y), Point(offset_x + window_size / 2, output.rows - (offset_y + window_size * 1 / 2)), Scalar(255), 1);
                offset_y += window_size * 1 / 2;
                if (LRflag == 1)
                {
                    laneDetected[numOfLanePoint_R][0] = offset_x - targetMat.cols / 2;
                    laneDetected[numOfLanePoint_R++][1] = offset_y;
                    laneDetected[numOfLanePoint_R][0] = ranP22 + past_offset_x - window_size / 2 - targetMat.cols / 2;
                    laneDetected[numOfLanePoint_R++][1] = offset_y - window_size * 1 / 4;
                    // cout<<"Yoff"<<next_Y<<"Rtheta"<<theta<<endl;
                }
                else
                {
                    laneDetected[numOfLanePoint_L][3] = offset_x - targetMat.cols / 2;
                    laneDetected[numOfLanePoint_L++][4] = offset_y;
                    laneDetected[numOfLanePoint_L][3] = ranP22 + past_offset_x - window_size / 2 - targetMat.cols / 2;
                    laneDetected[numOfLanePoint_L++][4] = offset_y - window_size * 1 / 4;
                }
                getLine(targetMat, output, offset_x, offset_y, window_size);
            }
        }
        else
        {
            //cout<<"offset_Y:"<<offset_y<<endl;

            //cout<<"next PT: "<<"x: "<<offset_x<<"y: "<<offset_y<<"wid"<<window_size<<endl;
            rectangle(output, Point(offset_x - window_size / 2, output.rows - offset_y), Point(offset_x + window_size / 2, output.rows - (offset_y + window_size * 1 / 2)), Scalar(255), 1);
            offset_y += window_size * 1 / 2;
            if (LRflag == 1)
            {
                laneDetected[numOfLanePoint_R][0] = offset_x - targetMat.cols / 2;
                laneDetected[numOfLanePoint_R++][1] = offset_y;
                laneDetected[numOfLanePoint_R][0] = ranP22 + past_offset_x - window_size / 2 - targetMat.cols / 2;
                laneDetected[numOfLanePoint_R++][1] = offset_y - window_size * 1 / 4;
                // cout<<"Yoff"<<next_Y<<"Rtheta"<<theta<<endl;
            }
            else
            {
                laneDetected[numOfLanePoint_L][3] = offset_x - targetMat.cols / 2;
                laneDetected[numOfLanePoint_L++][4] = offset_y;
                laneDetected[numOfLanePoint_L][3] = ranP22 + past_offset_x - window_size / 2 - targetMat.cols / 2;
                laneDetected[numOfLanePoint_L++][4] = offset_y - window_size * 1 / 4;
            }
            getLine(targetMat, output, offset_x, offset_y, window_size);
            //fillPoly(Road_MaterialImg,Road_ppt,R_E_npt,1,Scalar(255),8);
        }
    }
    else
    {
    }
    //left

    //right
}
void brakeCar()
{
     geometry_msgs::Twist traffic_twist;
    if (carSpeed > 0)
    {
        carSpeed -= 5;
    }

    if (carSpeed < 0)
    {
        carSpeed = 0;
    }
    traffic_twist.angular.z = 7;
    traffic_twist.linear.x = carSpeed;
    twist_data.publish(traffic_twist);
}
void accelCar()
{
    if (end_flag == 0)
    {
        if (carSpeed < MAX_VELO)
        {
            carSpeed += 15;
        }
    }
    else
    {
        brakeCar();
    }
}

void subEndCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "end")
    {
        printf("subscine end!!!!|\n\n\n\n");
        end_flag = 1;
        // STOP CAR
    }
}

void subStrCallback(const std_msgs::String::ConstPtr &msg)
{
    std::string tfMsg[5]; // "light1/RED/(double)remain_time"
    //change string to char... for f**king token
    char *Cstr_tfMsg = (char *)malloc(sizeof(msg->data) + 10);
    strcpy(Cstr_tfMsg, msg->data.c_str());
    double tRemain = 0;
    double carDist1 = 10, carDist2 = 10;
    char *tok = strtok(Cstr_tfMsg, "/");
    geometry_msgs::Twist error_twist;

    if ((string(tok) == "light1" || string(tok) == "light2"))
    {
        for (int str_idx = 0; str_idx < 3; str_idx++)
        {
            tfMsg[str_idx] = std::string(tok);
            tok = strtok(nullptr, "/");
        }
        string red = "RED", green = "GREEN", yellow = "YELLOW";
        tRemain = std::stod(tfMsg[2]);
        if (tfMsg[0] == "light1")
        {

            trffic_flag[0] = 1;
            printf("| %s | sign: %s | time: %lf | carSpeed: %d | ", tfMsg[0].c_str(), tfMsg[1].c_str(), tRemain, carSpeed);
            if ((tfMsg[1] == red) || (tfMsg[1] == yellow))
            {
                // printf("STOP\n");
                traffic_r_flag =0;
                brakeCar();
            }
            else if ((tfMsg[1] == green))
            {

                traffic_r_flag =1;
                if (traffic_st_flag)
                {
                    printf("reduce flag... %d \n", traffic_st_flag);
                    brakeCar();
                    traffic_st_flag--;
                }
                else if (traffic_st_flag == 0)
                {
                    accelCar();
                }
            }
        }
    }

    else
    {
        for (int str_idx = 0; str_idx <2; str_idx++)
        {
            tfMsg[str_idx] = std::string(tok);
            tok = strtok(nullptr, "/");
        }

        if (tfMsg[0] == "car1")
        {
            trffic_flag[2] = 1;
            carDist1 = stod(tfMsg[1]);
        }
        else if (tfMsg[0] == "car2")
        {
            trffic_flag[3] = 1;
            carDist2 = stod(tfMsg[1]);
            printf("get dist| car2 | %lf \n", carDist2);
            traffic_st_flag = 4;
        }
    }
}
// string temp1 = "stop";
// string temp2 = "go";
// if (msg->data.c_str() == temp1)
// {
//     start_flag = 0;
// }
// else if (msg->data.c_str() == temp2)
// {
//     start_flag = 1;
// }
void subImgCallback(const sensor_msgs::CompressedImageConstPtr &subImgMsgs) //bag test
{

    if (subImgMsgs->data.size())
    {
        if (start_flag == 1)
        {
            laneDetected[3][0] = -520;
            laneDetected[0][0] = 520;
            //cout<<"start"<<endl;
            rawImage = imdecode(Mat(subImgMsgs->data), 1);

            cvtColor(rawImage, GrayImg, COLOR_BGR2GRAY);

            Mat rawWarpImg, warpImg;
            Size warpSize(1600, 800);

            // namedWindow("White binary img");
            // createTrackbar("WHigh Hue", "White binary img", &Whigh_H, 180);
            // createTrackbar("WHigh Sat", "White binary img", &Whigh_S, 255);
            // createTrackbar("WHigh Val", "White binary img", &Whigh_V, 255);
            // createTrackbar("Wlow Hue", "White binary img", &Wlow_H, 180);
            // createTrackbar("Wlow Sat", "White binary img", &Wlow_S, 255);
            // createTrackbar("Wlow Vral", "White binary img", &Wlow_V, 255);

            namedWindow("Yellow binary img");
            createTrackbar("YHigh Hue", "Yellow binary img", &Yhigh_H, 180);
            createTrackbar("YHigh Sat", "Yellow binary img", &Yhigh_S, 255);
            createTrackbar("YHigh Val", "Yellow binary img", &Yhigh_V, 255);
            createTrackbar("Ylow Hue", "Yellow binary img", &Ylow_H, 180);
            createTrackbar("Ylow Sat", "Yellow binary img", &Ylow_S, 255);
            createTrackbar("Ylow Val", "Yellow binary img", &Ylow_V, 255);

            // namedWindow("Red binary img");
            // createTrackbar("BHigh Hue", "Red binary img", &Bhigh_H, 180);
            // createTrackbar("BHigh Sat", "Red binary img", &Bhigh_S, 255);
            // createTrackbar("BHigh Val", "Red binary img", &Bhigh_V, 255);
            // createTrackbar("Blow Hue", "Red binary img", &Blow_H, 180);
            // createTrackbar("Blow Sat", "Red binary img", &Blow_S, 255);
            // createTrackbar("Blow Val", "Red binary img", &Blow_V, 255);

            namedWindow("rawImage");
            createTrackbar("raw", "rawImage", &_a, 500);
            createTrackbar("raw2", "rawImage", &_b, 500);
            createTrackbar("raw3", "rawImage", &_c, 500);
            createTrackbar("raw---", "rawImage", &__a, 500);
            createTrackbar("raw2---", "rawImage", &__b, 500);
            createTrackbar("raw3--", "rawImage", &__c, 500);

            createTrackbar("raw4", "rawImage", &_a2, 5000);
            createTrackbar("raw5", "rawImage", &_b2, 5000);
            createTrackbar("raw6", "rawImage", &_c2, 5000);
            createTrackbar("raw7---", "rawImage", &__a2, 5000);
            createTrackbar("raw8---", "rawImage", &__b2, 5000);
            createTrackbar("raw9--", "rawImage", &__c2, 5000);

            double c[9] = {1026.641371, 0, 781.842229,
                           0, 1024.528647, 428.861134,
                           0, 0, 1};
            Mat cameraMatrix(3, 3, CV_64FC1, c);
            double d[4] = {0.005902, -0.015082, 0.003146, -0.000331};
            Mat distCoeffs(4, 1, CV_64FC1, d);

            double r[3] = {-0.575 + (double)(425 - __a) / (double)2000,
                           -0.59 + (double)(450 - __b) / (double)2000,
                           -1.497 + (double)(_c - 80) / (double)2000};

            Mat rvec(3, 1, CV_64FC1, r);
            double t[3] = {-1100 + (double)(_a2 - __a2) / (double)10,
                           -100.4 + (double)(_b2 - 107) / (double)10,
                           255 + (double)(_c2 - __c2) / (double)10};
            Mat tvec(3, 1, CV_64FC1, t);

            Mat R_desired = (Mat_<double>(3, 3) << 0, 1, 0,
                             -1, 0, 0,
                             0, 0, 1);
            Mat R;

            Rodrigues(rvec, R);
            Mat normal = (Mat_<double>(3, 1) << 0, 0, 1);
            Mat normal1 = R * normal;

            Mat origin(3, 1, CV_64F, Scalar(0));

            Mat origin1 = R * origin + tvec;

            double d_inv1 = 1.0 / normal1.dot(origin1);
            Mat R_1to2, tvec_1to2;
            Mat tvec_desired = tvec.clone();
            computeC2MC1(R, tvec, R_desired, tvec_desired, R_1to2, tvec_1to2);
            Mat H = R_1to2 + d_inv1 * tvec_1to2 * normal1.t();
            H = cameraMatrix * H * cameraMatrix.inv();
            H = H / H.at<double>(2, 2);
            //std::cout << "H:\n" << H << std::endl;

            warpPerspective(rawImage, rawWarpImg, H, warpSize);
            // flip(rawWarpImg, rawWarpImg, 1);

            //hconcat(rawImage, rawWarpImg, compare);
            //imshow("Bird eye view", compare);

            cvtColor(rawWarpImg, rstImg, COLOR_BGR2HSV);
            imshow("WarpImg", rawWarpImg);
            // flip(rstImg, rstImg, 1);

            // inRange(rstImg, Scalar(Wlow_H, Wlow_S, Wlow_V), Scalar(Whigh_H, Whigh_S, Whigh_V), hsvImgWhite);
            inRange(rstImg, Scalar(Ylow_H, Ylow_S, Ylow_V), Scalar(Yhigh_H, Yhigh_S, Yhigh_V), hsvImgYellow);
            inRange(rstImg, Scalar(Blow_H, Blow_S, Blow_V), Scalar(Bhigh_H, Bhigh_S, Bhigh_V), hsvImgBlue);

            //            fillPoly(LMaterialImg,Lppt,Lnpt,1,Scalar(255),8);
            //            bitwise_and(hsvImgWhite,RMaterialImg,hsvImgWhite);

            // add(hsvImgWhite, hsvImgYellow, hsvImg);
            add(hsvImgYellow, hsvImgBlue, hsvImg);
            // flip(hsvImgBlue, hsvImgBlue, 1);
            // flip(hsvImgYellow, hsvImgYellow, 1);

            // flip(rstImg, rstImg, 1);

            // imshow("hhssvv img", rstImg);
            // imshow("Red binary img", hsvImgBlue);
            // imshow("Yellow binary img", hsvImgYellow);
            //            imshow("Rhsv img",hsvImgWhite);

            //            laneConerOutput[0] = Point2f(0,0);
            //            laneConerOutput[1] = Point2f(rawImage.cols-1,0);
            //            laneConerOutput[2] = Point2f(rawImage.cols-1,rawImage.rows-1);
            //            laneConerOutput[3] = Point2f(0,rawImage.rows-1);
            Mat homoWarp;
            Mat sobelX;
            homoWarp = findHomography(homoInput, homoOutput);
            //warpPerspective(rawImage,rawWarpImg,homoWarp,warpSize);

            warpMat = getPerspectiveTransform(laneConerInput, laneConerOutput);
            warpPerspective(hsvImg, warpImg, warpMat, warpImg.size());

            aruco::drawAxis(rawImage, cameraMatrix, distCoeffs, rvec, tvec, 50);
            MaterialImg = warpImg.clone(); // img for set ROI
            MaterialImg = Scalar(0);
            fillPoly(MaterialImg, ppt, npt, 1, Scalar(255), 8);

            //imshow("rawImage", rawImage);

            //imshow("warp",hsvImg);
            cvtColor(rawWarpImg, GrayImg, COLOR_BGR2GRAY);

            warpImg = hsvImg;
            // flip(warpImg, warpImg, 1);

            Mat convolLaneFilter = warpImg.clone();

            imshow("warpImg", warpImg);

            int init_MAX_Y = 200;

            dilate(warpImg, warpImg, Mat());
            erode(warpImg, warpImg, Mat());
            //ROS_INFO("Lane_init_ROI??");
            //ROS_INFO("Lane_init_ROI??");
            int W_cols = warpImg.cols;
            int W_rows = warpImg.rows;
            //ROS_INFO("%d %d",W_cols,W_rows);

            Mat Lane_init_ROI(warpImg, Rect(0, W_rows * 2 / 3, W_cols, W_rows / 3));
            //ROS_INFO("Lane_init_ROI is not");

            //ROS_INFO("threshold??");
            threshold(Lane_init_ROI, Lane_init_ROI, 100, 255, THRESH_BINARY);
            //ROS_INFO("circcle??");

            circle(Lane_init_ROI, Point((start_X1 + start_X2) / 2, 90), 3, Scalar(255));
            //ROS_INFO("Lane_init_ROI is not");

            // imshow("Roi", Lane_init_ROI);

            Mat sum_show = Mat::zeros(warpImg.rows, warpImg.cols, warpImg.type());
            Mat leftLanedetect = Mat::zeros(warpImg.rows, warpImg.cols, warpImg.type());
            Mat rightLanedetect = Mat::zeros(warpImg.rows, warpImg.cols, warpImg.type());

            uchar sum_data_arr[2000] = {};
            uchar *data_output;
            uchar *data_temp;
            //ROS_INFO("sum_Data??");

            data_output = sum_show.ptr<uchar>(0);
            for (int i = 0; i < Lane_init_ROI.rows; i++)
            {
                data_temp = Lane_init_ROI.ptr<uchar>(i);
                for (int j = 0; j < Lane_init_ROI.cols; j++)
                {
                    if (data_temp[j] != 0)
                    {

                        sum_data_arr[j]++;
                        data_output = sum_show.ptr<uchar>(sum_data_arr[j]);
                        data_output[j] = 255;
                    }
                }
            }
            // imshow("sum", sum_show);

            data_temp = sum_show.ptr<uchar>(0);
            int max1 = 0, max2 = 0;
            int sumtemp = 0;
            //ROS_INFO("Get mid?");

            if (detectFlag == 1)
            {
                if (1)
                {
                    for (int i = 800; i > 30; i--)
                    {
                        sumtemp = 0;
                        for (int j = i - 3; j < i + 3; j++)
                        {
                            sumtemp += sum_data_arr[j];
                        }
                        if (max1 < sumtemp)
                        {
                            max1 = sumtemp;
                            start_X1 = i;
                        }
                        if (max1 > 1200)
                            break;
                    }
                    for (int i = 800; i < 1570; i++)
                    {
                        sumtemp = 0;
                        for (int j = i - 3; j < i + 3; j++)
                        {
                            sumtemp += sum_data_arr[j];
                        }
                        if (max2 < sumtemp)
                        {
                            max2 = sumtemp;
                            start_X2 = i;
                        }
                        if (max2 > 1200)
                            break;
                    }
                }
                else
                {
                    for (int i = car_position - 10; i > car_position - 210 && i > 3; i--)
                    {
                        sumtemp = 0;
                        for (int j = i - 3; j < i + 3; j++)
                        {
                            sumtemp += sum_data_arr[j];
                        }
                        if (max1 < sumtemp)
                        {
                            max1 = sumtemp;
                            start_X1 = i;
                        }
                        if (max1 > 700)
                            break;
                        //cout<<"working"<<i<<endl;
                    }
                    for (int i = car_position + 10; i < sum_show.cols && i < car_position + 207; i++)
                    {
                        sumtemp = 0;
                        for (int j = i - 3; j < i + 3; j++)
                        {
                            sumtemp += sum_data_arr[j];
                        }
                        if (max2 < sumtemp)
                        {
                            max2 = sumtemp;
                            start_X2 = i;
                        }
                        if (max2 > 700)
                            break;
                    }
                }
                // detectFlag = 0;
                // cout<<"max1:"<<max1<<" max2:"<<max2<<endl;
            }

            // start_X2 = start_X1 + 195;
            // cout << "start_X1:"<<start_X1<<" start_X2:"<<start_X2<<start_X2-start_X1<<endl;

            //}

            int window_SZ = 160;
            start_Y1 = 0;
            start_Y2 = 0;
            numOfLanePoint_L = 0;
            LRflag = -1;
            pastTheta = 0;
            // ROS_INFO("GetLine??");
            getLine(warpImg, leftLanedetect, start_X1, start_Y1, window_SZ);

            start_Y1 = 0;
            start_Y2 = 0;
            numOfLanePoint_R = 0;
            LRflag = 1;
            pastTheta = 0;
            //ROS_INFO("GetLine??");

            getLine(warpImg, rightLanedetect, start_X2, start_Y2, window_SZ);
            //cout<<"endendend"<<endl
            // ROS_INFO("End");
            if (numOfLanePoint_L > 5)
            {
                int Lsum = 0;
                for (int Lidx = 2; Lidx < 5; Lidx++)
                {
                    Lsum += laneDetected[Lidx][3] - laneDetected[Lidx - 2][3];
                }

                errorL = Lsum / 3;
                //errorL /=3;
            }
            // else
            else if (numOfLanePoint_L > 3)
            {
                int Lsum = 0;
                for (int Lidx = 2; Lidx < numOfLanePoint_L; Lidx++)
                {
                    Lsum += laneDetected[Lidx][3] - laneDetected[Lidx - 2][3];
                }
                errorL = Lsum / (numOfLanePoint_L - 2);
                //errorL /=3;
                // errorL = laneDetected[numOfLanePoint_L][3] - laneDetected[numOfLanePoint_L - 1][3];
                //errorL /=3;
            }
            else
            {
                //do nohting
            }
            //  else if(numOfLanePoint_L ==0){
            //     //do nothing
            // }
            // else{
            // errorL = laneDetected[numOfLanePoint_L][3] + 490;
            // errorL /=3;
            // }

            if (numOfLanePoint_R > 5)
            {
                int Rsum = 0;
                for (int Ridx = 2; Ridx < 5; Ridx++)
                {
                    Rsum += laneDetected[Ridx][0] - laneDetected[Ridx - 2][0];
                }

                errorR = Rsum / 3;
                //errorL /=3;
            }
            // else
            else if (numOfLanePoint_R > 3)
            {
                int Rsum = 0;
                for (int Ridx = 2; Ridx < numOfLanePoint_R; Ridx++)
                {
                    Rsum += laneDetected[Ridx][0] - laneDetected[Ridx - 2][0];
                }
                errorR = Rsum / (numOfLanePoint_R - 2);
                //errorL /=3;
                // errorL = laneDetected[numOfLanePoint_L][3] - laneDetected[numOfLanePoint_L - 1][3];
                //errorL /=3;
            }
            else
            {
                //do nohting
            }
            // else if(numOfLanePoint_R ==0){
            //     //do nothing
            // }
            // else{
            // errorR = laneDetected[numOfLanePoint_R][0] - 490;
            // errorR /=3;
            // }

            circle(leftLanedetect, Point(laneDetected[2][3] + 1180, 440), 10, Scalar(255));
            cout << "errorL:" << numOfLanePoint_L << " " << errorL << " "
                 << " errorR: " << numOfLanePoint_R << " " << errorR << endl;
            imshow("wwewewe", rightLanedetect + leftLanedetect + warpImg);
            int cloudNum = 0;
            int error = errorR + errorL;
            if (error > 13 || error < -13)
                error = error * 1.2;
            geometry_msgs::Twist error_twist;
            if (error > 30)
            {
                error = 30;
            }
            else if (error < -30)
            {
                error = -30;
            }
            error = error * (-1);
            printf("error : %d\n\n", error);
            error_twist.angular.z = error;
            error_twist.linear.x = carSpeed;
            if(traffic_r_flag){
            twist_data.publish(error_twist);
            }
            //ROS_INFO("End");

            waitKey(10);
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Camera");
    srand(clock());
    Mat tmp1(640, 480, CV_8UC3);

    Road_Of_Interest = Mat::zeros(360, 640, CV_8U);
    Road_MaterialImg = Mat::zeros(360, 640, CV_8U);

    //640, 480
    lineForm[0][0] = Point(140, 0);
    lineForm[0][1] = Point(140, 359);
    lineForm[0][2] = Point(500, 359);
    lineForm[0][3] = Point(500, 0);

    homoOutput.push_back(Point2f(0, 0));
    homoOutput.push_back(Point2f(639, 0));
    homoOutput.push_back(Point2f(639, 479));
    homoOutput.push_back(Point2f(0, 479));

    homoInput.push_back(Point2f(18, 0));
    homoInput.push_back(Point2f(101, 0));
    homoInput.push_back(Point2f(123, 73));
    homoInput.push_back(Point2f(0, 73));

    ros::NodeHandle nh1;
    ros::Subscriber subImage1 = nh1.subscribe("usb_cam/image_raw/compressed", 1, subImgCallback);
    ros::Subscriber subStr1 = nh1.subscribe("traffic_light1_Info", 1, subStrCallback);
    ros::Subscriber subStr2 = nh1.subscribe("traffic_light2_Info", 1, subStrCallback);
    ros::Subscriber subStr3 = nh1.subscribe("aruco_single/car12_distance_info", 1, subStrCallback);
    ros::Subscriber subEnd = nh1.subscribe("/end_flag", 1, subEndCallback);

    ros::NodeHandle nh2;
    ros::NodeHandle laneNh;
    lane_tracking = laneNh.advertise<sensor_msgs::PointCloud2>("lane_tracking", 480);
    lane_waypoint = laneNh.advertise<sensor_msgs::PointCloud2>("lane_waypoint", 480);
    twist_data = laneNh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    waypoint.data = 0;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
