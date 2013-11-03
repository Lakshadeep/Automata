#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "math.h"

using namespace cv;
Mat arena0,arena,arena1,arena2,points;
char* RadiusC1_min,RadiusC1_max;
void Threshold_Demo1(int,void*);
int radiusc1_max,radiusc1_min;
float centerx,centery,radius1,radius2,radius3,radius4;
int discx1,discy1,discx2,discy2,discx3,discy3,discxs2,discys2,discxs3,discys3;
float radiusm1,radiusm2,radiusm3;
float centerx1,centery1,centerx2,centery2,centerx3,centery3,radius1b,radius2b;
int points1[35][2];

char* Top;
char* Bottom;
char* Left;
char* Right;
void crop(int,void*);
int left=1,right=640,top=1,bottom=480;

char* lowerH;
char* lowerS;
char* lowerV;
char* upperH;
char* upperS;
char* upperV;
void Threshold(int,void*);
int lowerh,lowers,lowerv,upperh,uppers,upperv;

char* lowerH1;
char* lowerS1;
char* lowerV1;
char* upperH1;
char* upperS1;
char* upperV1;
void Threshold_Demo44(int,void*);
int lowerh1,lowers1,lowerv1,upperh1,uppers1,upperv1;


char* lowerH2;
char* lowerS2;
char* lowerV2;
char* upperH2;
char* upperS2;
char* upperV2;
void Threshold_Demo55(int,void*);
int lowerh2,lowers2,lowerv2,upperh2,uppers2,upperv2;


int num = 0;
int main(int, char**)
{
    VideoCapture cap(0);
    if(!cap.isOpened()) return -1;

    bool running = true;


        while(running){



        if(num == 0){
        std::cout << "Enter particular numbers to perform specific operations" << std::endl <<"11-to take snapshot for calibration"<<std::endl<<"22-to crop the arena"<<std::endl<<"33-to calibrate for circle detection"<<std::endl<<"44,55-to calibrate bot front and back color"<<std::endl<<"77,88-to check center detection of bot front,back color respectively"<<std::endl<<""<< "1-to calibrate for circle detection"<< std::endl <<"2,3,4,5-to detect 1st,2nd,3rd,4th circle respectively"<<std::endl<< "6-to verify discontinuity points" << std::endl;
        std::cout<<"7-To plot path for bot motion" << std::endl << "8-To start execution of program"<< std::endl;
        std::cin >> num ;
        }
        if(num==11){
                 cap >> arena0;
                 imshow("Arena", arena0);

        }
        else if(num==22){
            namedWindow("Trackbars",CV_GUI_EXPANDED);
            createTrackbar( "Top","Trackbars", &top,arena0.rows,crop);
            createTrackbar( "Bottom","Trackbars", &bottom,arena0.rows,crop );
            createTrackbar( "Left","Trackbars", &left,arena0.cols,crop);
            createTrackbar( "Right","Trackbars", &right,arena0.cols,crop);


        }
        else if(num==33){
            namedWindow("Trackbars",CV_GUI_EXPANDED);
            createTrackbar( "lowerH","Trackbars", &lowerh,255,Threshold);
            createTrackbar( "LowerS","Trackbars", &lowers,255, Threshold );
            createTrackbar( "LowerV","Trackbars", &lowerv,255, Threshold );
            createTrackbar( "upperH","Trackbars", &upperh,255,Threshold);
            createTrackbar( "upperS","Trackbars", &uppers,255, Threshold );
            createTrackbar( "upperV","Trackbars", &upperv,255, Threshold );

        }
        else if(num == 1){
                //namedWindow("arena",1);
                //inRange(arena,Scalar(lowerh,lowers,lowerv),Scalar(upperh,uppers,upperv),arena);
                namedWindow("Trackbars",CV_GUI_EXPANDED);
                createTrackbar( "RadiusC1_min","Trackbars", &radiusc1_min,600,Threshold_Demo1);
                createTrackbar( "RadiusC1_max","Trackbars", &radiusc1_max,600, Threshold_Demo1 );
                //imshow("Arena",arena);

        }
        else if(num==2){
            cvtColor( arena,arena1, CV_BGR2GRAY );
            inRange(arena,Scalar(lowerh,lowers,lowerv),Scalar(upperh,uppers,upperv),arena1);
            GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
            vector<Vec3f> circles;
            HoughCircles( arena1, circles, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25,radiusc1_min,radiusc1_max);
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                std::cout << cvRound(circles[i][0])<< cvRound(circles[i][1]) <<std::endl;
                *&centerx = cvRound(circles[i][0]);
                *&centery = cvRound(circles[i][1]);
                *&radius1 = cvRound(circles[i][2]);

                circle( arena1, center, 3, Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                circle( arena1, center, radius, Scalar(0,45,255), 3, 8, 0 );
            }
            imshow("Circle",arena1);




        }
        else if(num==3){
            cvtColor( arena,arena1, CV_BGR2GRAY );
            inRange(arena,Scalar(lowerh,lowers,lowerv),Scalar(upperh,uppers,upperv),arena1);
            GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
            vector<Vec3f> circles;
            HoughCircles( arena1, circles, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25,radiusc1_min,radiusc1_max);
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                std::cout << cvRound(circles[i][0])<< cvRound(circles[i][1]) <<std::endl;
                //*&centerx = cvRound(circles[i][0]);
                //*&centery = cvRound(circles[i][1]);
                *&radius2 = cvRound(circles[i][2]);

                circle( arena1, center, 3, Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                circle( arena1, center, radius, Scalar(0,45,255), 3, 8, 0 );
            }
            imshow("Circle",arena1);




        }
        else if(num==4){
            cvtColor( arena,arena1, CV_BGR2GRAY );
            inRange(arena,Scalar(lowerh,lowers,lowerv),Scalar(upperh,uppers,upperv),arena1);
            GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
            vector<Vec3f> circles;
            HoughCircles( arena1, circles, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25,radiusc1_min,radiusc1_max);
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                std::cout << cvRound(circles[i][0])<< cvRound(circles[i][1]) <<std::endl;
               // *&centerx = cvRound(circles[i][0]);
               // *&centery = cvRound(circles[i][1]);
                *&radius3 = cvRound(circles[i][2]);

                circle( arena1, center, 3, Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                circle( arena1, center, radius, Scalar(0,45,255), 3, 8, 0 );
            }
            imshow("Circle",arena1);




        }
        else if(num==5){
            cvtColor( arena,arena1, CV_BGR2GRAY );
            inRange(arena,Scalar(lowerh,lowers,lowerv),Scalar(upperh,uppers,upperv),arena1);
            GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
            vector<Vec3f> circles;
            HoughCircles( arena1, circles, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25,radiusc1_min,radiusc1_max);
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                std::cout << cvRound(circles[i][0])<< cvRound(circles[i][1]) <<std::endl;
               // *&centerx = cvRound(circles[i][0]);
               // *&centery = cvRound(circles[i][1]);
                *&radius4 = cvRound(circles[i][2]);

                circle( arena1, center, 3, Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                circle( arena1, center, radius, Scalar(0,45,255), 3, 8, 0 );
            }
            imshow("Circle",arena1);



        }
        else if(num==6){

                    //arena = imread("realarena.jpg");
                    //cvtColor( arena,arena1, CV_BGR2GRAY );
                    //threshold(arena1,arena_binary,800,1000, THRESH_BINARY);



                    radiusm1= (radius1+radius2)/2;

                    int discx1 = centerx - (radiusm1*cos(3.14/6));
                    int discy1 = centery + (radiusm1*sin(3.14/6));
                    Point disc1(discx1,discy1);
                    circle( arena, disc1, 3, Scalar(0,255,0), -1, 8, 0 );



                    radiusm2 = (radius2+radius3)/2;

                    int discxs2 = centerx - (radiusm2*cos(3.14/6));
                    int discys2 = centery + (radiusm2*sin(3.14/6));
                    Point discs2(discxs2,discys2);
                    circle( arena, discs2, 3, Scalar(0,255,0), -1, 8, 0 );

                    int discx2 = centerx;
                    int discy2 = centery - radiusm2;
                    Point disc2(discx2,discy2);
                    circle( arena, disc2, 3, Scalar(0,255,0), -1, 8, 0 );



                    radiusm3 = (radius3+radius4)/2;

                    int discxs3 = centerx;
                    int discys3 = centery - radiusm3;
                    Point discs3(discxs3,discys3);
                    circle( arena, discs3, 3, Scalar(0,255,0), -1, 8, 0 );

                    int discx3 = centerx + (radiusm3*cos(3.14/6));
                    int discy3 = centery + (radiusm3*sin(3.14/6));
                    Point disc3(discx3,discy3);
                    circle( arena, disc3, 3, Scalar(0,255,0), -1, 8, 0 );


                    imshow("Discontinuity Points",arena);



        }
        else if(num==7){
                    double distbx = centerx-centerx1;              //calculates object angle
                    double distby = centery-centery1;
                    double angleb,anglebd;
                    if(distbx<0 && distby >0){
                    distbx = distbx*-1;
                    angleb = atan(distby/distbx);
                    anglebd = angleb*180/3.14;
                    }
                    else if(distbx>0 && distby >0){
                    angleb = atan(distby/distbx);
                    anglebd = 180 -(angleb*180/3.14);
                    }
                    else if(distbx>0 && distby <0){
                    distby = distby*-1;
                    angleb = atan(distby/distbx);
                    anglebd = 180+(angleb*180/3.14);

                    }
                    else if(distbx<0 && distby <0){
                    distby = distby*-1;
                    distbx = distbx*-1;
                    angleb = atan(distby/distbx);
                    anglebd = 360 -(angleb*180/3.14);
                    }
                        std::cout << anglebd << std::endl;

                    float angle1=210,angles2=210,angle2=90,angles3=90,angle3=330;
                     int k = 0;
                     if(angle1>anglebd){

                            int increm=15;

                            for(int i = anglebd;i<=angle1;){

                                if(i<=90){
                                    points1[k][0]=centerx + (radiusm1*cos(i*3.14/180));
                                    points1[k][1]=centery - (radiusm1*sin(i*3.14/180));
                                    k++;
                                }
                                else if(i>90 && i<=180){
                                    points1[k][0]=centerx - (radiusm1*cos(3.14-(i*3.14/180)));
                                    points1[k][1]=centery - (radiusm1*sin(3.14-(i*3.14/180)));
                                    k++;
                                }
                                else{
                                    points1[k][0]=centerx - (radiusm1*cos((i*3.14/180)-3.14));
                                    points1[k][1]=centery + (radiusm1*sin((i*3.14/180)-3.14));
                                    k++;
                                }


                                        i=i+increm;

                            }




                    }
                    else{

                         int increm=15;
                        //int k = 0;
                        for(int i = anglebd;i>=angle1;){

                                if(i>=270){
                                    points1[k][0]=centerx + (radiusm1*cos(6.28-(i*3.14/180)));
                                    points1[k][1]=centery + (radiusm1*sin(6.28-(i*3.14/180)));
                                    k++;
                                }
                                else{
                                    points1[k][0]=centerx - (radiusm1*cos((i*3.14/180)-3.14));
                                    points1[k][1]=centery + (radiusm1*sin((i*3.14/180)-3.14));
                                    k++;
                                }
                                i=i-increm;

                            }

                    }

                        for(int p =210;p>=90;){

                        if(p<=180){
                                    points1[k][0]=centerx - (radiusm2*cos(3.14-(p*3.14/180)));
                                    points1[k][1]=centery - (radiusm2*sin(3.14-(p*3.14/180)));
                                    k++;

                        }else{
                                    points1[k][0]=centerx - (radiusm2*cos((p*3.14/180)-3.14));
                                    points1[k][1]=centery + (radiusm2*sin((p*3.14/180)-3.14));
                                    k++;

                        }

                        p=p-18;
                        }

                        for(int q =90;q>=0;){


                                    points1[k][0]=centerx + (radiusm3*cos((q*3.14/180)));
                                    points1[k][1]=centery - (radiusm3*sin((q*3.14/180)));
                                    k++;
                                    q=q-30;

                        }

                        for(int s = 0;s<30;){
                                    points1[k][0]=centerx + (radiusm3*cos((s*3.14/180)));
                                    points1[k][1]=centery + (radiusm3*sin((s*3.14/180)));
                                    k++;
                                    s=s+20;

                        }

                        points1[k][0]=centerx;
                        points1[k][1]=centery;

                          //points = imread("realarena.jpg");




                          for(int t = 0;t<35;t++){
                                    Point pt(points1[t][0],points1[t][1]);
                                    circle(arena,pt, 3, Scalar(0,255,0), -1, 8, 0 );

                                }

                                    imshow("arena",arena);



    }
    else if(num == 44){
            namedWindow("Trackbars",CV_GUI_EXPANDED);
            createTrackbar( "lowerH1","Trackbars", &lowerh1,255,Threshold_Demo44);
            createTrackbar( "LowerS1","Trackbars", &lowers1,255, Threshold_Demo44 );
            createTrackbar( "LowerV1","Trackbars", &lowerv1,255, Threshold_Demo44 );
            createTrackbar( "upperH1","Trackbars", &upperh1,255,Threshold_Demo44);
            createTrackbar( "upperS1","Trackbars", &uppers1,255, Threshold_Demo44 );
            createTrackbar( "upperV1","Trackbars", &upperv1,255, Threshold_Demo44 );

        }
        else if(num==55){
             namedWindow("Trackbars",CV_GUI_EXPANDED);
            createTrackbar( "lowerH2","Trackbars", &lowerh2,255,Threshold_Demo55);
            createTrackbar( "LowerS2","Trackbars", &lowers2,255, Threshold_Demo55 );
            createTrackbar( "LowerV2","Trackbars", &lowerv2,255, Threshold_Demo55 );
            createTrackbar( "upperH2","Trackbars", &upperh2,255,Threshold_Demo55);
            createTrackbar( "upperS2","Trackbars", &uppers2,255, Threshold_Demo55 );
            createTrackbar( "upperV2","Trackbars", &upperv2,255, Threshold_Demo55);

            }
            else if(num==77){

            cvtColor( arena,arena1, CV_BGR2GRAY );
            inRange(arena,Scalar(lowerh1,lowers1,lowerv1),Scalar(upperh1,uppers1,upperv1),arena1);
            GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
            vector<Vec3f> circles;
            HoughCircles( arena1, circles, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25, 10, 60);
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                std::cout << cvRound(circles[i][0]) << cvRound(circles[i][1]) <<std::endl;
                *&centerx1 = cvRound(circles[i][0]);
                *&centery1 = cvRound(circles[i][1]);

            // circle center
                circle( arena, center, 3, Scalar(0,255,0), -1, 8, 0 );
   // circle outline
                circle(arena, center, radius, Scalar(0,45,255), 3, 8, 0 );
            }

                imshow( "NewArena", arena );






            }
            else if(num==88){
                cvtColor( arena,arena1, CV_BGR2GRAY );
            inRange(arena,Scalar(lowerh2,lowers2,lowerv2),Scalar(upperh2,uppers2,upperv2),arena1);
            GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
            vector<Vec3f> circles;
            HoughCircles( arena1, circles, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25, 10, 60);
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                std::cout << cvRound(circles[i][0]) << cvRound(circles[i][1]) <<std::endl;
                *&centerx2 = cvRound(circles[i][0]);
                *&centery2 = cvRound(circles[i][1]);

            // circle center
                circle(arena, center, 3, Scalar(0,255,0), -1, 8, 0 );
   // circle outline
                circle(arena, center, radius, Scalar(0,45,255), 3, 8, 0 );
            }

                imshow( "NewArena", arena );

        }
        else if(num==8){
             int  j = 0;
            for(;;)
                {
                    Rect rect;
                    cap >> arena0;                                  //capture snapshot
                    rect = Rect(left,top,right-left,bottom-top);    //crops the image
                    arena = arena0(rect);


                    cvtColor(arena,arena1, CV_BGR2GRAY );
                    inRange(arena,Scalar(lowerh1,lowers1,lowerv1),Scalar(upperh1,uppers1,upperv1),arena1);
                    GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
                    vector<Vec3f> circles1;
                    HoughCircles( arena1, circles1, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25, 10, 60);

                    for( size_t i = 0; i < circles1.size(); i++ )
                    {
                        Point center(cvRound(circles1[i][0]), cvRound(circles1[i][1]));
                        int radius = cvRound(circles1[i][2]);
                        std::cout << cvRound(circles1[i][0]) << cvRound(circles1[i][1]) <<std::endl;
                        *&centerx1 = cvRound(circles1[i][0]);
                        *&centery1 = cvRound(circles1[i][1]);
                        *&radius1b = cvRound(circles1[i][2]);


                    }


                    cvtColor(arena,arena1, CV_BGR2GRAY );
                    inRange(arena,Scalar(lowerh2,lowers2,lowerv2),Scalar(upperh2,uppers2,upperv2),arena1);
                    GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
                    vector<Vec3f> circles2;
                    HoughCircles( arena1, circles2, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25, 10, 60);

                    for( size_t i = 0; i < circles2.size(); i++ )
                    {
                        Point center(cvRound(circles2[i][0]), cvRound(circles2[i][1]));
                        int radius = cvRound(circles2[i][2]);
                        std::cout << cvRound(circles2[i][0]) << cvRound(circles2[i][1]) <<std::endl;
                        *&centerx2 = cvRound(circles2[i][0]);
                        *&centery2 = cvRound(circles2[i][1]);
                        *&radius2b = cvRound(circles2[i][2]);

                    }

                        *&centerx3 = points1[j][0];
                        *&centery3 = points1[j][1];


                    Point center1(centerx1, centery1);
                    circle( arena,center1, 3, Scalar(0,255,0), -1, 8, 0 );    //plot bot front center
                    circle( arena,center1 ,radius1b, Scalar(0,255,0), 3, 8, 0 );

                    Point center2(centerx2, centery2);
                    circle( arena,center2, 3, Scalar(0,255,0), -1, 8, 0 );    //plot bot back center
                    circle( arena,center2 ,radius2b, Scalar(0,00,255), 3, 8, 0 );

                    Point center3(centerx3, centery3);
                    circle( arena,center3, 3, Scalar(0,255,0), -1, 8, 0 );    //plot object center


                    imshow("current frame",arena);

                    if(centerx3!=0 && centery3!=0){

                    std::cout<<"Now following point "<<j<<std::endl;
                    if(circles1.size()!=0 && circles2.size()!=0){
                    double distox = centerx2-centerx3;              //calculates object angle
                    double distoy = centery2-centery3;
                    double angleo,angleod;
                    if(distox<0 && distoy >0){
                    distox = distox*-1;
                    angleo = atan(distoy/distox);
                    angleod = angleo*180/3.14;
                    }
                    else if(distox>0 && distoy >0){
                    angleo = atan(distoy/distox);
                    angleod = 180 -(angleo*180/3.14);
                    }
                    else if(distox>0 && distoy <0){
                    distoy = distoy*-1;
                    angleo = atan(distoy/distox);
                    angleod = 180+angleo*180/3.14;

                    }
                    else if(distox<0 && distoy <0){
                    distoy = distoy*-1;
                    distox = distox*-1;
                    angleo = atan(distoy/distox);
                    angleod = 360 -(angleo*180/3.14);
                    }
                        std::cout << angleod << std::endl;


                    double distbx = centerx2-centerx1;              //calculates bot angle
                    double distby = centery2-centery1;
                    double angleb,anglebd;
                    if(distbx<0 && distby >0){
                    distbx = distbx*-1;
                    angleb = atan(distby/distbx);
                    anglebd = angleb*180/3.14;
                    }
                    else if(distbx>0 && distby >0){
                    angleb = atan(distby/distbx);
                    anglebd = 180 -(angleb*180/3.14);
                    }
                    else if(distbx>0 && distby <0){
                    distby = distby*-1;
                    angleb = atan(distby/distbx);
                    anglebd = 180+angleb*180/3.14;

                    }
                    else if(distbx<0 && distby <0){
                    distby = distby*-1;
                    distbx = distbx*-1;
                    angleb = atan(distby/distbx);
                    anglebd = 360 -(angleb*180/3.14);
                    }
                        std::cout << anglebd << std::endl;

                    if(angleod>=anglebd){
                            if(anglebd<90 && angleod>270){
                                std::cout<<"Turn left"<< std::endl;
                            }
                            else if(angleod-anglebd<5){
                                int distx = (centerx1-centerx3)*(centerx1-centerx3);
                                int disty = (centery1-centery3)* (centery1-centery3);
                                double dist = sqrt(distx+disty);
                                if(dist<20){
                                    std::cout<<"Stop"<< std::endl;
                                    j=j+1;
                                }
                                else{
                                    std::cout<<"Move forward"<<std::endl;
                                }

                        }
                        else{
                        std::cout << "move left" << std::endl;
                        }
                    }
                    else{
                            if(angleod<90 && anglebd>270){
                                std::cout<<"Turn left"<< std::endl;
                            }
                            else if(anglebd-angleod<5){
                                int distx = (centerx1-centerx3)*(centerx1-centerx3);
                                int disty = (centery1-centery3)* (centery1-centery3);
                                double dist = sqrt(distx+disty);
                                if(dist<20){
                                    std::cout<<"Stop"<< std::endl;
                                    j=j+1;
                                }
                                else{
                                    std::cout<<"Move forward"<<std::endl;
                                }

                            }
                            else{
                        std::cout << "move right" << std::endl;


                            }
                    }






                }


            }     std::cout<<"Something not right.."<<std::endl;
                     if(waitKey(30) >= 0)     break;
        }
        }
        num = 0;


//}

    waitKey( 1200000 );
    destroyAllWindows();
    }


    return 0;
}
void Threshold_Demo1(int,void*){
            cvtColor( arena,arena1, CV_BGR2GRAY );
            inRange(arena,Scalar(lowerh,lowers,lowerv),Scalar(upperh,uppers,upperv),arena1);
            //inRange(arena1,Scalar(140,120,120),Scalar(255,200,240),arena1);
            GaussianBlur( arena1, arena1, Size(9, 9), 2, 2 );
            vector<Vec3f> circles;
            HoughCircles( arena1, circles, CV_HOUGH_GRADIENT, 1, arena1.rows/16, 200, 25,radiusc1_min,radiusc1_max);
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);


                // circle center
                circle( arena1, center, 3, Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                circle( arena1, center, radius, Scalar(0,45,255), 3, 8, 0 );
            }
            imshow("Circle",arena1);

}
void crop( int,void*)
{
            Rect rect;
            rect = Rect(left,top,right-left,bottom-top);
            *&left = left;
            *&top = top;
            *&right = right;
            *&bottom = bottom;

            arena = arena0(rect);
            imshow("NewArena",arena);
}
void Threshold( int,void*)
{
    namedWindow("NewArena",1);

    inRange(arena,Scalar(lowerh,lowers,lowerv),Scalar(upperh,uppers,upperv),arena2);
    *&lowerh = lowerh;
    *&lowers = lowers;
    *&lowerv = lowerv;
    *&upperh = upperh;
    *&uppers = uppers;
    *&upperv = upperv;
    imshow( "NewArena",arena2 );
}
void Threshold_Demo44( int,void*)
{
    namedWindow("NewArena",1);

    inRange(arena,Scalar(lowerh1,lowers1,lowerv1),Scalar(upperh1,uppers1,upperv1),arena2);
    *&lowerh1 = lowerh1;
    *&lowers1 = lowers1;
    *&lowerv1 = lowerv1;
    *&upperh1 = upperh1;
    *&uppers1 = uppers1;
    *&upperv1 = upperv1;
    imshow( "NewArena", arena2 );
}
void Threshold_Demo55( int,void*)
{
    namedWindow("NewArena",1);

    inRange(arena,Scalar(lowerh2,lowers2,lowerv2),Scalar(upperh2,uppers2,upperv2),arena2);
    *&lowerh2 = lowerh2;
    *&lowers2 = lowers2;
    *&lowerv2 = lowerv2;
    *&upperh2 = upperh2;
    *&uppers2 = uppers2;
    *&upperv2 = upperv2;
    imshow( "NewArena", arena2 );
}
