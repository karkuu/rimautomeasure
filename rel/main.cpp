#include <iostream>	// for standard I/O
#include <string>   // for strings
#include <vector>
#include <stdio.h>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/opencv.hpp>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <bcm2835.h>

#define PIN1 RPI_V2_GPIO_P1_11
#define PIN2 RPI_V2_GPIO_P1_16


using namespace std;
using namespace cv;

#define PI 3.14159265

//using namespace CVD;

// Image matrixes
Mat frame;
Mat imgroi;
Mat unedited;

// Mouse and area of interest
const int captureSizeX = 320;
const int captureSizeY = 240;
const int initialBoxHeight = 20;
const int initialBoxWidth = 40;
int Roix1 = (captureSizeX/2) - (initialBoxWidth/2);
int Roiy1 = (captureSizeY/2) - (initialBoxHeight/2);
int Roiwidth = initialBoxWidth;
int Roiheight = initialBoxHeight;

bool centerIsSet = false;
int rimCenter;
int rimR;
int rimL;

vector<int> rimPoints;

Rect areaOfInterest(0,0,0,0);
Rect currentAreaOfInterest(0,0,0,0);

int mousex,mousey,mousex2,mousey2;

bool newPressed = false;
bool calibPressed = false;
bool autoPressed = false;
bool settingsPressed = false;
bool helpPressed = false;
bool quitPressed = false;

bool firstclick = true;

// Sliders
const int canny_slider_max = 100;
const int c_slider_max = 600;
const int b_slider_max = 600;
const int threshold_low_max = 255;
const int threshold_hi_max = 255;

const int rimTolerance_max = 10;

int canny_slider;

int canny_onoff;
int gblur_onoff;
int threshold_onoff;

int contrast_slider = 225;
int brightness_slider = 141;
int threshold_low_slider = 0;
int threshold_hi_slider = 255;

int rimTolerance_slider = 3;


bool leftLight = false;
bool rightLight = false;

// Functions
void on_takenoaction (int, void* );
void on_contrastbrightness();
void on_canny (int, void* );
void on_threshold(int, void* );

static void onMouse( int event, int x, int y, int, void* );

void setAreaOfInterest(int x, int y);

int findCenter();
void lookDrift();
void autoCenter();
void changeLight(int);

void drawRim();



// Main
int main(int, char**)
{
    if (!bcm2835_init())
        return 1;

    // Set the pin to be an output
    bcm2835_gpio_fsel(PIN1, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(PIN2, BCM2835_GPIO_FSEL_OUTP);


    currentAreaOfInterest = {Roix1,Roiy1,Roiwidth,Roiheight};

    // Image compression parameters for writing png captures
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    VideoCapture cap; // open the default camera
    //cap.open("/home/pi/test/camtest/2.MPG");  // Capture from video
    cap.open(0);  // Capture from cam

    if(!cap.isOpened()) return -1; // check if we succeeded

    // Options for webcam
    cap.set(CV_CAP_PROP_FRAME_WIDTH, captureSizeX);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, captureSizeY);

    //cap.set(CV_CAP_PROP_FOCUS,160);

    // Create windows
    namedWindow("Unedited",1);
    namedWindow("Edited", 1);

    // Capture loop
    for(;;)
    {
        cap >> frame; // get a new frame from camera

        rectangle(frame,currentAreaOfInterest,Scalar(255,0,0),1,8,0);

        line(frame,Point(rimCenter,currentAreaOfInterest.y), Point(rimCenter,currentAreaOfInterest.y+currentAreaOfInterest.height/2-1),Scalar(255,255,255),1,8,0);
        line(frame,Point(currentAreaOfInterest.x+rimL,currentAreaOfInterest.y+currentAreaOfInterest.height/2+1), Point(currentAreaOfInterest.x+rimR,currentAreaOfInterest.y+currentAreaOfInterest.height/2+1),Scalar(255,255,255),1,8,0);
        line(frame,Point(currentAreaOfInterest.x+rimL,currentAreaOfInterest.y+currentAreaOfInterest.height/2-1), Point(currentAreaOfInterest.x+rimR,currentAreaOfInterest.y+currentAreaOfInterest.height/2-1),Scalar(255,255,255),1,8,0);


        rectangle(frame,Point(3,3),Point(317,220),Scalar(0,255,0),1,8,0);

        rectangle(frame,Point(3,223),Point(36,237),Scalar(255,0,0),-1,8,0);
        rectangle(frame,Point(39,223),Point(79,237),Scalar(255,0,0),-1,8,0);
        rectangle(frame,Point(82,223),Point(119,237),Scalar(255,0,0),-1,8,0);
        rectangle(frame,Point(122,223),Point(184,237),Scalar(255,0,0),-1,8,0);
        rectangle(frame,Point(187,223),Point(223,237),Scalar(255,0,0),-1,8,0);
        rectangle(frame,Point(226,223),Point(260,237),Scalar(255,0,0),-1,8,0);

        putText(frame,"NEW",Point(6,234),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);
        putText(frame,"CALIB",Point(42,234),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);
        putText(frame,"AUTO",Point(85,234),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);
        putText(frame,"SETTINGS",Point(125,234),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);
        putText(frame,"HELP",Point(190,234),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);
        putText(frame,"QUIT",Point(229,234),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);

        if (newPressed==true)
        {
            putText(frame,"Please select processed area with mouse.",Point(15,20),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);
            if (firstclick == true)
            {
                putText(frame,"Input first area cordinate.",Point(15,40),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);
            }
            else
            {
                putText(frame,"Input second area cordinate.",Point(15,40),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255),1,8,false);
            }
        }

        if (helpPressed==true)
        {
            rectangle(frame,Point(3,3),Point(317,220),Scalar(255,255,255),-1,8,0);
            putText(frame,"Line 1",Point(10,20),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 2",Point(10,35),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 3",Point(10,50),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 4",Point(10,65),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 5",Point(10,80),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 6",Point(10,95),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 7",Point(10,110),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 8",Point(10,125),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 9",Point(10,140),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 10",Point(10,155),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 11",Point(10,170),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 12",Point(10,185),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 13",Point(10,200),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);
            putText(frame,"Line 14",Point(10,215),FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,0),1,8,false);

            drawRim();
        }

        imshow("Unedited", frame); // Show unaltered image

        setMouseCallback( "Unedited", onMouse, 0 ); // Capture mouse events in highui window

        cvtColor(frame, frame, CV_BGR2GRAY); // Drop color information

        imgroi = Mat(frame,currentAreaOfInterest);  //

        on_contrastbrightness();

        if(threshold_onoff == 1)
        {
            threshold(imgroi,imgroi,threshold_low_slider,threshold_hi_slider,THRESH_BINARY);
        }

        if(gblur_onoff == 1)
        {
            GaussianBlur(imgroi, imgroi, Size(7,7), 1.5, 1.5);
        }

        if(canny_onoff == 1)
        {
            on_canny (canny_slider, 0 );
        }


        //line(imgroi,Point(rimCenter,0), Point(rimCenter,imgroi.rows),255,1,8,0);

        if (autoPressed == true)
        {
            autoCenter();
        }
        else
        {
            lookDrift();
        }



        imshow("Edited", imgroi);


        if(waitKey(30) >= 0 || quitPressed == true)
        {
            //Mat roi(frame,rect);
            // imwrite("testi.png", frame, compression_params);

            // Reset leds/pins
            bcm2835_gpio_write(PIN1, LOW);
            bcm2835_gpio_write(PIN2, LOW);

            break;
        }



        /*calibPressed = false;
        settingsPressed = false;
        helpPressed = false;
        quitPressed = false;*/
    }
    return 0;
}

void on_takenoaction (int, void* ) {}

void changeLight(int status)
{
    if (status == 0 && leftLight == false)
    {
        bcm2835_gpio_write(PIN1, HIGH);
        leftLight = true;
    }
    if (status == 1 && rightLight == false)
    {
        bcm2835_gpio_write(PIN2, HIGH);
        rightLight = true;
    }
    if (status == 3)
    {
        bcm2835_gpio_write(PIN1, LOW);
        bcm2835_gpio_write(PIN2, LOW);
        rightLight = false;
        leftLight = false;
    }

}

void autoCenter()
{
    int x,y;
    int left=0;
    int right = imgroi.cols;
    int center;

    //centerIsSet = true;

    for(x = 1; x < imgroi.cols; x++ )
    {
        //std::cout<< ":" << (int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) << std::endl;
        if ((int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) == 0)
        {
            left = x;
            //rimL = x;
            break;
        }
    }
    for(x = imgroi.cols-2; x > 1; x-- )
    {
        //std::cout<< ":" << (int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) << std::endl;
        if ((int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) == 0)
        {
            right = x;
            //rimR =x;
            break;
        }
    }

    //center = currentAreaOfInterest.x left + ((right - left)/2) + currentAreaOfInterest.x;
    center = currentAreaOfInterest.x + left + (int) ((right-left)/2);
    rimPoints.push_back(center);
}


void lookDrift()
{
    int x,y,left,right,center;

    int drift = 0;

    if (centerIsSet == true)
    {
        for(x = 1; x < imgroi.cols; x++ )
        {
            if ((int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) == 0)
            {
                left = x;
                rimL = x;
                break;
            }
        }
        for(x = imgroi.cols-2; x > 1; x-- )
        {
            if ((int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) == 0)
            {
                right = x;
                rimR =x;
                break;
            }
        }
        center = currentAreaOfInterest.x + left + (int) ((right-left)/2);

        drift = rimCenter - center;

        if (drift > rimTolerance_slider)
        {
            //std::cout<< drift << std::endl;
            changeLight(0);
        }
        else if (drift < -(rimTolerance_slider))
        {
            //std::cout<< drift << std::endl;
            changeLight(1);
        }
        else
        {
            changeLight(3);
        }
    }
}

int findCenter()
{
    int x,y;
    int left=0;
    int right = imgroi.cols;
    int center;

    centerIsSet = true;

    for(x = 1; x < imgroi.cols; x++ )
    {
        //std::cout<< ":" << (int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) << std::endl;
        if ((int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) == 0)
        {
            left = x;
            rimL = x;
            break;
        }
    }
    for(x = imgroi.cols-2; x > 1; x-- )
    {
        //std::cout<< ":" << (int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) << std::endl;
        if ((int) imgroi.at<unsigned char>((int)(imgroi.rows/2), x) == 0)
        {
            right = x;
            rimR =x;
            break;
        }
    }

    //center = currentAreaOfInterest.x left + ((right - left)/2) + currentAreaOfInterest.x;
    center = currentAreaOfInterest.x + left + (int) ((right-left)/2);

    //std::cout<< ":" << left << " " << right << std::endl;
    return center;

}

void on_contrastbrightness()
{
    double contrast = contrast_slider;
    int x,y;

    //std::cout<< "c" << imgroi.cols << " r" << imgroi.rows << std::endl;

    for(y = 0; y < imgroi.rows; y++ )
    {
        for(x = 0; x < imgroi.cols; x++ )
        {
            imgroi.at<unsigned char>(y, x) = saturate_cast<uchar>( (contrast/100)*( imgroi.at<unsigned char>(y,x) ) + (brightness_slider-(b_slider_max/2)) ); // For greyscale

            /* for( int c = 0; c < 3; c++ ) // For color data
            {

            imgroi.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( (contrast/100)*( imgroi.at<Vec3b>(y,x)[c] ) + (brightness_slider-(b_slider_max/2)) );
            }*/
        }
    }
}
void on_canny( int, void* )
{
    Canny(imgroi, imgroi, 0, canny_slider, 3);
}

void on_threshold (int, void* )
{
    //if (threshold_low_slider > threshold_hi_slider) threshold_low_slider = threshold_hi_slider;
    //if (threshold_hi_slider < threshold_low_slider) threshold_hi_slider = threshold_low_slider;
}

static void onMouse( int event, int x, int y, int, void* )
{
    int i=0;
    int sum=0;

    if( event == CV_EVENT_LBUTTONDOWN )
    {
        if (x>3 && x<36 && y>223 && y<237)
        {
            newPressed = true;
            centerIsSet = false;
            rimCenter = 0;
        }
        if (x>39 && x<79 && y>223 && y<237)
        {
            //calibPressed = true;
            rimCenter = findCenter();
        }
        if (x>82 && x<119 && y>223 && y<237)
        {
            if (autoPressed == true)
            {
                //std::cout<< "tot:" << rimPoints.size() << std::endl; // for testing purposes

                for ( i = 0; i < rimPoints.size(); i++)
                {
                    sum = sum+rimPoints[i];
                }

                //std::cout<< "sum:" << sum << std::endl; // for testing purposes

                rimCenter = (int) (sum/rimPoints.size());

                //std::cout<< "cen:" << rimCenter << std::endl; // for testing purposes

                vector<int>().swap(rimPoints); //Clear array and free mem

                autoPressed = false;
                centerIsSet = true;
            }
            else
            {
                autoPressed = true;
            }
        }
        if (x>122 && x<184 && y>223 && y<237)
        {
            if (settingsPressed == true)
            {
                settingsPressed = false;
                destroyWindow("Settings");
            }
            else
            {
                settingsPressed = true;

                namedWindow("Settings", 1);

                // Setting sliders
                createTrackbar( "Blur", "Settings", &gblur_onoff, 1, on_takenoaction);
                createTrackbar( "Canny On/Off", "Settings", &canny_onoff, 1, on_takenoaction);
                createTrackbar( "Canny edges", "Settings", &canny_slider, canny_slider_max, on_canny );
                createTrackbar( "Contrast", "Settings", &contrast_slider, c_slider_max, on_takenoaction );
                createTrackbar( "Brightness", "Settings", &brightness_slider, b_slider_max, on_takenoaction );
                createTrackbar( "Threshold On/Off", "Settings", &threshold_onoff, 1, on_takenoaction );
                createTrackbar( "Threshold low", "Settings", &threshold_low_slider, threshold_low_max, on_threshold );
                createTrackbar( "Threshold high", "Settings", &threshold_hi_slider, threshold_hi_max, on_threshold );
                createTrackbar( "Tolerance", "Settings", &rimTolerance_slider, rimTolerance_max, on_takenoaction );
            }
        }

        if (x>187 && x<223 && y>223 && y<237)
        {
            if (helpPressed == true)
            {
                helpPressed = false;
                //  bcm2835_gpio_write(PIN1, LOW);
                // bcm2835_gpio_write(PIN2, LOW);
            }
            else
            {
                helpPressed = true;
                // bcm2835_gpio_write(PIN1, HIGH);
                // bcm2835_gpio_write(PIN2, HIGH);
            }
        }
        if (x>226 && x<260 && y>223 && y<237)
        {
            quitPressed = true;
        }

        //std::cout<< "x" << x << " y" << y << std::endl; // for testing purposes

        if (newPressed == true)
        {
            setAreaOfInterest(x,y);
        }
    }
    else
        return;
}

void setAreaOfInterest(int x, int y)
{
    if (x>3 && x<317 && y>3 && y<220)
    {
        if (firstclick == true)
        {
            areaOfInterest = {x,y,1,1};
            firstclick = false;
        }
        else if (firstclick == false)
        {
            if (x < areaOfInterest.x)
            {
                areaOfInterest.width = areaOfInterest.x-x;
                areaOfInterest.x = x;
            }
            else
            {
                areaOfInterest.width = x-areaOfInterest.x;
            }
            if (y < areaOfInterest.y)
            {
                areaOfInterest.height = areaOfInterest.y-y;
                areaOfInterest.y = y;
            }
            else
            {
                areaOfInterest.height = y-areaOfInterest.y;
            }

            currentAreaOfInterest = areaOfInterest;

            firstclick = true;
            newPressed = false;
        }
    }
}

void drawRim()
{
    float x,y,ox,oy,r,a,v1;
    int i;
    int circleXorigin = 200;
    int circleYorigin = 110;
    int circleSize = 80;
    int measuredPoints;

    vector<Point> pointsInner;
    vector<Point> pointsOuter;

    Mat img(Size(320,240),CV_8UC3,Scalar(255,255,255));


    measuredPoints = 20;
    r=100;
    ox = circleXorigin;
    oy = circleYorigin;


    for (i=0; i<=measuredPoints; i++)
    {
        x = ox + r * cos (((360/measuredPoints)*i)*PI/180);
        y = oy + r * sin (((360/measuredPoints)*i)*PI/180);
        pointsOuter.push_back (Point(x,y));
    }

    /* for (vector<Point>::iterator i = pointsOuter.begin(); i != pointsOuter.end(); ++i)
     {
         cout << *i << endl;
         line(img, *i, *i, Scalar(0,0,0), 1, 8, 0);
     }
    */
    r=80;

    for (i=0; i<=measuredPoints; i++)
    {
        x = ox + r * cos (((360/measuredPoints)*i)*PI/180);
        y = oy + r * sin (((360/measuredPoints)*i)*PI/180);
        pointsInner.push_back (Point(x,y));
    }

    for(std::vector<Point>::size_type i = 0; i != pointsOuter.size(); i++)
    {
        line(img, pointsOuter[i], pointsOuter[i], Scalar(0,0,0), 1, 8, 0);
    }

    for(std::vector<Point>::size_type i = 0; i != pointsInner.size(); i++)
    {
        line(img, pointsInner[i], pointsInner[i], Scalar(0,0,0), 1, 8, 0);
    }

    int lineType = 8;

    Point rook_points[1][4];

    const Point* ppt[1] = { rook_points[0] };
    int npt[] = { 4 };

    for (i = 0; i<measuredPoints+1; i++)
    {
        if (i<measuredPoints)
        {
            rook_points[0][0] = pointsOuter[i];
            rook_points[0][1] = pointsOuter[i+1];
            rook_points[0][2] = pointsInner[i+1];
            rook_points[0][3] = pointsInner[i];
        }
        else
        {
            rook_points[0][0] = pointsOuter[i];
            rook_points[0][1] = pointsOuter[0];
            rook_points[0][2] = pointsInner[0];
            rook_points[0][3] = pointsInner[i];
        }


        fillPoly( frame,ppt,npt, 1, Scalar( (255/(measuredPoints+5))*i, (255/(measuredPoints+5))*i, (255/(measuredPoints+5))*i ), lineType );
    }

    //imshow("edges", img);
}

