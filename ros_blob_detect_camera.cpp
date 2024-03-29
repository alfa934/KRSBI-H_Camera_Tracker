#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>

// RESOLUTION WINDOW 640 x 480

using namespace cv;

SimpleBlobDetector::Params params;
void SetupBlobDetector()
{
    // Filter by color
    params.filterByColor = true;
    params.blobColor = 255; // White blobs
    // Filter by area
    params.filterByArea = true;
    params.minArea = 1000;
    params.maxArea = 100000;
    // Filter by circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.1;
    // Filter by convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.87;
    // Filter by inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;
}

int main(int argc, char **argv)
{
    /********************************** Ros Initialisation **********************************/
    
    ros::init(argc, argv, "blobCamera");

    ros::NodeHandle n;

    ros::Publisher pos_pub = n.advertise<std_msgs::Int32MultiArray>("servo_pos", 1000);
    
    std_msgs::Int32MultiArray pos_msg;

    ros::Rate loop_rate(10);


    /************************************** OpenCV stuff **************************************/
    
    // Opens the camera and check if it is available
    VideoCapture cap(0);

    if(!cap.isOpened())
    {
        std::cout << "Error opening video stream" << std::endl;
        return -1;
    }
    

    /********************* SimpleBlobDetector Setup (parameters, etc...) *********************/
    
    SetupBlobDetector();
    // initialise the blob detector
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);


    /************************************* MAIN LOOP *************************************/
    while(true)
    {
        // Capture frame-by-frame
        Mat frame, flippedFrame;  
        cap >> frame;
        flip(frame, flippedFrame, 1); // flip so that it's mirrored
        if(frame.empty())
            break;


        /******************************** Frame Manipulation ********************************/
        
        // show rectangle/square thing
        Rect rect(100, 100, frame.cols-200, frame.rows-200);
        rectangle(flippedFrame, rect, Scalar(0, 255 ,0), 2);

        // Convert from RGB to HSV color space
        Mat hsv;
        cvtColor(flippedFrame, hsv, COLOR_BGR2HSV);

        // Threshold the image based on color, try to calibrate for better results
        int low_H  =   0, 
            low_S  =  80, 
            low_V  = 140;
        int high_H = 180, 
            high_S = 255, 
            high_V = 255;
        Mat mask;     // might need to calibrate Scalar() values
        inRange(hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), mask);

        // Apply morphological operations to remove noise and fill small gaps in the blobs
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
        morphologyEx(mask, mask, MORPH_OPEN, kernel);


        /******************************* BLOB DETECTION *******************************/

        // Detect the BIGGEST blob in the masked image
        std::vector<KeyPoint> keypoints;
        detector->detect(mask, keypoints);

        int maxArea = 0;
        Point maxPt;
        for (int i = 0; i < keypoints.size(); i++) 
        {
            if (keypoints[i].size > maxArea) 
            {
                maxArea = keypoints[i].size;
                maxPt = keypoints[i].pt;
                
                // send position of the BIGGEST blob
                if(!rect.contains(maxPt))
                {
                    int x = (int)maxPt.x;
                    int y = (int)maxPt.y;

                    pos_msg.data.clear();
                    pos_msg.data.push_back(x);
                    pos_msg.data.push_back(y);
                    pos_pub.publish(pos_msg);

                    printf("X,Y : [ %d, %d ] \n", x, y);
                }   
            }
        }
        
        // Draw detected blobs as circles
        Mat img_keypoints;
        drawKeypoints(  flippedFrame, 
                        keypoints, 
                        img_keypoints, 
                        Scalar(0, 0, 255), 
                        DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        

        /******************************** SHOW FRAMES ********************************/
        
        // Show the image with keypoints
        imshow("Blob detection", img_keypoints);
        
        char c = (char)waitKey(25);
        if(c == 27 || c == 'q')
            break;
    }

    // Finish processing images
    cap.release();
    destroyAllWindows();

    return 0;
}

