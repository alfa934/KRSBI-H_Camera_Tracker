#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    // Open the default camera
    VideoCapture cap(0);

    // Check if camera opened successfully
    if(!cap.isOpened()){
        std::cout << "Error opening video stream" << std::endl;
        return -1;
    }

    // Set up the SimpleBlobDetector parameters
    SimpleBlobDetector::Params params;

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

    // Set up the blob detector with the above parameters
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // Capture loop
    while(true){
        // Capture frame-by-frame
        Mat frame, flippedFrame;
        cap >> frame;

        // Flip the frame
        flip(frame, flippedFrame, 1);
        // Check if frame is empty
        if(frame.empty())
            break;

        // Convert from RGB to HSV color space
        Mat hsv;
        cvtColor(flippedFrame, hsv, COLOR_BGR2HSV);

        // Threshold the image based on color
        Mat mask;
        inRange(hsv, Scalar(0, 100, 179), Scalar(100, 255, 255), mask);

        // Apply morphological operations to remove noise and fill small gaps in the blobs
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
        morphologyEx(mask, mask, MORPH_OPEN, kernel);

        // Detect blobs in the masked image
        std::vector<KeyPoint> keypoints;
        detector->detect(mask, keypoints);

        // Draw detected blobs as circles
        Mat im_with_keypoints;
        drawKeypoints(flippedFrame, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Show the image with keypoints
        imshow("Blob detection", im_with_keypoints);

        // Exit on ESC key
        char c=(char)waitKey(25);
        if(c == 27 || c == 'q')
            break;
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    destroyAllWindows();

    return 0;
}