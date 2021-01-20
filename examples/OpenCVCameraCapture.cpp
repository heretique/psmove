//#include "opencv2/core.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/highgui.hpp"
//
//
//int main(int c, char* v[])
//{ 
//    cv::VideoCapture cam(0);
//    cv::Mat image;
//    cv::OutputArray outImage(image);
//    while (true)
//    {
//        cam.read(outImage);
//        cv::imshow("frame", outImage);
//        if (cv::waitKey(1) == 'q')
//            break;
//    }
//
//    return 0;
//}


#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudafilters.hpp"

using namespace cv;
int main(int, char**)
{
    VideoCapture cap(0);
    if (!cap.isOpened()) return -1;
    cap.set(CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(CAP_PROP_FRAME_HEIGHT, 1080);

    Mat frame;

    cuda::GpuMat gframe, gedges;
    Ptr<cuda::Filter> gaussFilter = cuda::createGaussianFilter(gedges.type(), gedges.type(), Size(7, 7), 1.5, 1.5);
    Ptr<cuda::CannyEdgeDetector> cannyEdge = cuda::createCannyEdgeDetector(0, 30, 3); 
    namedWindow("edges", WINDOW_AUTOSIZE);
    for (;;)
    {
        cap >> frame;
        gframe.upload(frame);
        cuda::cvtColor(gframe, gedges, COLOR_BGR2GRAY);
        //cvtColor(frame, edges, COLOR_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
        gaussFilter->apply(gedges, gedges);
        cannyEdge->detect(gedges, gedges);
        // Canny(edges, edges, 0, 30, 3);
        gedges.download(frame);
        imshow("edges", frame);
        if (waitKey(30) >= 0) break;
    }
    return 0;
}