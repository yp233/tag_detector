/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Answer for Bito Company Recuitment 
//
// Author: yp
//
// Node: video_frame_sub
//
// Finished tasks on this node:
//             1、 subscribe 20s video on topic "/image_raw"
//             2、 implement a rectangle detector to detect the boundary of 2D square tags in the image, visualize
//                          the detected rectangle with OpenCV tool, and measure computation time.
//             3、 evaluate the sharpness of the image by using FFT.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include<stdio.h> 
#include<math.h> 
#include<vector> 
#include<string.h>
#include<ctime>
#include<iostream>
#include<ros/ros.h> 
#include<cv_bridge/cv_bridge.h> 
#include<sensor_msgs/image_encodings.h> 
#include<image_transport/image_transport.h> 
#include<opencv2/opencv.hpp> 
#include<opencv2/highgui/highgui.hpp> 
#include<opencv2/imgproc/imgproc.hpp> 

#include "cvTools.h"
#include "cvDeconv.h"
#include "cvQM.h"
using namespace std;
using namespace cv;

int thresh = 50;
IplImage* img =NULL;
IplImage* img0 = NULL;
CvMemStorage* storage =NULL;
const char * wndname = "square detection";

//return cos(angle)
double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* findSquares4( IplImage* img, CvMemStorage* storage )
{
    clock_t startTime,endTime;
    startTime = clock();
    CvSeq* contours;
    int i, c, l, N = 11;
    CvSize sz = cvSize( img->width & -2, img->height & -2 ); 
 
    IplImage* timg = cvCloneImage( img );
    IplImage* gray = cvCreateImage( sz, 8, 1 );
    IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3);
    IplImage* tgray;
    CvSeq* result;
    double s, t;
 
    // 创建一个空序列用于存储轮廓角点
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );
    cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));
 
    // remove noise
    cvPyrDown( timg, pyr, 7 );
    cvPyrUp( pyr, timg, 7 );
    tgray = cvCreateImage( sz, 8 , 1 );

   // find squares in every color plane of the image
    for( c = 0; c < 3; c++ )
    {
        // 提取 the c-th color plane
        cvSetImageCOI( timg, c+1 );
        cvCopy( timg, tgray, 0 );

        // 尝试各种阈值提取得到的（N=11）
        for( l = 0; l < N; l++ )
        {
            // apply Canny. Take the upper threshold from slider
            // Canny helps to catch squares with gradient shading  
            if( l == 0 )
            {
                cvCanny( tgray, gray, 0, thresh, 5 );
                //使用任意结构元素膨胀图像
                cvDilate( gray, gray, 0, 1 );
            }
            else
            {
                // apply threshold if l!=0:
                cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );
            }

            // 找到所有轮廓并且存储在序列中
            cvFindContours( gray, storage, &contours, sizeof(CvContour),
                    CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

            // 遍历找到的每个轮廓contours
            while( contours )
            {
                //用指定精度逼近多边形曲线
                result = cvApproxPoly( contours, sizeof(CvContour), storage,
                    CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
                  
                if( result->total == 4 &&
                    fabs(cvContourArea(result,CV_WHOLE_SEQ)) > 500 &&
                    fabs(cvContourArea(result,CV_WHOLE_SEQ)) < 100000 &&
                    cvCheckContourConvexity(result) )
                {
                    s = 0;

                    for( i = 0; i < 5; i++ )
                    {
                        // find minimum angle between joint edges (maximum of cosine)
                        if( i >= 2 )
                        {
                            t = fabs(angle(
                                (CvPoint*)cvGetSeqElem( result, i ),
                                (CvPoint*)cvGetSeqElem( result, i-2 ),
                                (CvPoint*)cvGetSeqElem( result, i-1 )));
                            s = s > t ? s : t;
                        }
                    }

                    // if 余弦值 足够小，可以认定角度为90度直角
                    //cos0.1=83度，能较好的趋近直角
                    if( s < 0.1 )  
                        for( i = 0; i < 4; i++ )
                            cvSeqPush( squares,(CvPoint*)cvGetSeqElem( result, i ));
                }           

                // 继续查找下一个轮廓
                contours = contours->h_next;
            }
        }
    }
    cvReleaseImage( &gray );
    cvReleaseImage( &pyr );
    cvReleaseImage( &tgray );
    cvReleaseImage( &timg );
    endTime = clock();
    cout<<"Square Detection cost:"<<(double)(endTime-startTime)/CLOCKS_PER_SEC<<"s"<<endl;
    return squares;
}

//drawSquares函数用来画出在图像中找到的所有正方形轮廓
void drawSquares( IplImage* img, CvSeq* squares )
{
    CvSeqReader reader;
    IplImage* cpy = cvCloneImage( img );
    int i;
    cvStartReadSeq( squares, &reader, 0 );

    // read 4 sequence elements at a time (all vertices of a square)
    for( i = 0; i < squares->total; i += 4 )
    {
        CvPoint pt[4], *rect = pt;
        int count = 4;

        // read 4 vertices
        CV_READ_SEQ_ELEM( pt[0], reader );
        CV_READ_SEQ_ELEM( pt[1], reader );
        CV_READ_SEQ_ELEM( pt[2], reader );
        CV_READ_SEQ_ELEM( pt[3], reader );

        // draw the square as a closed polyline
        cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(0,255,0), 2, CV_AA, 0 );
    }

    cvShowImage( wndname, cpy );
    cvReleaseImage( &cpy );
}

//FFT
void fft(const Mat image)
{
    Mat dst;
    Mat padded; 
    cvtColor(image, dst, CV_RGB2GRAY);
    
    int m = getOptimalDFTSize(dst.rows);
    int n = getOptimalDFTSize(dst.cols);

    copyMakeBorder(dst, padded, 0, m - dst.rows, 0, n - dst.cols, BORDER_CONSTANT, Scalar::all(0));
    Mat planes[] = { Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F) };

    Mat complexI;
    merge(planes, 2, complexI);
    dft(complexI, complexI);  
    split(complexI, planes); 
    magnitude(planes[0], planes[1], planes[0]);
    Mat magI = planes[0];
    magI += Scalar::all(1); 
    log(magI, magI);
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

    int cx = magI.cols / 2;
    int cy = magI.rows / 2;

    Mat q0(magI, Rect(0, 0, cx, cy));
    Mat q1(magI, Rect(cx, 0, cx, cy));
    Mat q2(magI, Rect(0, cy, cx, cy));
    Mat q3(magI, Rect(cx, cy, cx, cy));

    Mat tmp;  
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
    normalize(magI, magI, 0, 1, CV_MINMAX); 

    //imshow("Input Image", I);
    imshow("spectrum magnitude", magI);
    waitKey(20);
}

void imageCalllback(const sensor_msgs::ImageConstPtr& msg) 
{
    
	ROS_INFO("Received \n"); 
    cv::Mat image1 = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::imshow("video",image1);
    cv::waitKey(20); 
//////////////FFT evluate the sharpness of the images/////////////////////////////
    fft(image1);

/////////////Richardson-Lucy算法去除模糊////////////////////////////////////////////
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat ref = cv_bridge::toCvCopy(msg, "bgr8")->image;
    //cv::normalize(image, image, 0.0, 1.0, CV_MINMAX,CV_64F);   
    //cv::normalize(ref, ref, 0.0, 1.0, CV_MINMAX,CV_64F);
    cv::Mat kernel;
    cvTools::getGaussianKernel(kernel, 21, 3);
    cv::Mat deconvlucy;
    int numit = 1;
    cvDeconv::richardsonLucyDeconv(image, deconvlucy, kernel, numit);
    cv::imshow("deblur_video",deconvlucy);
    std::cout<<"Richardson-Lucy : "<<std::endl;
    std::cout<<"PSNR deconv : "<<cvQM::psnr(ref, deconvlucy, cvTools::max(ref))<<std::endl;
    std::cout<<"SSIM deconv : "<<cvQM::ssim(ref, deconvlucy)<<std::endl;


//////////////////////////////square detection////////////////////////////////////
    IplImage img3 = deconvlucy;
    img0=cvCloneImage(&img3);
    drawSquares( img0, findSquares4( img0, storage ) );
    //cvReleaseImage( (&img3) );
    cvReleaseImage( &img0 );
    //cvClearMemStorage( storage );*/
	cv::waitKey(20); 
} 


int main(int argc, char** argv) 
{ 
    storage = cvCreateMemStorage(0);
	ros::init(argc, argv, "video_frame_sub"); 
	ros::NodeHandle n; 
	//cv::namedWindow("video"); 
	cv::startWindowThread(); 
    cv::namedWindow("video"); 
    cvNamedWindow( wndname, 1 );
	image_transport::ImageTransport it(n); 
	image_transport::Subscriber sub = it.subscribe( "image_raw", 1, imageCalllback );
    //cvReleaseImage( &img0 ); 
    cvClearMemStorage( storage ); 
	ros::spin(); 
    cvDestroyWindow( wndname );
	cv::destroyWindow("video"); 
	return 0; 
}

