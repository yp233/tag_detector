

#ifndef DEBLURRINGCV_OPENCVUTILS_H
#define DEBLURRINGCV_OPENCVUTILS_H

#include <string>
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "shift.hpp"

/*
 * Provides general purpose tools for image processing
 * TODO : Change curvature implementation (do everything w/out OpenCV?)
 * TODO : Factorize finite difference schemes
 */
class cvTools{
public:

    /*
     * Loads an image file into a cv::Mat
     *
     * @param imagePath : path to the image to be loaded
     * @return a cv::Mat containing the image bands
     */
    static cv::Mat loadImageToGrayCvMat(std::string imagePath);

    /*
     * Blurs an image with a given blurring kernel with mirror boundary conditions
     *
     * @param image : the image to be blurred
     * @param kernel : the blurring kernel
     */
    static void blurredGrayImage(cv::Mat & image, const cv::Mat & kernel);

    /*
     * Applies a gaussian noise of given variance and zero-mean to an input image
     *
     * @param image : the image to be "noised"
     * @param var : the variance of the gaussian noise
     */
    static void applyGaussianNoise(cv::Mat & image, double var);

    /*
     * Displays an input image in a specific window
     *
     * @param image : the image to be displayed
     * @param windowName : the name of the window
     */
    static void displayImage(const cv::Mat & image, std::string windowName = "Image");

    /*
     * Applies gaussian blur + noise to an image
     *
     * @param image : the image to be degraded
     * @param kernel : the blurring kernel
     * @param sigma : the standard deviation of the gaussian noise
     */
    static void blurNoise(cv::Mat & image, const cv::Mat & kernel, double sigma);

    /*
     * Matlab-like operation on an image. Shifts the pixels values.
     *
     * @param image : the image to be shifted
     * @param shift : the 2-D parameters of the shift (can be float-valued)
     * @return cv::Mat : shifted matrix
     */
    static cv::Mat circshift(const cv::Mat & image, cv::Point2f shift);

    /*
     * Matlab-like operation on an image. Pads the image with given borders and values
     *
     * @param image : the image to be padded
     * @param paddedImage : padded output image
     * @param top : top border size
     * @param bottom : bottom border size
     * @param left : left border size
     * @param right : right border size
     * @param borderType : type of border to be used (cv::BORDER_* available)
     * @param value : value to use in the border (for instance when borderType = BORDER_CONSTANT)
     */
    static void padarray(const cv::Mat & image,
                         cv::Mat & paddedImage,
                         int top, int bottom,
                         int left, int right,
                         int borderType,
                         double value);

    /*
     * Converts a blurring kernel (psf matrix) to an Optical Transfer Function (psf in frequency domain)
     *  - Pads the psf with zeros to obtain desired OTF size
     *  - Applies fourier transform to the padded psf
     *
     *  @param psf : input blurring kernel
     *  @param otf : output optical transfer function
     *  @param s : size of the desired otf
     */
    static void psf2otf(const cv::Mat & psf, cv::Mat & otf, const cv::Size & s);

    /*
     * Obtain a 2-D gaussian blurring kernel of given standard deviation and size
     *
     * @param kernel : output blurring kernel
     * @param size : size of the blurring kernel
     * @param sigma : spatial standard deviation of the kernel
     */
    static void getGaussianKernel(cv::Mat & kernel, int size, double sigma);

    /*
     * Obtain a string describing the data type corresponding to a cv::Mat type
     *
     * @param number : the number given by cv::Mat::getType()
     * @return std::string describing OpenCV datatype
     */
    static std::string getImageType(int number);

    /*
     * Computes the maximum value of the first band of a cv::Mat
     *
     * @param m : input cv::Mat
     * @return double maximum value
     */
    static double max(const cv::Mat & m);

    /*
     * Computes the maximum value of the first band of a cv::Mat
     *
     * @param m : input cv::Mat
     * @return double maximum value
     */
    static double min(const cv::Mat & m);

    /*
     * Computes the standard deviation of the first band of a cv::Mat
     *
     * @param m : input cv::Mat
     * @return double standard deviation
     */
    static double std(const cv::Mat & m);

    /*
     * Computes the "forward" col gradient of an image
     *
     */
    static cv::Mat gradXPlus(const cv::Mat & m);

    /*
     * Computes the "forward" row gradient of an image
     *
     */
    static cv::Mat gradYPlus(const cv::Mat & m);

    /*
     * Computes the "forward" col gradient of an image
     *
     */
    static cv::Mat gradXMinus(const cv::Mat & m);

    /*
     * Computes the "forward" row gradient of an image
     *
     */
    static cv::Mat gradYMinus(const cv::Mat & m);

    /*
     * Computes the "centered" col gradient of an image
     *
     */
    static cv::Mat gradXCenter(const cv::Mat & m);

    /*
     * Computes the "centered" col gradient of an image
     *
     */
    static cv::Mat gradYCenter(const cv::Mat & m);

    /*
     * Computes the divergence of the normalized gradient of an image
     */
    static cv::MatExpr curvature(const cv::Mat & m);

};

double cvTools::std(const cv::Mat & m)
{
    cv::Scalar std, mean;
    cv::meanStdDev(m,mean,std);
    return std[0];
}

double cvTools::max(const cv::Mat & m)
{
    double min, max;
    cv::minMaxLoc(m, &min, &max);
    return max;
}


double cvTools::min(const cv::Mat & m)
{
    double min, max;
    cv::minMaxLoc(m, &min, &max);
    return min;
}

std::string cvTools::getImageType(int number)
{
    // find type
    int imgTypeInt = number%8;
    std::string imgTypeString;

    switch (imgTypeInt)
    {
        case 0:
            imgTypeString = "8U";
            break;
        case 1:
            imgTypeString = "8S";
            break;
        case 2:
            imgTypeString = "16U";
            break;
        case 3:
            imgTypeString = "16S";
            break;
        case 4:
            imgTypeString = "32S";
            break;
        case 5:
            imgTypeString = "32F";
            break;
        case 6:
            imgTypeString = "64F";
            break;
        default:
            break;
    }

    // find channel
    int channel = (number/8) + 1;

    std::stringstream type;
    type<<"CV_"<<imgTypeString<<"C"<<channel;

    return type.str();
}

void cvTools::getGaussianKernel(cv::Mat & kernel, int size, double sigma)
{
    cv::Mat kernelX = cv::getGaussianKernel(size,sigma);
    cv::Mat kernelY = cv::getGaussianKernel(size,sigma);
    kernel = kernelX * kernelY.t();
}

void cvTools::padarray(const cv::Mat & image,cv::Mat & paddedImage, int top, int bottom, int left, int right, int borderType, double value)
{
    cv::copyMakeBorder(image, paddedImage, top, bottom, left, right, borderType, value);
}

cv::Mat cvTools::circshift(const cv::Mat &image, cv::Point2f shiftCoords)
{
    cv::Mat out;
    shift(image, out, shiftCoords, cv::BORDER_REFLECT);
    return out;
}

void cvTools::psf2otf(const cv::Mat & psf, cv::Mat & otf, const cv::Size & s)
{
    padarray(psf, otf, 0, s.height - psf.rows, 0, s.width - psf.cols, cv::BORDER_CONSTANT, 0);
    cv::dft(otf,otf,cv::DFT_COMPLEX_OUTPUT);
}
cv::Mat cvTools::loadImageToGrayCvMat(std::string imagePath)
{
    return cv::imread(imagePath.c_str(), CV_LOAD_IMAGE_ANYDEPTH);
}

void cvTools::blurNoise(cv::Mat & image, const cv::Mat & kernel, double sigma)
{
    blurredGrayImage(image,kernel);
    applyGaussianNoise(image,sigma);
}

void cvTools::applyGaussianNoise(cv::Mat & image, double sigma)
{
    cv::Mat noise = cv::Mat(image.size(),CV_64F);
    cv::randn(noise, 0, sigma);
    image = image + noise;
}

void cvTools::blurredGrayImage(cv::Mat & image,const cv::Mat & kernel)
{
    cv::filter2D(image, image, image.depth(), kernel,cv::Point(-1,-1), 0, cv::BORDER_REFLECT);
}

void cvTools::displayImage(const cv::Mat &image, std::string windowName)
{
    namedWindow( windowName, cv::WINDOW_AUTOSIZE );
    imshow( windowName, image);
    cv::waitKey(0);
}

cv::Mat cvTools::gradXPlus(const cv::Mat & m)
{
    cv::Mat kernel(2,1,CV_64F);
    kernel.at<double>(0) = -1;
    kernel.at<double>(1) = 1;
    cv::Mat out;
    cv::filter2D(m, out, m.depth(), kernel,cv::Point(-1,-1), 0, cv::BORDER_REFLECT);
    return out;
}

cv::Mat cvTools::gradYPlus(const cv::Mat & m)
{
    cv::Mat kernel(1,2,CV_64F);
    kernel.at<double>(0) = -1;
    kernel.at<double>(1) = 1;
    cv::Mat out;
    cv::filter2D(m, out, m.depth(), kernel,cv::Point(-1,-1), 0, cv::BORDER_REFLECT);
    return out;
}

cv::Mat cvTools::gradXMinus(const cv::Mat & m)
{
    cv::Mat out = gradXPlus(m);
    return circshift(out, cv::Point2f(1,0));
}

cv::Mat cvTools::gradYMinus(const cv::Mat & m)
{
    cv::Mat out = gradYPlus(m);
    return circshift(out, cv::Point2f(0,1));
}

cv::Mat cvTools::gradXCenter(const cv::Mat & m)
{
    cv::Mat kernel(3,1,CV_64F);
    kernel.at<double>(0) = -1/2.0;
    kernel.at<double>(1) = 0;
    kernel.at<double>(2) = 1/2.0;
    cv::Mat out;
    cv::filter2D(m, out, m.depth(), kernel,cv::Point(-1,-1), 0, cv::BORDER_REFLECT101);
    return out;
}

cv::Mat cvTools::gradYCenter(const cv::Mat & m)
{
    cv::Mat kernel(1,3,CV_64F);
    kernel.at<double>(0) = -1/2.0;
    kernel.at<double>(1) = 0;
    kernel.at<double>(2) = 1/2.0;
    cv::Mat out;
    cv::filter2D(m, out, m.depth(), kernel,cv::Point(-1,-1), 0, cv::BORDER_REFLECT101);
    return out;
}

cv::MatExpr cvTools::curvature(const cv::Mat & m)
{
    cv::Mat p1 = gradXPlus(m);
    cv::Mat p2 = gradYPlus(m);

    cv::Mat p1_c = gradXCenter(m);
    cv::Mat p2_c = gradYCenter(m);

    double * p1_pnt = (double*)p1.ptr();
    double * p2_pnt = (double*)p2.ptr();
    double * p1_c_pnt = (double*)p1_c.ptr();
    double * p2_c_pnt = (double*)p2_c.ptr();

    // Gradient normalisation
    for (int i = 0; i < m.rows * m.cols; i++)
    {
        p1_pnt[i] /= std::sqrt(p1_pnt[i]*p1_pnt[i] + p2_c_pnt[i]*p2_c_pnt[i]);
        p2_pnt[i] /= std::sqrt(p2_pnt[i]*p2_pnt[i] + p1_c_pnt[i]*p1_c_pnt[i]);
    }

    p1 = gradXMinus(p1);
    p2 = gradYMinus(p2);

    return p1 + p2;
}

#endif //DEBLURRINGCV_OPENCVUTILS_H
