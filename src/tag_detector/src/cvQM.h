
#ifndef DEBLURRINGCV_CVQUALITYMEASURES_H
#define DEBLURRINGCV_CVQUALITYMEASURES_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
 * Common quality metrics used to evaluate deconvolution algorithms
 * Based on an openCV tutorial
 * http://docs.opencv.org/2.4/doc/tutorials/highgui/video-input-psnr-ssim/video-input-psnr-ssim.html
 *
 *  - SSIM (Structural Similarity Measure)
 *  - PSNR (Peak Signal to Noise Ratio)
 *
 */
class cvQM
{
public:

    /*
     * Peak Signal to Noise Ratio
     *  - Gives a measure of similarity of two images, based on the
     *    mean square error.
     *
     * @param i1 : First image to compare (usually, reference image)
     * @param i2 : Second image to compare (usually, degraded image)
     * @param dyn : dynamic of the reference image
     */
    static double psnr(const cv::Mat& i1, const cv::Mat & i2, double dyn=1.0);

    /*
     * Structural Similarity Measure
     *  - Gives a measure of similarity of two images, based on
     *  statistical measures (mean, covariance, variance)
     *
     * @param i1 : First image to compare (usually, reference image)
     * @param i2 : Second image to compare (usually, degraded image)
     * @param dyn : dynamic of the reference image
     */
    static double ssim(const cv::Mat& i1, const cv::Mat i2, double dyn=1.0);
};


double cvQM::psnr(const cv::Mat& i1, const cv::Mat & i2, double dyn)
{
    cv::Mat s1;
    cv::absdiff(i1, i2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    cv::Scalar s = sum(s1);         // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if( sse <= 1e-10) // for small values return zero
        return 0;
    else
    {
        double  mse =sse /(double)(i1.channels() * i1.total());
        double result = 10.0*log10((dyn*dyn)/mse);
        return result;
    }
}

double cvQM::ssim(const cv::Mat &i1, const cv::Mat i2, double dyn)
{
    const double C1 = (0.01*dyn)*(0.01*dyn), C2 = (0.03*dyn)*(0.03*dyn);
    /***************************** INITS **********************************/
    int d     = CV_32F;

    cv::Mat I1, I2;
    i1.convertTo(I1, d);           // cannot calculate on one byte large values
    i2.convertTo(I2, d);

    cv::Mat I2_2   = I2.mul(I2);        // I2^2
    cv::Mat I1_2   = I1.mul(I1);        // I1^2
    cv::Mat I1_I2  = I1.mul(I2);        // I1 * I2

    /*************************** END INITS **********************************/

    cv::Mat mu1, mu2;   // PRELIMINARY COMPUTING
    GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
    GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);

    cv::Mat mu1_2   =   mu1.mul(mu1);
    cv::Mat mu2_2   =   mu2.mul(mu2);
    cv::Mat mu1_mu2 =   mu1.mul(mu2);

    cv::Mat sigma1_2, sigma2_2, sigma12;

    cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;

    cv::GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;

    cv::GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;

    ///////////////////////////////// FORMULA ////////////////////////////////
    cv::Mat t1, t2, t3;

    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

    cv::Mat ssim_map;
    divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;

    cv::Scalar mssim = mean( ssim_map ); // mssim = average of ssim map
    return mssim[0];
}


#endif //DEBLURRINGCV_CVQUALITYMEASURES_H
