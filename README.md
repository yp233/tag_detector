# tag_detector
Answer for Bito Company by yp
# 1、STEPS
### STEP 1 : Compile project
```
    $cd tag_detector/src
    $catkin_make  
```  
### STEP 2 : launch the node for publishing video
```
    $roscore
```
##### type in a new terminal
```
    $source devel/setup.bash
    $rosrun tag_detector video_frame_pub
```
### STEP 3 : launch the node for subscribing video
##### type in a new terminal
```
    $source devel/setup.bash
    $rosrun tag_detector video_frame_sub
```
### STEP 4 : call rosservice
##### type in a new terminal
```
    $source devel/setup.bash
    $rosservice call set_blur_window_size 11 11 
        (the window size should be odd and > 0 )
```
# 2、Result
The relationship between nodes:

![avatar](/home/yp/Pictures/rqt.png)

The result(one frame of all images):

![avatar](/home/yp/Pictures/result.png)

# 3、problems
In the question 8 , i think the question should be solved like this:
```
J = I o K；  
```
I represents original image;
o represents convolution;
K represents blur kernel;
J represents the blurred image;

So we should implement a deblur filter like this:   
```
FFT(J) = FFT(I o K) = FFT(I) × FFT(K);
I = IFFT( FFT(J) / FFT(K) );
```

And I take the wiener filter and Richardson_Lucy Algorithm as a comparison

![avatar](/home/yp/Pictures/deconv.jpeg)

_The right upper image is the blurred image with Gaussian kernel._

_The right lower image named Deconv image was got by wiener filter ,somehow the content of image was damaged. Because of time , i give up the method._

_The left lower image was got by Richardson_Lucy Algorithm.However,it needs many iterations to improve the result, so applying it to the video deblurr problem is not a good idea!_

_So my result was not as good as i imagined because of lack of time._
