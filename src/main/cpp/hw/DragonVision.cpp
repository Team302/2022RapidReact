
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

 #include <hw/DragonVision.h>

 //Constructor
DragonVision::DragonVision()
{

 }

 //Destructor
DragonVision::~DragonVision()
{

 }

 cv::Mat  DragonVision::getImage()
{
    return image;
}

 int DragonVision::getWidth(cv::Mat &image)
{
    return image.size().width;
}

 int DragonVision::getHeight(cv::Mat &image)
{
    return image.size().height;
}

 void DragonVision::showCircle(cv::Mat &image, int &width, int &height)
{
    //create circle in the middle of the screen
    int radius= 40;
    cv::Scalar lightgreen(0,255,0);
    cv::Point center(width/2, height/2);
    cv::circle(image, center, radius, lightgreen, 3, 8);
}

 void DragonVision::showCross(cv::Mat &image, int &width, int &height)
{
    //create cross in the middle of the screen
    cv::Scalar lightgreen(0,255,0);
    cv::Point p1(width/2-40, height/2);
    cv::Point p2(width/2+40, height/2);
    cv::Point p3(width/2, height/2-40);
    cv::Point p4(width/2, height/2+40);
    cv::line(image,p1,p2,lightgreen, 3);//horizontal line
    cv::line(image,p3,p4,lightgreen, 3);//vertical line
}

  void DragonVision::RotateVertically(cv::Mat &image, cv::Mat &outputimage)
 {
     cv::flip(image, outputimage, 0);
 } 