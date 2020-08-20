#include <iostream>
#include <cmath>
#include <array>
#//include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include </home/robotics/catkin_ws/src/vision_opencv-opencv4/cv_bridge/include/cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv4/opencv2/opencv.hpp>
//#include <opencv4/opencv2/imgproc/imgproc.hpp>
//#include <opencv4/opencv2/highgui/highgui.hpp>
#include "img_recog/points.h"

cv::Mat inner;
cv::Mat rot;
cv::Mat mov;
cv::Mat outer;
cv::Mat mtx;
cv::Mat dist;
cv::Mat world;
cv::Mat rect1;
cv::Mat rect2;
cv::Mat a;

std::array<float, 2> b;
std::array<float, 2> b_3;
std::array<float, 2> c;
std::array<float, 2> c2;
std::array<float, 2> target;



int width, height;
int dis,i;
cv::Size imageSize(width, height);
cv::Mat mapx, mapy;
cv::Mat undistorted;

void triangle(cv::Mat input, int x, int y){
    cv::Point pt[3];
    pt[0] = cv::Point(x, y);
    pt[1] = cv::Point(x-10, y+20);
    pt[2] = cv::Point(x+10, y+20);
    //cv::circle(image, pt1, 0, scalar, -1);
    //cv::circle(image, pt2, 0, scalar, -1);
    //cv::circle(image, pt3, 0, scalar, -1);
    cv::fillConvexPoly(input, pt, 3, cv::Scalar(0,0,255));
}

std::array<float, 2> project_n_undistort(cv::Mat input){
    cv::Mat b;
    cv::Mat new_b;
    cv::Mat outputUndistortedPoints;
    //static float output_new_b[2];
    b = a * input;
    b = b / b.at<float>(0, 2);
    new_b = (cv::Mat_<float>(2, 1) << b.at<float>(0,0),b.at<float>(1,0));
    cv::undistortPoints(new_b, outputUndistortedPoints, mtx, dist, cv::noArray(), mtx);
    //output_new_b[0] = new_b.at<float>(0,0);, int x, int y)
    //output_new_b[1] = new_b.at<float>(1,0);
    return {new_b.at<float>(0,0),new_b.at<float>(1,0)};
}

class Sight
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub1_;
  image_transport::Publisher sight_pub_;
  image_transport::Publisher undistort_pub_;
  ros::Publisher point_pub_ = nh_.advertise<img_recog::points>("target_point", 1);

  image_transport::Publisher spectrum_pub_;
  //image_transport::Subscriber image_sub2_;
  
  
public:
  // コンストラクタ
  Sight()
    : it_(nh_)
  {
    // カラー画像をサブスクライブ
    image_sub1_ = it_.subscribe("/image_raw", 1, 
      &Sight::callback, this);
    sight_pub_ = it_.advertise("Sight", 1);
    undistort_pub_ = it_.advertise("undistort_image", 1);
 
    //spectrum_pub_ = it_.advertise("Rotated_spectrum_image", 1);
    //image_sub2 = it_.subscribe("spectrum/image_raw",
    //  &Sight::rotate_callback, this);
    
    cv::namedWindow("OPENCV_WINDOW");
 }

  // デストラクタ
  ~Sight()
  {
    // 全てのウインドウは破壊
    cv::destroyAllWindows();
  }

  // コールバック関数
  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr,cv_ptr2;
    img_recog::points point;
    try
    {
      // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Size imageSize(width, height);
    imageSize.width = cv_ptr->image.cols;
    imageSize.height = cv_ptr->image.rows;
    // 歪みマップを求める  
    cv::initUndistortRectifyMap(mtx, dist, cv::Mat(), mtx, imageSize, CV_32FC1, mapx, mapy);
    //歪み修正
    cv::remap(cv_ptr->image, cv_ptr2->image, mapx, mapy, cv::INTER_LINEAR);
    undistort_pub_.publish(cv_ptr2->toImageMsg());

    
    target[0] = (c[0]+c2[0])/2;
    target[1] = (c[1]+c2[1])/2;
    //std::cout<<target<<std::endl;
    
    point.point1_x = target[0];
    point.point1_y = target[1];
    point.point2_x = c[0];
    point.point2_y = c[1];
    point.point3_x = c2[0];
    point.point3_y = c2[1];
    
    point_pub_.publish(point);
    
    //三角形描画
    //input distance
    triangle(cv_ptr2->image,(int)b[0], (int)b[1]);
    cv::rectangle(cv_ptr2->image, cv::Point((int)c[0], (int)c[1]), cv::Point((int)c2[0], (int)c2[1]),CV_RGB(0,255,0),2, cv::LINE_8, 0);
    cv::putText(cv_ptr2->image, dis+"m", cv::Point((int)(c[0]+10), (int)b[1]), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,0,255), 2);
    
    //>3m
    triangle(cv_ptr2->image,(int)b_3[0], (int)b_3[1]);
    cv::putText(cv_ptr2->image, ">3m", cv::Point((int)(b_3[0]-90), (int)b_3[1]), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,0,255), 2);

    sight_pub_.publish(cv_ptr2->toImageMsg());

/*  
    cv::Mat hsv_image, color_mask, gray_image, cv_image2, cv_image3;
    // RGB表色系をHSV表色系へ変換して、hsv_imageに格納
    cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
    
    // // 色相(Hue), 彩度(Saturation), 明暗(Value, brightness)
    // 指定した範囲の色でマスク画像color_mask(CV_8U:符号なし8ビット整数)を生成 
    // マスク画像はcv:c:Mat_<float>(1, 5) 指定した範囲の色に該当する要素は255(8ビットすべて1)、それ以外は0        
    cv::inRange(hsv_image, cv::Scalar(150, 100, 50, 0) , cv::Scalar(180, 255, 255, 0), color_mask);
    cv::bitwise_and(cv_ptr->image, cv_ptr->image, cv_image2, color_mask);.at<float>
    // エッジを検出するためにCannyアルゴリズムを適用
    cv::Canny(cv_ptr3->image, cv_ptr3->image, 15.0, 30.0, 3);

    // ウインドウに円を描画。中心(100, 100), 半径20[pixel]、色緑
    cv::circle(cv_ptr->image, cv::Point(100, 100), 20, CV_RGB(0,255,0));

    // 画像サイズを縦横半分に変更
    cv::Mat cv_half_image, cv_half_image2, cv_half_image3;
    cv::resize(cv_ptr->image, cv_half_image,cv::Size(),0.5,0.5);
    cv::resize(cv_image2, cv_half_image2,cv::Size(),0.5,0.5);
    cv::resize(cv_ptr3->image, cv_half_image3,cv::Size(),0.5,0.5);

    std::cout << "Mat type1: " << cv_half_image.type() << std::endl;
    //std::cout << "Mat type3: " << cv_ptr3->image.type() << std::endl;
    // ウインドウ表示
    cv::imshow("Original Image", cv_half_image);
    cv::imshow("Result Image", cv_half_image2);
    //cv::imshow("Edge Image", cv_ptr3->image);/////////////
    cv::waitKey(3);
    
    //エッジ画像をパブリッシュ。OpenCVからROS形式にtoImageMsg()で変換。
    image_pub_.publish(cv_ptr3->toImageMsg());
    */
    //std::cout << "version =" <<CV_VERSION<<std::endl;
  }
};

int main(int argc, char** argv)
{ 
  //inner = np.array([[614.470458984375, 0, 1036.625311183976, 0], [0, 730.07958984375, 773.5514250291162, 0], [0, 0, 1, 0]])#内部パラメーター 2048*1536
  //outer = np.dot(np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),np.array([[1,0,0,0],[0,1,0,88],[0,0,1,0],[0,0,0,1]]))外部パラメーター 2048*1536
  //inner = np.array([[307.6017761230469, 0, 500.9403417435533, 0], [0, 342.7832641601562, 379.5612670888731, 0], [0, 0, 1, 0]])#内部パラメーター 1024*768
  inner = (cv::Mat_<float>(3, 4) << 307.6017761230469, 0, 512, 0,
           0, 342.7832641601562, 384, 0, 
           0, 0, 1, 0);//内部パラメーター 1024*768

  rot = (cv::Mat_<float>(4, 4) << cos(M_PI *2/180),0,-sin(M_PI *2/180),0,
        0,1,0,0,
        sin(M_PI *2/180),0,cos(M_PI *2/180),0,
        0,0,0,1);

  
  mov = (cv::Mat_<float>(4, 4) << 1,0,0,0,
        0,1,0,85,
        0,0,1,0,
        0,0,0,1);  

  outer = rot * mov;//外部パラメーター 1024*768

  //mtx = np.array([[898.740576, 0, 1047.656497], [0, 899.086181, 771.153764], [0, 0, 1]])#camera_matrix 2048*1536
  //dist = np.array([-0.202203, 0.029082, 0.000228target, -0.0005, 0])#distortion coefficient 2048*1536
  
  mtx = (cv::Mat_<float>(3, 3) << 437.7949802332059, 0, 502.174798617283,
         0, 437.0115481181825, 380.0865280376445,
         0, 0, 1);//camera_matrix 1024*768

  dist = (cv::Mat_<float>(1, 5) << -0.2171050089304354, 0.03583959522345996, 0.0009684942829751926, 0.0006630572393120683, 0);//distortion coefficient 1024*768

  std::cout << "Enter current distance(m)" << std::endl;
  std::cin >> dis;

  i = dis * 1000 + 111.5;

  world = (cv::Mat_<float>(4, 1) << 0, (i + 93) * tan(M_PI *2.85/180), i , 1);
  
  rect1 = (cv::Mat_<float>(4, 1) << (i+93) * tan(M_PI *1.2/180),(i + 93) * tan(M_PI *2.85/180), i, 1);
  
  rect2 = (cv::Mat_<float>(4, 1) << -(i+93) * tan(M_PI *1.2/180),-(i + 93) * tan(M_PI *5.75/180), i, 1);

  a = inner * outer;

  b = project_n_undistort(world);
  b_3 = project_n_undistort((cv::Mat_<float>(4, 1) << 0, 3204.5 * tan(M_PI *2.85/180), 3111.5, 1));
  c = project_n_undistort(rect1);
  c2 = project_n_undistort(rect2);
  //std::cout<<"OpenCV Version used:"<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<std::endl;
  ros::init(argc, argv, "sight");
  Sight ic;
  std::cout << "check in viewer" << std::endl;
  ros::spin();
  return 0;
}