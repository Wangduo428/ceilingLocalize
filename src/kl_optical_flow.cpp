#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std; 

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include<opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include<opencv2/opencv.hpp>



struct cameraParamater
{
  double cx,cy,fx,fy;
};

cv::Point3f points2DTo3D(cv::Point2f p2d, cameraParamater camera, double height);

int main(int argc, char** argv)
{
  string dataset = "/home/wangduo/Code/ceilingLocalize/dataset/image_";
  
  cv::Mat color,last_color;
  list<cv::Point2f> keypoints;
  
  //相机参数
  cameraParamater camera;
  camera.cx=644;
  camera.cy=484;
  camera.fx=518.0;
  camera.fy=519.0;
  double height = 2.3;
  
  for(int i = 0; i<300; i++)
  {
    ostringstream filename;
    filename << dataset<<i<<".bmp";
    color = cv::imread(filename.str().c_str());
    if(i == 0)
    {
      //对第一帧提取Fast特征点
      vector<cv::KeyPoint> kps;
      cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
      detector->detect(color, kps);
      for(auto kp:kps)
	keypoints.push_back(kp.pt);
      last_color = color;
      continue;
    }
    
    vector<cv::Point2f> prev_keypoints, next_keypoints;
    
    for(auto kp:keypoints)
      prev_keypoints.push_back(kp);
    vector<unsigned char> status;
    vector<float> error;
    
    cv::calcOpticalFlowPyrLK(last_color, color, prev_keypoints, next_keypoints, status, error);
    prev_keypoints.clear();
    vector<cv::Point3f> srcPoints;
    
    vector<cv::KeyPoint> kp1,kp2;
    
    //删除跟丢的点
    int j = 0;
    for(auto iter = keypoints.begin(); iter!=keypoints.end(); j++)
    {
      if(status[j] = 0)
      {
	iter = keypoints.erase(iter);
	continue;
      }
      prev_keypoints.push_back(*iter);
      srcPoints.push_back(points2DTo3D(*iter, camera, height));
      cv::KeyPoint kp;
      //kp.pt.x = iter->x;
      //kp.pt.y = iter->y;
      kp.pt = *iter;
      kp1.push_back(kp);
      *iter = next_keypoints[j];
      iter++;
    }
    
    if(keypoints.size() == 0)
    {
      cout<<"All points are lost!"<<endl;
      break;
    }
    
    for(auto p2d:next_keypoints)
    {
      cv::KeyPoint kp;
      kp.pt = p2d;
      //kp.pt.y = p2d.y;
      kp2.push_back(kp);
    }
    
    cout<<"tracked keypoints: "<<keypoints.size()<<endl;
    
    //solvePnP 求解运动
    cv::Mat rvec, tvec, inliers;
    double camera_matrix[3][3] = {{camera.fx, 0, camera.cx},{0, camera.fy, camera.cy},{0,0,1}};
    cv::Mat matcamera_matrix(3,3,CV_64F,camera_matrix);
    cv::solvePnPRansac(srcPoints, next_keypoints, matcamera_matrix, cv::Mat(), rvec, tvec, false, 100, 2, 0.99, inliers);
    
    //画出内点匹配
//     vector<cv::DMatch> inliers_match;
//     for(unsigned k = 0; k<inliers.rows; k++)
//     {
// //       cout<<inliers.ptr<int>(k)[0]<<endl;
// //       cout<<inliers.ptr<int>(k)[1]<<endl;
//       cv::DMatch dm(inliers.ptr<int>(k)[0], inliers.ptr<int>(k)[0], 1.0);
//       inliers_match.push_back(dm);
// //       inliers_match[k].queryIdx = inliers.ptr<int>(k)[0];
// //       inliers_match[k].trainIdx = inliers.ptr<int>(k)[1];
//     }
//     cv::Mat inliersMatchShow;
//     cout<<kp1.size()<<"   "<<kp2.size()<<"   "<<inliers_match.size()<<"   "<<endl;
//     cv::drawMatches(last_color, kp1, color, kp2, inliers_match, inliersMatchShow);
    

    cv::namedWindow("imageprev", cv::WINDOW_NORMAL);
    cv::namedWindow("imagenext", cv::WINDOW_NORMAL);
    cv::Mat im_show_prev = last_color.clone();
    for(auto kp:prev_keypoints)
      cv::circle(im_show_prev, kp, 10, cv::Scalar(0,240,0), 1);
    cv::imshow("imageprev", im_show_prev);
    
    for(int k =0; k<inliers.rows; k++)
      cv::circle(im_show_prev, prev_keypoints[inliers.ptr<int>(k)[0]], 10, cv::Scalar(0,0,240), 1);
    cv::imshow("imageprev", im_show_prev);
    
    cv::Mat im_show_next = color.clone();
    for(auto kp:keypoints)
      cv::circle(im_show_next, kp, 10, cv::Scalar(0,240,0), 1);
    cv::imshow("imagenext", im_show_next);
    
    for(int k =0; k<inliers.rows; k++)
      cv::circle(im_show_next, next_keypoints[inliers.ptr<int>(k)[0]], 10, cv::Scalar(0,0,240), 1);
    cv::imshow("imagenext", im_show_next);

    cv::waitKey(0);
    last_color = color;
  }
  return 0;
}

cv::Point3f points2DTo3D(cv::Point2f p2d, cameraParamater camera, double c_height)
{
  cv::Point3f p3d;
  p3d.z = c_height;
  p3d.x = (p2d.x - camera.cx)*p3d.z/camera.fx;
  p3d.y = (p2d.y - camera.cy)*p3d.z/camera.fy;
//   cout<<"p3d "<<p3d<<endl;
  return p3d;
}