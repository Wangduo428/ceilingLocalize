#include "voBase.h"
#include "voEnd.h"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

int main(int argc, char** argv)
{
  FRAME f1,f2;
  f1.image = cv::imread("/home/wangduo/Code/ceilingLocalize/src/test3.bmp");
  f2.image = cv::imread("/home/wangduo/Code/ceilingLocalize/src/test4.bmp");
  f1.frameID = 1;
  f2.frameID = 2;
  computeOrbKeyPointsAndDesp(f1);
  computeOrbKeyPointsAndDesp(f2);
  
  double ceiling_height = 2.3;
  RESULT_OF_Motion homo;
  cameraParamater camera = getDefaultCamera();
  
  homo = estimateMotion(f1,f2,ceiling_height,camera);
  cout<<homo.rmat<<endl;
  cout<<homo.tvec<<endl;
  
  Eigen::Isometry3d T = cvMat2Eigen(homo.rmat, homo.tvec); //eigen库中矩阵为按列储存，opencv中的Mat类型为按行储存,注意类型转换时的区别
  cout<<"T = "<<T.matrix() <<endl; //matrix()是EIgen库类型取数据的函数
  
  return 0;
}