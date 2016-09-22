#include "voBase.h"

//函数实现
void computeOrbKeyPointsAndDesp( FRAME& frame)
{
  static ParameterReader pd;
  vector<cv::KeyPoint> kp;
  cv::Mat desp;
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  //static int maxFeatures = atoi( pd.getData("maxFeatures").c_str() ); //最大特征点数量
  int maxFeatures = 500;
  orb->setMaxFeatures(maxFeatures);
  orb->detectAndCompute(frame.image, cv::noArray(), kp, desp);
  cout<<"find " << kp.size() << " feature points in frame NO"<< frame.frameID <<endl;
  
  frame.kp = kp;
  frame.desp = desp;
}

RESULT_OF_Motion estimateMotion( FRAME& frame1, FRAME& frame2, const double ceiling_height, cameraParamater& camera )
{
    static ParameterReader pd;
    RESULT_OF_Motion result;
    //特征点匹配跟踪，使用BruteForce-Hamming法
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    vector<vector<cv::DMatch> > matches;                    // 匹配结果
    matcher->knnMatch(frame1.desp, frame2.desp, matches, 2);            // 2近邻
    
    //选择好的匹配，选择原则是最近邻的特征点描述子距离小于二近邻距离的0.6倍
    //即两个匹配上的特征点的描述子足够相似并且与其他描述子的差异足够明显
    vector<cv::DMatch> goodmatches;
    for (unsigned i = 0;i<matches.size();i++) {
      if (matches[i][0].distance < matches[i][1].distance *0.6) {
	goodmatches.push_back(matches[i][0]);
      }
    }
    cout<<"find "<<goodmatches.size()<<" good matches between frame "<< frame1.frameID<<" and frame "<<frame2.frameID<<endl;
    
    static int min_good_match = atoi( pd.getData("min_good_match").c_str()); //good matches数量下限
    if(goodmatches.size()<min_good_match){
      result.num_inliers = -1;
      return result;
    }
    
    vector<cv::Point3f> srcPoints3D;
    vector<cv::Point2f> dstPoints2D;

    for(unsigned i =0;i<goodmatches.size();i++){    
      srcPoints3D.push_back(points2DTo3D(frame1.kp[goodmatches[i].queryIdx].pt, camera, ceiling_height));
      dstPoints2D.push_back(frame2.kp[goodmatches[i].trainIdx].pt);
    }
    
    //构建相机矩阵
    static double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    
    cv::Mat rvec, tvec, inliers;
    
    //利用3D-2D法，即求解PnP问题
    cv::solvePnPRansac(srcPoints3D, dstPoints2D, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 2.0, 0.99, inliers);
    
    //对求解结果利用g2o进行优化
    

    //构造输出结果
    cv::Mat rmat = cv::Mat::zeros(3,3,CV_64F);
    cv::Rodrigues(rvec, rmat);
    result.rmat = rmat;
    result.tvec = tvec;
    result.num_inliers = inliers.rows;
    
    return result;
}
cv::Point3f points2DTo3D(cv::Point2f p2d, cameraParamater camera, double c_height){
  cv::Point3f p3d;
  p3d.z = c_height;
  p3d.x = (p2d.x - camera.cx)*p3d.z/camera.fx;
  p3d.y = (p2d.y - camera.cy)*p3d.z/camera.fy;
//   cout<<"p3d "<<p3d<<endl;
  return p3d;
}

