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

RESULT_OF_Homogrophy estimateMotion( FRAME& frame1, FRAME& frame2, cameraParamater& camera )
{
    static ParameterReader pd;
    RESULT_OF_Homogrophy result;
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
    
    //由于天花板定位特征点共面，这里利用Homography计算相机相对运动
    cv::Mat H;
    vector<unsigned char> mask;
    vector<cv::Point2f> srcPoints , dstPoints;
    for(unsigned i =0;i<goodmatches.size();i++){
      srcPoints.push_back(frame1.kp[goodmatches[i].queryIdx].pt);
      dstPoints.push_back(frame2.kp[goodmatches[i].trainIdx].pt);
    }
    H = cv::findHomography(srcPoints, dstPoints, cv::RANSAC, 3, mask, 2000, 0.995);
    cout<<"H computed successfully!"<<endl;
    cout<<"H = "<<H<<endl;
    //内点
    vector<cv::DMatch> inliersmatches;
    vector<cv::Point2f> inliersSrcPoints , inliersDstPoints;
    for(size_t i = 0; i<goodmatches.size(); i++){
      if(mask[i] == 1){
	inliersmatches.push_back(goodmatches[i]);
	inliersSrcPoints.push_back(srcPoints[i]);
	inliersDstPoints.push_back(dstPoints[i]);
      }
    }
//     cv::Mat inlierMatchesShow;
//     cv::drawMatches( frame1.image, frame1.kp, frame2.image, frame2.kp, inliersmatches, inlierMatchesShow );
//     cv::namedWindow("inlier matches", cv::WINDOW_NORMAL);
//     cv::imshow( "inlier matches", inlierMatchesShow );
//     cv::waitKey( 0 );
    
    //构建相机矩阵
    static double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );

    vector<cv::Mat> rmat, tvec, nvec; // 由于H分解会得到4个结果，因此此处需要定义vector类型
    
    //H分解得到旋转和平移矩阵,注意得到的变换矩阵是src->dst 点坐标的变换
    cv::decomposeHomographyMat(H, cameraMatrix, rmat, tvec, nvec); // 得到的rvec 3x3, tvec 3x1, nvec 3x1
    
    cout<<"H decomposed successfully!"<<endl;

    // 选择符合实际的旋转平移
    //根据三个条件判断：
    cv::Mat temp;
    int num_feasible = rmat.size();
    cout<<"num_feasible = "<<num_feasible<<endl;
    bool motion_feasible[4] = {1,1,1,1};
    
    for(unsigned i = 0; i<rmat.size(); i++){
      temp = nvec[i].t() * rmat[i].t() * tvec[i];
      if(temp.at<double>(0) <= -1){ //1.移动前后相机在平面的同一侧  
	motion_feasible[i] = false;
	num_feasible = num_feasible-1;
      }
      temp = cv::abs(nvec[i]);
      if(temp.at<double>(0) > temp.at<double>(2) || temp.at<double>(1) > temp.at<double>(2)){
	motion_feasible[i] = false; //2.天花板基本与相机XOY平面平行,即平面法向量的Z分量最大
	num_feasible = num_feasible-1;
      }     
    }
    
    cout<<"Done!"<<endl;
    cout<<"num_feasible = "<<num_feasible<<endl;
    //3.特征点在相机的前面
    if(num_feasible >1){
    cv::Mat instrinsic_para_inv = cameraMatrix.inv();
    
    int num_front[4] = {0};
    
    for(unsigned i = 0; i<rmat.size(); i++){
      if(!motion_feasible[i])
	continue;
      for(unsigned j = 0; j<srcPoints.size(); j++){
	if(mask[j] == 1){     //为内点
	  //构造2维齐次坐标
	  float homo_2d[3] = {srcPoints[j].x , srcPoints[j].y , 1};
	  cv::Mat homo_coord_2d(3,1,CV_64F, homo_2d);
	  cv::Mat temp2 = homo_coord_2d.t() * instrinsic_para_inv.t() * rmat[i] * nvec[i];
	  if(temp2.at<double>(0) >0)
	    num_front[i]++;
	} 
      }
      //static double prop_feasible_points = atof( pd.getData("prop_feasible_points").c_str());
      double prop_feasible_points = 0.1;
      if(num_front[i] < prop_feasible_points*srcPoints.size())
	motion_feasible[i] = false;
	num_feasible = num_feasible-1;
    }
    }
    cout<<"Done!"<<endl;
    cout<<"num_feasible = "<<num_feasible<<endl;
//     cout<<rmat[0]<<endl;
//     cout<<tvec[0]<<endl;
//     cout<<nvec[0]<<endl;

    //TODO 如果到这步还是有多个解怎么办？

    //构造输出结果
    for(unsigned i =0; i<rmat.size(); i++){
      if(motion_feasible[i] == true){
	result.rmat = rmat[i];
	
	//确定平移的尺度，通过优化求解
	result.tvec = computeScaledTrans(inliersSrcPoints, inliersDstPoints, rmat[i], tvec[i], camera); 
	result.nvec = nvec[i];
	result.num_inliers = countNonZero(cv::Mat(mask));
	break;
      }
      else{
	result.num_inliers = -1; //没有可行的运动解
      }
    }
    return result;
}
cv::Mat computeScaledTrans(vector<cv::Point2f> inliersSrcPoints, vector<cv::Point2f> inliersDstPoints, cv::Mat R, cv::Mat t, cameraParamater& camera){
  int num_inliers = inliersSrcPoints.size();
  double scale;
  Eigen::MatrixXd inliersSrcPointsEigen(3,num_inliers), inliersDstPointsEigen(3,num_inliers);
  Eigen::Matrix3d rotate;
  Eigen::Vector3d trans;
  cv::cv2eigen(R, rotate);
  cv::cv2eigen(t, trans);
  if(trans.norm() < 0.001 ){
    return t;
  }
  else{
  for(unsigned i = 0; i<num_inliers; i++){
    inliersSrcPointsEigen(0,i) = inliersSrcPoints[i].x;
    inliersSrcPointsEigen(1,i) = inliersSrcPoints[i].y;
    inliersSrcPointsEigen(2,i) = 1;
    inliersDstPointsEigen(0,i) = inliersDstPoints[i].x;
    inliersDstPointsEigen(1,i) = inliersDstPoints[i].y;
    inliersDstPointsEigen(2,i) = 1;
  }
  
  inliersSrcPointsEigen = camera.matrix2dTo3d * inliersSrcPointsEigen;
  inliersDstPointsEigen = camera.matrix2dTo3d * inliersDstPointsEigen;
  
  scale = (trans.transpose() * (inliersDstPointsEigen - rotate * inliersSrcPointsEigen)).sum() / (num_inliers * trans.transpose() * trans);
  
  double tras_scaled[3] = {scale*trans(0), scale*trans(1), scale*trans(2)};
  cv::Mat tvec_scale = cv::Mat::zeros(3,1,CV_64F);
  tvec_scale.at<double>(0) = tras_scaled[0];
  tvec_scale.at<double>(1) = tras_scaled[1];
  tvec_scale.at<double>(2) = tras_scaled[2];
  return tvec_scale;
  }
}
