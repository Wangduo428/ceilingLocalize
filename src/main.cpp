//C++标准库
#include<iostream>
#include<vector>
#include<fstream>

//Opencv的可视化模块
#include<opencv2/viz.hpp>

//相机模块
#include"camera.h"

//vo算法模块
#include"voBase.h"
#include"voEnd.h"

//参数读取对象声明
static ParameterReader pd;

using namespace std;


int main( int argc , char** argv ){
  //与相机有关的变量
  Error error; //相机调用过程中可能产生的各种错误信息
  Camera cam; //相机类
  Image image; // 图片类
  int imageState;
  int fail_times = 0;
  double ceiling_height = 2.1;
  
  int min_kps_extracted = 5; //提取特征点数量下限
  
  //所有关键帧
  vector<FRAME> keyFrames;
  
  //当前帧
  FRAME currentFrame;
  
  //关键帧标号
  int currentId = 1;
  
  //与g2o有关的变量
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2> > Block;
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(linearSolver);
  
  //g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>* linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
  //linearSolver->setBlockOrdering(false);
  //g2o::BlockSolver_6_3* blockSolver = new g2o::BlockSolver_6_3(linearSolver);
  
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
  g2o::SparseOptimizer g2oOptimizer;  // 最后用这个类进行优化
  g2oOptimizer.setAlgorithm( solver ); 
  g2oOptimizer.setVerbose( false );// 不输出优化过程信息
  
  //初始化
  cout<<"The programe is initializing ... "<<endl;
  
  //相机初始化
  int cameraState = CameraInitialize(error, cam);
  
  if(cameraState<0){
    //相机错误，给出提示
    cout<<"Failed to initialize camera. Please check the device and try again!"<<endl;
    return -1;
  }
  
  //从相机获取图片
  while(fail_times<30){
    imageState = QueryImages(error, cam, image);
    if(imageState<0){ 
      fail_times ++;
    }
    else{ 
      fail_times = 0; 
      break;
    }
  }
  
  if(fail_times==30){ 
    return -1;
  }
  
  //构造第一帧
  currentFrame.frameID = currentId;
  currentFrame.image = cv::Mat::zeros(image.GetRows(), image.GetCols(), CV_8UC4);
  currentFrame.image.data = image.GetData(); 
  computeOrbKeyPointsAndDesp( currentFrame );

  //将第一帧的相机位姿作为世界坐标系，驱动可视化模块
  cv::viz::Viz3d vis("Ceiling Visual Odometry");
  cv::viz::WCoordinateSystem world_coor(1.0), curr_coor(0.5);
  cv::Point3d campos(0,-1,-1), camfocalpoint(0,0,0), camupvec(0,0,1);
  cv::Affine3d campose = cv::viz::makeCameraPose(campos, camfocalpoint, camupvec);
  vis.setViewerPose(campose);
  world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
  curr_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
  vis.showWidget("World", world_coor);
  vis.showWidget("Current Pose", curr_coor);

  //第一帧加入g2o
  g2o::VertexSE3* v = new g2o::VertexSE3();
  v->setId(currentFrame.frameID);
  v->setEstimate(Eigen::Isometry3d::Identity());
  v->setFixed( true ); //第一个顶点固定不用优化
  g2oOptimizer.addVertex( v );
  
  keyFrames.push_back(currentFrame);//加入关键帧集合
  
  cout<<"Initialize Done!"<<endl;
  
  while(fail_times<30){
    //从相机获取新的一帧
    imageState = QueryImages(error, cam, image);
    if(imageState>0){
      currentId++;
      currentFrame.frameID = currentId;
      currentFrame.image.data = image.GetData(); // 获取一帧图像
      computeOrbKeyPointsAndDesp( currentFrame );
      if(currentFrame.kp.size()<min_kps_extracted)
	continue;
    
      CHECK_KF_RESULT kf_result = checkKeyframes(keyFrames.back(), currentFrame, ceiling_height, g2oOptimizer, false);
      switch(kf_result)
      {
	//TODO:匹配丢失后怎样处理有待进一步考虑
      
	case NOT_MATCHED:
	  //图片未匹配上
	  cout<<"Not enough inliers."<<endl;
	  fail_times++;
	  break;
	case TOO_FAR_AWAY:
	  // 计算结果太远了，出现错误
	  cout<<"Too far away, may be an error."<<endl;
	  fail_times++;
	  break;
	case TOO_CLOSE:
	  // 太近了，不是关键帧
	  cout<<"Too close, not a keyframe"<<endl;
	  fail_times = 0;
	  break;
	case KEYFRAME:
	  cout<<"This is a new keyframe"<<endl;
	  
	  fail_times = 0;
	
	  g2oOptimize(keyFrames, currentFrame, ceiling_height,g2oOptimizer);
	
	  keyFrames.push_back(currentFrame);
	  cout<<"Keyframes found: "<<keyFrames.size()<<endl;
	  
	  //输出当前帧优化后的位姿
          g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(g2oOptimizer.vertex( keyFrames.back().frameID ));
          Eigen::Isometry3d pose = vertex->estimate();
          cout<<"The robot's current pose matrix is :"<<endl;
          cout<<pose.matrix()<<endl;
	  
	  cv::Affine3d M( cv::Affine3d::Mat3( pose(0,0), pose(0,1), pose(0,2), pose(1,0), pose(1,1), pose(1,2), pose(2,0), pose(2,1), pose(2,2)), 
	  cv::Affine3d::Vec3(pose(0,3), pose(1,3), pose(2,3)) );

	  vis.setWidgetPose( "Current Pose", M);
	  vis.spinOnce(1, false);
	  int key = cv::waitKey(25);
	  if(key == 27) break;
      }
    
//       //输出当前帧优化后的位姿
//       g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(g2oOptimizer.vertex( keyFrames.back().frameID ));
//       Eigen::Isometry3d pose = vertex->estimate();
//       cout<<"The robot's current pose matrix is :"<<endl;
//       cout<<pose.matrix()<<endl;
  
      //驱动可视化模块viz进行可视化显示
      //TODO 确定T是谁相对于谁的
    
//       cv::Affine3d M( cv::Affine3d::Mat3( pose(0,0), pose(0,1), pose(0,2), pose(1,0), pose(1,1), pose(1,2), pose(2,0), pose(2,1), pose(2,2)), 
//       cv::Affine3d::Vec3(pose(0,3), pose(1,3), pose(2,3)) );
//       vis.setWidgetPose( "Current Pose", M);
//       vis.spinOnce(1, false);
//       int key = cv::waitKey(25);
//       if(key == 27) break;
    }
    else{
      cout<<"Failed to get image from camera. Retrying..."<<endl;
      fail_times++;
    }
  }
  
  if(fail_times==30)
    cout<<"VO stopped unexpectedly!"<<endl;
  else
    cout<<"VO stopped by user"<<endl;
  return 0;
}