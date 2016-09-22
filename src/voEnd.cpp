#include "voBase.h"
#include "voEnd.h"

//函数实现
CHECK_KF_RESULT checkKeyframes( FRAME& f1, FRAME& f2, const double ceiling_height, g2o::SparseOptimizer& opti, bool isLoops){
    static ParameterReader pd;
    static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static cameraParamater camera = getDefaultCamera();
    static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );
    // 比较f1 和 f2
    RESULT_OF_Motion result = estimateMotion( f1, f2, ceiling_height, camera );
    cout<<"motion calculate done!"<<endl;
    if ( result.num_inliers < min_inliers ) //inliers不够，放弃该帧
        return NOT_MATCHED;
    // 计算运动范围是否太大
    double norm = normofTransform(result.rmat, result.tvec);
    Eigen::Isometry3d T = cvMat2Eigen(result.rmat, result.tvec);
    cout<<"convert mat 2 eigen done!"<<endl;
    if(!isLoops){
      if ( norm >= max_norm )
	  return TOO_FAR_AWAY;   // too far away, may be error
    }
    else{
      if ( norm >= max_norm_lp )
	  return TOO_FAR_AWAY;   // too far away, may be error
    }
    
    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;   // too adjacent frame
    // 剩下的情况就是关键帧了
    //将关键帧添加到g2o中
    if (!isLoops)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f2.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    //构造边
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->vertices() [0] = opti.vertex(f1.frameID);
    edge->vertices() [1] = opti.vertex(f2.frameID);
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    // 也可以将角度设大一些，表示对角度的估计更加准确
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    edge->setInformation(information);
    edge->setRobustKernel( robustKernel );
    edge->setMeasurement( T );
    opti.addEdge(edge);
    
    return KEYFRAME;
}

double normofTransform(cv::Mat rmat, cv::Mat tvec){
  cv::Mat rvec;
  cv::Rodrigues(rmat,tvec);
  return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rmat, cv::Mat& tvec )
{
    Eigen::Matrix3d r;
    cv::cv2eigen(rmat, r);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(0,1); 
    T(2,3) = tvec.at<double>(0,2);
    return T;
}

void g2oOptimize(vector<FRAME>& keyFrames, FRAME& currentFrame, const double ceiling_height, g2o::SparseOptimizer& opti){ 
  static ParameterReader pd;
  //检测当前帧与之前的几帧是否可以形成闭环
  //static int lastFramesNum =  atoi( pd.getData("lastFramesNum").c_str() ); //将最多考虑前面多少帧
  static int lastFramesNum = 6;
  if(keyFrames.size()<=lastFramesNum){
    for(size_t i = 0; i<keyFrames.size(); i++){
      checkKeyframes(keyFrames[i], currentFrame, ceiling_height, opti, true);
    }
  }
  else{
    for(size_t i = keyFrames.size() - lastFramesNum; i<keyFrames.size(); i++){
      checkKeyframes(keyFrames[i], currentFrame, ceiling_height, opti, true);
    }
  }
  //选取最后几帧进行优化,把以前的节点的setFixed设为true,不再优化
  if(keyFrames.size()>lastFramesNum){
    opti.vertex(keyFrames[keyFrames.size() - lastFramesNum + 1].frameID)->setFixed(true);
  }
  
  //优化得到最新帧的位姿
  opti.initializeOptimization();
  opti.optimize(50);
}
