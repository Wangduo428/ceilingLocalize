# pragma once
#include"voBase.h"
//C++标准库
#include<iostream>
#include<vector>
#include<fstream>
//Opencv库
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/features2d.hpp>
#include<opencv2/core/eigen.hpp>
//g2o头文件
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

// 通过检测当前帧和上一个关键帧来判断，结果定义
enum CHECK_KF_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME}; 

// 函数说明：判断当前帧是否为关键帧
CHECK_KF_RESULT checkKeyframes( FRAME& f1, FRAME& f2, const double ceiling_height, g2o::SparseOptimizer& opti, bool isLoops);

//函数说明：计算运动参数的范数
double normofTransform(cv::Mat rmat, cv::Mat tvec);

//函数说明：cv::Mat 类型转换成Eigen库类型
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rmat, cv::Mat& tvec );

//函数说明：对最近的几帧利用g2o进行位姿优化
void g2oOptimize(vector<FRAME>& keyFrames, FRAME& currentFrame, const double ceiling_height, g2o::SparseOptimizer& opti);

