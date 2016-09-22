# pragma once
//C++标准库
#include<iostream>
#include<vector>
#include<fstream>
using namespace std;
// Eigen库
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<eigen3/Eigen/Dense>
//Opencv库
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/features2d.hpp>
#include<opencv2/core/eigen.hpp>

//******一些结构******//
//相机内参结构
struct cameraParamater{
    double cx ,cy ,fx, fy;
    Eigen::Matrix3d matrix2dTo3d;
};

// 帧结构
struct FRAME
{
    int frameID;  //帧标号
    cv::Mat image; //该帧对应的图像
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// Homogrophy 结果
struct RESULT_OF_Homogrophy
{
    cv::Mat rmat, tvec, nvec;
    int num_inliers;
};

/********函数定义*******/
 //提取图片特征，计算描述子
void computeOrbKeyPointsAndDesp( FRAME& frame);
 //计算相机相对运动
RESULT_OF_Homogrophy estimateMotion( FRAME& frame1, FRAME& frame2, cameraParamater& camera );
//计算实际尺度下的平移量
cv::Mat computeScaledTrans(vector<cv::Point2f> inliersSrctPoints, vector<cv::Point2f> inliersDstPoints, cv::Mat R, cv::Mat t, cameraParamater& camera);

// 参数读取类
class ParameterReader
{
public:
    ParameterReader( string filename="/home/wangduo/Code/ceilingLocalize/parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

inline static cameraParamater getDefaultCamera()
{
    ParameterReader pd;
    cameraParamater camera;
    double ceiling_height = 2.3;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    //double ceiling_height = atof( pd.getData("ceiling_height").c_str());
    //cout<<"ceiling height = "<<ceiling_height<<endl;
    camera.matrix2dTo3d << camera.fx, 0,  camera.cx,
                           0, camera.fy, camera.cy,
			   0,0,1;
    camera.matrix2dTo3d = camera.matrix2dTo3d.inverse().eval() * ceiling_height;
   
    return camera;
}