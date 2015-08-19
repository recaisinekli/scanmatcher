#ifndef ITERATIVECLOSESTPOINT_H
#define ITERATIVECLOSESTPOINT_H

#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <ctime>
#include "closest.h"
#include <fstream>
#include <iterator>
#include <QString>
#include <numeric>
#include <QDebug>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/ransac.h>

class IterativeClosestPoint
{

public:
    IterativeClosestPoint();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr set1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr set1_tr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr set2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr set2_tr;
    double median(std::vector<double> distance);
    void mean(pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::VectorXf &p, bool correspOnly, const std::vector<int> &corresp =  std::vector<int>());
    void rt2tr(Eigen::MatrixXf &R, Eigen::VectorXf &t, Eigen::Matrix4f &T);
    void mabs(Eigen::Matrix4f &T);
    void addNoise(Eigen::Matrix4f &transformationMatrix, float maxR, float maxT);
    Eigen::Matrix4f getFinalTransformation();
    clock_t t1,t2;
    void setSource(pcl::PointCloud<pcl::PointXYZRGB> &source);
    void setTarget(pcl::PointCloud<pcl::PointXYZRGB> &target);
    void align(Eigen::Matrix4f IMU=Eigen::Matrix4f::Identity ());
    void step();
    double map(double x, double in_min, double in_max, double out_min, double out_max);
    int maxIteration;
    Eigen::Matrix4f T,Tprev;
    double mediandiff, kdtreediff, medianCoef, lastSumDist, sumDist, breakCriteria, useRANSAC;
    int totalMatchedPoint;
    int iterbr,stepNum;
    Closest closest;
    std::vector<int> source_indices;
    std::vector<int> kdcorresp;
    std::vector<double> kddistance;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    bool onetoone;
    bool fixMedianCoefficient;
    bool randomMedianCoefficient;
    bool adaptiveMedianCoefficient;
    bool pclLikeBreak;
    bool pomerleauLikeBreak;
    QString dirName;
    int icpCount;
};

#endif // ITERATIVECLOSESTPOINT_H
