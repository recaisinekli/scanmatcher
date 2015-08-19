#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include "closest.h"
#include "iterativeclosestpoint.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include "vtkRenderWindow.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <ctime>
#include <cmath>
#include "csvrow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <boost/filesystem.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewerPCD;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewerPCDfiltered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewerPCDtransformed;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewerPCDtransformedFiltered;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matcherPrevScan;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matcherCurrScan;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matcherPrevScanOriginal;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matcherCurrScanOriginal;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matcher_source_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matcher_source_filtered_tr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matcher_target_filtered;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prevScan;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currScan;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prevScanOriginal;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currScanOriginal;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_filtered_tr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr set1_tr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr set2_tr;
    Closest closest;
    IterativeClosestPoint icp, matcherICP;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> PCLICP;
    clock_t t1,t2,t3,t4;
    void updateView();
    void updateMatcherView();
    void drawPointCloud(Eigen::Matrix4f &tempT);
    void getEulerAngles(const Eigen::Matrix3f& t, float& roll, float& pitch, float& yaw);
    void getAxisAngle(const Eigen::Matrix3f& t, Eigen::Vector3f& u, float& angle);
    int v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12;
    Eigen::Matrix4f T, Tg;
    std::vector<Eigen::Matrix4f> GroundTruth, GroundTruthNoisy;
    void clear(bool display=true);
    void clearMatcher();
    void loadPCDFiles(const int file1, const int file2, bool display=true);
    void loadPCDFiles_PCL(const int file1, const int file2);
    void addNoise(Eigen::Matrix4f &transformationMatrix, float maxR, float maxT);
    bool isIMUloaded, isPCDLoaded, isViewerPCDLoaded, isMatcherSourceLoaded, isMatcherTargetLoaded;
    double strToDbl(std::string);
    void viewer_applyFilter();
    void viewer_transform();
    float leafsize;
    QString multifolderpath;
    QString pclfolderpath;
    
private slots:
    void on_multi_icpAlign_clicked();
    void on_multi_clearScreen_clicked();
    void on_multi_onetoone_clicked();
    void on_multi_IMU_clicked();
    void on_multi_readGT_clicked();

    void on_pcd_viewer_load_clicked();
    void on_viewer_leafSize_valueChanged();
    void on_saveFiltered_clicked();
    void on_saveTransformed_clicked();

    void on_selectSource_clicked();
    void on_selectTarget_clicked();
    void on_matcherICP_step_clicked();
    void on_matcher_clearScreen_clicked();
    void on_matcherICP_align_clicked();
    void on_rotationX_valueChanged();
    void on_trX_valueChanged();
    void on_rotationY_valueChanged();
    void on_trY_valueChanged();
    void on_rotationZ_valueChanged();
    void on_trZ_valueChanged();

    void on_viewerAddNoise_clicked();

    void on_arrow_clicked();

    void on_matcher_onetoone_clicked();

    void on_fixMedianCoefficient_clicked();

    void on_randomMedianCoefficient_clicked();

    void on_useRANSAC_clicked();

    void on_adaptiveMedianCoefficient_clicked();

    void on_pclICPAlign_clicked();

    void on_pclICP_readGT_clicked();

    void on_pclICP_IMU_clicked();

    void on_pomerleauBreak_clicked();

    void on_pclBreak_clicked();

    void on_selectMultiFolder_clicked();

    void on_selecPCLFolder_clicked();

    void on_pclICP_clearScreen_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
