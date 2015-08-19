#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <Eigen/Geometry>

pcl::visualization::PCLVisualizer pvis1 ("test", false); //Multi Matcher
pcl::visualization::PCLVisualizer pvis2 ("test", false); //Viewer
pcl::visualization::PCLVisualizer pvis3 ("test", false); //Matcher
pcl::visualization::PCLVisualizer pvis4 ("test", false); //PCL ICP

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    matcherCurrScan.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    matcherPrevScan.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    matcherPrevScanOriginal.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    matcherCurrScanOriginal.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    matcher_source_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    matcher_source_filtered_tr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    matcher_target_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewerPCD.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewerPCDfiltered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewerPCDtransformed.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewerPCDtransformedFiltered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    prevScan.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    currScan.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    prevScanOriginal.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    currScanOriginal.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    source_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    source_filtered_tr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    target_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    set1_tr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    set2_tr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    ui->qvtkWidget->SetRenderWindow(pvis1.getRenderWindow());
    ui->qvtkWidget_2->SetRenderWindow(pvis2.getRenderWindow());
    ui->qvtkWidget_3->SetRenderWindow(pvis3.getRenderWindow());
    ui->qvtkWidget_5->SetRenderWindow(pvis4.getRenderWindow());

    pvis1.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    pvis1.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    pvis1.setBackgroundColor(0.1, 0.1, 0.1, v2);

    pvis2.createViewPort(0.0, 0.0, 0.5, 1.0, v5);
    pvis2.createViewPort(0.5, 0.0, 1.0, 1.0, v6);
    pvis2.setBackgroundColor(0.1, 0.1, 0.1, v6);

    pvis3.createViewPort(0.0, 0.0, 0.5, 0.5, v9);
    pvis3.createViewPort(0.5, 0.0, 1.0, 0.5, v10);
    pvis3.createViewPort(0.0, 0.5, 0.5, 1.0, v7);
    pvis3.createViewPort(0.5, 0.5, 1.0, 1.0, v8);
    pvis3.setBackgroundColor(0.0, 0.0, 0.0, v7);
    pvis3.setBackgroundColor(0.2, 0.2, 0.2, v8);
    pvis3.setBackgroundColor(0.2, 0.2, 0.2, v9);
    pvis3.setBackgroundColor(0.0, 0.0, 0.0, v10);

    pvis4.createViewPort(0.0, 0.0, 0.5, 1.0, v11);
    pvis4.createViewPort(0.5, 0.0, 1.0, 1.0, v12);
    pvis4.setBackgroundColor(0.1, 0.1, 0.1, v12);

    T = Eigen::Matrix4f::Identity ();
    Tg = T;

    srand(time(0));
    isIMUloaded=false;
    isPCDLoaded=false;
    isViewerPCDLoaded=false;
    isMatcherSourceLoaded=false;
    isMatcherTargetLoaded=false;

    ui->matcher_clearScreen->setFocusPolicy(Qt::NoFocus);
    ui->selectSource->setFocusPolicy(Qt::NoFocus);
    ui->selectTarget->setFocusPolicy(Qt::NoFocus);
    ui->pcd_viewer_load->setFocusPolicy(Qt::NoFocus);
    ui->saveFiltered->setFocusPolicy(Qt::NoFocus);
    ui->saveTransformed->setFocusPolicy(Qt::NoFocus);
    ui->matcherICP_step->setFocusPolicy(Qt::NoFocus);
    ui->matcherICP_align->setFocusPolicy(Qt::NoFocus);
    ui->multi_icpAlign->setFocusPolicy(Qt::NoFocus);
    ui->multi_clearScreen->setFocusPolicy(Qt::NoFocus);
    ui->multi_readGT->setFocusPolicy(Qt::NoFocus);
    ui->tabWidget->setFocusPolicy(Qt::NoFocus);
    ui->arrow->setFocusPolicy(Qt::NoFocus);
    ui->viewerAddNoise->setFocusPolicy(Qt::NoFocus);
    ui->matcher_onetoone->setFocusPolicy(Qt::NoFocus);
    ui->fixMedianCoefficient->setFocusPolicy(Qt::NoFocus);
    ui->randomMedianCoefficient->setFocusPolicy(Qt::NoFocus);
    ui->useRANSAC->setFocusPolicy(Qt::NoFocus);
    ui->adaptiveMedianCoefficient->setFocusPolicy(Qt::NoFocus);
    ui->pclICPAlign->setFocusPolicy(Qt::NoFocus);
    ui->pclICP_readGT->setFocusPolicy(Qt::NoFocus);
    ui->pclBreak->setFocusPolicy(Qt::NoFocus);
    ui->pomerleauBreak->setFocusPolicy(Qt::NoFocus);
    ui->multi_IMU->setFocusPolicy(Qt::NoFocus);
    ui->displayClouds->setFocusPolicy(Qt::NoFocus);
    ui->pclICP_IMU->setFocusPolicy(Qt::NoFocus);

    ui->tabWidget->setCurrentIndex(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadPCDFiles(const int file1, const int file2, bool display){
    QString target=multifolderpath+"/"+QString::number(file1)+".pcd";
    QString source=multifolderpath+"/"+QString::number(file2)+".pcd";
    icp.stepNum=0;
    pcl::io::loadPCDFile (target.toStdString(), *prevScan);
    pcl::io::loadPCDFile (source.toStdString(), *currScan);

    pcl::copyPointCloud(*currScan,*currScanOriginal);
    pcl::copyPointCloud(*prevScan,*prevScanOriginal);

    isPCDLoaded=true;
}

void MainWindow::loadPCDFiles_PCL(const int file1, const int file2){
    QString target=pclfolderpath+"/"+QString::number(file1)+".pcd";
    QString source=pclfolderpath+"/"+QString::number(file2)+".pcd";

    pcl::io::loadPCDFile (target.toStdString(), *prevScan);
    pcl::io::loadPCDFile (source.toStdString(), *currScan);

    pcl::copyPointCloud(*currScan,*currScanOriginal);
    pcl::copyPointCloud(*prevScan,*prevScanOriginal);

    isPCDLoaded=true;
}

void MainWindow::on_multi_icpAlign_clicked()
{
    pvis1.removeAllPointClouds();
    pvis1.removeAllShapes();

    int start=ui->multi_pcd_start->text().toInt();
    int end=ui->multi_pcd_end->text().toInt();
    int step=ui->multi_pcd_step->text().toInt();

    Tg = Eigen::Matrix4f::Identity();

    std::vector<Eigen::Matrix4f> T_history;
    int k=0;

    boost::filesystem::create_directories("benchmark_results");
    t3 = clock();

    double med=ui->multi_medyanKatsayisi->text().toDouble();
    double ls=ui->multi_leafSize->text().toDouble();
    QString onetoone;
    if(ui->multi_onetoone->isChecked()){
        onetoone="1";
    }else{
        onetoone="0";
    }

    QString dirName;
    if(ui->fixMedianCoefficient->isChecked()){
        dirName = "benchmark_results/M_"+QString::number(med)+"_LS_"+QString::number(ls)+"_1to1_"+onetoone;
    }
    if(ui->randomMedianCoefficient->isChecked()){
        dirName = "benchmark_results/M_RND_LS_"+QString::number(ls)+"_1to1_"+onetoone;
    }
    if(ui->useRANSAC->isChecked()){
        dirName = "benchmark_results/RANSAC_LS_"+QString::number(ls)+"_1to1_"+onetoone;
    }
    if(ui->adaptiveMedianCoefficient->isChecked()){
        dirName = "benchmark_results/M_ADA_LS_"+QString::number(ls)+"_1to1_"+onetoone;
    }
    boost::filesystem::create_directories(dirName.toStdString().c_str());
    k=0;
    T_history.clear();
    Tg=Eigen::Matrix4f::Identity();
    for(int i=start; i<=end-step;i+=step){
        bool usedisplay = ui->displayClouds->isChecked();
        loadPCDFiles(i, i+step, usedisplay);
        qDebug()<<i+step<<"to"<<start<<" ---------------------------------------";
        icp.stepNum=0;
        ui->multi_icpAlign->setEnabled(false);
        ui->multi_leafSize->setEnabled(false);
        icp.breakCriteria = ui->multi_breakCriteria->text().toDouble();
        icp.medianCoef = med;
        icp.dirName=dirName;
        icp.icpCount=i;
        if(ui->fixMedianCoefficient->isChecked()){
            icp.fixMedianCoefficient=true;
        }else{
            icp.fixMedianCoefficient=false;
        }

        if(ui->randomMedianCoefficient->isChecked()){
            icp.randomMedianCoefficient=true;
        }else{
            icp.randomMedianCoefficient=false;
        }

        if(ui->adaptiveMedianCoefficient->isChecked()){
            icp.adaptiveMedianCoefficient=true;
        }else{
            icp.adaptiveMedianCoefficient=false;
        }

        if(ui->useRANSAC->isChecked()){
            icp.useRANSAC=true;
        }else{
            icp.useRANSAC=false;
        }

        if(ui->pomerleauBreak->isChecked()){
            icp.pomerleauLikeBreak=true;
        }else{
            icp.pomerleauLikeBreak=false;
        }

        if(ui->pclBreak->isChecked()){
            icp.pclLikeBreak=true;
        }else{
            icp.pclLikeBreak=false;
        }
        leafsize=ls;
        clear(usedisplay);
        if(ui->multi_IMU->isChecked()){
            Eigen::Matrix4f T;
            T = Eigen::Matrix4f::Identity();
            for(int j=i;j<i+step;j++){
                T*=GroundTruthNoisy.at(j+1);
            }
            t1 = clock();
            icp.align(T);
        }else{
            t1 = clock();
            icp.align();
        }
        t2 = clock();
        updateView();
        ui->multi_icpAlign->setEnabled(true);
        ui->multi_leafSize->setEnabled(true);
        T_history.push_back(T);
        Tg *= T;
        std::cout<<Tg<<endl;
        if(usedisplay){
            drawPointCloud(Tg);
        }

        std::ofstream myfile;

        //Write to File------------------------------------------------------------------------------------
        QString poseFile = dirName+"/pose_scanner_leica.csv";
        myfile.open (poseFile.toStdString().c_str(), ios::app);
        if(k==0){
            myfile<<"poseId, timestamp, T00, T01, T02, T03, T10, T11, T12, T13, T20, T21, T22, T23, T30, T31, T32, T33"<<endl;
            myfile<<"0, 0, 1, -0, -0, 0, -0, 1, -0, 0, 0, -0, 1, 0, 0, 0, 0, 1"<<endl;
            k++;
        }
        myfile<<k<<", 0, ";
        for(int x=0;x<4;x++){
            for(int y=0;y<4;y++){
                myfile<<Tg(x,y);
                if(x!=3 || y!=3)
                    myfile<<", ";
            }
        }
        myfile<<endl;
        myfile.close();
        k++;
        //-------------------------------------------------------------------------------------------------

        //Write to File------------------------------------------------------------------------------------
        QString timeFile = dirName+"/time_elapsed_ms.txt";
        myfile.open (timeFile.toStdString().c_str(), ios::app);
        myfile<<(double)(t2-t1)*1000/CLOCKS_PER_SEC<<endl;
        myfile.close();

        //Write to File------------------------------------------------------------------------------------
        QString iterFile = dirName+"/iteration.txt";
        myfile.open (iterFile.toStdString().c_str(), ios::app);
        myfile<<icp.iterbr+1<<endl;
        myfile.close();
    }

    t4 = clock();
}

void MainWindow::updateView(){
    T = icp.getFinalTransformation();

    Eigen::Vector3f u; float angle;
    getAxisAngle(T.block<3,3>(0,0), u, angle);
}

void MainWindow::drawPointCloud(Eigen::Matrix4f &tempT){
    //ADD POINT CLOUDS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempdata(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*source_filtered,*tempdata);
    pcl::transformPointCloud(*tempdata,*tempdata,tempT);

    QString id="ps"+QString::number(icp.icpCount);
    pvis1.addPointCloud<pcl::PointXYZRGB>(tempdata,id.toStdString().c_str(), v2);
    pvis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,1.0, id.toStdString().c_str(), v2);
    ui->qvtkWidget->update();
}

void MainWindow::getEulerAngles(const Eigen::Matrix3f& t, float& roll, float& pitch, float& yaw)
{
    roll  = atan2(t(2,1), t(2,2));
    pitch = asin(-t(2,0));
    yaw   = atan2(t(1,0), t(0,0));
}

void MainWindow::getAxisAngle(const Eigen::Matrix3f& R, Eigen::Vector3f& u, float& angle){
    angle=acos((R(0,0)+R(1,1)+R(2,2)-1)/2);
    double temp = sqrt(pow((R(2,1)-R(1,2)),2)+pow((R(0,2)-R(2,0)),2)+pow((R(1,0)-R(0,1)),2));
    u(0)=(R(2,1)-R(1,2))/temp;
    u(1)=(R(0,2)-R(2,0))/temp;
    u(2)=(R(1,0)-R(0,1))/temp;
}

void MainWindow::clear(bool display){
    ui->multi_leafSize->setEnabled(false);

    if(icp.stepNum==0){
        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        grid.setLeafSize(leafsize,leafsize,leafsize);
        grid.setInputCloud (currScanOriginal);
        grid.filter (*source_filtered);

        grid.setInputCloud (prevScanOriginal);
        grid.filter (*target_filtered);

        if(display){
            //ADD POINT CLOUDS
            Eigen::Matrix4f tempT;
            tempT = Eigen::Matrix4f::Identity();
            if(ui->multi_IMU->isChecked()){
                for(int j=0;j<icp.icpCount+1;j++){
                    tempT*=GroundTruthNoisy.at(j);
                }
            }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempdata(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*source_filtered,*tempdata);
            pcl::transformPointCloud(*tempdata,*tempdata,tempT);

            QString id="cs"+QString::number(icp.icpCount);
            pvis1.addPointCloud<pcl::PointXYZRGB>(tempdata,id.toStdString().c_str(), v1);
            pvis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, id.toStdString().c_str(), v1);
            ui->qvtkWidget->update();
        }
        T = Eigen::Matrix4f::Identity ();
        icp.setSource(*source_filtered);
        icp.setTarget(*target_filtered);
        icp.maxIteration=ui->multi_maxIter->text().toInt();
    }
}

void MainWindow::on_multi_clearScreen_clicked()
{
    pvis1.removeAllPointClouds();
    pvis1.removeAllShapes();
    ui->qvtkWidget->update();
    ui->multi_IMU->setChecked(false);
    ui->multi_IMU->setEnabled(false);
}

void MainWindow::on_multi_onetoone_clicked()
{
    if(ui->multi_onetoone->isChecked()){
        icp.onetoone=true;
    }else{
        icp.onetoone=false;
    }
}

void MainWindow::addNoise(Eigen::Matrix4f &transformationMatrix, float maxR, float maxT){
    Eigen::Affine3f transformation;
    maxR=maxR*M_PI/180;
    maxT/=100;
    float r = (static_cast <float> (rand()) / (static_cast <float> (RAND_MAX)))*maxR-maxR/2;
    float t = (static_cast <float> (rand()) / (static_cast <float> (RAND_MAX)))*maxT-maxT/2;
    transformation = pcl::getTransformation(t,t,t,r,r,r);
    transformationMatrix*=transformation.matrix();
}

void MainWindow::on_multi_IMU_clicked()
{
    if(isPCDLoaded){
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempdata(new pcl::PointCloud<pcl::PointXYZRGB>);
        if(ui->multi_IMU->isChecked()){
            Eigen::Matrix4f T;
            T = Eigen::Matrix4f::Identity ();
            for(int i=ui->multi_pcd_start->text().toInt();i<ui->multi_pcd_end->text().toInt();i++){
                T*=GroundTruthNoisy.at(i+1);
            }
        }
        ui->qvtkWidget->update();
    }
}

void MainWindow::on_multi_readGT_clicked()
{
    QString csvfile = QFileDialog::getOpenFileName(this,
                                                   tr("Open IMU File"), QDir::currentPath(), tr("CSV Files (*.csv)"));
    if(csvfile!=NULL){
        GroundTruth.clear();
        GroundTruthNoisy.clear();
        //std::ifstream file("pose_scanner_leica.csv");
        std::ifstream file(csvfile.toStdString().c_str());

        CSVRow row;
        while(row.readNextRow(file))
        {
            char* p;
            strtod(row[0].c_str(), &p);
            if (!*p) {
                Eigen::Matrix4f T,N;

                T(0,0)=strToDbl(row[2]);T(0,1)=strToDbl(row[3]);T(0,2)=strToDbl(row[4]);T(0,3)=strToDbl(row[5]);
                T(1,0)=strToDbl(row[6]);T(1,1)=strToDbl(row[7]);T(1,2)=strToDbl(row[8]);T(1,3)=strToDbl(row[9]);
                T(2,0)=strToDbl(row[10]);T(2,1)=strToDbl(row[11]);T(2,2)=strToDbl(row[12]);T(2,3)=strToDbl(row[13]);
                T(3,0)=strToDbl(row[14]);T(3,1)=strToDbl(row[15]);T(3,2)=strToDbl(row[16]);T(3,3)=strToDbl(row[17]);

                if(GroundTruth.size()==0){
                    GroundTruthNoisy.push_back(T);
                    GroundTruth.push_back(T);
                }else{
                    N=T;
                    //N=GroundTruth.back().inverse()*T;
                    //addNoise(N, ui->multi_noiseR->text().toInt(), ui->multi_noiseT->text().toInt());
                    GroundTruthNoisy.push_back(N);
                    GroundTruth.push_back(T);
                }

            }
        }
        isIMUloaded=true;
        ui->multi_IMU->setEnabled(true);

        //Write to File------------------------------------------------------------------------------------
        /*
    std::ofstream myfile;
    int k=0;
    while(k<=35){
        QString poseFile = "ground_truth_noisy.csv";
        myfile.open (poseFile.toStdString().c_str(), ios::app);
        if(k==0){
            myfile<<"poseId, timestamp, T00, T01, T02, T03, T10, T11, T12, T13, T20, T21, T22, T23, T30, T31, T32, T33"<<endl;
            myfile<<"0, 0, 1, -0, -0, 0, -0, 1, -0, 0, 0, -0, 1, 0, 0, 0, 0, 1"<<endl;
            k++;
        }
        myfile<<k<<", 0, ";
        for(int x=0;x<4;x++){
            for(int y=0;y<4;y++){
                Tg=GroundTruthNoisy.at(k);
                myfile<<Tg(x,y);
                if(x!=3 || y!=3)
                    myfile<<", ";
            }
        }
        myfile<<endl;
        myfile.close();
        k++;
    }
    */
        //-------------------------------------------------------------------------------------------------
    }
}

double MainWindow::strToDbl(std::string s){
    std::stringstream ss(s);
    double d;
    ss >> d;
    return d;
}

void MainWindow::on_pcd_viewer_load_clicked()
{
    QString pcdfile = QFileDialog::getOpenFileName(this,
                                                   tr("Open PCD File"), QDir::currentPath(), tr("PCD Files (*.pcd)"));
    if(pcdfile!=NULL){
        pvis2.removeAllPointClouds();
        pvis2.removeAllShapes();

        pvis2.addText(".",0,310,11,0,0,0,"otitle",v5);
        pvis2.addText(".",0,310,11,0,0,0,"ftitle",v6);
        pvis2.addText(".",0,325,11,0,0,0,"kns",v5);
        pvis2.addText(".",0,310,11,0,0,0,"knsf",v6);

        pcl::io::loadPCDFile (pcdfile.toStdString(), *viewerPCD);

        pvis2.addPointCloud<pcl::PointXYZRGB>(viewerPCD,"viewerPCD", v5);
        pvis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,1.0, "viewerPCD", v5);
        ui->qvtkWidget->update();
        isViewerPCDLoaded=true;
        viewer_transform();
    }
}

void MainWindow::viewer_transform()
{
    pvis2.removeAllPointClouds();
    pvis2.removeAllShapes();
    pvis2.addText(".",0,310,11,0,0,0,"otitle",v5);
    pvis2.addText(".",0,310,11,0,0,0,"ftitle",v6);
    pvis2.addText(".",0,325,11,0,0,0,"kns",v5);
    pvis2.addText(".",0,310,11,0,0,0,"knsf",v6);
    Eigen::Affine3f transformation;
    float roll=ui->rotationX->text().toFloat()*M_PI/180;
    float pitch=ui->rotationY->text().toFloat()*M_PI/180;
    float yaw=ui->rotationZ->text().toFloat()*M_PI/180;
    float x=ui->trX->text().toFloat()/100;
    float y=ui->trY->text().toFloat()/100;
    float z=ui->trZ->text().toFloat()/100;
    transformation = pcl::getTransformation(x,y,z,roll,pitch,yaw);
    pcl::transformPointCloud(*viewerPCD, *viewerPCDtransformed, transformation.matrix());
    pvis2.addPointCloud<pcl::PointXYZRGB>(viewerPCD,"viewerPCD", v5);
    pvis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9,0.9,0.9, "viewerPCD", v5);
    pvis2.addPointCloud<pcl::PointXYZRGB>(viewerPCDtransformed,"viewerPCDtransformed", v5);
    pvis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,0.0, "viewerPCDtransformed", v5);
    ui->qvtkWidget_2->update();
    viewer_applyFilter();
}

void MainWindow::viewer_applyFilter()
{
    if(isViewerPCDLoaded){
        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        grid.setLeafSize (ui->viewer_leafSize->text().toDouble(), ui->viewer_leafSize->text().toDouble(), ui->viewer_leafSize->text().toDouble());
        grid.setInputCloud (viewerPCD);
        grid.filter (*viewerPCDfiltered);

        pvis2.removePointCloud("viewerPCDfiltered",v6);
        for (unsigned int i = 0; i < viewerPCDfiltered->size();i++)
        {
            viewerPCDfiltered->points[i].r=230;
            viewerPCDfiltered->points[i].g=230;
            viewerPCDfiltered->points[i].b=230;
        }
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (viewerPCDfiltered);
        pvis2.addPointCloud<pcl::PointXYZRGB>(viewerPCDfiltered,rgb,"viewerPCDfiltered", v6);

        QString kns="Point Number = "+QString::number(viewerPCD->size());
        QString knsf="Point Number = "+QString::number(viewerPCDfiltered->size());

        pvis2.updateText("Original Data\n-------------------------------",10,25,11,1.0,1.0,1.0,"otitle");
        pvis2.updateText(kns.toStdString().data(),10,10,11,1.0,1.0,1.0,"kns");

        pvis2.updateText("Filtered Data\n-------------------------------",10,25,11,1.0,1.0,1.0,"ftitle");
        pvis2.updateText(knsf.toStdString().data(),10,10,11,1.0,1.0,1.0,"knsf");

        Eigen::Affine3f transformation;
        float roll=ui->rotationX->text().toFloat()*M_PI/180;
        float pitch=ui->rotationY->text().toFloat()*M_PI/180;
        float yaw=ui->rotationZ->text().toFloat()*M_PI/180;
        float x=ui->trX->text().toFloat();
        float y=ui->trY->text().toFloat();
        float z=ui->trZ->text().toFloat();
        transformation = pcl::getTransformation(x,y,z,roll,pitch,yaw);
        pcl::transformPointCloud(*viewerPCDfiltered, *viewerPCDtransformedFiltered, transformation.matrix());

        pvis2.removePointCloud("viewerPCDtransformedFiltered",v6);
        pvis2.addPointCloud<pcl::PointXYZRGB>(viewerPCDtransformedFiltered,"viewerPCDtransformedFiltered", v6);
        pvis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,0.0, "viewerPCDtransformedFiltered", v6);

        //pvis2.addCoordinateSystem(1.0, v5);
        //pvis2.addCoordinateSystem(1.0, v6);
        ui->qvtkWidget_2->update();
    }
}

void MainWindow::on_viewer_leafSize_valueChanged()
{
    viewer_applyFilter();
}

void MainWindow::on_saveFiltered_clicked()
{
    QString filename=QFileDialog::getSaveFileName(0,"Save file",QDir::currentPath(),"PCD files (*.pcd)",new QString("PCD files (*.pcd)"));
    if(!filename.contains(".pcd"))
        filename+=".pcd";
    if(isViewerPCDLoaded)
        pcl::io::savePCDFileASCII(filename.toStdString(),*viewerPCDfiltered);
}

void MainWindow::on_saveTransformed_clicked()
{
    QString filename=QFileDialog::getSaveFileName(0,"Save file",QDir::currentPath(),"PCD files (*.pcd)",new QString("PCD files (*.pcd)"));
    if(!filename.contains(".pcd"))
        filename+=".pcd";
    pcl::io::savePCDFileASCII(filename.toStdString(),*viewerPCDtransformedFiltered);
}

void MainWindow::on_selectSource_clicked()
{
    QString pcdfile = QFileDialog::getOpenFileName(this,
                                                   tr("Open PCD File"), QDir::currentPath(), tr("PCD Files (*.pcd)"));
    if(pcdfile!=NULL){
        if(isMatcherSourceLoaded){
            pvis3.removePointCloud("matcherCurrScan",v7);
        }

        pcl::io::loadPCDFile (pcdfile.toStdString(), *matcherCurrScan);
        pcl::copyPointCloud(*matcherCurrScan,*matcherCurrScanOriginal);

        matcherICP.stepNum=0;

        pvis3.addPointCloud<pcl::PointXYZRGB>(matcherCurrScan,"matcherCurrScan", v7);
        pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, "matcherCurrScan", v7);
        ui->qvtkWidget_3->update();
        isMatcherSourceLoaded=true;
        if(isMatcherSourceLoaded && isMatcherTargetLoaded){
            ui->matcherICP_step->setEnabled(true);
            ui->matcherICP_align->setEnabled(true);
        }
    }
}

void MainWindow::on_selectTarget_clicked()
{
    QString pcdfile = QFileDialog::getOpenFileName(this,
                                                   tr("Open PCD File"), QDir::currentPath(), tr("PCD Files (*.pcd)"));
    if(pcdfile!=NULL){
        if(isMatcherTargetLoaded){
            pvis3.removePointCloud("matcherPrevScan",v7);
        }

        pcl::io::loadPCDFile (pcdfile.toStdString(), *matcherPrevScan);
        pcl::copyPointCloud(*matcherPrevScan,*matcherPrevScanOriginal);

        matcherICP.stepNum=0;

        pvis3.addPointCloud<pcl::PointXYZRGB>(matcherPrevScan,"matcherPrevScan", v7);
        pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,1.0, "matcherPrevScan", v7);
        ui->qvtkWidget_3->update();
        isMatcherTargetLoaded=true;
        if(isMatcherSourceLoaded && isMatcherTargetLoaded){
            ui->matcherICP_step->setEnabled(true);
            ui->matcherICP_align->setEnabled(true);
        }
    }
}

void MainWindow::on_matcherICP_step_clicked()
{
    ui->matcherICP_step->setEnabled(false);
    ui->matcherICP_align->setEnabled(false);
    QApplication::processEvents();
    matcherICP.medianCoef = ui->matcher_medianCoefficient->text().toDouble();
    matcherICP.breakCriteria = ui->matcher_breakCriteria->text().toDouble();
    clearMatcher();
    t1 = clock();
    matcherICP.step();
    t2 = clock();
    updateMatcherView();
    ui->matcherICP_step->setEnabled(true);
    ui->matcherICP_align->setEnabled(true);
}

void MainWindow::clearMatcher(){
    ui->matcher_leafSize->setEnabled(false);
    pvis3.removeAllShapes();
    pvis3.removePointCloud("ICPtarget");
    pvis3.removePointCloud("ICPsource");
    pvis3.removePointCloud("targetLine");
    pvis3.removePointCloud("sourceLine");
    pvis3.addText(".",550,310,11,0,0,0,"icptitle",v10);
    pvis3.addText(".",550,370,11,0,0,0,"ea",v10);
    pvis3.addText(".",550,355,11,0,0,0,"tra",v10);
    pvis3.addText(".",550,340,11,0,0,0,"gs",v10);
    pvis3.addText(".",550,265,11,0,0,0,"iterbr",v10);
    pvis3.addText(".",550,265,11,0,0,0,"ens",v9);
    pvis3.addText(".",550,310,11,0,0,0,"otitle",v7);
    pvis3.addText(".",550,325,11,0,0,0,"kns",v7);
    pvis3.addText(".",550,295,11,0,0,0,"hns",v7);
    pvis3.addText(".",550,310,11,0,0,0,"ftitle",v8);
    pvis3.addText(".",550,310,11,0,0,0,"knsf",v8);
    pvis3.addText(".",550,280,11,0,0,0,"hnsf",v8);

    if(matcherICP.stepNum==0){
        pvis3.removePointCloud("source");
        pvis3.removePointCloud("target");
        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        grid.setLeafSize (ui->matcher_leafSize->text().toDouble(), ui->matcher_leafSize->text().toDouble(), ui->matcher_leafSize->text().toDouble());
        grid.setInputCloud (matcherCurrScanOriginal);
        grid.filter (*matcher_source_filtered);

        grid.setInputCloud (matcherPrevScanOriginal);
        grid.filter (*matcher_target_filtered);

        pvis3.addPointCloud<pcl::PointXYZRGB>(matcher_source_filtered,"source", v8);
        pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, "source", v8);

        pvis3.addPointCloud<pcl::PointXYZRGB>(matcher_target_filtered,"target", v8);
        pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,1.0, "target", v8);
        ui->qvtkWidget_3->update();
        T = Eigen::Matrix4f::Identity ();
        matcherICP.setSource(*matcher_source_filtered);
        matcherICP.setTarget(*matcher_target_filtered);
        matcherICP.maxIteration=ui->matcher_maxIter->text().toInt();
    }
}

void MainWindow::updateMatcherView(){
    double gsure=(double)(t2-t1)*1000/CLOCKS_PER_SEC;
    T = matcherICP.getFinalTransformation();

    Eigen::Affine3f transformation;
    transformation = pcl::getTransformation(-ui->trX->text().toDouble(),
                                            -ui->trY->text().toDouble(),
                                            -ui->trZ->text().toDouble(),
                                            -ui->rotationX->text().toDouble()*M_PI/180,
                                            -ui->rotationY->text().toDouble()*M_PI/180,
                                            -ui->rotationZ->text().toDouble()*M_PI/180);

    float roll,pitch,yaw;
    getEulerAngles(T.block<3,3>(0,0),roll,pitch,yaw);
    //std::cout<<roll*180/M_PI <<"\t" <<pitch*180/M_PI <<"\t" << yaw*180/M_PI<<endl;

    pcl::copyPointCloud(*matcherCurrScanOriginal,*matcherCurrScan);
    pcl::transformPointCloud(*matcherCurrScan,*matcherCurrScan,T);

    Eigen::Vector3f u; float angle;
    getAxisAngle(T.block<3,3>(0,0), u, angle);

    //std::cout<<angle*180/M_PI<<endl;
    //std::cout<<u<<endl;

    QString ea="Euler Acilari (deg) = "+QString::number(roll*180/M_PI,'f',3)+"  "+QString::number(pitch*180/M_PI,'f',3)+"  "+QString::number(yaw*180/M_PI,'f',3);
    QString tra="Oteleme (mm) = "+QString::number(T(12),'f',3)+"  "+QString::number(T(13),'f',3)+"  "+QString::number(T(14),'f',3);
    QString gs="Gecen Sure (ms) = "+QString::number(gsure);
    QString kns="Kaynak Nokta Sayisi = "+QString::number(matcherCurrScan->size());
    QString knsf="Kaynak Nokta Sayisi = "+QString::number(matcher_source_filtered->size());
    QString hns="Hedef Nokta Sayisi = "+QString::number(matcherPrevScan->size());
    QString hnsf="Hedef Nokta Sayisi = "+QString::number(matcher_target_filtered->size());
    QString ens="Eslesen Nokta Sayisi = "+QString::number(matcherICP.totalMatchedPoint);
    QString iterbr;
    if(matcherICP.stepNum!=0){
        iterbr = "Toplam Iterasyon = "+QString::number(matcherICP.stepNum);
    }else{
        iterbr = "Toplam Iterasyon = "+QString::number(matcherICP.iterbr+1);
    }
    pvis3.updateText("Orijinal Veriler\n-------------------------------",0,285,11,1.0,1.0,1.0,"otitle");
    pvis3.updateText(kns.toStdString().data(),0,270,11,1.0,1.0,1.0,"kns");
    pvis3.updateText(hns.toStdString().data(),0,255,11,1.0,1.0,1.0,"hns");

    pvis3.updateText("Filtrelendikten Sonraki\n-------------------------------",0,285,11,1.0,1.0,1.0,"ftitle");
    pvis3.updateText(knsf.toStdString().data(),0,270,11,1.0,1.0,1.0,"knsf");
    pvis3.updateText(hnsf.toStdString().data(),0,255,11,1.0,1.0,1.0,"hnsf");

    pvis3.addPointCloud<pcl::PointXYZRGB>(matcherPrevScan,"ICPtarget", v10);
    pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1.0,1.0, "ICPtarget", v10);
    pvis3.addPointCloud<pcl::PointXYZRGB>(matcherCurrScan,"ICPsource",v10);
    pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, "ICPsource",v10);

    pvis3.updateText(ens.toStdString().data(),0,295,11,1.0,1.0,1.0,"ens");
    pvis3.addPointCloud<pcl::PointXYZRGB>(matcher_target_filtered,"targetLine", v9);
    pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1.0,1.0, "targetLine", v9);
    pcl::transformPointCloud(*matcher_source_filtered,*matcher_source_filtered_tr,matcherICP.Tprev);
    pvis3.addPointCloud<pcl::PointXYZRGB>(matcher_source_filtered_tr,"sourceLine",v9);
    pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, "sourceLine",v9);

    for(unsigned int i=0;i<matcherICP.kdcorresp.size();i++){
        QString lname = QString::number(i);
        pvis3.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB> (matcherICP.set2_tr->points[i], matcherICP.set1_tr->points[matcherICP.kdcorresp[i]], 0.0, 1.0, 0.0, lname.toStdString().data(), v9);
    }

    pvis3.updateText("ICP Algoritmasi Sonucu\n-------------------------------",0,285,11,1.0,1.0,1.0,"icptitle");
    pvis3.updateText(ea.toStdString().data(),0,270,11,1.0,1.0,1.0,"ea");
    pvis3.updateText(tra.toStdString().data(),0,255,11,1.0,1.0,1.0,"tra");
    pvis3.updateText(gs.toStdString().data(),0,240,11,1.0,1.0,1.0,"gs");
    pvis3.updateText(iterbr.toStdString().data(),0,225,11,1.0,1.0,1.0,"iterbr");

    ui->qvtkWidget_3->update();
}

void MainWindow::on_matcher_clearScreen_clicked()
{
    pvis3.removeAllPointClouds();
    pvis3.removeAllShapes();
    ui->qvtkWidget_3->update();
    ui->matcherICP_step->setEnabled(false);
    ui->matcherICP_align->setEnabled(false);
    isMatcherSourceLoaded=false;
    isMatcherTargetLoaded=false;
}

void MainWindow::on_matcherICP_align_clicked()
{
    matcherICP.stepNum=0;
    ui->matcherICP_align->setEnabled(false);
    ui->matcherICP_step->setEnabled(false);
    ui->matcher_leafSize->setEnabled(false);
    QApplication::processEvents();
    matcherICP.medianCoef = ui->matcher_medianCoefficient->text().toDouble();
    matcherICP.breakCriteria = ui->matcher_breakCriteria->text().toDouble();
    matcherICP.fixMedianCoefficient=true;
    clearMatcher();
    t1 = clock();
    matcherICP.align();
    t2 = clock();
    updateMatcherView();
    ui->matcherICP_align->setEnabled(true);
    ui->matcherICP_step->setEnabled(true);
    ui->matcher_leafSize->setEnabled(true);
}

void MainWindow::on_rotationX_valueChanged()
{
    viewer_transform();
}

void MainWindow::on_trX_valueChanged()
{
    viewer_transform();
}

void MainWindow::on_rotationY_valueChanged()
{
    viewer_transform();
}

void MainWindow::on_trY_valueChanged()
{
    viewer_transform();
}

void MainWindow::on_rotationZ_valueChanged()
{
    viewer_transform();
}

void MainWindow::on_trZ_valueChanged()
{
    viewer_transform();
}

void MainWindow::on_viewerAddNoise_clicked()
{
    if(isViewerPCDLoaded){
        pvis2.removePointCloud("viewerPCDtransformedFiltered",v6);
        for(unsigned int i=0;i<viewerPCDtransformedFiltered->size();i++){
            viewerPCDtransformedFiltered->points[i].x+=0.05*random()/RAND_MAX-0.025;
            viewerPCDtransformedFiltered->points[i].y+=0.05*random()/RAND_MAX-0.025;
            viewerPCDtransformedFiltered->points[i].z+=0.05*random()/RAND_MAX-0.025;
        }
        pvis2.addPointCloud<pcl::PointXYZRGB>(viewerPCDtransformedFiltered,"viewerPCDtransformedFiltered", v6);
        pvis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,0.0, "viewerPCDtransformedFiltered", v6);
        ui->qvtkWidget_2->update();
    }
}

void MainWindow::on_arrow_clicked()
{
    if(isViewerPCDLoaded){
        pvis3.removeAllShapes();
        pvis3.removeAllPointClouds();
        if(isMatcherSourceLoaded){
            pvis3.removePointCloud("matcherCurrScan",v7);
        }
        pcl::copyPointCloud(*viewerPCDtransformedFiltered,*matcherCurrScan);
        pcl::copyPointCloud(*matcherCurrScan,*matcherCurrScanOriginal);
        pvis3.addPointCloud<pcl::PointXYZRGB>(matcherCurrScan,"matcherCurrScan", v7);
        pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, "matcherCurrScan", v7);
        isMatcherSourceLoaded=true;

        if(isMatcherTargetLoaded){
            pvis3.removePointCloud("matcherPrevScan",v7);
        }
        pcl::copyPointCloud(*viewerPCDfiltered,*matcherPrevScan);
        pcl::copyPointCloud(*matcherPrevScan,*matcherPrevScanOriginal);
        pvis3.addPointCloud<pcl::PointXYZRGB>(matcherPrevScan,"matcherPrevScan", v7);
        pvis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,1.0, "matcherPrevScan", v7);
        isMatcherTargetLoaded=true;
        matcherICP.stepNum=0;
        ui->matcherICP_step->setEnabled(true);
        ui->matcherICP_align->setEnabled(true);

        ui->qvtkWidget_3->update();
        ui->tabWidget->setCurrentIndex(1);
    }else{
        QMessageBox::information( this, "Point Cloud Viewer",
                                  "Load a PCD file first to jump to the Point Cloud Matcher.", QMessageBox::Ok, 0 );
    }
}

void MainWindow::on_matcher_onetoone_clicked()
{
    if(ui->matcher_onetoone->isChecked()){
        matcherICP.onetoone=true;
    }else{
        matcherICP.onetoone=false;
    }
}

void MainWindow::on_fixMedianCoefficient_clicked()
{
    if(ui->fixMedianCoefficient->isChecked()){
        ui->randomMedianCoefficient->setChecked(false);
        ui->useRANSAC->setChecked(false);
        ui->adaptiveMedianCoefficient->setChecked(false);
    }
}

void MainWindow::on_randomMedianCoefficient_clicked()
{
    if(ui->randomMedianCoefficient->isChecked()){
        ui->fixMedianCoefficient->setChecked(false);
        ui->useRANSAC->setChecked(false);
        ui->adaptiveMedianCoefficient->setChecked(false);
    }
}

void MainWindow::on_useRANSAC_clicked()
{
    if(ui->useRANSAC->isChecked()){
        ui->randomMedianCoefficient->setChecked(false);
        ui->fixMedianCoefficient->setChecked(false);
        ui->adaptiveMedianCoefficient->setChecked(false);
    }
}

void MainWindow::on_adaptiveMedianCoefficient_clicked()
{
    if(ui->adaptiveMedianCoefficient->isChecked()){
        ui->randomMedianCoefficient->setChecked(false);
        ui->fixMedianCoefficient->setChecked(false);
        ui->useRANSAC->setChecked(false);
    }
}

void MainWindow::on_pclICPAlign_clicked()
{
    int start=ui->pclICP_start->text().toInt();
    int end=ui->pclICP_end->text().toInt();
    int step=ui->pclICP_step->text().toInt();

    Tg = Eigen::Matrix4f::Identity();

    std::vector<Eigen::Matrix4f> T_history;
    int k=0;

    boost::filesystem::create_directories("benchmark_results");
    t3 = clock();

    QString dirName = "benchmark_results/PCLICP_LS_"+ui->pcl_leafsize->text()+"_MCD_"+ui->pcl_maxcorrespdist->text();
    boost::filesystem::create_directories(dirName.toStdString().c_str());
    k=0;
    T_history.clear();
    Tg=Eigen::Matrix4f::Identity();

    PCLICP.setMaxCorrespondenceDistance (ui->pcl_maxcorrespdist->text().toDouble());
    PCLICP.setTransformationEpsilon(ui->pcl_transEpsilon->text().toDouble());
    PCLICP.setMaximumIterations(ui->pcl_maxiter->text().toInt());

    for(int i=start; i<=end-step;i+=step){
        bool usedisplay = ui->pclDisplayClouds->isChecked();
        loadPCDFiles_PCL(i, i+step);
        qDebug()<<i+step<<"to"<<start<<" ---------------------------------------";

        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        grid.setLeafSize(ui->pcl_leafsize->text().toDouble(),ui->pcl_leafsize->text().toDouble(),ui->pcl_leafsize->text().toDouble());
        grid.setInputCloud (currScanOriginal);
        grid.filter (*source_filtered);

        grid.setInputCloud (prevScanOriginal);
        grid.filter (*target_filtered);

        PCLICP.setInputCloud(source_filtered);
        PCLICP.setInputTarget(target_filtered);

        if(usedisplay){
            //ADD POINT CLOUDS
            Eigen::Matrix4f tempT;
            tempT = Eigen::Matrix4f::Identity();
            if(ui->pclICP_IMU->isChecked()){
                for(int j=0;j<i+1;j++){
                    tempT*=GroundTruthNoisy.at(j);
                }
            }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempdata(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*source_filtered,*tempdata);
            pcl::transformPointCloud(*tempdata,*tempdata,tempT);

            QString id="cs"+QString::number(i);
            pvis4.addPointCloud<pcl::PointXYZRGB>(tempdata,id.toStdString().c_str(), v11);
            pvis4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, id.toStdString().c_str(), v11);
            ui->qvtkWidget->update();
        }

        if(ui->pclICP_IMU->isChecked()){
            Eigen::Matrix4f T;
            T = Eigen::Matrix4f::Identity();
            for(int j=i;j<i+step;j++){
                T*=GroundTruthNoisy.at(j+1);
            }
            t1 = clock();
            PCLICP.align(*source_filtered, T);
        }else{
            t1 = clock();
            PCLICP.align(*source_filtered);
        }
        t2 = clock();
        T = PCLICP.getFinalTransformation();

        T_history.push_back(T);
        Tg *= T;
        //std::cout<<Tg<<endl;

        if(usedisplay){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempdata(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*source_filtered,*tempdata);
            pcl::transformPointCloud(*tempdata,*tempdata,Tg);

            QString id="ps"+QString::number(i);
            pvis4.addPointCloud<pcl::PointXYZRGB>(tempdata,id.toStdString().c_str(), v12);
            pvis4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,1.0, id.toStdString().c_str(), v12);
            ui->qvtkWidget_5->update();
        }

        std::ofstream myfile;

        //Write to File------------------------------------------------------------------------------------
        QString poseFile = dirName+"/pose_scanner_leica.csv";
        myfile.open (poseFile.toStdString().c_str(), ios::app);
        if(k==0){
            myfile<<"poseId, timestamp, T00, T01, T02, T03, T10, T11, T12, T13, T20, T21, T22, T23, T30, T31, T32, T33"<<endl;
            myfile<<"0, 0, 1, -0, -0, 0, -0, 1, -0, 0, 0, -0, 1, 0, 0, 0, 0, 1"<<endl;
            k++;
        }
        myfile<<k<<", 0, ";
        for(int x=0;x<4;x++){
            for(int y=0;y<4;y++){
                myfile<<Tg(x,y);
                if(x!=3 || y!=3)
                    myfile<<", ";
            }
        }
        myfile<<endl;
        myfile.close();
        k++;
        //-------------------------------------------------------------------------------------------------

        //Write to File------------------------------------------------------------------------------------
        QString timeFile = dirName+"/time_elapsed_ms.txt";
        myfile.open (timeFile.toStdString().c_str(), ios::app);
        myfile<<(double)(t2-t1)*1000/CLOCKS_PER_SEC<<endl;
        myfile.close();

        //Write to File------------------------------------------------------------------------------------
        QString iterFile = dirName+"/iteration.txt";
        myfile.open (iterFile.toStdString().c_str(), ios::app);
        myfile<<PCLICP.nr_iterations_<<endl;
        myfile.close();
    }
}

void MainWindow::on_pclICP_readGT_clicked()
{
    QString csvfile = QFileDialog::getOpenFileName(this,
                                                   tr("Open IMU File"), QDir::currentPath(), tr("CSV Files (*.csv)"));
    if(csvfile!=NULL){
        GroundTruth.clear();
        GroundTruthNoisy.clear();
        //std::ifstream file("pose_scanner_leica.csv");
        std::ifstream file(csvfile.toStdString().c_str());

        CSVRow row;
        while(row.readNextRow(file))
        {
            char* p;
            strtod(row[0].c_str(), &p);
            if (!*p) {
                Eigen::Matrix4f T,N;

                T(0,0)=strToDbl(row[2]);T(0,1)=strToDbl(row[3]);T(0,2)=strToDbl(row[4]);T(0,3)=strToDbl(row[5]);
                T(1,0)=strToDbl(row[6]);T(1,1)=strToDbl(row[7]);T(1,2)=strToDbl(row[8]);T(1,3)=strToDbl(row[9]);
                T(2,0)=strToDbl(row[10]);T(2,1)=strToDbl(row[11]);T(2,2)=strToDbl(row[12]);T(2,3)=strToDbl(row[13]);
                T(3,0)=strToDbl(row[14]);T(3,1)=strToDbl(row[15]);T(3,2)=strToDbl(row[16]);T(3,3)=strToDbl(row[17]);

                if(GroundTruth.size()==0){
                    GroundTruthNoisy.push_back(T);
                    GroundTruth.push_back(T);
                }else{
                    N=T;
                    //N=GroundTruth.back().inverse()*T;
                    //addNoise(N, ui->multi_noiseR->text().toInt(), ui->multi_noiseT->text().toInt());
                    GroundTruthNoisy.push_back(N);
                    GroundTruth.push_back(T);
                }

            }
        }
        isIMUloaded=true;
        ui->pclICP_IMU->setEnabled(true);
    }
}

void MainWindow::on_pclICP_IMU_clicked()
{
    if(isPCDLoaded){
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempdata(new pcl::PointCloud<pcl::PointXYZRGB>);
        if(ui->pclICP_IMU->isChecked()){
            pcl::copyPointCloud(*currScan,*tempdata);
            Eigen::Matrix4f T;
            T = Eigen::Matrix4f::Identity ();
            for(int i=ui->multi_pcd_start->text().toInt();i<ui->multi_pcd_end->text().toInt();i++){
                T*=GroundTruthNoisy.at(i+1);
            }
            pcl::transformPointCloud(*currScan,*currScan,T);
            pvis1.updatePointCloud(currScan, "currScan");
        }else{
            pcl::copyPointCloud(*tempdata,*currScan);
            pvis1.removePointCloud("currScan", v11);
            pvis1.addPointCloud<pcl::PointXYZRGB>(tempdata,"currScan", v11);
            pvis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, "currScan", v11);
        }
        ui->qvtkWidget_5->update();
    }
}

void MainWindow::on_pomerleauBreak_clicked()
{
    if(ui->pomerleauBreak->isChecked()){
        ui->pclBreak->setChecked(false);
    }
}

void MainWindow::on_pclBreak_clicked()
{
    if(ui->pclBreak->isChecked()){
        ui->pomerleauBreak->setChecked(false);
    }
}

void MainWindow::on_selectMultiFolder_clicked()
{
    QString multifolder = QFileDialog::getExistingDirectory(this, tr("Select Directory"),
                                                            "/home",
                                                            QFileDialog::ShowDirsOnly
                                                            | QFileDialog::DontResolveSymlinks);
    multifolderpath=multifolder;
}

void MainWindow::on_selecPCLFolder_clicked()
{
    QString pclfolder = QFileDialog::getExistingDirectory(this, tr("Select Directory"),
                                                            "/home",
                                                            QFileDialog::ShowDirsOnly
                                                            | QFileDialog::DontResolveSymlinks);
    pclfolderpath=pclfolder;
}

void MainWindow::on_pclICP_clearScreen_clicked()
{
    pvis4.removeAllPointClouds();
    pvis4.removeAllShapes();
    ui->qvtkWidget_5->update();
    ui->pclICP_IMU->setChecked(false);
    ui->pclICP_IMU->setEnabled(false);
}
