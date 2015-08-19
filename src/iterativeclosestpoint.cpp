#include "iterativeclosestpoint.h"

IterativeClosestPoint:: IterativeClosestPoint()
{
    set1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    set1_tr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    set2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    set2_tr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    kdtreediff=mediandiff=lastSumDist=sumDist=stepNum=0;
    medianCoef=3;
    onetoone=false;
    fixMedianCoefficient=false;
    randomMedianCoefficient=false;
    adaptiveMedianCoefficient=false;
    useRANSAC=false;
    pclLikeBreak=false;
    pomerleauLikeBreak=false;
}

void IterativeClosestPoint::setSource(pcl::PointCloud<pcl::PointXYZRGB> &source){
    *set1 = source;
}

void IterativeClosestPoint::setTarget(pcl::PointCloud<pcl::PointXYZRGB> &target){
    *set2 = target;
}

void IterativeClosestPoint::step(){
    static Eigen::VectorXf p1(3);
    static Eigen::VectorXf p2(3);
    static Eigen::VectorXf t(3);
    static Eigen::VectorXf tempt(3);
    if(stepNum==0){
        T = Eigen::Matrix4f::Identity ();
    }

    pcl::copyPointCloud(*set1, *set1_tr);
    pcl::transformPointCloud(*set1_tr,*set1_tr,T);
    Tprev = T;

    //k-D Tree Closest -----------------------------------------------------------------------------
    kdcorresp.resize(set2->points.size());
    kddistance.resize(set2->points.size());

    kdtree.setInputCloud (set1_tr);
    int K = 1;

    pcl::PointXYZRGB searchPoint;
    for(unsigned int i=0;i<set2->points.size();i++){

        searchPoint.x = set2->points[i].x;
        searchPoint.y = set2->points[i].y;
        searchPoint.z = set2->points[i].z;

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            kdcorresp[i]=pointIdxNKNSearch[0];
            kddistance[i]=pointNKNSquaredDistance[0];
        }
    }
    //-------------------------------------------------------------------------------------------------
    //find correspondences where the distance is above threshold (3*median)
    std::vector<int> k;
    t1 = clock();
    double threshold = medianCoef*median(kddistance);
    t2 = clock();
    mediandiff += double(t2-t1)*1000/CLOCKS_PER_SEC;
    //std::cout<<kddistance.size()<<"---median-------------"<<mediandiff<<std::endl;
    for(unsigned int i=0;i<kdcorresp.size();i++){
        if(kddistance[i]>threshold){
            k.push_back(i);
        }
    }

    //remove them from corresp and distance vectors
    for(unsigned int i=0;i<k.size();i++){
        kdcorresp.erase(kdcorresp.begin()+k[i]-i,kdcorresp.begin()+k[i]-i+1);
        kddistance.erase(kddistance.begin()+k[i]-i,kddistance.begin()+k[i]-i+1);
    }

    pcl::copyPointCloud(*set2,*set2_tr);
    for(unsigned int i=0;i<k.size();i++){
        set2_tr->points.erase(set2_tr->points.begin()+k[i]-i,set2_tr->points.begin()+k[i]-i+1);
    }
    //-------------------------------------------------------------------------------------------------
    //DEBUG
    /*
    for(int i=0;i<kdcorresp.size();i++){
       qDebug()<<kdcorresp[i]<<"\t"<<kddistance[i];
    }
    */

    //Ağırlık merkezlerini hesapla
    mean(*set1_tr, p1, true, kdcorresp); //Sadece ilişkili noktalar
    mean(*set2_tr, p2, false);

    Eigen::MatrixXf M(3,3);
    M.setZero(3,3);
    for(unsigned int i=0;i<set2_tr->points.size();i++){
        Eigen::VectorXf temp(3);
        temp(0)=set1_tr->points[kdcorresp[i]].x-p1(0);
        temp(1)=set1_tr->points[kdcorresp[i]].y-p1(1);
        temp(2)=set1_tr->points[kdcorresp[i]].z-p1(2);

        Eigen::RowVectorXf temp2(3);
        temp2(0)=set2_tr->points[i].x-p2(0);
        temp2(1)=set2_tr->points[i].y-p2(1);
        temp2(2)=set2_tr->points[i].z-p2(2);

        M += temp*temp2;
    }

    //SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXf R(3,3);
    Eigen::MatrixXf U(3,3);
    Eigen::MatrixXf V(3,3);

    U = svd.matrixU();
    V = svd.matrixV();

    if (U.determinant () * V.determinant () < 0)
    {
        for (int x = 0; x < 3; ++x)
            V (x, 2) *= -1;
    }

    R = V*U.transpose();

    //compute the incremental translation
    tempt = t;
    t = p2-R*p1;

    //compute the tranformation
    Eigen::Matrix4f tempT = Eigen::Matrix4f::Identity ();
    rt2tr(R,t,tempT);
    T = T * tempT;

    totalMatchedPoint = kdcorresp.size();
    stepNum++;
}

void IterativeClosestPoint::align(Eigen::Matrix4f IMU){
    Eigen::VectorXf p1(3);
    mean(*set1, p1, false);

    Eigen::VectorXf p2(3);
    mean(*set2, p2, false);

    Eigen::VectorXf t(3);
    Eigen::VectorXf tempt(3);
    //t = p2-p1;

    T = IMU;
    //T.block<3,1>(0,3) = t;

    iterbr=0;

    double Eupper;
    double previousError;
    //double previousError2;
    int noChangeinError=0;

    std::ofstream myfile;
    QString sumdistfile = dirName+"/sumdist"+QString::number(icpCount)+".txt";

    for(int m=0; m<maxIteration; m++){

        pcl::copyPointCloud(*set1, *set1_tr);
        pcl::transformPointCloud(*set1_tr,*set1_tr,T);
        Tprev = T;

        //Closest --------------------------------------------------------------------------------------
        /*
        kdcorresp.resize(set2->points.size());
        kddistance.resize(set2->points.size());
        t1 = clock();
        closest.search(*set2,*set1_tr, kdcorresp, kddistance);
        t2 = clock();
        kdtreediff += (double)(t2-t1);
        //std::cout<<"diff = "<<kdtreediff<<std::endl;
        //std::cout<<"time = "<<kdtreediff*1000/CLOCKS_PER_SEC<<std::endl;
        */
        //----------------------------------------------------------------------------------------------


        //k-D Tree Closest -----------------------------------------------------------------------------
        source_indices.clear();
        source_indices.resize(set2->points.size());
        kdcorresp.resize(set2->points.size());
        kddistance.resize(set2->points.size());
        std::vector<int> kdcorresp2;
        std::vector<double> kddistance2;

        int cnt = 0;

        kdtree.setInputCloud (set1_tr);
        int K = 1;

        pcl::PointXYZRGB searchPoint;
        //        t1 = clock();
        for(unsigned int i=0;i<set2->points.size();i++){

            searchPoint.x = set2->points[i].x;
            searchPoint.y = set2->points[i].y;
            searchPoint.z = set2->points[i].z;

            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                source_indices[cnt]=i;
                kdcorresp[i]=pointIdxNKNSearch[0];
                kddistance[i]=pointNKNSquaredDistance[0];
                cnt++;
            }
        }
        //        t2 = clock();
        //        kdtreediff += (double)(t2-t1);
        //        std::cout<<"diff = "<<kdtreediff<<std::endl;
        //        std::cout<<"time = "<<kdtreediff*1000/CLOCKS_PER_SEC<<std::endl;

        //-------------------------------------------------------------------------------------------------

        //Write to File------------------------------------------------------------------------------------
        /*
        QString filename=QString::number(m)+".txt";
        std::ofstream output_file(filename.toStdString().data());
        std::ostream_iterator<double> output_iterator(output_file, "\n");
        std::copy(kdcorresp.begin(), kdcorresp.end(), output_iterator);
        */
        //-------------------------------------------------------------------------------------------------

        //MEDIAN
        if(!useRANSAC){
            //find correspondences where the distance is above threshold (3*median)
            std::vector<int> k;
            t1 = clock();
            double threshold;
            if(fixMedianCoefficient){
                /*
                if(noChangeinError==5){
                    qDebug()<<"JUMP TO 15";
                    double mc_temp=medianCoef;
                    medianCoef= 15;
                    threshold = medianCoef*median(kddistance);
                    noChangeinError=0;
                    qDebug()<<"previous error="<<previousError;
                    qDebug()<<"medianCoef="<<medianCoef;
                    medianCoef=mc_temp;
                }else{
                    qDebug()<<"previous error="<<previousError;
                    qDebug()<<"medianCoef="<<medianCoef;
                    threshold = medianCoef*median(kddistance);
                }
                */
                threshold = medianCoef*median(kddistance);
            }

            if(randomMedianCoefficient){
                medianCoef= 3 + (rand() % 13);
                threshold = medianCoef*median(kddistance);
            }

            if(adaptiveMedianCoefficient){
                if(noChangeinError==5){
                    qDebug()<<"ADAPTIVE";
                    //medianCoef= 3 + (rand() % 13);
                    medianCoef= 15;
                    threshold = medianCoef*median(kddistance);

                    Eigen::Matrix4f randomT = Eigen::Matrix4f::Identity();
                    addNoise(randomT, 1, 1);
                    pcl::transformPointCloud(*set1_tr, *set1_tr, randomT);

                    noChangeinError=0;
                }
                else{
                    if(m==0){
                        threshold = medianCoef*median(kddistance);
                    }else{
                        medianCoef=map(previousError, breakCriteria, Eupper, 3, 15);
                        threshold = medianCoef*median(kddistance);
                    }
                }
            }
            t2 = clock();
            mediandiff += double(t2-t1)*1000/CLOCKS_PER_SEC;
            for(unsigned int i=0;i<kdcorresp.size();i++){
                if(kddistance[i]>threshold){
                    k.push_back(i);
                }
            }

            //------------------------------------------------------------------------------------------------
            //remove them from corresp and distance vectors
            for(unsigned int i=0;i<k.size();i++){
                kdcorresp.erase(kdcorresp.begin()+k[i]-i,kdcorresp.begin()+k[i]-i+1);
                kddistance.erase(kddistance.begin()+k[i]-i,kddistance.begin()+k[i]-i+1);
            }

            pcl::copyPointCloud(*set2,*set2_tr);
            for(unsigned int i=0;i<k.size();i++){
                set2_tr->points.erase(set2_tr->points.begin()+k[i]-i,set2_tr->points.begin()+k[i]-i+1);
            }
            //-------------------------------------------------------------------------------------------------

            //------------------------------------------------------------------------------------------------
            // 1 to 1 Correspondence
            if(onetoone){
                std::vector<int> silinecek;

                double min_val;int min_val_ind;
                std::vector<int> corresp_copy=kdcorresp;

                for(unsigned int i=0;i<kdcorresp.size();i++){
                    if(corresp_copy[i]==0){
                        continue;
                    }else{
                        min_val=kddistance[i];
                        min_val_ind=i;
                    }

                    for(unsigned int j=0;j<kdcorresp.size();j++){
                        if(kdcorresp[j]==kdcorresp[i] && j!=i && kddistance[j]<min_val){
                            corresp_copy[min_val_ind]=0;
                            silinecek.push_back(min_val_ind);

                            min_val = kddistance[j];
                            min_val_ind = j;
                        }else if(kdcorresp[j]==kdcorresp[i] && j!=i && kddistance[j]>=min_val){
                            corresp_copy[j]=0;
                            silinecek.push_back(j);
                        }
                    }
                }

                std::sort(silinecek.begin(), silinecek.end());

                silinecek.erase(
                            std::unique(silinecek.begin(), silinecek.end()),
                            silinecek.end());

                //------------------------------------------------------------------------------------------------
                //remove them from corresp and distance vectors

                for(unsigned int i=0;i<silinecek.size();i++){
                    kdcorresp.erase(kdcorresp.begin()+silinecek[i]-i,kdcorresp.begin()+silinecek[i]-i+1);
                    kddistance.erase(kddistance.begin()+silinecek[i]-i,kddistance.begin()+silinecek[i]-i+1);
                }

                for(unsigned int i=0;i<silinecek.size();i++){
                    set2_tr->points.erase(set2_tr->points.begin()+silinecek[i]-i,set2_tr->points.begin()+silinecek[i]-i+1);
                }

                //-------------------------------------------------------------------------------------------------

            }
        }
        else{
            //RANSAC
            //-------------------------------------------------------------------------------------------------
            std::vector<int> inliers;

            pcl::SampleConsensusModelRegistration< pcl::PointXYZRGB >::Ptr model_r;
            model_r.reset(new pcl::SampleConsensusModelRegistration< pcl::PointXYZRGB > (set2, source_indices));
            model_r->setInputTarget (set1_tr, kdcorresp);

            // Create a RANSAC model
            pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (model_r);

            sac.setMaxIterations (1000);
            sac.setDistanceThreshold(0.1);
            sac.computeModel();
            sac.getInliers (inliers);

            std::vector<int> source_indices_good;
            std::vector<int> target_indices_good;

            source_indices_good.resize (inliers.size ());
            target_indices_good.resize (inliers.size ());

            boost::unordered_map<int, int> source_to_target;
            for (unsigned int i = 0; i < source_indices.size(); ++i){
                source_to_target[source_indices[i]] = kdcorresp[i];
            }

            // Copy just the inliers
            std::copy(inliers.begin(), inliers.end(), source_indices_good.begin());
            for (size_t i = 0; i < inliers.size (); ++i){
                target_indices_good[i] = source_to_target[inliers[i]];
            }

            kdcorresp2.clear();
            kdcorresp2.resize(target_indices_good.size());
            kdcorresp2=target_indices_good;

            kddistance2.clear();
            kddistance2.resize(target_indices_good.size());

            for(unsigned int i=0;i<kdcorresp2.size();i++){
                kddistance2[i]=kddistance[kdcorresp2[i]];
            }

            set2_tr->clear();
            set2_tr->height=1;
            set2_tr->width=source_indices_good.size();
            set2_tr->resize(source_indices_good.size());

            for(unsigned int i=0;i<source_indices_good.size();i++){
                set2_tr->points[i]=set2->points[source_indices_good[i]];
            }

            kdcorresp.clear();
            kdcorresp.resize(kdcorresp2.size());
            kdcorresp=kdcorresp2;

            kddistance.clear();
            kddistance.resize(kddistance2.size());
            kddistance=kddistance2;
        }

        //Ağırlık merkezlerini hesapla
        mean(*set1_tr, p1, true, kdcorresp); //Sadece ilişkili noktalar
        mean(*set2_tr, p2, false);

        Eigen::MatrixXf M(3,3);
        M.setZero(3,3);
        for(unsigned int i=0;i<set2_tr->points.size();i++){
            Eigen::VectorXf temp(3);
            temp(0)=set1_tr->points[kdcorresp[i]].x-p1(0);
            temp(1)=set1_tr->points[kdcorresp[i]].y-p1(1);
            temp(2)=set1_tr->points[kdcorresp[i]].z-p1(2);

            Eigen::RowVectorXf temp2(3);
            temp2(0)=set2_tr->points[i].x-p2(0);
            temp2(1)=set2_tr->points[i].y-p2(1);
            temp2(2)=set2_tr->points[i].z-p2(2);

            M += temp*temp2;
        }

        //SVD
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXf R(3,3);
        Eigen::MatrixXf U(3,3);
        Eigen::MatrixXf V(3,3);

        U = svd.matrixU();
        V = svd.matrixV();


        if (U.determinant () * V.determinant () < 0)
        {
            for (int x = 0; x < 3; ++x)
                V (x, 2) *= -1;
        }

        R = V*U.transpose();

        //compute the incremental translation
        tempt = t;
        t = p2-R*p1;

        //compute the tranformation
        Eigen::Matrix4f tempT = Eigen::Matrix4f::Identity ();
        rt2tr(R,t,tempT);
        Eigen::Matrix4f previousT = T;
        T = T * tempT;

        //Break Criteria-----------------------------------------------------------------------------------
        iterbr=m;
        if(pclLikeBreak){
            Eigen::Matrix4f abs_temp = T-previousT;
            mabs(abs_temp);
            double diff = abs_temp.sum();
            if(m==0){
                Eupper=diff;
                previousError = diff;
            }
            if(diff<breakCriteria){
                std::cout<<"break at "<<m<<std::endl;
                break;
            }else{
                if(std::abs(diff-previousError)<0.0001 && m!=0){
                    noChangeinError++;
                }else{
                    noChangeinError=0;
                }
            }
            previousError = diff;
        }
        else if(pomerleauLikeBreak){
            Eigen::Matrix4f deltaT;
            deltaT = T*previousT.transpose();
            double errorT = sqrt(pow(deltaT(0,3),2)+pow(deltaT(1,3),2)+pow(deltaT(2,3),2));
            double errorR = acos((deltaT.trace()-2)/2);

            if(m==0){
                Eupper=errorT;
                previousError = errorT;
            }
            if(errorT<0.01 && errorR<0.001){
                std::cout<<"break at "<<m<<std::endl;
                break;
            }else{
                if(std::abs(errorT-previousError)<0.0001 && m!=0){
                    noChangeinError++;
                }else{
                    noChangeinError=0;
                }
            }
            previousError = errorT;
        }
        else{
            if(m==0){
                for(unsigned int s=0;s<kddistance.size();s++)
                    sumDist += kddistance[s];
                Eupper=sumDist;
                previousError = Eupper;
            }else{
                lastSumDist = sumDist;
                sumDist=0;
                for(unsigned int s=0;s<kddistance.size();s++)
                    sumDist += kddistance[s];
            }

            if(std::abs(sumDist-lastSumDist)<breakCriteria){
                std::cout<<"diff="<<std::abs(sumDist-lastSumDist)<<std::endl;
                std::cout<<"break at"<<m<<std::endl;
                break;
            }
            else{
                if(std::abs(std::abs(sumDist-lastSumDist)-previousError)<0.0001 && m!=0){
                    noChangeinError++;
                }else{
                    noChangeinError=0;
                }
            }
            previousError = std::abs(sumDist-lastSumDist);
        }
        //-------------------------------------------------------------------------------------------------

        totalMatchedPoint = kdcorresp.size();
    }
}

double IterativeClosestPoint::map(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void IterativeClosestPoint::mean(pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::VectorXf &p,
                                 bool correspOnly,const std::vector<int> &corresp){
    double x,y,z;
    x=y=z=0;
    if(correspOnly){
        for(unsigned int i=0;i<corresp.size();i++){
            x+=cloud.points[corresp[i]].x;
            y+=cloud.points[corresp[i]].y;
            z+=cloud.points[corresp[i]].z;
        }
        p(0)=x/corresp.size();
        p(1)=y/corresp.size();
        p(2)=z/corresp.size();
    }else{
        for(unsigned int i=0;i<cloud.points.size();i++){
            x+=cloud.points[i].x;
            y+=cloud.points[i].y;
            z+=cloud.points[i].z;
        }
        p(0)=x/cloud.points.size();
        p(1)=y/cloud.points.size();
        p(2)=z/cloud.points.size();
    }
}

double IterativeClosestPoint::median(std::vector<double> distance){
    double median;
    std::sort(distance.begin(),distance.end());
    if(distance.size()%2==0){
        median = (distance[distance.size()/2-1]+distance[distance.size()/2])/2;
    }else{
        median = distance[distance.size()/2];
    }
    return median;
}

void IterativeClosestPoint::rt2tr(Eigen::MatrixXf &R, Eigen::VectorXf &t, Eigen::Matrix4f &T){
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
}

Eigen::Matrix4f IterativeClosestPoint::getFinalTransformation(){
    return T;
}

void IterativeClosestPoint::mabs(Eigen::Matrix4f &T){
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            T(i,j)=std::abs(T(i,j));
}

void IterativeClosestPoint::addNoise(Eigen::Matrix4f &transformationMatrix, float maxR, float maxT){
    Eigen::Affine3f transformation;
    maxR=maxR*M_PI/180;
    maxT/=100;
    float r = (static_cast <float> (rand()) / (static_cast <float> (RAND_MAX)))*maxR-maxR/2;
    float t = (static_cast <float> (rand()) / (static_cast <float> (RAND_MAX)))*maxT-maxT/2;
    transformation = pcl::getTransformation(t,t,t,r,r,r);
    transformationMatrix*=transformation.matrix();
}
