#include "closest.h"

Closest::Closest()
{
}

void Closest::search(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, std::vector<int> &J, std::vector<double> &distance){

    std::vector< std::vector<double> > e(target.points.size(),std::vector<double>(source.points.size()));
    double sourceX,sourceY,sourceZ,targetX,targetY,targetZ;

    double min_element_in_col;
    int min_element_indice;

    for(int i=0;i<target.points.size();i++){
        sourceX=target.points[i].x;
        sourceY=target.points[i].y;
        sourceZ=target.points[i].z;
        for(int j=0;j<source.points.size();j++){
            targetX=source.points[j].x;
            targetY=source.points[j].y;
            targetZ=source.points[j].z;
            e[i][j]=pow((targetX-sourceX),2)+pow((targetY-sourceY),2)+pow((targetZ-sourceZ),2);
        }
    }

    // e.size() = number of rows
    // e.[0].size() = number of columns

    for(int col=0;col<e[0].size();col++){
        min_element(min_element_in_col,min_element_indice,e,col);
        J[col] = min_element_indice;
        distance[col] = min_element_in_col;
    }
}

void Closest::min_element(double &element, int &indice, std::vector< std::vector<double> > &e, int col){
    element=e[0][col];
    indice=0;
    for(int i=1;i<e.size();i++){
        if(e[i][col]<element){
            element=e[i][col];
            indice=i;
        }
    }
}
