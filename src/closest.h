#ifndef CLOSEST_H
#define CLOSEST_H

#include <pcl/io/io.h>
#include <math.h>

class Closest
{
public:
    Closest();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt;
    void search(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, std::vector<int> &J, std::vector<double> &distance);
    void min_element(double &element, int &indice, std::vector< std::vector<double> > &e, int col);
    
};

#endif // CLOSEST_H
