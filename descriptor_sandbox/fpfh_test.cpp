/* Standard Includes*/
#include <fstream>
#include <iostream>
#include <set>
#include <stdio.h>
#include <string>
#include <vector>

/* Boost Includes */
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
//#include <boost/timer/timer.hpp>

/* PCL Includes */
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/visualization/histogram_visualizer.h>

namespace fs = boost::filesystem;

class Trainer {
    /* Handles a training instance for one class. This class will
     * parse through a directory contianing labels and datasets for a
     * class. FPFH features will be generated for each class, exported to
     * a text file, and then a classifier will be trainined on the generated
     * features
     */
};

class Class_Trainer {
private:
    /* TODO: Add support for PCL visualizer*/
    std::string pclass;
    std::string feature_type;
    std::vector<std::string> pcd_directories;
    pcl::visualization::PCLHistogramVisualizer* pcl_hv;

    /* clang-format off */
    std::vector<pcl::PointCloud<pcl::PointXYZ> > parsed_clouds;
    std::vector<pcl::PointCloud<pcl::FPFHSignature33> > pcd_features;
    /* clang format on */

public:
    Class_Trainer(std::string pclass, std::string feature_type, fs::path working_directory);
    void parse_directory(fs::path dp);
    void parse_pcd();
    void run_pipeline(bool output = false);
};

Class_Trainer::Class_Trainer(std::string pclass, std::string feature_type, fs::path working_directory) {
	this->pclass = pclass;
	this->feature_type = feature_type;	
}

void Class_Trainer::parse_directory(fs::path dp){
    fs::directory_iterator it(dp), eod;
    BOOST_FOREACH (const fs::path &dp, std::make_pair(it, eod)){
	if(fs::is_regular_file(dp) && (fs::extension(dp) == ".pcd")){
	    this->pcd_directories.push_back(fs::canonical(dp).string());
	}
    }
}

void Class_Trainer::parse_pcd(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    BOOST_FOREACH(std::string pcd_directory, this->pcd_directories){ 
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_directory, *cloud_ptr) == -1) {
	    PCL_ERROR("Couldn't read file \n");
        }
	else{
	    this->parsed_clouds.push_back(*cloud_ptr);
	}	
    }
    std::cout << "Completed parsing provided PCD directory" << std::endl;
    std::cout << "   Total Number of Clouds: " << this->parsed_clouds.size() << std::endl;    
    std::cout << "   Amount of Memory Consumed: " << sizeof(this->parsed_clouds) << " MB"<< std::endl;
}

void Class_Trainer::run_pipeline(bool output){
    std::cout << "Running feature pipeline"<< std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fh;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    fh.setRadiusSearch(0.1);	/* Might have to be tuned; Our cloud is relatively sparse */
    
    BOOST_FOREACH(pcl::PointCloud<pcl::PointXYZ> cloud, this->parsed_clouds){
	/* TODO: Place detected features in map with PCD filename */
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
	ne.setInputCloud(cloud.makeShared());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.05);
	ne.compute(*normals);

	fh.setInputCloud(cloud.makeShared());
	fh.setInputNormals(normals);
	fh.compute(*fpfhs);

    	this->pcd_features.push_back(*fpfhs);
    }
    std::cout << "Finished generating features" << std::endl;
    std::cout << "   Generated " << this->pcd_features.size() << " features"<< std::endl;
}

int main(int argc, char **argv) {
    fs::path f = fs::current_path();
    f /= "training/";
    
    /* This will all be managed by a Trainer Manager instance */
    Class_Trainer* c_;
    c_ = new Class_Trainer("buoy", "fpfh", fs::current_path());
    c_->parse_directory(f);
    c_->parse_pcd();
    c_->run_pipeline();
}
