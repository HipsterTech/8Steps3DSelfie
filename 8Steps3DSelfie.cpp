// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

#include <iostream>
#include <mutex>
#include <math.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/transforms.h>

using std::cout;
using std::endl;


static pcl::Grabber* kinectGrabber;
static boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
static std::mutex _mtx;  
static bool saveCloud(false);
static unsigned int filesSaved = 0;
static Eigen::Matrix4f transform [8];
static double d = 1.2;
static double v = sqrt(2)/2;

// 

void 
populateTransformationMatrices()
{
    Eigen::Matrix4f m0;
    m0=Eigen::Matrix4f::Identity ();
    transform[0]=m0;

    std::cout << " Created m0= "<<std::endl << m0 << std::endl;

    Eigen::Matrix4f m1;
    m1 << v,0,-v,d,
          0,1,0,0,
          v,0,v,0,
          0,0,0,1;
    transform[1]=m1;

    std::cout << " Created m1= "<<std::endl << m1 << std::endl;

    Eigen::Matrix4f m2;
    m2 << 0,0,-1,d,
          0,1,0,0,
          -1,0,0,d,
          0,0,0,1;
    transform[2]=m2;

    std::cout << " Created m2= "<<std::endl << m2 << std::endl;

    Eigen::Matrix4f m3;
    m3 << -v,0,-v,d,
          0,1,0,0,
          v,0,-v,2*d,
          0,0,0,1;
    transform[3]=m3;

    std::cout << " Created m3= "<<std::endl << m3 << std::endl;

    Eigen::Matrix4f m4;
    m4 << -1,0,0,0,
          0,1,0,0,
          0,0,-1,2*d,
          0,0,0,1;
    transform[4]=m4;

    std::cout << " Created m4= "<<std::endl << m4 << std::endl;

    Eigen::Matrix4f m5;
    m5 << -v,0,v,-d,
          0,1,0,0,
          -v,0,-v,2*d,
          0,0,0,1;
    transform[5]=m5;

    std::cout << " Created m5= "<<std::endl << m5 << std::endl;

    Eigen::Matrix4f m6;
    m6 << 0,0,1,-d,
          0,1,0,0,
          -1,0,0,d,
          0,0,0,1;
    transform[6]=m6;

    std::cout << " Created m6= "<<std::endl << m6 << std::endl;

    Eigen::Matrix4f m7;
    m7 << v,0,v,-d,
          0,1,0,0,
          -v,0,v,0,
          0,0,0,1;
    transform[7]=m7;

    std::cout << " Created m7= "<<std::endl << m7 << std::endl;



}



//Called every time there's a new frame 
void
grabber_callback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{        
    static bool first = true;
    if(first)
    {
        _mtx.lock();
        _viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "samplecloud");
        _mtx.unlock();
        //_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        first = false;
        return;
    }
    else
    {
        _mtx.lock();
        _viewer->updatePointCloud(cloud,"samplecloud");
        if (saveCloud)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    
            Eigen::Matrix4f m=transform[filesSaved];
            pcl::transformPointCloud (*cloud, *transformed_cloud, m);
            std::stringstream stream;
            //mac
            stream << "inputCloud" << filesSaved << ".pcd";
            //linux
            //stream << "inputCloud" << filesSaved << ".pcd";
            std::string filename = stream.str();
            if (pcl::io::savePCDFile(filename, *transformed_cloud, true) == 0)
            {
                filesSaved++;
                cout << "Saved " << filename << "." << endl;
            }
            else PCL_ERROR("Problem saving %s.\n", filename.c_str());     
            saveCloud = false;
        }
        _mtx.unlock();    
    }

}

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "Return" && event.keyDown())
    {
        saveCloud = true;
    }
    else if (event.getKeySym() == "r" && event.keyDown())
    {
        _viewer->setCameraPosition(0,0,-1,0,0,1,0,-1,0); 
        std::cout << "Camera viewpoint reset" << std::endl;
    } 
    // else if(event.keyDown())
    // {
    //     std::cout << "key pressed: " <<event.getKeySym() << std::endl;
    // }
    

}

//Creates and configures the viewer, returning a shared pointer to it
boost::shared_ptr<pcl::visualization::PCLVisualizer>
create_viewer()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.21, 0.27, 0.22);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0,0,-1,0,0,1,0,-1,0); 
    viewer->registerKeyboardCallback(keyboardEventOccurred);
    return (viewer);    
} 



int
main(int argc, char** argv)
{

    //Set up viewer
    _viewer = create_viewer();
    
    //populate the matrices
    populateTransformationMatrices();

    //Set up the grabber and register the callback function
    kinectGrabber = new pcl::OpenNIGrabber();
    if (kinectGrabber == 0) return false;
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&grabber_callback, _1);
    kinectGrabber->registerCallback(f);

    //Start the loop
    kinectGrabber->start();
    while (!_viewer->wasStopped ())
    {
        _mtx.lock();
        _viewer->spinOnce (100);
        _mtx.unlock();
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
        
    kinectGrabber->stop();
}
