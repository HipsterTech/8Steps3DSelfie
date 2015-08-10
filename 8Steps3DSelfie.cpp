// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

#include <iostream>
#include <mutex>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;


static pcl::Grabber* kinectGrabber;
static boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
static std::mutex _mtx;  
static bool saveCloud(false);
static unsigned int filesSaved = 0;

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
            std::stringstream stream;
            stream << "inputCloud" << filesSaved << ".pcd";
            std::string filename = stream.str();
            if (pcl::io::savePCDFile(filename, *cloud, true) == 0)
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
    if (event.getKeySym() == "space" && event.keyDown())
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
