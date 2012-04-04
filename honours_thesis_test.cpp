#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#ifdef _WIN32
  #define sleep(x) Sleep((x)*1000)
#endif

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer( "Mats Thesis" ) {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      double kinect_capture_time; 
      double kinect_capture_rate; 
      double noise_reduction_time;         

      // Input timing to see how quickly (slowly) we are getting data from the Kinect. 

      static double start_kinect_capture = pcl::getTime(); 

      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30)
      {
        double now = pcl::getTime ();
        //std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        kinect_capture_rate = double(count)/double(now - last); 
        std::cout << "Kinect Data Capture:\t" << (double)( now - start_kinect_capture ) << std::endl; 
        count = 0;
        last = now;
      }


      static double start_noise_reduction = pcl::getTime(); 


      double end_noise_reduction = pcl::getTime(); 
      std::cout << "Noise Reduction:\t" << (double)( end_noise_reduction - start_noise_reduction ) << std::endl; 

      

      if (!viewer.wasStopped())
      {
        viewer.setBackgroundColor (1, 1, 1);
        viewer.removePointCloud( "cloud", 0 );
        viewer.addPointCloud( cloud, "cloud", 0 ); 
        

        //viewer.showCloud(cloud);
        
        static unsigned count = 0;
        std::stringstream ss;
        ss << "Kinect Capture Rate:\t" << 5 << "Hz"; 
        viewer.removeShape ("text", 0);
        viewer.addText (ss.str(), 200, 300, "text", 0);

        viewer.spinOnce(); 
      }

      start_kinect_capture = 0; 
      start_noise_reduction = 0; 
    }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

         // Set up viewer :) 
         viewer.setBackgroundColor (0, 0, 0);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::PCLVisualizer viewer; 
     
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }