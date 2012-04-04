#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

// SEGMENTATION INCLUDES
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

// Filtering Stuff
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#ifdef _WIN32
  #define sleep(x) Sleep((x)*1000)
#endif

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("Mats Thesis") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
        pcl::PointCloud< pcl::PointXYZRGBA > point_cloud; 
        pcl::PointCloud< pcl::PointXYZRGBA > temp_point_cloud; 
        pcl::PointCloud< pcl::PointXYZRGBA > output_point_cloud;

        pcl::ExtractIndices< pcl::PointXYZRGBA > extract;

        double kinect_capture_time; 
        double kinect_capture_rate; 
        double noise_reduction_time;         

        // Input timing to see how quickly (slowly) we are getting data from the Kinect. 
        
        double start_kinect_capture = pcl::getTime(); 

        point_cloud = *cloud; 

        static unsigned count = 0;
        static double last = pcl::getTime ();
       

        double end_kinect_capture = pcl::getTime(); 

         std::cout << "Kinect Data Capture:\t" << ( end_kinect_capture - start_kinect_capture ) <<std::endl; 

        // ---------- SIZE REDUCTION ----------
        double start_size_reduction = pcl::getTime(); 

        //std::cerr << "PointCloud Size:\t\t" << point_cloud.points.size( ) << std::endl;

        pcl::VoxelGrid< pcl::PointXYZRGBA > sor; 
        sor.setInputCloud( point_cloud.makeShared() ); 
        //sor.setLeafSize( 0.01f, 0.01f, 0.01f ); 
        sor.setLeafSize( 0.005f, 0.005f, 0.005f ); 
        sor.filter( temp_point_cloud ); 
        
        point_cloud = temp_point_cloud; 

        //std::cerr << "PointCloud Size:\t\t" << point_cloud.points.size( ) << std::endl;

        double end_size_reduction = pcl::getTime(); 
        std::cout << "Size Reduction:\t\t\t" << ( end_size_reduction - start_size_reduction ) << std::endl;
        // ---------- END SIZE REDUCTION ----------

        // ---------- NOISE REDUCTION ----------
        double start_noise_reduction = pcl::getTime(); 
        double end_noise_reduction = pcl::getTime(); 
        std::cout << "Noise Reduction:\t\t" << (end_noise_reduction - start_noise_reduction ) << std::endl;
        // ---------- END NOISE REDUCTION ----------

        // ---------- SEGEMENTATION CODE ----------
        double start_segmentation = pcl::getTime(); 

        temp_point_cloud.points.clear(); 

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices );
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
        seg.setModelType(pcl::SACMODEL_SPHERE );
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud ( point_cloud.makeShared ());
        seg.segment( *inliers, *coefficients);

        // remove found cloud and modify it to stick out :)
        extract.setInputCloud( point_cloud.makeShared( ) );
        extract.setIndices( inliers );
        extract.setNegative( false );
        extract.filter( temp_point_cloud );

        int color = rand( ) * 10000;
        for( int i = 0; i <= temp_point_cloud.points.size( ); i++ )
        {
            temp_point_cloud.points[ i ].rgba = color;
        }

        // remove the found cloud permanately.
        extract.setInputCloud( point_cloud.makeShared( ) );
        extract.setIndices( inliers  );
        extract.setNegative( false );
        extract.filter( temp_point_cloud );

        // put object into our output cloud. 
        output_point_cloud.header.frame_id = temp_point_cloud.header.frame_id; 
        output_point_cloud += temp_point_cloud; 

        double end_segmentation = pcl::getTime();         
        std::cout << "Environment Segmentation:\t" << (double)( end_segmentation - start_segmentation ) << std::endl;
        // ---------- END SEGEMENTATION CODE ----------

        // once all objects have been found put remaining points into the cloud. 
        output_point_cloud += point_cloud;

         if (++count == 30)
        {
          double now = pcl::getTime ();
          std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
         count = 0;
          last = now;
        }

        if (!viewer.wasStopped())
        {
         viewer.showCloud( output_point_cloud.makeShared() );
        }

        start_kinect_capture = 0; 
        start_size_reduction = 0; 
        start_noise_reduction = 0; 
        start_segmentation = 0; 
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;

     
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }