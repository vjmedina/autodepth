#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

//#include <process.h>//To be able to call external programs from the code. 

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <flann/flann.h>

#include <vector>


void wait()
{
    cout<<"Press ENTER to continue....."<<endl<<endl;
    cin.ignore(1);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  
  viewer->setBackgroundColor (0, 0, 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);  
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
  return (viewer);
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> fitToPlanes()
{
	sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	float depth_levels[20];
	std::vector<pcl::ModelCoefficients> depth_planes;

  	
	// Fill in the cloud data
	pcl::PCDReader reader;
	//reader.read ("table_scene_lms400.pcd", *cloud_blob);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud (new pcl::PointCloud<pcl::PointXYZ>); 
	//pcl::io::loadPLYFile("apple.ply",*cloud_blob);
	pcl::io::loadPLYFile("kermit.ply",*cloud_blob);	

	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// Write the downsampled version to disk
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	uint8_t r(255), g(15), b(15);
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	
	
	//viewer = customColourVis(cloud_p);

	int i = 0, nr_points = (int) cloud_filtered->points.size ();
	// While 30% of the original cloud is still there
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{		
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		depth_levels[i]= coefficients->values[3];
		depth_planes.push_back(*coefficients);

		// Extract the inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		std::stringstream ss;
		ss << "table_scene_lms400_plane_" << i << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);

		//viewer->addPlane (*coefficients, "plane"+i);
						
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_p, (rand()%255)+1, (rand()%255)+1, (rand()%255)+1);  
		ss << "cloud" << i;
		viewer->addPointCloud<pcl::PointXYZ> (cloud_p, single_color, ss.str());
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
				
		//wait();
	
		i++;		
	}
		
	
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (0.1);
	//viewer = customColourVis(cloud_p);
	
	pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
	coeffs->values.resize (4);

	std::stringstream plane_name;
	int j=0;
	
	for (std::vector<pcl::ModelCoefficients>::iterator i = depth_planes.begin(); i != depth_planes.end(); ++i)
	{
		plane_name << "plane" << j;
		coeffs->values[0] = 0;
		coeffs->values[1] = 0;
		coeffs->values[2] = 1;
		coeffs->values[3] = i->values[3];
		
		viewer->addPlane (*coeffs, plane_name.str());
		//viewer->addPlane (*i, plane_name.str());
		j++;
	}
	/*
	for (j=0; j<i; j++)
	{		
		plane_name << "plane" << j;
		
		coeffs->values[0] = 0;
		coeffs->values[1] = 0;
		coeffs->values[2] = 1;
		coeffs->values[3] = depth_levels[j];
		
		viewer->addPlane (*coeffs, plane_name.str());
	}*/
	
	/*	
	coeffs->values[0] = 0;
	coeffs->values[1] = 1;
	coeffs->values[2] = 0;
	coeffs->values[3] = 0.01;
	viewer->addPlane (*coeffs, "testplane");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,0,"testplane"); 
	*/

	return viewer;
}

int main (int argc, char** argv)
{
	//system("test.exe"); Run the generation of ply file.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = fitToPlanes();

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

  /*
	while (!viewer.wasStopped (30)) 
	{ 

	}

	while (!viewer->wasStopped ())
	  {
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
	*/

  return (0);
}


/*
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud (new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::io::loadPLYFile("apple.ply",*pcloud);

  std::cerr << "Point cloud data: " << pcloud->points.size () << " points" << std::endl;
  for (size_t i = 0; i < pcloud->points.size (); ++i)
    std::cerr << "    " << pcloud->points[i].x << " " 
                        << pcloud->points[i].y << " " 
                        << pcloud->points[i].z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (pcloud->makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << pcloud->points[inliers->indices[i]].x << " "
                                               << pcloud->points[inliers->indices[i]].y << " "
                                               << pcloud->points[inliers->indices[i]].z << std::endl;

  return (0);
}
*/

/*
int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud (new pcl::PointCloud<pcl::PointXYZ>); 
	
	pcl::io::loadPLYFile("test.ply",*pcloud);
	
	pcl::visualization::CloudViewer viewer("Cloud Viewer"); 
	
	viewer.showCloud(pcloud); 
	
	while (!viewer.wasStopped (30)) 
	{ 

	} 

	return (0);
}
*/