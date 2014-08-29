// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _color_interesting_points_deformable_alg_h_
#define _color_interesting_points_deformable_alg_h_

#include <iri_color_interesting_points_deformable/ColorInterestingPointsDeformableConfig.h>

//include color_interesting_points_deformable_alg main library

#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class ColorInterestingPointsDeformableAlgorithm {
 protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the ColorInterestingPointsDeformableConfig. All driver implementations
    * will then use the same variable type Config.
    */
	pthread_mutex_t access_;

	// private attributes and methods

 public:
   /**
    * \brief define config type
    *
    * Define a Config type with the ColorInterestingPointsDeformableConfig. All driver implementations
    * will then use the same variable type Config.
    */
	typedef iri_color_interesting_points_deformable::
	    ColorInterestingPointsDeformableConfig Config;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
	Config config_;

   /**
    * \brief constructor
    *
    * In this constructor parameters related to the specific driver can be
    * initalized. Those parameters can be also set in the openDriver() function.
    * Attributes from the main node driver class IriBaseDriver such as loop_rate,
    * may be also overload here.
    */
	 ColorInterestingPointsDeformableAlgorithm(void);

   /**
    * \brief Lock Algorithm
    *
    * Locks access to the Algorithm class
    */
	void lock(void) {
		pthread_mutex_lock(&this->access_);
	};

   /**
    * \brief Unlock Algorithm
    *
    * Unlocks access to the Algorithm class
    */
	void unlock(void) {
		pthread_mutex_unlock(&this->access_);
	};

   /**
    * \brief Tries Access to Algorithm
    *
    * Tries access to Algorithm
    * 
    * \return true if the lock was adquired, false otherwise
    */
	bool try_enter(void) {
		if (pthread_mutex_trylock(&this->access_) == 0)
			return true;
		else
			return false;
	};

   /**
    * \brief config update
    *
    * In this function the driver parameters must be updated with the input
    * config variable. Then the new configuration state will be stored in the 
    * Config attribute.
    *
    * \param new_cfg the new driver configuration state
    *
    * \param level level in which the update is taken place
    */
	void config_update(Config & new_cfg, uint32_t level = 0);

	// here define all color_interesting_points_deformable_alg interface methods to retrieve and set
	// the driver parameters

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
	~ColorInterestingPointsDeformableAlgorithm(void);

   /** My Vars **/

	struct interest_point {
		int id_color;
		int U, V;
		float X, Y, Z;
		char type;
	};

   /** My Functions **/

	void extract_interest_points(const sensor_msgs::
				     Image::ConstPtr & rgb_msg,
				     const sensor_msgs::
				     PointCloud2::ConstPtr & points_msg,
				     sensor_msgs::Image & cp_im,
				     std::vector < interest_point > &v);
	void process_image(const sensor_msgs::Image::ConstPtr & msg,
			   sensor_msgs::Image & cp_img);
	void compute_color_centroid(const cv::Mat & imgOriginal,
				    int OriginCropX, int OriginCropY,
				    int RangeHSV[6], cv::Point & Centroid);
	void copy_image(const sensor_msgs::Image::ConstPtr & msg,
			sensor_msgs::Image & cp_img);
	void obtain_3D(const pcl::PointCloud < pcl::PointXYZ > &cloud,
		       const cv::Point & Centroid,
		       std::vector < interest_point > &v, const int color);

};

#endif
