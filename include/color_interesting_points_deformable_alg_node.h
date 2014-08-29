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

#ifndef _color_interesting_points_deformable_alg_node_h_
#define _color_interesting_points_deformable_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "color_interesting_points_deformable_alg.h"

// [publisher subscriber headers]
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

// [service client headers]
#include <iri_color_interesting_points_deformable/InterestPoints.h>

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class ColorInterestingPointsDeformableAlgNode:public
    algorithm_base::IriBaseAlgorithm <
    ColorInterestingPointsDeformableAlgorithm > {
 private:
	// [publisher attributes]
	ros::Publisher interest_points_image_publisher_;
	sensor_msgs::Image interest_points_image_Image_msg_;

	// [subscriber attributes]
	ros::Subscriber point_cloud_subscriber_;
	void point_cloud_callback(const sensor_msgs::
				  PointCloud2::ConstPtr & msg);
	pthread_mutex_t point_cloud_mutex_;
	void point_cloud_mutex_enter(void);
	void point_cloud_mutex_exit(void);
	ros::Subscriber img_rgb_subscriber_;
	void img_rgb_callback(const sensor_msgs::Image::ConstPtr & msg);
	pthread_mutex_t img_rgb_mutex_;
	void img_rgb_mutex_enter(void);
	void img_rgb_mutex_exit(void);

	// [service attributes]
	ros::ServiceServer interest_points_server_;
	bool interest_pointsCallback
	    (iri_color_interesting_points_deformable::InterestPoints::
	     Request & req,
	     iri_color_interesting_points_deformable::InterestPoints::
	     Response & res);
	pthread_mutex_t interest_points_mutex_;
	void interest_points_mutex_enter(void);
	void interest_points_mutex_exit(void);

	// [client attributes]

	// [action server attributes]

	// [action client attributes]

	// MY CLASS VARS
	sensor_msgs::Image::ConstPtr Img_Color_Msg;
	sensor_msgs::PointCloud2::ConstPtr Img_Points_Msg;

	bool ProcessingImgs;

 public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
	ColorInterestingPointsDeformableAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
	~ColorInterestingPointsDeformableAlgNode(void);

 protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
	void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
	void node_config_update(Config & config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
	void addNodeDiagnostics(void);

	// [diagnostic functions]

	// [test functions]
};

#endif
