#include "color_interesting_points_deformable_alg.h"

ColorInterestingPointsDeformableAlgorithm::ColorInterestingPointsDeformableAlgorithm
    (void)
{
	pthread_mutex_init(&this->access_, NULL);
}

ColorInterestingPointsDeformableAlgorithm::~ColorInterestingPointsDeformableAlgorithm
    (void)
{
	pthread_mutex_destroy(&this->access_);
}

void ColorInterestingPointsDeformableAlgorithm::config_update(Config & new_cfg,
							      uint32_t level)
{
	this->lock();

	// save the current configuration
	this->config_ = new_cfg;

	this->unlock();
}

// ColorInterestingPointsDeformableAlgorithm Public API

/*
  Algorithm to extract all the interest points of an image. This algorithm also draws
  all the interest points in the image publisher to obtain a visual feedback.
  This is the main function of this node
*/
void ColorInterestingPointsDeformableAlgorithm::
extract_interest_points(const sensor_msgs::Image::ConstPtr & rgb_msg,
			const sensor_msgs::PointCloud2::ConstPtr & points_msg,
			sensor_msgs::Image & img_publish,
			std::vector < interest_point > &v)
{

	/// OpenCV bridge for input and output images
	cv_bridge::CvImagePtr cv_ptr =
	    cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);
	cv_bridge::CvImagePtr cv_ptr_detected =
	    cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);

	/// Get the original Image
	cv::Mat imgOriginal = cv_ptr->image;

	/// from ROS msg to PLC XYZ
	pcl::PointCloud < pcl::PointXYZ > cloud;
	pcl::fromROSMsg(*points_msg, cloud);

	/// Crop the image to delete some rubbish that appear in the image borders. 
	// Crop from the Origin values to (the original size - the_same_origin_value - Offset)
	// Croping Values in Pixels
	int OriginCropX = 50;
	int OriginCropY = 50;
	int SizeX = imgOriginal.size().width - OriginCropX - OriginCropX;
	int SizeY = imgOriginal.size().height - OriginCropY - OriginCropY;
	//Show the region we crop, in the original image, so add offset size 
	rectangle(imgOriginal, cv::Point(OriginCropX, OriginCropY),
		  cv::Point(SizeX + OriginCropX, SizeY + OriginCropY),
		  cv::Scalar(0), 2, 8, 0);

	/// Known colors that are going to be detected
	/* HUE range --> Color
	   [0]: iLowH = 0
	   [1]: iHighH = 179
	   Stauration range --> like brightness, 0 is white
	   [2]: iLowS = 0 
	   [3]: iHighS =255
	   Value range --> like darkness, 0 always black
	   [4]: iLowV = 0
	   [5]: iHighV =255
	 */
	int MyColors[8][6] = { 158, 179, 80, 255, 0, 255,	//RedTapeHSV
		65, 100, 80, 255, 0, 155,						//GreenTapeHSV
		0, 20, 80, 255, 50, 150,						//BrownTapeHSV
		100, 130, 110, 255, 50, 200,					//BlueTapeHSV

		145, 157, 150, 255, 125, 255,					//FucsiaTapeHSV
		100, 115, 55, 155, 155, 255,					//CyanHSV
		0, 179, 0, 1, 235, 255,							//WhiteTapeHSV	BAD COLOR!! CHANGE IT!
		25, 40, 100, 255, 140, 255						//YellowTapeHSV
	};			

	/// Once the color is detected a similar color is drawn in the pub img to have some feedback 
	int MyColorsBGR[8][3] = { 0, 0, 255,		//Red
		0, 255, 0,								//Green
		128, 128, 128,							//Gray
		255, 0, 0,								//Blue

		114, 128, 250,							//Salmon
		225, 255, 0,							//Cyan
		255, 0, 255,							//Purple
		0, 255, 255								//Yellow
	};			      

	/// Converting image from BRG to HSV for the color detection 
	cv::Mat imgHSV;
	cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

	/// Trying to find Centroide for each color
	// var to store Centroides
	std::vector < cv::Point > List_Centroids(8);
	int i = 0;
	while (i < 8) {
		compute_color_centroid(imgHSV, OriginCropX, OriginCropY,	// Image and cropping
				       MyColors[i],									// HSV values
				       List_Centroids[i]);							// Return values
		i++;
	}

	/// Find Centroid 3d Position
	i = 0;
	while (i < 8) {
		if (List_Centroids[i].x > 0 && List_Centroids[i].y > 0)
			obtain_3D(cloud, List_Centroids[i], v, i);
		i++;
	}

	/// DRAWING STUFF
	// Drawing a circle in the centroids grasping position U V, BGR
	i = 0;
	while (i < v.size()) {
		cv::circle(imgOriginal, cv::Point(v[i].U, v[i].V), 10,
			   cv::Scalar(MyColorsBGR[v[i].id_color][0],
				      MyColorsBGR[v[i].id_color][1],
				      MyColorsBGR[v[i].id_color][2]), 2, 8, 0);
		// Drawing number of color
		char text[10];
		std::sprintf(text, " #:%d", v[i].id_color);
		cv::putText(imgOriginal, text, cv::Point(v[i].U, v[i].V + 10),
			    cv::FONT_HERSHEY_SIMPLEX, 0.75,
			    cv::Scalar(MyColorsBGR[v[i].id_color][0],
				       MyColorsBGR[v[i].id_color][1],
				       MyColorsBGR[v[i].id_color][2]), 2, 8, 0);

		i++;
	}

	/// From OpenCV to ROS Image msg. img_publish is the image message to publish
	cv_ptr_detected->image = imgOriginal;
	copy_image(cv_ptr_detected->toImageMsg(), img_publish);

}

/* 
  Algorithm that coputes the centroid of a color in HSV range  
*/
void ColorInterestingPointsDeformableAlgorithm::
compute_color_centroid(const cv::Mat & imgHSV, int OriginCropX, int OriginCropY,
		       int RangeHSV[6], cv::Point & Centroid)
{

	/// Threshold the image
	cv::Mat imgDetect;		//B&W image where the color detected is white
	cv::Mat imgCropped;		//B&W image, color detected and cropped
	cv::Mat imgThresholded;	//B&W image where the centroids are computed
	//                   (iLowH, iLowS, iLowV), (iHighH, iHighS, iHighV) 
	cv::inRange(imgHSV, cv::Scalar(RangeHSV[0], RangeHSV[2], RangeHSV[4]),
		    cv::Scalar(RangeHSV[1], RangeHSV[3], RangeHSV[5]),
		    imgDetect);

	/// Crop the image, from the Origin to (the original size - the same origin value - Offset)
	int SizeX = imgDetect.size().width - OriginCropX - OriginCropX;
	int SizeY = imgDetect.size().height - OriginCropY - OriginCropY;
						//originX, originY  width, height
	cv::Rect extractROI(OriginCropX, OriginCropY, SizeX, SizeY);	
	cv::Mat(imgDetect, extractROI).copyTo(imgCropped);

	/// morphological opening (remove small objects from the foreground)
	cv::erode(imgCropped, imgThresholded,
		  getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::dilate(imgThresholded, imgThresholded,
		   getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	/// morphological closing (fill small holes in the foreground)
	cv::dilate(imgThresholded, imgThresholded,
		   getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::erode(imgThresholded, imgThresholded,
		  getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	/// Calc moments of the image to find centroid. 
	cv::Moments ObjMoments = cv::moments(imgThresholded);
	//Position of the cropped image, so offset added
	Centroid.x = (int)(ObjMoments.m10 / ObjMoments.m00 + OriginCropX);	//U 
	Centroid.y = (int)(ObjMoments.m01 / ObjMoments.m00 + OriginCropY);	//V

}

/* 
  Algorithm that uses the PointCloud2 to obtain the Z axis (3D) form a list of 2D positions
*/
void ColorInterestingPointsDeformableAlgorithm::
obtain_3D(const pcl::PointCloud < pcl::PointXYZ > &cloud,
	  const cv::Point & Centroid, std::vector < interest_point > &v,
	  const int color)
{

	//We need the 3d coordinates, so we're going to use pcl::PointXYZRGB points
	if (Centroid.x > 0 && Centroid.y > 0) {

		interest_point IPoint;
		IPoint.U = Centroid.x;
		IPoint.V = Centroid.y;
		IPoint.X = cloud(Centroid.x, Centroid.y).x;
		IPoint.Y = cloud(Centroid.x, Centroid.y).y;
		IPoint.Z = cloud(Centroid.x, Centroid.y).z;
		IPoint.type = 'C';	//Centroid
		IPoint.id_color = color;

		/// Insert IP in the vector 
		v.push_back(IPoint);

	}

}

/* 
  Algorithm that copies an image from a subscriber message format to a publisher message format
*/
void ColorInterestingPointsDeformableAlgorithm::
copy_image(const sensor_msgs::Image::ConstPtr & msg,
	   sensor_msgs::Image & cp_img)
{

	cp_img.data = msg->data;
	cp_img.height = msg->height;
	cp_img.width = msg->width;
	cp_img.encoding = msg->encoding;
	cp_img.is_bigendian = msg->is_bigendian;
	cp_img.step = msg->step;
	cp_img.header = msg->header;

}
