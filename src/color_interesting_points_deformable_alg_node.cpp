#include "color_interesting_points_deformable_alg_node.h"

ColorInterestingPointsDeformableAlgNode::ColorInterestingPointsDeformableAlgNode
    (void):algorithm_base::IriBaseAlgorithm <
    ColorInterestingPointsDeformableAlgorithm > ()
{
	//init class attributes if necessary
	this->ProcessingImgs = false;

	//this->loop_rate_ = 2;//in [Hz]

	// [init publishers]
	this->interest_points_image_publisher_ =
	    this->public_node_handle_.advertise < sensor_msgs::Image >
	    ("interest_points_image", 1);

	// [init subscribers]
	this->point_cloud_subscriber_ =
	    this->public_node_handle_.subscribe("point_cloud", 1,
						&ColorInterestingPointsDeformableAlgNode::point_cloud_callback,
						this);
	pthread_mutex_init(&this->point_cloud_mutex_, NULL);

	this->img_rgb_subscriber_ =
	    this->public_node_handle_.subscribe("img_rgb", 1,
						&ColorInterestingPointsDeformableAlgNode::img_rgb_callback,
						this);
	pthread_mutex_init(&this->img_rgb_mutex_, NULL);

	// [init services]
	this->interest_points_server_ =
	    this->public_node_handle_.advertiseService("interest_points",
						       &ColorInterestingPointsDeformableAlgNode::interest_pointsCallback,
						       this);
	pthread_mutex_init(&this->interest_points_mutex_, NULL);

	// [init clients]

	// [init action servers]

	// [init action clients]
}

ColorInterestingPointsDeformableAlgNode::~ColorInterestingPointsDeformableAlgNode
    (void)
{
	// [free dynamic memory]
	pthread_mutex_destroy(&this->interest_points_mutex_);
	pthread_mutex_destroy(&this->point_cloud_mutex_);
	pthread_mutex_destroy(&this->img_rgb_mutex_);
}

void ColorInterestingPointsDeformableAlgNode::mainNodeThread(void)
{
	// [fill msg structures]
	//this->interest_points_image_Image_msg_.data = my_var;

	// [fill srv structure and make request to the server]

	// [fill action structure and make request to the action server]

	// [publish messages]
	this->interest_points_image_publisher_.
	    publish(this->interest_points_image_Image_msg_);
}

/*  [subscriber callbacks] */
void ColorInterestingPointsDeformableAlgNode::
point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
	//ROS_INFO("ColorInterestingPointsDeformableAlgNode::point_cloud_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	this->alg_.lock();
	this->point_cloud_mutex_enter();

	if (!ProcessingImgs)
		Img_Points_Msg = msg;

	//unlock previously blocked shared variables 
	this->alg_.unlock();
	this->point_cloud_mutex_exit();
}

void ColorInterestingPointsDeformableAlgNode::point_cloud_mutex_enter(void)
{
	pthread_mutex_lock(&this->point_cloud_mutex_);
}

void ColorInterestingPointsDeformableAlgNode::point_cloud_mutex_exit(void)
{
	pthread_mutex_unlock(&this->point_cloud_mutex_);
}

void ColorInterestingPointsDeformableAlgNode::
img_rgb_callback(const sensor_msgs::Image::ConstPtr & msg)
{
	//ROS_INFO("ColorInterestingPointsDeformableAlgNode::img_rgb_callback: New Message Received");

	//use appropiate mutex to shared variables if necessary 
	this->alg_.lock();
	this->img_rgb_mutex_enter();

	if (!ProcessingImgs)
		Img_Color_Msg = msg;

	//unlock previously blocked shared variables 
	this->alg_.unlock();
	this->img_rgb_mutex_exit();
}

void ColorInterestingPointsDeformableAlgNode::img_rgb_mutex_enter(void)
{
	pthread_mutex_lock(&this->img_rgb_mutex_);
}

void ColorInterestingPointsDeformableAlgNode::img_rgb_mutex_exit(void)
{
	pthread_mutex_unlock(&this->img_rgb_mutex_);
}

/*  [service callbacks] */
bool ColorInterestingPointsDeformableAlgNode::interest_pointsCallback
    (iri_color_interesting_points_deformable::InterestPoints::Request & req,
     iri_color_interesting_points_deformable::InterestPoints::Response & res)
{
	ROS_INFO
	    ("ColorInterestingPointsDeformableAlgNode::interest_pointsCallback: New Request Received!");

	//use appropiate mutex to shared variables if necessary 
	this->alg_.lock();
	this->interest_points_mutex_enter();

	ROS_INFO
	    ("ColorInterestingPointsDeformableAlgNode::interest_pointsCallback: Processing New Request!");

	if (this->Img_Color_Msg != NULL && this->Img_Points_Msg != NULL
	    && !ProcessingImgs) {

		//Init
		ProcessingImgs = true;
		std::vector <
		    ColorInterestingPointsDeformableAlgorithm::interest_point >
		    v;

		//Obtain interest points
		this->alg_.extract_interest_points(Img_Color_Msg,
						   Img_Points_Msg,
						   this->interest_points_image_Image_msg_,
						   v);

		res.amount = v.size();
		res.Color.resize(res.amount);
		res.U.resize(res.amount);
		res.V.resize(res.amount);
		res.X.resize(res.amount);
		res.Y.resize(res.amount);
		res.Z.resize(res.amount);
		res.object_type.resize(res.amount);

		//Fill return variables
		int i = 0;
		while (i < res.amount) {
			res.U[i] = v[i].U;
			res.V[i] = v[i].V;
			res.X[i] = v[i].X;
			res.Y[i] = v[i].Y;
			res.Z[i] = v[i].Z;
			res.Color[i] = v[i].id_color;
			res.object_type[i] = v[i].type;
			i++;
		}

		//unlock previously blocked shared variables 
		this->interest_points_mutex_exit();
		this->alg_.unlock();
		ProcessingImgs = false;
		return true;

	}
	else {
		//unlock previously blocked shared variables 
		this->interest_points_mutex_exit();
		this->alg_.unlock();
		return false;
	}

}

void ColorInterestingPointsDeformableAlgNode::interest_points_mutex_enter(void)
{
	pthread_mutex_lock(&this->interest_points_mutex_);
}

void ColorInterestingPointsDeformableAlgNode::interest_points_mutex_exit(void)
{
	pthread_mutex_unlock(&this->interest_points_mutex_);
}

/*  [action callbacks] */

/*  [action requests] */

void ColorInterestingPointsDeformableAlgNode::
node_config_update(Config & config, uint32_t level)
{
	this->alg_.lock();

	this->alg_.unlock();
}

void ColorInterestingPointsDeformableAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
	return algorithm_base::main < ColorInterestingPointsDeformableAlgNode >
	    (argc, argv, "color_interesting_points_deformable_alg_node");
}
