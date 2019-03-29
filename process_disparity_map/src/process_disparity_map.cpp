#include "process_disparity_map/process_disparity_map.h"

DisparityProc::DisparityProc() : it_(nh_)
{
  // Subscribers
  image_sub_ = it_.subscribe("/uvc_camera/cam_0/image_depth", 100, &DisparityProc::imageCb, this);

  // Publishers
  image_pub_ = it_.advertise("/disp_proc/image_depth", 100);
  bicaval_x_pub = nh_.advertise<std_msgs::Float64MultiArray>("/disp_proc/bicaval/x", 100);
  bicaval_y_pub = nh_.advertise<std_msgs::Float64MultiArray>("/disp_proc/bicaval/y", 100);
  short_axis_x_pub = nh_.advertise<std_msgs::Float64MultiArray>("/disp_proc/short_axis/x", 100);
  short_axis_y_pub = nh_.advertise<std_msgs::Float64MultiArray>("/disp_proc/short_axis/y", 100);
  four_chamber_x_pub = nh_.advertise<std_msgs::Float64MultiArray>("/disp_proc/four_chamber/x", 100);
  four_chamber_y_pub = nh_.advertise<std_msgs::Float64MultiArray>("/disp_proc/four_chamber/y", 100);
  needle_loc_pub = nh_.advertise<std_msgs::Float64MultiArray>("/disp_proc/needle_loc", 100);

  // ROI - Boundary
  x_min  = 372;
  y_min  = 255;
  width  = 100;
  height = 100;

  // After configuration values on Fossa should be 0
  config_iter = 0;
  config_done = false;
  config_img = cv::Mat::zeros(height+1, width+1, CV_8UC1);
  //niveau = 20;
  niveau = 120;

  // TO BE IMPLEMENTED
  std::cerr << "Z_Scaling initialized to 100! Find good choice!" << std::endl;
  z_scaling = 100;

  proj_bicaval = std::make_pair(0,0);
  proj_short_axis = std::make_pair(0,0);
  proj_four_chamber = std::make_pair(0,0);
  old_spot_init = false;

  x_pt_bicaval = {0, 0, 0};
  y_pt_bicaval = {0, 0, 0};
  x_pt_short_axis = {0, 0, 0};
  y_pt_short_axis = {0, 0, 0};
  x_pt_four_chamber = {0, 0, 0};
  y_pt_four_chamber = {0, 0, 0};
}

DisparityProc::~DisparityProc() { }

/* --------------------------------------------------------------------------------------------
	Call Back function for disparity map - manipulate data here
	TODO: 	- Make X,Y,mask_ell,mask_line, ... member variable -> only need to be computed once!
			- 
   -------------------------------------------------------------------------------------------*/

void DisparityProc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);		// Choose 8 BIT Format, e.g. TYPE_8UC1, MONO8, ...
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  /* --------------------------------------------------------------------------------------------
				Select ROI & Inititialize Masks
     -------------------------------------------------------------------------------------------*/

  cv::Mat image = cv_ptr->image;
  cv::Mat ROI(image, cv::Rect(x_min, y_min, width+1, height+1));

  cv::Mat X = cv::Mat::zeros(height+1, width+1, CV_32FC1);
  cv::Mat Y = cv::Mat::zeros(height+1, width+1, CV_32FC1);

  // Create Meshgrid
  cv::Range x_bound(0, width);
  cv::Range y_bound(0, height);
  meshgrid(x_bound, y_bound, X, Y);

  // Initialize and compute Masks
  const float PI = 3.14159265358979f;
  Line line1(-2.3, 175, 2);						// Bicaval
  Line line2(0.8, 20, 1);						// Short Axis
  Line line3(0, 60, 1);							// 4 Chamber
  Ellipse FO(50, 50, 35, 33, -50 * PI/180);		// Constructor(x0, y0, a, b, phi)
  Rectangle needle(0, 0 ,10, 10);					// Constructor(x0, y0, width, height)

  cv::Mat mask_line1;
  cv::Mat mask_line2;
  cv::Mat mask_line3;
  cv::Mat mask_FO;
  cv::Mat mask_needle;

  //std::cerr << "X: " << X.rows << "x" << X.cols << std::endl;
  //std::cerr << "Y: " << Y.rows << "x" << Y.cols << std::endl;

  line1.calc_mask(mask_line1, X, Y);
  line2.calc_mask(mask_line2, X, Y);
  line3.calc_mask(mask_line3, X, Y);
  FO.calc_mask(mask_FO, X, Y);

  // Auto Configuration
  // ROI = compute_config(ROI);

  /* --------------------------------------------------------------------------------------------
				Compute Start & End Points of Cuts
     -------------------------------------------------------------------------------------------*/

  std::pair<int,int> bicaval_start;
  std::pair<int,int> bicaval_stop;
  std::pair<int,int> short_axis_start;
  std::pair<int,int> short_axis_stop;
  std::pair<int,int> four_chamber_start;
  std::pair<int,int> four_chamber_stop;

  start_end_coord(line1, mask_FO, X, Y, bicaval_start, bicaval_stop);
  start_end_coord(line2, mask_FO, X, Y, short_axis_start, short_axis_stop);
  start_end_coord(line3, mask_FO, X, Y, four_chamber_start, four_chamber_stop);

  /* --------------------------------------------------------------------------------------------
				Compute Update of Tenting Curve
    Main Idea: Minimize difference between SPOT & OLD SPOT and introduce some kind of spacial delay
     -------------------------------------------------------------------------------------------*/

  // Check whether or not tenting even occurs
  bool th_exceeded = tenting_ocurrs(ROI, mask_FO);

  // Update Spot Location if Threshold is exceeded
  if (th_exceeded) {

    // Estimate current position of needle
    find_needle_tip(ROI, mask_FO, needle);
	needle_tip_ = needle.get_mid();

    if (old_spot_init) {
      // Compute whether or not intersection occurs - for each Projection
	  std::pair<int,int> tmp_proj1 = line1.projection(needle_tip_, bicaval_start);
	  std::pair<int,int> tmp_proj2 = line2.projection(needle_tip_, short_axis_start);
	  std::pair<int,int> tmp_proj3 = line3.projection(needle_tip_, four_chamber_start);
      bool moving1 = needle_moving(old_proj_bicaval, tmp_proj1);
	  bool moving2 = needle_moving(old_proj_short_axis, tmp_proj2);
	  bool moving3 = needle_moving(old_proj_four_chamber, tmp_proj3);

	  // Update Bicaval
      if (!moving1) {
		proj_bicaval = old_proj_bicaval;				// Stay Still
      }
      else {
		minimize_with_constraint(needle, line1, 0);		// Minimize Scalar Product
      }

	  // Update Short Axis
      if (!moving2) {
		proj_short_axis = old_proj_short_axis;			// Stay Still
      }
      else {
		minimize_with_constraint(needle, line2, 1);		// Minimize Scalar Product
      }

	  // Update 4 Chamber
      if (!moving3) {
		proj_four_chamber = old_proj_four_chamber;		// Stay Still
      }
      else {
		minimize_with_constraint(needle, line3, 2);		// Minimize Scalar Product
      }
    }
    else {
      // Old Spot not initialized -> Initialize
      initialize_spot(needle,line1, bicaval_start, 0);
      initialize_spot(needle,line2, short_axis_start, 1);
      initialize_spot(needle,line3, four_chamber_start, 2);
      old_spot_init = true;
    }
  }
  else {
    //std::cerr << "Threshold NOT exceeded..." << std::endl;
    // Draw straight line
    proj_bicaval = bicaval_start;
    proj_short_axis = short_axis_start;
    proj_four_chamber = four_chamber_start;
    old_spot_init = false;
  }

  // Derive x/y _ points (new 1D coord sys) from spot
  derive_xy_points(bicaval_start, bicaval_stop, ROI, 0);
  derive_xy_points(short_axis_start, short_axis_stop, ROI, 1);
  derive_xy_points(four_chamber_start, four_chamber_stop, ROI, 2);

  // Update Old Spot
  old_proj_bicaval = line1.projection(proj_bicaval, bicaval_start);
  old_proj_short_axis = line2.projection(proj_short_axis, short_axis_start);
  old_proj_four_chamber = line3.projection(proj_four_chamber, four_chamber_start);

  // Print Values of Needle Location
/*
  std::cerr << "Needle Location:" << std::endl;
  int x_print = needle_tip_.first;
  int y_print = needle_tip_.second;
  int value   = ROI.at<uint8_t>(y_print, x_print);
  std::cerr << "(X,Y) = (" << x_print << ", " << y_print << ") : " << value << std::endl;
*/

  // For Debugging -> Draw on FO
  ROI = draw_on_FO (
	ROI,
	mask_FO,
	mask_line1,
	mask_line2,
	mask_line3,
	bicaval_start,
	bicaval_stop,
	short_axis_start,
	short_axis_stop,
	four_chamber_start,
	four_chamber_stop,
	needle );

  /* --------------------------------------------------------------------------------------------
	Interpoplate the two parabolas & evaluate on a fine mesh
     -------------------------------------------------------------------------------------------*/
/*
 *  depricated...
 *

  Parabola left(x_points[0], y_points[0], x_points[1], y_points[1]);
  Parabola right(x_points[2], y_points[2], x_points[1], y_points[1]);

  int N_vis = 1000;
  std::vector<float> x_vis(N_vis, 0);
  std::vector<float> y_vis(N_vis, 0);

  float h = (x_points[2]-x_points[0])/(N_vis-1);
  for(unsigned int i=0; i<x_vis.size(); ++i) {
    x_vis[i] = x_points[0] + i*h;
    if (x_vis[i] < x_points[1])
      y_vis[i] = left.eval_para(x_vis[i]);
    else
      y_vis[i] = right.eval_para(x_vis[i]);
  }
*/
  /* --------------------------------------------------------------------------------------------
	Create Ros - Messages and publish data
     -------------------------------------------------------------------------------------------*/

  // Write ROI to published image
  cv_ptr->image = ROI;

  // Build Messages - BICAVAL
  std_msgs::Float64MultiArray bicaval_x;
  std_msgs::Float64MultiArray bicaval_y;

  bicaval_x.layout.dim.push_back(std_msgs::MultiArrayDimension());
  bicaval_x.layout.dim[0].size = x_pt_bicaval.size();
  bicaval_x.layout.dim[0].stride = 1;
  bicaval_x.layout.dim[0].label = "Bicaval_X_Val";

  bicaval_x.data.clear();
  bicaval_x.data.insert(bicaval_x.data.end(), x_pt_bicaval.begin(), x_pt_bicaval.end());

  bicaval_y.layout.dim.push_back(std_msgs::MultiArrayDimension());
  bicaval_y.layout.dim[0].size = y_pt_bicaval.size();
  bicaval_y.layout.dim[0].stride = 1;
  bicaval_y.layout.dim[0].label = "Bicaval_Y_Val";

  bicaval_y.data.clear();
  bicaval_y.data.insert(bicaval_y.data.end(), y_pt_bicaval.begin(), y_pt_bicaval.end());

  // Build Messages - SHORT AXIS
  std_msgs::Float64MultiArray short_axis_x;
  std_msgs::Float64MultiArray short_axis_y;

  short_axis_x.layout.dim.push_back(std_msgs::MultiArrayDimension());
  short_axis_x.layout.dim[0].size = x_pt_short_axis.size();
  short_axis_x.layout.dim[0].stride = 1;
  short_axis_x.layout.dim[0].label = "Short_Axis_X_Val";

  short_axis_x.data.clear();
  short_axis_x.data.insert(short_axis_x.data.end(), x_pt_short_axis.begin(), x_pt_short_axis.end());

  short_axis_y.layout.dim.push_back(std_msgs::MultiArrayDimension());
  short_axis_y.layout.dim[0].size = y_pt_short_axis.size();
  short_axis_y.layout.dim[0].stride = 1;
  short_axis_y.layout.dim[0].label = "Short_Axis_Y_Val";

  short_axis_y.data.clear();
  short_axis_y.data.insert(short_axis_y.data.end(), y_pt_short_axis.begin(), y_pt_short_axis.end());

  // Build Messages - 4 CHAMBER
  std_msgs::Float64MultiArray four_chamber_x;
  std_msgs::Float64MultiArray four_chamber_y;

  four_chamber_x.layout.dim.push_back(std_msgs::MultiArrayDimension());
  four_chamber_x.layout.dim[0].size = x_pt_four_chamber.size();
  four_chamber_x.layout.dim[0].stride = 1;
  four_chamber_x.layout.dim[0].label = "Four_Chamber_X_Val";

  four_chamber_x.data.clear();
  four_chamber_x.data.insert(four_chamber_x.data.end(), x_pt_four_chamber.begin(), x_pt_four_chamber.end());

  four_chamber_y.layout.dim.push_back(std_msgs::MultiArrayDimension());
  four_chamber_y.layout.dim[0].size = y_pt_four_chamber.size();
  four_chamber_y.layout.dim[0].stride = 1;
  four_chamber_y.layout.dim[0].label = "Four_Chamber_Y_Val";

  four_chamber_y.data.clear();
  four_chamber_y.data.insert(four_chamber_y.data.end(), y_pt_four_chamber.begin(), y_pt_four_chamber.end());

  // Build Messages - NEEDLE LOCATION
  std_msgs::Float64MultiArray needle_loc_msg;

  bool punctured = true;
  std::pair<float,float> tmp_ = FO.map_to_unit_circle(needle_tip_);
  tmp_.second = -tmp_.second;

  needle_loc_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  needle_loc_msg.layout.dim[0].size = 3;
  needle_loc_msg.layout.dim[0].stride = 1;
  needle_loc_msg.layout.dim[0].label = "Needle_Location";

  needle_loc_msg.data.clear();
  needle_loc_msg.data.push_back(punctured);
  needle_loc_msg.data.push_back(tmp_.first);
  needle_loc_msg.data.push_back(tmp_.second);

  // Publish
  image_pub_.publish(cv_ptr->toImageMsg());
  bicaval_x_pub.publish(bicaval_x);
  bicaval_y_pub.publish(bicaval_y);
  short_axis_x_pub.publish(short_axis_x);
  short_axis_y_pub.publish(short_axis_y);
  four_chamber_x_pub.publish(four_chamber_x);
  four_chamber_y_pub.publish(four_chamber_y);
  needle_loc_pub.publish(needle_loc_msg);
}

void DisparityProc::meshgrid(cv::Range x_bound, cv::Range y_bound, cv::Mat& X, cv::Mat& Y)
{
  for (int i = x_bound.start; i <= x_bound.end; i++) {
    for (int j = y_bound.start; j <= y_bound.end; j++) {
      X.at<float>(i,j) = j;
      Y.at<float>(i,j) = i;
    }
  }
}

cv::Mat DisparityProc::compute_config (cv::Mat depth_img)
{
  if (!config_done) {
	std::cerr << "Save Reference Values: Iter " << config_iter << std::endl;
	for (int i = 0; i < depth_img.rows; ++i) {
	  for (int j = 0; j < depth_img.cols; ++j) {
		if ( depth_img.at<uint8_t>(i,j) > config_img.at<uint8_t>(i,j) ) {
		  config_img.at<uint8_t>(i,j) = depth_img.at<uint8_t>(i,j);
		}
	  }
	}
	config_iter++;
	if (config_iter == 60) {
	  for (int i = 0; i < depth_img.rows; ++i) {
		for (int j = 0; j < depth_img.cols; ++j) {
		  config_img.at<uint8_t>(i,j) -= 5;
		}
	  }
	  config_done = true;
	}
  }
  else {
	int px_c, px_r;
	for (int i = 0; i < depth_img.rows; ++i) {
	  for (int j = 0; j < depth_img.cols; ++j) {
		px_c = config_img.at<uint8_t>(i,j);
		px_r = depth_img.at<uint8_t>(i,j);
		if (px_r > px_c) {
		  depth_img.at<uint8_t>(i,j) = px_r - px_c;
		}
		else {
		  depth_img.at<uint8_t>(i,j) = 0;
		}
	  }
	}
  }
  return depth_img;
}

bool DisparityProc::tenting_ocurrs(cv::Mat depth_img, cv::Mat mask_FO)
{
  int th = 250;
  int ctr = 0;
  for (int i = 0; i < depth_img.rows; ++i) {
    for (int j = 0; j < depth_img.cols; ++j) {
      if (!mask_FO.at<uint8_t>(i,j) && depth_img.at<uint8_t>(i,j) > niveau) {
	ctr++;
      }
    }
  }
  if (ctr > th) {
    return true;
  }
  return false;
}

bool DisparityProc::needle_moving (std::pair<int,int> pt_1, std::pair<int,int> pt_2)
{
  int y_diff = pt_1.first - pt_2.first;
  int x_diff = pt_1.second - pt_2.second;
  float norm = sqrt(x_diff*x_diff + y_diff*y_diff);
  if (norm > 5) {
	return true;
  }
  return false;
}

void DisparityProc::find_needle_tip(
	//input
	const cv::Mat& depth_img,
	const cv::Mat& mask_FO,
	//output
	Rectangle& needle_tip)
{
  // Needle tip size needs to be initialized
  if ( !needle_tip.height || !needle_tip.width ) {
    std::cerr << "Needle tip was initialized with zero size, shutdown..." << std::endl;
    ros::shutdown();
  }
  int width = needle_tip.width;
  int height = needle_tip.height;

  float spot_px_max_avg = 0;
  float spot_px_sum     = 0;
  int spot_px_ctr       = 0;

  for (int i = 0; i < (depth_img.rows - height); ++i) {
    for (int j = 0; j < (depth_img.cols - width); ++j) {
      // Restrict search to Ellipse representing FO
      if ( !mask_FO.at<uint8_t>(i,j) ) {
	// Average Values over Rectangle of needle tip
	for (int n = i; n < (i + width); ++n) {
	  for (int m = j; m < (j + height); ++m) {
	    if ( !mask_FO.at<uint8_t>(n,m) ) {
	      spot_px_sum += depth_img.at<uint8_t>(n,m);
	      spot_px_ctr++;
	    }
	  }
	}
	if (spot_px_ctr > 9) {					// Some threshold for minimum amount of pixels
	  float cmp = spot_px_sum / spot_px_ctr;
	  if (cmp > spot_px_max_avg) {
	    spot_px_max_avg = cmp;
	    needle_tip.x0 = j;
	    needle_tip.y0 = i;
	  }
	}
	spot_px_ctr = 0;
	spot_px_sum = 0;
      }
    }
  }
}

void DisparityProc::start_end_coord(
	// input
	const Line& cut_line,
	const cv::Mat& mask_FO,
	const cv::Mat& X,
	const cv::Mat& Y,
	// output
	std::pair<int,int>& start,
	std::pair<int,int>& end)
{
  // Compute Start & End Coordinates along the cut specified by Line
  float a = cut_line.a;
  cv::Mat mask_line;
  cut_line.calc_mask(mask_line, X, Y);

  std::vector<float> cut_x;
  std::vector< std::pair<int,int> > cut_xy;

  for (int i = 0; i < mask_line.rows; ++i) {
    for (int j = 0; j < mask_line.cols; ++j) {
      if (mask_line.at<uint8_t>(i,j) && !mask_FO.at<uint8_t>(i,j)) {
	cut_xy.push_back( std::make_pair(j,i) );
	cut_x.push_back(sqrt(1 + a*a) * j);
      }
    }
  }
  if (cut_x.empty() || cut_xy.empty()) {
    std::cerr << "Cut is not hitting Fossa Ovalis, shutdown..." << std::endl;
    ros::shutdown();
  }

  // Sort the cut according to the x values
  std::vector< std::pair <float, std::pair<int,int> > > sort_container;
  for (unsigned int i = 0; i < cut_x.size(); ++i) {
    sort_container.push_back( std::make_pair(cut_x[i], cut_xy[i]) );
  }
  sort(sort_container.begin(), sort_container.end());
  for (unsigned int i = 0; i < sort_container.size(); ++i) {
    cut_x[i] = sort_container[i].first;
    cut_xy[i] = sort_container[i].second;
  }
  start = cut_xy[0];
  end = cut_xy.back();
}

void DisparityProc::initialize_spot (Rectangle needle, Line cut_line, std::pair<int,int> start_pt, int view)
{
  std::pair<int,int> needle_tip = needle.get_mid();
  switch (view) {
	case 0:
	  proj_bicaval = cut_line.projection(needle_tip, start_pt);
	  break;
	case 1:
	  proj_short_axis = cut_line.projection(needle_tip, start_pt);
	  break;
	case 2:
	  proj_four_chamber = cut_line.projection(needle_tip, start_pt);
	  break;
	default:
	  std::cerr << "Valid Views are:\n0: Bicaval\n1: Short Axis\n2: 4 Chamber" << std::endl;
  }

}

void DisparityProc::minimize_with_constraint (Rectangle needle, Line cut_line, int view)
{
  // Definitions of updating scheme
  int max_px_mv = 3;

  // Directional Vector of cutting line
  float norm = sqrt(1 + cut_line.a * cut_line.a);
  std::pair<float,float> norm_vec = std::make_pair(1/norm, cut_line.a/norm);

  // Difference Vector of needle tip and old spot
  std::pair<int,int> needle_tip = needle.get_mid();
  std::pair<float,float> diff_vec;
  switch (view) {
	case 0:
	  diff_vec = std::make_pair(needle_tip.first - old_proj_bicaval.first, needle_tip.second - old_proj_bicaval.second);
	  break;
	case 1:
	  diff_vec = std::make_pair(needle_tip.first - old_proj_short_axis.first, needle_tip.second - old_proj_short_axis.second);
	  break;
	case 2:
	  diff_vec = std::make_pair(needle_tip.first - old_proj_four_chamber.first, needle_tip.second - old_proj_four_chamber.second);
	  break;
	default:
	  std::cerr << "Valid Views are:\n0: Bicaval\n1: Short Axis\n2: 4 Chamber" << std::endl;
  }

  // Scalar Product
  float scalar = diff_vec.first * norm_vec.first + diff_vec.second * norm_vec.second;
  int scalar_sgn = (scalar > 0) - (scalar < 0);

  // Compute Vector to new Spot
  std::pair<float,float> update_vec;
  if (abs(scalar) > max_px_mv) {
    update_vec.first  = scalar_sgn * max_px_mv * norm_vec.first;
    update_vec.second = scalar_sgn * max_px_mv * norm_vec.second;
  }
  else {
    update_vec.first  = scalar_sgn * norm_vec.first;
    update_vec.second = scalar_sgn * norm_vec.second;
  }

  // Update Spot Location
  switch (view) {
	case 0:
	  proj_bicaval.first  = old_proj_bicaval.first + update_vec.first;
	  proj_bicaval.second = old_proj_bicaval.second + update_vec.second;
	  break;
	case 1:
	  proj_short_axis.first  = old_proj_short_axis.first + update_vec.first;
	  proj_short_axis.second = old_proj_short_axis.second + update_vec.second;
	  break;
	case 2:
	  proj_four_chamber.first  = old_proj_four_chamber.first + update_vec.first;
	  proj_four_chamber.second = old_proj_four_chamber.second + update_vec.second;
	  break;
	default:
	  std::cerr << "Valid Views are:\n0: Bicaval\n1: Short Axis\n2: 4 Chamber" << std::endl;
  }
}

void DisparityProc::derive_xy_points(std::pair<int,int> start_pt, std::pair<int,int> end_pt, const cv::Mat& depth_img, int view)
{
  // Compute normalizing Factors
  float a = sqrt( pow((end_pt.first - start_pt.first),2) + pow((end_pt.second - start_pt.second),2) );
  float b;

  switch (view) {
	case 0:
	  b = sqrt( pow((proj_bicaval.first - start_pt.first),2) + pow((proj_bicaval.second - start_pt.second),2) );

	  x_pt_bicaval[0] = 0;
	  x_pt_bicaval[1] = b/a;
	  x_pt_bicaval[2] = 1;

	  y_pt_bicaval[0] = 0;
	  if (x_pt_bicaval[1] == x_pt_bicaval[0]) {
    	y_pt_bicaval[1] = 0;
	  }
	  else {
 	   y_pt_bicaval[1] = depth_img.at<uint8_t>(proj_bicaval.second,proj_bicaval.first) / float(z_scaling);
	  }
	  y_pt_bicaval[2] = 0;
	  break;
	case 1:
	  b = sqrt( pow((proj_short_axis.first - start_pt.first),2) + pow((proj_short_axis.second - start_pt.second),2) );

	  x_pt_short_axis[0] = 0;
	  x_pt_short_axis[1] = b/a;
	  x_pt_short_axis[2] = 1;

	  y_pt_short_axis[0] = 0;
	  if (x_pt_short_axis[1] == x_pt_short_axis[0]) {
    	y_pt_short_axis[1] = 0;
	  }
	  else {
 	   y_pt_short_axis[1] = depth_img.at<uint8_t>(proj_short_axis.second,proj_short_axis.first) / float(z_scaling);
	  }
	  y_pt_short_axis[2] = 0;
	  break;
	case 2:
	  b = sqrt( pow((proj_four_chamber.first - start_pt.first),2) + pow((proj_four_chamber.second - start_pt.second),2) );

	  x_pt_four_chamber[0] = 0;
	  x_pt_four_chamber[1] = b/a;
	  x_pt_four_chamber[2] = 1;

	  y_pt_four_chamber[0] = 0;
	  if (x_pt_four_chamber[1] == x_pt_four_chamber[0]) {
    	y_pt_four_chamber[1] = 0;
	  }
	  else {
 	   y_pt_four_chamber[1] = depth_img.at<uint8_t>(proj_four_chamber.second,proj_four_chamber.first) / float(z_scaling);
	  }
	  y_pt_four_chamber[2] = 0;
	  break;
	default:
	  std::cerr << "Valid Views are:\n0: Bicaval\n1: Short Axis\n2: 4 Chamber" << std::endl;
  }

}

cv::Mat DisparityProc::draw_on_FO (
	cv::Mat depth_img,
	cv::Mat mask_FO,
	cv::Mat mask_bicaval,
	cv::Mat mask_short_axis,
	cv::Mat mask_four_chamber,
	std::pair<int,int> bicaval_start,
	std::pair<int,int> bicaval_stop,
	std::pair<int,int> short_axis_start,
	std::pair<int,int> short_axis_stop,
	std::pair<int,int> four_chamber_start,
	std::pair<int,int> four_chamber_stop,
	Rectangle needle )
{
  // Draw Mask arounf FO & Cutting Lines
  for (int i = 0; i < depth_img.rows; ++i) {
    for (int j = 0; j < depth_img.cols; ++j) {
      if (	mask_bicaval.at<uint8_t>(i,j) ||
			mask_short_axis.at<uint8_t>(i,j) ||
			mask_four_chamber.at<uint8_t>(i,j) ||
			mask_FO.at<uint8_t>(i,j) )
	  {
		depth_img.at<uint8_t>(i,j) = 0;
      }
    }
  }

  // Draw Start & End Points
  depth_img.at<uint8_t>(bicaval_start.second, bicaval_start.first) = 255;
  depth_img.at<uint8_t>(bicaval_stop.second, bicaval_stop.first) = 255;
  depth_img.at<uint8_t>(short_axis_start.second, short_axis_start.first) = 255;
  depth_img.at<uint8_t>(short_axis_stop.second, short_axis_stop.first) = 255;
  depth_img.at<uint8_t>(four_chamber_start.second, four_chamber_start.first) = 255;
  depth_img.at<uint8_t>(four_chamber_stop.second, four_chamber_stop.first) = 255;

  // Draw Needle Tip
  std::pair<int,int> helper_tip = needle.get_mid();
  for (int i = (helper_tip.second - 2); i <= (helper_tip.second + 2); ++i) {
	for (int j = (helper_tip.first - 2); j <= (helper_tip.first + 2); ++j) {
	  depth_img.at<uint8_t>(i,j) = 255;
	}
  }

  // Draw Spot Location
  depth_img.at<uint8_t>(proj_bicaval.second, proj_bicaval.first) = 255;
  depth_img.at<uint8_t>(proj_short_axis.second, proj_short_axis.first) = 255;
  depth_img.at<uint8_t>(proj_four_chamber.second, proj_four_chamber.first) = 255;

  return depth_img;
}

void DisparityProc::interpolate_natural_cubic_spline (
	// input
	std::vector<double> const & x,
	std::vector<double> const & y,
	int l_clamped,
	int r_clamped,
	std::vector<double> const & x_vis,
	// output
	std::vector<double> & y_vis )
{
  // Assemble the system; note that it is a tridiagonal one, so we store it as 3 vectors 

  int N = x.size();

  std::vector<double> lower(N-1, 0);
  std::vector<double> diag(N, 0);
  std::vector<double> upper(N-1, 0);
  std::vector<double> rhs(N);

  // Compute difference between data points
  std::vector<double> delta(N-1, 0);
  for (unsigned int i = 0; i < delta.size(); ++i)
    delta[i] = x[i+1] - x[i];

  // Initialize the system
  if (l_clamped) {
    upper[0] = (delta[0])/6;
    diag[0] = (delta[0])/3;
    rhs[0] = (y[1] - y[0])/(delta[0]);
  }
  else
    diag[0] = 1;

  for (int i = 1; i < N - 1; ++i) {
    upper[i] = (delta[i])/6;
    lower[i-1] = (delta[i-1])/6;
    diag[i] = (x[i+1] - x[i-1])/3;
    rhs[i] = ((y[i+1] - y[i])/delta[i]) - ((y[i] - y[i-1])/delta[i-1]);
  }

  if (r_clamped) {
    diag[N-1] = -(delta[N-2])/3;
    lower[N-2] = -(delta[N-2])/6;
    rhs[N-1] = (y[N-1] - y[N-2])/(delta[N-2]);
  }
  else
    diag[N-1] = 1;

  // Solve the system and store the result in d
  std::vector<double> d(N,0);
  solve_thomas(lower,diag,upper,rhs,d);

  // Evaluate the interploated curve at x_vis and store data in y_vis
  // We assume a sorted data list -> maybe implement later
  // The counter keeps track of which cubic polynomial the current x_vis values are
  int counter = 0;
  for (unsigned int i = 0; i < x_vis.size(); ++i) {
    while (x_vis[i] >= x[counter]) {
      counter++;
      if (counter == N) {
	counter--;
	break;
      }
    }
    y_vis[i] =	d[counter-1] * ((pow(x[counter] - x_vis[i], 3))/(6*delta[counter-1])) + 
		d[counter]*((pow(x_vis[i] - x[counter-1], 3))/(6*delta[counter-1])) + 
		((y[counter] - y[counter-1])/delta[counter-1] - (d[counter] -
		d[counter-1])*(delta[counter-1])/6)*(x_vis[i] - x[counter-1]) + (y[counter-1] - 
		d[counter-1]*(delta[counter-1]*delta[counter-1]/6));
  }
}

int DisparityProc::solve_thomas(
	// input: matrix & rhs
	std::vector<double> const & lower,
	std::vector<double> const & diag,
	std::vector<double> const & upper,
	std::vector<double> const & rhs,
	// output
	std::vector<double> & solution)
{
  int N = rhs.size();
  std::vector<double> c(N), d(N);
  double t;
    
  // forward sweep
  t = diag[0];
  if(fabs(t) < 1e-16)
    return -1;

  c[0] = upper[0]/t;
  d[0] = rhs[0]/t;

  for(int i=1; i<N-1; ++i) {
    t = diag[i]-lower[i-1]*c[i-1];
    if(fabs(t) < 1e-16)
      return -1;

    c[i] = upper[i]/t;
    d[i] = (rhs[i]-lower[i-1]*d[i-1])/t;
  }

  t = diag[N-1]-lower[N-2]*c[N-2];
  if(fabs(t) < 1e-16)
    return -1;

  d[N-1] = (rhs[N-1]-lower[N-2]*d[N-2])/t;

  // backward substitution
  solution[N-1] = d[N-1];
  for(int i=N-2; i>=0; --i)
    solution[i] = d[i]-c[i]*solution[i+1];

  return 0;
}

void Line::calc_mask (cv::Mat& mask_line, const cv::Mat& X, const cv::Mat& Y) const
{
  mask_line = (abs(a * X - Y + b) < tol);
}

std::pair<int,int> Line::projection (std::pair<int,int> pt, std::pair<int,int> start_pt)
{
  // Directional Vector of line
  float norm = sqrt(1 + a*a);
  std::pair<float,float> norm_vec = std::make_pair(1/norm, a/norm);

  // Difference Vector
  std::pair<int,int> diff_vec = std::make_pair(pt.first - start_pt.first, pt.second - start_pt.second);

  // Scalar Product
  float scalar = norm_vec.first * diff_vec.first + norm_vec.second * diff_vec.second;

  // Compute Vector to Result
  std::pair<int,int> offset_vec = std::make_pair(scalar * norm_vec.first, scalar * norm_vec.second);

  // Return Result
  std::pair<int,int> proj;
  proj.first  = start_pt.first + offset_vec.first;
  proj.second = start_pt.second + offset_vec.second;

  return proj;
}

void Ellipse::calc_mask (cv::Mat& mask_ell, const cv::Mat& X, const cv::Mat& Y) const
{
  cv::Mat x_norm = (1/a * std::cos(phi) * (X - x0) + 1/a * std::sin(phi) * (Y - y0));
  cv::Mat y_norm = (-1/b * std::sin(phi) * (X - x0) + 1/b * std::cos(phi) * (Y - y0));
  for (int i = 0; i < X.rows; ++i) {
    for (int j = 0; j < X.cols; ++j) {
      x_norm.at<float>(i,j) = x_norm.at<float>(i,j) * x_norm.at<float>(i,j);
      y_norm.at<float>(i,j) = y_norm.at<float>(i,j) * y_norm.at<float>(i,j);
    }
  }
  mask_ell = (x_norm + y_norm > 1);
}

std::pair<float,float> Ellipse::map_to_unit_circle (std::pair<int,int> point)
{
  int x_pt = point.first;
  int y_pt = point.second;
  float x_uc = 1/a * std::cos(phi) * (x_pt - x0) + 1/a * std::sin(phi) * (y_pt - y0);
  float y_uc = -1/b * std::sin(phi) * (x_pt - x0) + 1/b * std::cos(phi) * (y_pt - y0);

  return std::make_pair(x_uc, y_uc);
}

void Rectangle::calc_mask (cv::Mat& mask_rect, const cv::Mat& X, const cv::Mat& Y) const
{
  mask_rect = cv::Mat::zeros(X.rows, X.cols, CV_8UC1);
  if ( (x0 + width) > X.cols || (y0 + height) > X.rows) {
    std::cerr << "Error, while computing mask for rectangle!" << std::endl;
    std::cerr << "Rectangle outside Coordinate System..." << std::endl;
    ros::shutdown();
  }
  for (int i = x0; i < (x0 + width); ++i) {
    for (int j = y0; j < (y0 + height); ++j) {
      mask_rect.at<uint8_t>(i,j) = 1;
    }
  }
}

std::pair<int,int> Rectangle::get_mid()
{
  int xm = x0 + width/2;
  int ym = y0 + height/2;
  return std::make_pair(xm,ym);
}

void Rectangle::infer_from_mid(std::pair<int,int> mid)
{
  if (!width || !height) {
    std::cerr << "Warning! Zero Size Rectangle called..." << std::endl;
  }
  x0 = mid.first - width/2;
  y0 = mid.second - height/2;
}

Parabola::Parabola (float xs, float ys, float xp, float yp)
{
  a = (yp - ys) /( (xp - xs)*(xp - xs) );
  b = -2*a*xs;
  c = ys + a*xs*xs;
}

float Parabola::eval_para (float x)
{
  return a*x*x + b*x + c;
}

