#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <assert.h>
#include <cmath>
#include <algorithm>

struct Line
{
  float a;
  float b;
  float tol;

  Line () {}
  ~Line () {}
  Line (float a, float b, float tol) : a(a), b(b), tol(tol) {}
  void calc_mask (cv::Mat& mask_line, const cv::Mat& X, const cv::Mat& Y) const;
  std::pair<int,int> projection (std::pair<int,int> pt, std::pair<int,int> start_pt);
};

struct Ellipse
{
  float x0;
  float y0;
  float a;
  float b;
  float phi;

  Ellipse () {}
  ~Ellipse () {}
  Ellipse (float x0, float y0, float a, float b, float phi) : x0(x0), y0(y0), a(a), b(b), phi(phi) {}
  void calc_mask (cv::Mat& mask_ell, const cv::Mat& X, const cv::Mat& Y) const;
  std::pair<float,float> map_to_unit_circle (std::pair<int,int> point);
};

struct Rectangle
{
  int x0;
  int y0;
  int width;
  int height;

  Rectangle () {}
  ~Rectangle () {}
  Rectangle (int x0, int y0, int width, int height) : x0(x0), y0(y0), width(width), height(height) {}
  void calc_mask (cv::Mat& mask_rect, const cv::Mat& X, const cv::Mat& Y) const;
  std::pair<int,int> get_mid();
  void infer_from_mid(std::pair<int,int> mid);
};

struct Parabola
{
  float a;
  float b;
  float c;

  // Constructor for a parabola with known vertex (xs,ys) & some additional point (xp,yp)
  Parabola (float xs, float ys, float xp, float yp);

  // Evaluate the Parabola
  float eval_para (float x);

};

class DisparityProc
{
private:
  // ROS Handles for Communication
  ros::NodeHandle nh_;
  ros::Publisher bicaval_x_pub;
  ros::Publisher bicaval_y_pub;
  ros::Publisher short_axis_x_pub;
  ros::Publisher short_axis_y_pub;
  ros::Publisher four_chamber_x_pub;
  ros::Publisher four_chamber_y_pub;
  ros::Publisher needle_loc_pub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // Boundary for ROI
  int x_min;
  int y_min;
  int width;
  int height;

  // Configuration Handles
  cv::Mat config_img;
  int config_iter;
  bool config_done;

  // Important Stuff :)
  int niveau;
  int z_scaling;
  bool old_spot_init;

  std::pair<int,int> proj_bicaval;
  std::pair<int,int> old_proj_bicaval;
  std::pair<int,int> proj_short_axis;
  std::pair<int,int> old_proj_short_axis;
  std::pair<int,int> proj_four_chamber;
  std::pair<int,int> old_proj_four_chamber;
  std::pair<int,int> needle_tip_;

  std::vector<float> x_pt_bicaval;
  std::vector<float> y_pt_bicaval;
  std::vector<float> x_pt_short_axis;
  std::vector<float> y_pt_short_axis;
  std::vector<float> x_pt_four_chamber;
  std::vector<float> y_pt_four_chamber;

public:
  DisparityProc ();
  ~DisparityProc ();
  void imageCb (const sensor_msgs::ImageConstPtr& msg);
  void meshgrid (cv::Range x_bound, cv::Range y_bound, cv::Mat& X, cv::Mat& Y);
  cv::Mat compute_config (cv::Mat depth_img);
  bool tenting_ocurrs (cv::Mat depth_img, cv::Mat mask_FO);
  bool needle_moving (std::pair<int,int> pt_1, std::pair<int,int> pt_2);
  void find_needle_tip (
	//input
	const cv::Mat& depth_img,
	const cv::Mat& mask_ell,
	//output
	Rectangle& needle_tip);
  void start_end_coord (
	// input
	const Line& cut_line,
	const cv::Mat& mask_FO,
	const cv::Mat& X,
	const cv::Mat& Y,
	// output
	std::pair<int,int>& start,
	std::pair<int,int>& end);
  void initialize_spot (Rectangle needle, Line cut_line ,std::pair<int,int> start_pt, int view);
  void minimize_with_constraint (Rectangle needle, Line cut_line, int view);
  void derive_xy_points(std::pair<int,int> start_pt, std::pair<int,int> end_pt, const cv::Mat& depth_img, int view);
  cv::Mat draw_on_FO (
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
	Rectangle needle );
  void interpolate_natural_cubic_spline (
	// input
	std::vector<double> const & x,
	std::vector<double> const & y,
	int l_clamped,
	int r_clamped,
	std::vector<double> const & x_vis,
	// output
	std::vector<double> & y_vis );
  int solve_thomas (
	// input: matrix & rhs
	std::vector<double> const & lower,
	std::vector<double> const & diag,
	std::vector<double> const & upper,
	std::vector<double> const & rhs,
	// output
	std::vector<double> & solution );
};
