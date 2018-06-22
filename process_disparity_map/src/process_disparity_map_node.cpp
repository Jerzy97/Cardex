#include "process_disparity_map/process_disparity_map.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "disparity_processing");
  DisparityProc ic;
  ros::spin();
  return 0;
}
