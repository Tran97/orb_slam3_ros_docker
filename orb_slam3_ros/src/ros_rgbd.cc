/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
 *
 */

#include "common.h"
#include <sys/stat.h>

using namespace std;

class ImageGrabber {
public:
  ImageGrabber(){};

  void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB,
                const sensor_msgs::ImageConstPtr &msgD);
};

bool fileExists(const std::string &path) {
  struct stat buffer;
  return (stat(path.c_str(), &buffer) == 0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "RGBD");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  if (argc > 1) {
    ROS_WARN("Arguments supplied via command line are ignored.");
  }

  std::string node_name = ros::this_node::getName();

  ros::NodeHandle node_handler;
  image_transport::ImageTransport image_transport(node_handler);

  std::string voc_file, settings_file;
  node_handler.param<std::string>(node_name + "/voc_file", voc_file,
                                  "file_not_set");
  node_handler.param<std::string>(node_name + "/settings_file", settings_file,
                                  "file_not_set");

  if (voc_file == "file_not_set" || settings_file == "file_not_set") {
    ROS_ERROR("Please provide voc_file and settings_file in the launch file");
    ros::shutdown();
    return 1;
  }

  std::string load_atlas_from_file;
  node_handler.param<std::string>(node_name + "/load_atlas_from_file",
                                  load_atlas_from_file, "file_not_set");

  if (load_atlas_from_file == "file_not_set") {
    ROS_ERROR("Parameter '%s/load_atlas_from_file' not "
              "found. Continuing without it.",
              node_name.c_str());
  }
  if (fileExists(load_atlas_from_file)) {
    ROS_ERROR(
        "Parameter '%s/load_atlas_from_file' is provided and a valid path: %s",
        node_name.c_str(), load_atlas_from_file.c_str());
  } else {
    ROS_ERROR("File does not exist: %s. Setting to empty string.",
              load_atlas_from_file.c_str());
    load_atlas_from_file = "";
  }

  node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id,
                                  "map");
  node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id,
                                  "camera");

  bool enable_pangolin;
  node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin,
                           true);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  sensor_type = ORB_SLAM3::System::RGBD;
  pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type,
                                load_atlas_from_file, enable_pangolin);

  ImageGrabber igb;

  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(
      node_handler, "/camera/rgb/image_raw", 100);
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(
      node_handler, "/camera/depth_registered/image_raw", 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img,
                                               sub_depth_img);
  sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

  setup_publishers(node_handler, image_transport, node_name);
  setup_services(node_handler, node_name);

  ros::spin();

  // Stop all threads
  pSLAM->Shutdown();
  ros::shutdown();

  return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB,
                            const sensor_msgs::ImageConstPtr &msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  float imageScale = pSLAM->GetImageScale();
  cv::Mat imRGB = cv_ptrRGB->image;
  cv::Mat imD = cv_ptrD->image;
  if (imageScale != 1.f) {
    int width = imRGB.cols * imageScale;
    int height = imRGB.rows * imageScale;
    cv::resize(imRGB, imRGB, cv::Size(width, height));
    cv::resize(imD, imD, cv::Size(width, height));

  } // ORB-SLAM3 runs in TrackRGBD()
  Sophus::SE3f Tcw = pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image,
                                      cv_ptrRGB->header.stamp.toSec());

  ros::Time msg_time = cv_ptrRGB->header.stamp;

  publish_topics(msg_time);
}