#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <dense_reconstruction/DisparitySGBMConfig.h>
#include <dynamic_reconfigure/server.h>

namespace dense_reconstruction {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class DisparitySGBMNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_disparity_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef dense_reconstruction::DisparitySGBMConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  cv::StereoSGBM sgbm; // contains scratch buffers for block matching
  int channels;

  virtual void onInit();

  void connectCb();

  void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
               const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

  void configCb(Config &config, uint32_t level);
};

void DisparitySGBMNodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                 sub_l_image_, sub_l_info_,
                                                 sub_r_image_, sub_r_info_) );
    approximate_sync_->registerCallback(boost::bind(&DisparitySGBMNodelet::imageCb,
                                                    this, _1, _2, _3, _4));
  }
  else
  {
    exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                     sub_l_image_, sub_l_info_,
                                     sub_r_image_, sub_r_info_) );
    exact_sync_->registerCallback(boost::bind(&DisparitySGBMNodelet::imageCb,
                                              this, _1, _2, _3, _4));
  }

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f = boost::bind(&DisparitySGBMNodelet::configCb,
                                                  this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparitySGBMNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_disparity_ = nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);
  channels = 1; //default value
}

// Handles (un)subscribing when clients (un)subscribe
void DisparitySGBMNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_disparity_.getNumSubscribers() == 0)
  {
    sub_l_image_.unsubscribe();
    sub_l_info_ .unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_ .unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    /// @todo Allow remapping left, right?
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
    sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
    sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
  }
}

void DisparitySGBMNodelet::imageCb(const ImageConstPtr& l_image_msg,
                               const CameraInfoConstPtr& l_info_msg,
                               const ImageConstPtr& r_image_msg,
                               const CameraInfoConstPtr& r_info_msg)
{
	//NODELET_INFO("Image pair received in SGBM nodelet!!");

  /// @todo Convert (share) with new cv_bridge
  assert(l_image_msg->encoding == sensor_msgs::image_encodings::MONO8);
  assert(r_image_msg->encoding == sensor_msgs::image_encodings::MONO8);

  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);
  
  // Allocate new disparity image message
  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header         = l_info_msg->header;
  disp_msg->image.header   = l_info_msg->header;
  disp_msg->image.height   = l_image_msg->height;
  disp_msg->image.width    = l_image_msg->width;
  disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.step     = disp_msg->image.width * sizeof(float);
  disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);

  // Stereo parameters
  disp_msg->f = model_.right().fx();
  disp_msg->T = model_.baseline();

//  CvStereoBMState* s =  cvCre(100,100);
  // Compute window of (potentially) valid disparities
//  cv::Ptr<CvStereoBMState> params =  .state;
 // cv::Ptr<CvStereoBMState> params = block_matcher_.state;
  int border   = sgbm.SADWindowSize / 2;
  int left   = sgbm.numberOfDisparities + sgbm.minDisparity + border - 1;
  int wtf = (sgbm.minDisparity >= 0) ? border + sgbm.minDisparity : std::max(border, -sgbm.minDisparity);
  int right  = disp_msg->image.width - 1 - wtf;
  int top    = border;
  int bottom = disp_msg->image.height - 1 - border;
  disp_msg->valid_window.x_offset = left;
  disp_msg->valid_window.y_offset = top;
  disp_msg->valid_window.width    = right - left;
  disp_msg->valid_window.height   = bottom - top;

  // Disparity search range
  disp_msg->min_disparity = sgbm.minDisparity;
  disp_msg->max_disparity = sgbm.minDisparity + sgbm.numberOfDisparities - 1;
  disp_msg->delta_d = 1.0 / 16; // OpenCV uses 16 disparities per pixel

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image(l_image_msg->height, l_image_msg->width,
                                  const_cast<uint8_t*>(&l_image_msg->data[0]),
                                  l_image_msg->step);
  const cv::Mat_<uint8_t> r_image(r_image_msg->height, r_image_msg->width,
                                  const_cast<uint8_t*>(&r_image_msg->data[0]),
                                  r_image_msg->step);
  cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                             reinterpret_cast<float*>(&disp_msg->image.data[0]),
                             disp_msg->image.step);


  channels = l_image.channels();

 // NODELET_INFO("4 SGBM nodelet!!");
  // Perform block matching to find the disparities
 // sgbm(l_image, r_image, disp_image); disp_image
  cv::Mat disp;
  sgbm(l_image, r_image,disp);

  //NODELET_INFO("5 SGBM nodelet!!");


 //disparity.copyTo(disp_image);
  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r)
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);

 //cv::imshow("disparity cv",disp);
 //cv::waitKey();

  disp.convertTo(disp_image,CV_32F,1/16.);

  pub_disparity_.publish(disp_msg);
}

void DisparitySGBMNodelet::configCb(Config &config, uint32_t level)
{
  // Tweak all settings to be valid
//  config.prefilter_size |= 0x1; // must be odd
//  config.correlation_window_size |= 0x1; // must be odd
	config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16

  // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
  // concurrently, so this is thread-safe.

	  NODELET_INFO("SGBM nodelet: configCb!!");
  sgbm.preFilterCap = config.prefilter_cap;
  sgbm.SADWindowSize = config.SADWindowSize; //config.SADWindowSize > 0 ? config.SADWindowSize : 3;
  sgbm.P1 = 64*channels*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.P2 = 128*channels*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.minDisparity = config.min_disparity;
  sgbm.numberOfDisparities = config.disparity_range;
  sgbm.uniquenessRatio = config.uniqueness_ratio;
  sgbm.speckleWindowSize = config.speckle_size;
  sgbm.speckleRange = config.speckle_range;
  sgbm.disp12MaxDiff = config.disp12MaxDiff;
  sgbm.fullDP = config.fullDP;
  //~ sgbm.fullDP = alg == STEREO_HH;

}

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(dense_reconstruction, disparitySGBM,
						dense_reconstruction::DisparitySGBMNodelet, nodelet::Nodelet)
