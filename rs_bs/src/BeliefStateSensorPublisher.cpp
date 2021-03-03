#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

using namespace uima;


class BeliefStateSensorPublisher : public Annotator
{
private:
  typedef pcl::PointXYZRGBA PointT;

  pcl::PointCloud<PointT>::Ptr bs_cloud_;
  cv::Mat object_, rgb_, depth_;

  bool use_hd_images_ = false;

  ros::Publisher rgb_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher object_pub_;
  ros::Publisher cloud_pub_;
  ros::NodeHandle nh_;



public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("useHDImages"))
    {
      ctx.extractValue("useHDImages", use_hd_images_);
      outInfo("Use HD Images: " << use_hd_images_);
    }

    rgb_pub_ = nh_.advertise<sensor_msgs::Image>("bs_rgb_image", 1, true);
    depth_pub_ = nh_.advertise<sensor_msgs::Image>("bs_depth_image", 1, true);
    object_pub_ = nh_.advertise<sensor_msgs::Image>("bs_object_image", 1, true);

//    cloud_pub_ = nh_.advertise<pcl::PointCloud>("bs_pointcloud", 1, true);

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);

    bs_cloud_ =  pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    if(use_hd_images_) {
      cas.get(VIEW_OBJECT_IMAGE_HD, object_);
      cas.get(VIEW_COLOR_IMAGE_HD, rgb_);
      cas.get(VIEW_DEPTH_IMAGE_HD, depth_);
    }
    else
    {
      cas.get(VIEW_OBJECT_IMAGE, object_);
      cas.get(VIEW_COLOR_IMAGE, rgb_);
      cas.get(VIEW_DEPTH_IMAGE, depth_);
    }

    cas.get(VIEW_CLOUD, *bs_cloud_);

    sensor_msgs::Image rgb_msg;
    cv_bridge::CvImage cv_image;
    cv_image.image = rgb_;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.toImageMsg(rgb_msg);
    rgb_pub_.publish(rgb_msg);

    // Doesn't work right now
    sensor_msgs::Image depth_msg;
    cv_bridge::CvImage cv_depth;
    cv_depth.image = depth_;
    cv_depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    cv_depth.toImageMsg(depth_msg);
    depth_pub_.publish(depth_msg);

    sensor_msgs::Image object_msg;
    cv_bridge::CvImage cv_object;
    cv_object.image = object_;
    cv_object.encoding = sensor_msgs::image_encodings::BGR8;
    cv_object.toImageMsg(object_msg);
    object_pub_.publish(object_msg);

//    sensor_msgs::PointCloud output;
//    pcl_conversions::fromPCL(*bs_cloud_, output);

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BeliefStateSensorPublisher)