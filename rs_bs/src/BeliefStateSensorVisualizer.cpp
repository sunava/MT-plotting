#include <uima/api.hpp>

//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/CASConsumerContext.h>

using namespace uima;


class BeliefStateSensorVisualizer : public DrawingAnnotator
{
private:
  typedef pcl::PointXYZRGBA PointT;

  cv::Mat object_, rgb_, depth_;
  cv::Mat other_cas_img_; // fill be filled if the correspondig mode is used
  double pointSize;

  enum
  {
      ONLY_RGB,
      MIXED_WITH_OTHER_CAS_RGB,
      ONLY_OBJECT_MASK
  } dispMode;

  pcl::PointCloud<PointT>::Ptr joined_cloud_;
  pcl::PointCloud<PointT>::Ptr bs_cloud_;
  pcl::PointCloud<PointT>::Ptr other_cas_cloud_;

  //
  // PARAMETERS
  //
  //
  // This identifier is used to reference the CAS of another AAE that ran before
  // *this* AAE. It is accessed via rs::CASConsumerContext.
  // This Annotator will fetch an VIEW_COLOR_IMAGE from this CAS and mix it
  // with the VIEW_COLOR_IMAGE in the CAS *this* Annotator is running in.
  std::string other_cas_id;
  bool use_hd_images_ = false;

public:
  BeliefStateSensorVisualizer(): DrawingAnnotator(__func__), dispMode(ONLY_RGB), pointSize(1)
  {

  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("otherCASId"))
    {
      ctx.extractValue("otherCASId", other_cas_id);
      outInfo("Using AAE/CAS identified by '" << other_cas_id << "' for mixing images");
    }

    if(ctx.isParameterDefined("useHDImages"))
    {
      ctx.extractValue("useHDImages", use_hd_images_);
      outInfo("Use HD Images: " << use_hd_images_);
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);

    joined_cloud_ =  pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    bs_cloud_ =  pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    other_cas_cloud_ =  pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

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

    *joined_cloud_ += *bs_cloud_;

    if(dispMode == MIXED_WITH_OTHER_CAS_RGB) {
      uima::CAS *other_cas;
      other_cas = rs::CASConsumerContext::getInstance().getCAS(other_cas_id);
      if (other_cas) {
        rs::SceneCas other_cas_scene(*other_cas);
        if(use_hd_images_)
        {
          other_cas_scene.get(VIEW_COLOR_IMAGE_HD, other_cas_img_);
        }
        else
        {
          other_cas_scene.get(VIEW_COLOR_IMAGE, other_cas_img_);
        }
        other_cas_scene.get(VIEW_CLOUD, *other_cas_cloud_);

        *joined_cloud_ += *other_cas_cloud_;
      } else {
        outWarn("Couldn't fetch CAS identified by '"
                << other_cas_id
                << "'. Make sure you have loaded an AAE with that name and "
                << " that you've set 'otherCASId' in this config");
      }
    }

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case '1':
     dispMode = ONLY_RGB;
     outWarn("Switching to ONLY_RGB mode");
     break;
    case '2':
     dispMode = MIXED_WITH_OTHER_CAS_RGB;
     outWarn("Switching to MIXED_WITH_OTHER_CAS_RGB");
     break;
    case '3':
     dispMode = ONLY_OBJECT_MASK;
     outWarn("Switching to OBJECT MASK");
     break;
    }

   return true;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    if(!rgb_.empty())
    {
      // Switch over the other cases
      switch(dispMode)
      {
      case MIXED_WITH_OTHER_CAS_RGB:
        if(other_cas_img_.empty()){
          outWarn("Other CAS img is empty. Can't mix.");
          disp = cv::Mat::ones(cv::Size(640, 480), CV_8UC3);
          return;
        }

        addWeighted(rgb_, 0.5, other_cas_img_, 0.5, 0.0, disp);
        break;
      case ONLY_OBJECT_MASK:
        disp = object_.clone();
        break;
      default:
        disp  = rgb_.clone();
      }
    }
    else
    {
      disp = cv::Mat::ones(cv::Size(640, 480), CV_8UC3);
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
    const std::string &cloudname = this->name;

    if(!firstRun)
    {
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    visualizer.removeAllPointClouds();
    visualizer.removeAllShapes();

    switch(dispMode) {
    case ONLY_RGB:
      visualizer.addPointCloud(bs_cloud_, cloudname);
      break;
    case MIXED_WITH_OTHER_CAS_RGB:
      visualizer.addPointCloud(joined_cloud_, cloudname);
      break;
    // TODO: Bring up new type?
    case ONLY_OBJECT_MASK:
      visualizer.addPointCloud(other_cas_cloud_, cloudname);
      break;
    }
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BeliefStateSensorVisualizer)