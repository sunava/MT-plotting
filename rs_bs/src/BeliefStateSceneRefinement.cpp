#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>

using namespace uima;


class BeliefStateSceneRefinement : public DrawingAnnotator
{
private:
  typedef pcl::PointXYZRGBA PointT;

  pcl::PointCloud<PointT>::Ptr bs_cloud_;
  pcl::PointCloud<PointT>::Ptr bs_cloud_downsampled_;
  pcl::PointCloud<PointT>::Ptr bs_cloud_downsampled_aligned_;
  pcl::PointCloud<PointT>::Ptr bs_aligned_cloud_;

  pcl::PointCloud<PointT>::Ptr other_cas_cloud_;
  pcl::PointCloud<PointT>::Ptr other_cas_cloud_downsampled_;

  pcl::PointCloud<PointT>::Ptr joined_cloud_;
  pcl::PointCloud<PointT>::Ptr aligned_joined_cloud_;

  std::string other_cas_id_;
  double pointSize;

  double rotation_threshold_ = 0.05; // in radian
  double translation_threshold_ = 0.05; // in meters
  enum
  {
    SHOW_UNALIGNED_CLOUDS,
    SHOW_ALIGNED_CLOUDS,
    SHOW_DOWNSAMPLED_OTHER_CAS_CLOUD
  } dispMode;

public:


  BeliefStateSceneRefinement(): DrawingAnnotator(__func__), other_cas_id_(""), pointSize(1), dispMode(SHOW_ALIGNED_CLOUDS)
  {

  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("otherCASId"))
    {
      ctx.extractValue("otherCASId", other_cas_id_);
      outInfo("Using AAE/CAS identified by '" << other_cas_id_ << "' for Clouds to refine");
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
    bs_cloud_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<PointT>());
    bs_cloud_downsampled_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<PointT>());
    bs_cloud_downsampled_aligned_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<PointT>());
    bs_aligned_cloud_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<PointT>());
    joined_cloud_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<PointT>());
    aligned_joined_cloud_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<PointT>());
    cas.get(VIEW_CLOUD,*bs_cloud_);

    uima::CAS *other_cas;
    other_cas_cloud_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<PointT>());
    other_cas_cloud_downsampled_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<PointT>());
    other_cas = rs::CASConsumerContext::getInstance().getCAS(other_cas_id_);
    if (other_cas) {
      rs::SceneCas other_cas_scene(*other_cas);
      other_cas_scene.get(VIEW_CLOUD, *other_cas_cloud_);
    } else {
      outWarn("Couldn't fetch CAS identified by '"
                  << other_cas_id_
                  << "'. Make sure you have loaded an AAE with that name and "
                  << " that you've set 'otherCASId' in this config");
      return UIMA_ERR_ANNOTATOR_MISSING_CONFIG;
    }

    *joined_cloud_ += *bs_cloud_;
    *joined_cloud_ += *other_cas_cloud_;

    pcl::VoxelGrid<pcl::PointXYZRGBA> other_cas_cloud_sampler;
    other_cas_cloud_sampler.setInputCloud(other_cas_cloud_);
    other_cas_cloud_sampler.setLeafSize(0.01f, 0.01f, 0.01f);
    other_cas_cloud_sampler.filter(*other_cas_cloud_downsampled_);
    outInfo("Cloud size after filter: " << other_cas_cloud_downsampled_->points.size());

    pcl::VoxelGrid<pcl::PointXYZRGBA> bs_cloud_sampler;
    other_cas_cloud_sampler.setInputCloud(bs_cloud_);
    other_cas_cloud_sampler.setLeafSize(0.01f, 0.01f, 0.01f);
    other_cas_cloud_sampler.filter(*bs_cloud_downsampled_);
    outInfo("Cloud size after filter: " << bs_cloud_downsampled_->points.size());



    // ALIGN THE BS CLOUD
//    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(bs_cloud_downsampled_);
    icp.setInputTarget(other_cas_cloud_downsampled_);
    icp.setMaximumIterations (20);
    icp.setEuclideanFitnessEpsilon(1e-2);
    icp.setMaxCorrespondenceDistance (0.05);
    icp.align(*bs_cloud_downsampled_aligned_);

    outInfo("ICP done. Has converged? " << icp.hasConverged()
                                        << " score: " << icp.getFitnessScore());
    outInfo("ICP found transform: " << icp.getFinalTransformation());

    Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
    pcl::transformPointCloud (*bs_cloud_, *bs_aligned_cloud_, icp_transform);

    *aligned_joined_cloud_ += *bs_aligned_cloud_;
    *aligned_joined_cloud_ += *other_cas_cloud_;

    if(!analyzeICPResults(icp))
    {
      outInfo("analyze false");
    }


//    outInfo("Cloud size: " << bs_cloud_->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  /**
   * Check if rotation is in threshold.
   * @param x rotation angle in deg.
   * @return true if OK
   */
  bool checkRotationalThreshold(double x) const
  {
    double x_abs = abs(x);
    if(x_abs <= rotation_threshold_)
      return true;

    if(M_PI - x_abs <= rotation_threshold_)
      return true;

    return false;
  }

  // Return false if icp transform shows translational or rotational diffs above threshold
  bool
  analyzeICPResults(pcl::IterativeClosestPoint<PointT, PointT> &icp) const {
    Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    rotation  =  icp.getFinalTransformation().block<3, 3>(0, 0);
    translation = icp.getFinalTransformation().block<3, 1>(0, 3);
    outInfo("Found translation: " << translation);
    Eigen::Vector3f rotation_as_euler_angles = rotation.eulerAngles(0,1,2);
    outInfo("Found rotation (RPY/XYZ) in RAD: " << rotation_as_euler_angles);

    if( !checkRotationalThreshold(rotation_as_euler_angles[0]) ||
        !checkRotationalThreshold(rotation_as_euler_angles[1]) ||
        !checkRotationalThreshold(rotation_as_euler_angles[2]) )
    {
      outError("Rotational error after ICP is larger than " << rotation_threshold_);
      return false;
    }

    if( translation[0] > translation_threshold_ ||
        translation[1] > translation_threshold_ ||
        translation[2] > translation_threshold_)
    {
      outError("Translational error after ICP is larger than " << rotation_threshold_);
      return false;
    }
    return true;
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case '1':
     dispMode = SHOW_UNALIGNED_CLOUDS;
     outWarn("Switching to SHOW_UNALIGNED_CLOUDS mode");
     break;
    case '2':
     dispMode = SHOW_ALIGNED_CLOUDS;
     outWarn("Switching to SHOW_ALIGNED_CLOUDS");
     break;
    case '3':
      dispMode = SHOW_DOWNSAMPLED_OTHER_CAS_CLOUD;
      outWarn("Switching to SHOW_DOWNSAMPLED_OTHER_CAS_CLOUD");
      break;
    }
   return true;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
      disp = cv::Mat::ones(cv::Size(640, 480), CV_8UC3);
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
    const std::string &cloudname = this->name;

    if(firstRun)
    {
//      visualizer.addPointCloud(viz_cloud_ptr_, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }else
    {
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    visualizer.removeAllPointClouds();
    visualizer.removeAllShapes();


    switch(dispMode) {
    case SHOW_UNALIGNED_CLOUDS:
      visualizer.addPointCloud(joined_cloud_, cloudname);
      break;
    case SHOW_ALIGNED_CLOUDS:
      visualizer.addPointCloud(aligned_joined_cloud_, cloudname);
      break;
    case SHOW_DOWNSAMPLED_OTHER_CAS_CLOUD:
      visualizer.addPointCloud(other_cas_cloud_downsampled_, cloudname);
      break;
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BeliefStateSceneRefinement)