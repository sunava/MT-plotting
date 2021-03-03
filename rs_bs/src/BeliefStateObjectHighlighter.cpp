#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/utils/common.h>

#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/compare.h>

#include <rapidjson/document.h>

// RS_BS
#include <rs_bs/types/all_types.h>
#include <rs_bs/ViewNames.h>
#include <rs_bs/BeliefStateRepresentation.h>
#include <rs_bs/BeliefStateCommunication.h>

using namespace uima;


class BeliefStateObjectHighlighter : public DrawingAnnotator
{
private:
  typedef pcl::PointXYZRGBA PointT;

  cv::Mat object_, rgb_, depth_;
  cv::Mat rgb_out_;

  pcl::PointCloud<PointT>::Ptr bs_cloud_;

  pcl::PointCloud<PointT>::Ptr viz_cloud_ptr_;

  double point_size_ = 1.0;

  // This contains the simulation actor IDs (UE4)!
  std::vector<std::string> belief_state_objects_to_display;
  int belief_state_object_counter_ = 0;

  std::vector<pcl::PointIndices::Ptr> cluster_indices_;

  std::vector<rs_bs::BeliefStateObject> bs_objs;

  bool use_hd_images_ = false;

  enum
  {
    CYCLE_THROUGH_OBJECTS,
    SHOW_BOXES,
    SHOW_FOOD,
    SHOW_DRINK
  } dispMode;


public:
  BeliefStateObjectHighlighter() : DrawingAnnotator(__func__), dispMode(CYCLE_THROUGH_OBJECTS)
  {

  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
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

  /**
   * Take the first color histograms of two clusters and compare them.
   *
   * @param c1 An Object that is a subtype of rs::ObjectHypothesis.
   * @param c2 An Object that is a subtype of rs::ObjectHypothesis
   * @return 0 if either c1 or c2 doesn't contain color annotations.
   *         Otherwise it returns the output of rs::compare for both color histograms.
   */
  double compareColorHistogramOnCluster(rs::ObjectHypothesis &c1, rs::ObjectHypothesis &c2)
  {
    std::vector<rs::ColorHistogram> colors1;
    c1.annotations.filter(colors1);

    std::vector<rs::ColorHistogram> colors2;
    c2.annotations.filter(colors2);

    if(colors1.size() == 0) {
      outWarn("No color annotations on c1");
      return 0;
    }
    if(colors2.size() == 0) {
      outWarn("No color annotations on c2");
      return 0;
    }

    return rs::compare(colors1[0], colors2[0]);
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

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
    bs_cloud_ =  pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    viz_cloud_ptr_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    cas.get(VIEW_CLOUD, *bs_cloud_);
    cas.get(VIEW_CLOUD, *viz_cloud_ptr_);

    rgb_out_ = rgb_.clone();

    belief_state_object_counter_ = 0;
    belief_state_objects_to_display.clear();
    bs_objs.clear();

    cas.get(VIEW_ALL_BS_OBJECTS, bs_objs);
    outInfo("Found "<<bs_objs.size()<<" Belief State Objects");

    rs::Query query = rs::create<rs::Query>(tcas);
    cas.getFS("BS_QUERY", query);
    std::string query_string = query.query();

    outInfo("Read query from BS_QUERY: " << query_string);
    if(query_string == "")
    {
      outInfo("took: " << clock.getTime() << " ms.");
      return UIMA_ERR_NONE;
    }

    rapidjson::Document jsonDoc;
    jsonDoc.Parse(query_string.c_str());

    if(jsonDoc.HasMember("type"))
    {
      rapidjson::Value::ConstMemberIterator colorMember = jsonDoc.FindMember("type");
      std::string typeValue = jsonDoc["type"].GetString();
      outInfo("The following type has been requested: " << typeValue);

      select_bs_objects_to_display_for_property(typeValue);
    }


    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void drawCluster(cv::Rect roi, const std::string &label)
  {
    cv::rectangle(rgb_out_, roi, CV_RGB(255, 0, 0));
    int offset = 7;
    int baseLine;
    cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 0.8, 1, &baseLine);
    cv::putText(rgb_out_, label, cv::Point(roi.x + (roi.width - textSize.width) / 2, roi.y - offset - textSize.height), cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(255, 255, 200), 1.0);
  }

  void drawImageWithLock(cv::Mat &disp) {
    rgb_out_ = rgb_.clone();

    outInfo("drawImageWithLock in BSOHighlighter with " << belief_state_objects_to_display.size() << " objects");

    for(const auto actorIdNameOfSimObject : belief_state_objects_to_display)
    {
      bool bsObjectFound = false;
      for(auto bsObject : bs_objs) {
        if (bsObject.simulationActorId() != actorIdNameOfSimObject)
          continue;

        bsObjectFound = true;

        // Draw the ROI for visualization
        cv::Rect rect;
        if(use_hd_images_)
        {
          rs::conversion::from(bsObject.rois().roi_hires(), rect);
        }
        else
        {
          rs::conversion::from(bsObject.rois().roi(), rect);
        }

        drawCluster(rect, bsObject.simulationActorId());
      }
      if(!bsObjectFound)
      {
        outError(actorIdNameOfSimObject <<
                 " was supposed to be highlighted but couldn't be found in the Belief State Object View");
      }
    }
    disp = rgb_out_;
  }

  bool callbackKey(const int key, const Source source)
  {
    belief_state_objects_to_display.clear();
    outInfo("BSR in callbackKey: " << rs_bs::BeliefStateRepresentation::getInstance().toString());
    outInfo("BSC in callbackKey: " << BeliefStateCommunication::getInstance().mapsToString());

//    /**
//     * TODO: REMOVE BS EXPERIMENTS
//     */
//    if(key == 'r')
//    {
//      rerun = rerun ? false : true;
//      outWarn("Set AAE RERUN to =" << rerun);
//    }

    if(bs_objs.size() == 0)
    {
      outWarn("No BS Objects found in BeliefStateObjectHighlighter. Nothing to visualize");
      return true;
    }

    switch(key)
    {
    case 'a':
      dispMode = CYCLE_THROUGH_OBJECTS;
      belief_state_object_counter_ == 0
          ? belief_state_object_counter_ = bs_objs.size() - 1 : belief_state_object_counter_--;

      belief_state_objects_to_display.push_back( bs_objs.at(belief_state_object_counter_).simulationActorId() );
      break;
    case 'd':
      dispMode = CYCLE_THROUGH_OBJECTS;
      belief_state_object_counter_ == bs_objs.size() - 1
          ? belief_state_object_counter_ = 0: belief_state_object_counter_++;

      belief_state_objects_to_display.push_back( bs_objs.at(belief_state_object_counter_).simulationActorId() );
      break;
    case 'b':
      dispMode = SHOW_BOXES;
      select_bs_objects_to_display_for_property("box");
      break;
    case 'v':
      dispMode = SHOW_FOOD;
      select_bs_objects_to_display_for_property("food");
      break;
    case 'c':
      dispMode = SHOW_DRINK;
      select_bs_objects_to_display_for_property("drink");
      break;
    }

    return true;
  }

  void select_bs_objects_to_display_for_property(std::string property) {
    for(const auto rs_obj_id : rs_bs::BeliefStateRepresentation::getInstance().has_property(property))
    {
      std::string object_name_in_game_engine;
      if(!rs_bs::BeliefStateRepresentation::getInstance().getObjectNameInGameEngine(rs_obj_id, object_name_in_game_engine))
      {
        outWarn("Could find Object Name in Game Engine. Can't visualize it.");
        continue;
      }
      belief_state_objects_to_display.push_back(object_name_in_game_engine);
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) override
  {
    const std::string &cloudname = this->name;
    outInfo("Size of cloud-ptr->points : " << viz_cloud_ptr_->points.size());

    cluster_indices_.clear();
    // Restore the colors in the viz cloud:
    for(int idx = 0; idx < bs_cloud_->points.size(); idx++)
    {
      viz_cloud_ptr_->points[idx].rgba = bs_cloud_->points[idx].rgba;
      viz_cloud_ptr_->points[idx].a = bs_cloud_->points[idx].a;
    }

    for(const auto actorIdNameOfSimObject : belief_state_objects_to_display) {
      for (auto bsObject : bs_objs) {
        if (bsObject.simulationActorId() != actorIdNameOfSimObject)
          continue;

        pcl::PointIndices::Ptr cluster_indices_for_bs_object(new pcl::PointIndices);
        rs::ReferenceClusterPoints clusterpoints(bsObject.points());
        rs::conversion::from(clusterpoints.indices(), *cluster_indices_for_bs_object);
        cluster_indices_.push_back(cluster_indices_for_bs_object);
      }
    }

    for(size_t i = 0; i < cluster_indices_.size(); ++i) {
      outInfo("Coloring " << cluster_indices_.size() << " clusters");
      const pcl::PointIndices &point_indices_for_single_cluster =
          *cluster_indices_[i];

      for (size_t idx : point_indices_for_single_cluster.indices) {
        viz_cloud_ptr_->points[idx].rgba =
            rs::common::colors[i % rs::common::numberOfColors];
        viz_cloud_ptr_->points[idx].a = 255;
      }
    }

    if(firstRun)
    {
      visualizer.addPointCloud(viz_cloud_ptr_, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(viz_cloud_ptr_, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, cloudname);
    }
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BeliefStateObjectHighlighter)