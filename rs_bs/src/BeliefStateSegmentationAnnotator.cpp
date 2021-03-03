#include <algorithm>
#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
#include <rs_bs/types/all_types.h>
//RS
#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/common.h>
#include <robosherlock/utils/time.h>

#include <rs_bs/BeliefStateCommunication.h>
#include <rs_bs/BeliefStateRepresentation.h>
#include <rs_bs/ViewNames.h>

using namespace uima;


/**
 * This Annotator reads the VIEW_OBJECTS from other CAS and tries
 * to segment the matching objects from the UE4 Belief State.
 * Please note that this is currently tailored towards Kinect-Style cameras
 * which are 4:3, have low and hi-res and the pointcloud data is only generated
 * on the low-res depth/color data!
 */
class BeliefStateSegmentationAnnotator : public DrawingAnnotator
{
private:
  typedef pcl::PointXYZRGBA PointT;

  cv::Mat object_, rgb_;
  pcl::PointCloud<PointT>::Ptr cloud_ptr_;
  pcl::PointCloud<PointT>::Ptr viz_cloud_ptr_;
  double point_size_ = 1.0;

  std::vector<pcl::PointIndices::Ptr> cluster_indices_;
  std::map<std::string, pcl::PointIndices::Ptr> cluster_indices_map_;

  std::vector<std::string> actor_names_of_detected_objects_;
  std::vector<std::string> actor_names_of_nondetected_objects_;

  std::string other_cas_id_;
  bool use_hd_images_ = false;

  bool use_hd_object_mask_for_segmentation_ = false;

  enum
  {
    HIGHLIGHT_ONLY_DETECTED_OBJECTS,
    HIGHLIGHT_ONLY_NON_DETECTED_OBJECTS,
    HIGHLIGHT_ALL_OBJECTS
  } dispMode;


  struct ObjectMaskSegmentationData
  {
      cv::Rect roi;
      cv::Mat mask = cv::Mat::zeros(3,3, CV_8UC1);
      pcl::PointIndices point_indices;
      int number_of_pixels;
  };

  // Define our own comparison function to make vec3b usable as a key in maps
  // Idea: Use the 3 bytes and construct a single 4-byte value from that
  struct Vec3bCompare
    {
        bool operator() (const cv::Vec3b& lhs, const cv::Vec3b& rhs) const
        {
            uint32_t lhs_one = (lhs.val[0] << 16) + (lhs.val[1] << 8) + (lhs.val[2]);
            uint32_t rhs_one = (rhs.val[0] << 16) + (rhs.val[1] << 8) + (rhs.val[2]);
            return lhs_one < rhs_one;
        }
    };

//    unsigned char blue   = vec.val[0];
//    unsigned char green  = vec.val[1];
//    unsigned char red    = vec.val[2];
  // This variable maps a single RGB color vector to a struct containg
  // information about this color in the full object mask.
  std::map<cv::Vec3b, ObjectMaskSegmentationData, Vec3bCompare> colorToSegmentationData;

  void clearObjectMaskSegmentation()
  {
      colorToSegmentationData.clear();
  }

  void printObjectMaskSegmentationData()
  {
      outInfo("Printing object mask stats: ");
      outInfo(" Number of unique colors: " << colorToSegmentationData.size());
      for(auto pair : colorToSegmentationData)
      {
        unsigned char blue   = pair.first.val[0];
        unsigned char green  = pair.first.val[1];
        unsigned char red    = pair.first.val[2];
        outInfo("Color(RGB)" << ((int)red) << " " << ((int)green) << " " << ((int)blue));

        auto& data = colorToSegmentationData[pair.first];
        outInfo("  Nr. of Pixels:" << data.number_of_pixels);
        outInfo("    ROI x,y,width,height: " << data.roi.x << "," << data.roi.y <<
                                    " " << data.roi.width << "x" << data.roi.height);
      }
  };

  /**
   * This method takes an object mask and iterates over it pixel by pixel.
   * For each color it encounters in the mask, it creates a
   * segmentation data struct that holds information like the ROI of the color
   * or the point indices.
   * This method could then be used to get the segmentation info
   * for each of the presented objects in the mask easily without re-iterating over
   * the object mask.
   *
   * @param object_mask blue   = vec.val[0], vec.val[2]
   */
  void computeObjectMaskSegmentationData(cv::Mat &object_mask)
  {
      for(int i = 0; i < object_mask.rows; i++)
      {
          const cv::Vec3b* pixel = object_mask.ptr<cv::Vec3b>(i);
          for(int j = 0; j < object_mask.cols; j++){
              // Optimize this lookup. You don't wanna call this everytime
              // in this loop.
              // Find a good way to init the color map with the right dimensions
              // and for the right colors before running this loop.
              if(colorToSegmentationData.count(pixel[j]) == 0)
              {
                  // Init the mask
                  colorToSegmentationData[pixel[j]].mask = cv::Mat::zeros(object_mask.rows,
                          object_mask.cols, CV_8U);
                  colorToSegmentationData[pixel[j]].roi = cv::Rect(j,i,1,1);
              }
              else
              {
                  // TODO This block needs a review
                  auto& segmentation_data = colorToSegmentationData[pixel[j]];

                  // If the value already exists, we have to adjust the ROI


                  if(j < segmentation_data.roi.x)
                  {
                      // Extend to the left
                      segmentation_data.roi.width += segmentation_data.roi.x - j;
                      segmentation_data.roi.x = j;
                  }
                  else if(j >= (segmentation_data.roi.x + segmentation_data.roi.width))
                  {
                    // Extend to the right
                      segmentation_data.roi.width = j - segmentation_data.roi.x + 1;
                  }

                  // TODO Check this. When using Contours + boundingRect the height is 2px higher.
                  // Looking at the crops and overlaying them on the original image, this seems to be correct.
                  segmentation_data.roi.height = i - segmentation_data.roi.y + 1;

              }

              colorToSegmentationData[pixel[j]].number_of_pixels++;

              // Fill the (binary) object mask for the individual object.
              auto *ptr_to_mask = colorToSegmentationData[pixel[j]].mask.ptr<uchar>(i);
              ptr_to_mask[j] = 255;

              // Save the pointcloud indices
              size_t index = i * object_mask.cols + j;
              colorToSegmentationData[pixel[j]].point_indices.indices.push_back(index);

          }
      }
  }


public:
  BeliefStateSegmentationAnnotator(): DrawingAnnotator(__func__), dispMode(HIGHLIGHT_ONLY_DETECTED_OBJECTS)
  {

  }

  TyErrorId initialize(AnnotatorContext &ctx) override
  {
    ctx.extractValue("otherCASId", other_cas_id_);

    if(ctx.isParameterDefined("useHDImages"))
    {
      ctx.extractValue("useHDImages", use_hd_images_);
      outInfo("Use HD Images: " << use_hd_images_);
    }

    if(ctx.isParameterDefined("useHDObjectMaskForSegmentation"))
    {
      ctx.extractValue("useHDObjectMaskForSegmentation", use_hd_object_mask_for_segmentation_);
      outInfo("Use HD object mask images: " << use_hd_object_mask_for_segmentation_);
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy() override
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  std::string getActorSimNameByColor(std::map<std::string, cv::Vec3b> &objectMap,
                                     unsigned char blue,
                                     unsigned char green,
                                     unsigned char red){
    // find actor sim name for color
    for(auto name_color_pair : objectMap)
    {
      unsigned char blueInObjectMap   = name_color_pair.second.val[0];
      unsigned char greenInObjectMap  = name_color_pair.second.val[1];
      unsigned char redInObjectMap    = name_color_pair.second.val[2];
      if(blue == blueInObjectMap &&
         green == greenInObjectMap &&
         red == redInObjectMap) {
        return name_color_pair.first;
      }
    }
    return "NotFoundInGetActorSimNameByColor";
  }

  /**
   * Iterate over the full object mask and annotate every visible object in the mask
   * @param tcas
   */
  void generateBeliefStateObjectsFromSegmentationData(CAS &tcas,
                                                      rs::SceneCas &cas,
                                                      rs::Scene &scene,
                                                      std::map<std::string, cv::Vec3b> &objectMap){

    std::vector<rs_bs::BeliefStateObject> all_beliefstate_objects;

    for(auto pair : colorToSegmentationData)
    {
      unsigned char blue   = pair.first.val[0];
      unsigned char green  = pair.first.val[1];
      unsigned char red    = pair.first.val[2];
//      outInfo("Color(RGB)" << ((int)red) << " " << ((int)green) << " " << ((int)blue));

      auto& data = colorToSegmentationData[pair.first];
      std::string actorSimName = getActorSimNameByColor(objectMap,blue,green,red);

      rs::ImageROI imageROI = rs::create<rs::ImageROI>(tcas);

      //
      // ROI/MASK Extraction after Object Segmentation
      //
      // **WARN**: This code was mainly written for kinect-style cameras!
      //       At some places it is assumed that HD quality is simply the
      //       double resolution of the normal quality. This is NOT true for many
      //       modern cameras. So use with care.
      //

      // Everything is in HD. Downsize the HD information to normal ROIS etc. by simply downscaling everything by half
      if(use_hd_images_ && use_hd_object_mask_for_segmentation_)
      {
        cv::Rect roi_hires = data.roi;
        cv::Rect roi_normal = cv::Rect(roi_hires.x >> 1, roi_hires.y >> 1, roi_hires.width >> 1, roi_hires.height >> 1);
        // mask_hires is segmented_object_from_mask
        cv::Mat mask_normal;
        cv::resize(data.mask, mask_normal, cv::Size(0, 0), 0.5, 0.5, cv::INTER_NEAREST);

        imageROI.mask_hires(rs::conversion::to(tcas, data.mask(roi_hires)));
        imageROI.mask(rs::conversion::to(tcas, mask_normal(roi_normal)));

        imageROI.roi_hires(rs::conversion::to(tcas, roi_hires));
        imageROI.roi(rs::conversion::to(tcas, roi_normal));
      }
      else if(use_hd_images_ && !use_hd_object_mask_for_segmentation_)
      {
        // We have extracted the segmentation info from low-res object masks
        // Upscale ROIs etc.
        cv::Rect roi_normal = data.roi;
        cv::Rect roi_hires = cv::Rect(roi_normal.x << 1, roi_normal.y << 1, roi_normal.width << 1, roi_normal.height << 1);

        cv::Mat mask_normal = data.mask;
        cv::Mat mask_hires;
        cv::resize(data.mask, mask_hires, cv::Size(0, 0), 2, 2, cv::INTER_NEAREST);

        imageROI.mask_hires(rs::conversion::to(tcas, mask_hires(roi_hires)));
        imageROI.mask(rs::conversion::to(tcas, mask_normal(roi_normal)));

        imageROI.roi_hires(rs::conversion::to(tcas, roi_hires));
        imageROI.roi(rs::conversion::to(tcas, roi_normal));
      }
      else
      {
        // We only fill the standard mask/roi attributes when HD mode is off
        imageROI.mask(rs::conversion::to(tcas, data.mask(data.roi)));
        imageROI.roi(rs::conversion::to(tcas, data.roi));
      }

      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      data.point_indices.header = cloud_ptr_->header;
      rs::PointIndices uimaIndices = rs::conversion::to(tcas, data.point_indices);
      rcp.indices.set(uimaIndices);

      rs_bs::BeliefStateObject bsObject = rs::create<rs_bs::BeliefStateObject>(tcas);

      bsObject.rois.set(imageROI);
      bsObject.source.set("BeliefStateSegmentationAnnotator");

      // Write already available information to the annotation
      bsObject.mask_color_b.set(blue);
      bsObject.mask_color_g.set(green);
      bsObject.mask_color_r.set(red);
      bsObject.simulationActorId.set(actorSimName);
      bsObject.points.set(rcp);

      all_beliefstate_objects.push_back(bsObject);
    }

    cas.set(VIEW_ALL_BS_OBJECTS, all_beliefstate_objects);
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) override
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    // Fetch sensor data from BS environment
    if(use_hd_images_) {
      cas.get(VIEW_COLOR_IMAGE_HD, rgb_);
    }
    else
    {
      cas.get(VIEW_COLOR_IMAGE, rgb_);
    }

    // Right now, the object mask is only used for segmentation
    // As this resolution might
    if(use_hd_object_mask_for_segmentation_)
    {
      cas.get(VIEW_OBJECT_IMAGE_HD, object_);
    }
    else
    {
      cas.get(VIEW_OBJECT_IMAGE, object_);
    }

    cloud_ptr_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    viz_cloud_ptr_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    cas.get(VIEW_CLOUD, *cloud_ptr_);
    cas.get(VIEW_CLOUD, *viz_cloud_ptr_);
    outInfo("Size of cloud-ptr->points : " << cloud_ptr_->points.size());

    std::map<std::string, cv::Vec3b> objectMap;
    cas.get(VIEW_OBJECT_MAP, objectMap);

    // Look up other CAS to know which objects from the BS sensor data is relevant.
    uima::CAS *other_cas;
    other_cas = rs::CASConsumerContext::getInstance().getCAS(other_cas_id_);
    if (!other_cas) {

      outWarn("Couldn't fetch CAS identified by '"
                  << other_cas_id_
                  << "'. Make sure you have loaded an AAE with that name and "
                  << " that you've set 'otherCASId' in this config");
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }

    rs::SceneCas other_cas_scene_cas(*other_cas);
    rs::Scene other_cas_scene = other_cas_scene_cas.getScene();

    std::vector<rs::Object> other_cas_object_annotations;
    other_cas_scene_cas.get(VIEW_OBJECTS, other_cas_object_annotations);
    outInfo("Found " << other_cas_object_annotations.size() << " object hypotheses in CAS '" << other_cas_id_ << "'");

    // Clear found clusters from last run
    cluster_indices_.clear();
    cluster_indices_map_.clear();
    actor_names_of_detected_objects_.clear();
    actor_names_of_nondetected_objects_.clear();


    /**
     * SEGMENTATION ON THE OBJECT MASK
     */
    auto t1 = std::chrono::high_resolution_clock::now();
    clearObjectMaskSegmentation();
    computeObjectMaskSegmentationData(object_);
    generateBeliefStateObjectsFromSegmentationData(tcas, cas, scene, objectMap);
//    printObjectMaskSegmentationData();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    outInfo("SINGLE Object Mask stuff took: " << duration << "ms");
    /**
     * SEGMENTATION ON THE OBJECT MASK DONE
     */

    processDataForVisualization(cas, scene, other_cas_object_annotations);
//    rs::ObjectHypothesis objt = rs::create<rs::ObjectHypothesis>(tcas);
//    scene.identifiables.append(objt);

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  /**
   * This method prepares the data structures to visualize different parts of the sensor
   * data and calls the draw method for visualizing the 2D regions of the detected objects
   */

  void processDataForVisualization(
      rs::SceneCas &cas, rs::Scene &scene,
      const std::vector<rs::Object> &other_cas_object_annotations) {
    std::vector<rs_bs::BeliefStateObject> bsobjs;
    cas.get(VIEW_ALL_BS_OBJECTS, bsobjs);
    outInfo("Found "<<bsobjs.size()<<" Belief State Objects");

    // Create a map for all the segmented belief state objects
    for(auto bsObject : bsobjs) {
      pcl::PointIndices::Ptr cluster_indices_for_bs_object(new pcl::PointIndices);
      rs::ReferenceClusterPoints clusterpoints(bsObject.points());
      rs::conversion::from(clusterpoints.indices(), *cluster_indices_for_bs_object);
      cluster_indices_.push_back(cluster_indices_for_bs_object);

      cluster_indices_map_[bsObject.simulationActorId()] = cluster_indices_for_bs_object;
    }

    // UnrealCV maps contain the ID NAME of the actor.
    // You can get this by hovering over an Actor in the UE4 World outlier
    int cluster_id = -1;

    for (auto other_cas_object_annotation : other_cas_object_annotations) {
      cluster_id++;
      outInfo("Object with Cluster ID: " << cluster_id
                                 << " has identifiable id: " << other_cas_object_annotation.id.get());

      std::vector<rs::Classification> classes;
      other_cas_object_annotation.annotations.filter(classes);
      if (classes.size() == 0) {
        outInfo("  This object doesn't have a class assigned. "
                "It's likely that it hasn't been spawned and is not in the BS. Ignoring.");
        continue;
      }
      std::string class_name = classes[0].classname.get();

      std::string actorIdNameOfSimObject;
      if(!rs_bs::BeliefStateRepresentation::getInstance().getObjectNameInGameEngine(other_cas_object_annotation.id.get(), actorIdNameOfSimObject))
      {
        outWarn("Could find Object Name in Game Engine. Can't visualize it.");
        continue;
      }


      // Fetch the desired rs object from the bs objects
      // Put additional information on the object and/or use it for visualization.
      // Add it to the scene identifiables.
      bool bsObjectFound = false;
      for(auto bsObject : bsobjs)
      {
        if(bsObject.simulationActorId() != actorIdNameOfSimObject)
          continue;

        outInfo("Found BS object for " << actorIdNameOfSimObject);

        // Use the same ID for the BS and the RW object
        bsObject.rsObjectId.set(other_cas_object_annotation.id.get());
        // pose of bsObject // TODO - Can we set this at this point in time already?

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

        actor_names_of_detected_objects_.push_back(bsObject.simulationActorId());

        scene.identifiables.append(bsObject);
        // Many of the other annotators in RS are working with rs::ObjectHypothesis.
        // Fill in a casted version of this BS object.
//        scene.identifiables.append(static_cast<rs::ObjectHypothesis>(bsObject));
//        std::vector<rs::ObjectHypothesis> clusters;

        bsObjectFound = true;
      }

      if(!bsObjectFound)
      {
        outError("Couldn't find " << actorIdNameOfSimObject << "Logging segmented images to /tmp.");
//        std::time_t current_time = std::time(nullptr);
//        std::string filepathWithoutExtension = std::string("/tmp/failed_contour_input_img-").append(std::to_string(current_time));
//        cv::imwrite(filepathWithoutExtension + "OBJECTMASK" + ".png", object_);
//        return UIMA_ERR_NONE;
      }
    }

    // Create list of non-detected objects
    outInfo("Iterating over " << cluster_indices_map_.size() << " elements in cluster_indices_map_.");
    for(auto pair : cluster_indices_map_)
    {
      const std::string &key = pair.first;

      // If key is not in in the detected objects list, then add it to the non  -detected objects list.
      if (std::find(actor_names_of_detected_objects_.begin(),
                    actor_names_of_detected_objects_.end(),
                    key) == actor_names_of_detected_objects_.end()) {
        outInfo("Adding " << key << " to the nondetected objects");
        actor_names_of_nondetected_objects_.push_back(key);
      }

    }
  }

  void drawCluster(cv::Rect roi, const std::string &label)
  {
    cv::rectangle(rgb_, roi, CV_RGB(255, 0, 0));
    int offset = 7;
    int baseLine;
    cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 0.8, 1, &baseLine);
    cv::putText(rgb_, label, cv::Point(roi.x + (roi.width - textSize.width) / 2, roi.y - offset - textSize.height), cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(255, 255, 200), 1.0);
  }

  void drawImageWithLock(cv::Mat &disp) override
  {
    // The actual ROI drawing is done in the processWithLock function
    disp = rgb_.clone();
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) override
  {
    const std::string &cloudname = this->name;
    outInfo("Size of cloud-ptr->points : " << viz_cloud_ptr_->points.size());
    outInfo("Visualizing for mode " << dispMode);

    cluster_indices_.clear();
    // Restore the colors in the viz cloud:
    for(int idx = 0; idx < cloud_ptr_->points.size(); idx++)
    {
      viz_cloud_ptr_->points[idx].rgba = cloud_ptr_->points[idx].rgba;
      viz_cloud_ptr_->points[idx].a = cloud_ptr_->points[idx].a;
    }

    for(auto ci_pair : cluster_indices_map_)
    {
      // Create the cluster_indices entries that are relevant
      const std::string &key = ci_pair.first;

      if(     dispMode == HIGHLIGHT_ALL_OBJECTS
          || (dispMode == HIGHLIGHT_ONLY_DETECTED_OBJECTS && std::find(actor_names_of_detected_objects_.begin(),
                                                                  actor_names_of_detected_objects_.end(),
                                                                  key) != actor_names_of_detected_objects_.end())
          || (dispMode == HIGHLIGHT_ONLY_NON_DETECTED_OBJECTS && std::find(actor_names_of_nondetected_objects_.begin(),
                                                                       actor_names_of_nondetected_objects_.end(),
                                                                    key) != actor_names_of_nondetected_objects_.end())
          )
      {
        cluster_indices_.push_back(ci_pair.second);
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
      // TODO
      // C&P from PointCloudClusterExtractor - Should this really be _G_etPCRP?
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, cloudname);
    }
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case '1':
      dispMode = HIGHLIGHT_ONLY_DETECTED_OBJECTS;
      outWarn("Switching to HIGHLIGHT_ONLY_DETECTED_OBJECTS mode");
      break;
    case '2':
      dispMode = HIGHLIGHT_ONLY_NON_DETECTED_OBJECTS;
      outWarn("Switching to HIGHLIGHT_ONLY_NON_DETECTED_OBJECTS");
      break;
    case '3':
      dispMode = HIGHLIGHT_ALL_OBJECTS;
      outWarn("Switching to OBJECT HIGHLIGHT_ALL_OBJECTS");
      break;
    }

    return true;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BeliefStateSegmentationAnnotator)