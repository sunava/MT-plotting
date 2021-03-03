#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>

#include "world_control_msgs/SetModelPose.h"
#include "rs_bs/BeliefStateCommunication.h"

#include <rapidjson/document.h>

using namespace uima;


/**
 * This Annotator looks up the camera pose from the CAS identified by otherCASId.
 * It will use this pose to adjust the camera at the same pose in the belief state.
 *
 * ROS Parameters exists (prefixed with rs_bs_ , try rosparam set rs_bs<TAB><TAB> to see a list)
 * that allow you to modify the camera pose of the belief state camera before
 * requesting the sensor data.
 *
 * If query_aware_ is set to true, this Annotator is analyzing the Query
 * which is fetched by BeliefStateQueryHandler (see rs_bs_query package).
 * Do change the BS camera pose from a query, send the following (exemplary )
 * service request to the BeliefStateQueryHandler:
 *   '{"camera" : {"x":-0.4, "y":0, "z":0, "pitch":0.2, "yaw":0.4}}'
 *
 */
class BeliefStateCameraUpdater : public Annotator
{
private:
  // This id is used to find the camera in the UE4 beliefstate among all other
  // UE4 actors. It's a usemlog tag from the uutils package.
  // Example in UE4:
  //   SemLog;id,urobovision_camera;
  // In that case, bs_camera_id_ should be set to urobovision_camera
  std::string bs_camera_id_;

  // This identifier is used to reference the CAS of another AAE that ran before
  // *this* AAE and holds the viewpoint information
  std::string other_cas_id;

  // If this is set to true, the camera updater will check if a belief state query
  // exists for the current run. It will then check it for any camera-related requests
  bool query_aware_ = true;

  ros::NodeHandle n;

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("bsCameraId", bs_camera_id_);
    ctx.extractValue("otherCASId", other_cas_id);

    outInfo("Loading the default transform onto the ROS parameter server");
    tf::Transform worldToUE4 = BeliefStateCommunication::getInstance().getTransformBetweenWorldAndUE4();
    ros::param::set("ue4_translation_x", worldToUE4.getOrigin()[0]);
    ros::param::set("ue4_translation_y", worldToUE4.getOrigin()[1]);
    ros::param::set("ue4_translation_z", worldToUE4.getOrigin()[2]);

    ros::param::set("ue4_rotation_x", worldToUE4.getRotation().getX());
    ros::param::set("ue4_rotation_y", worldToUE4.getRotation().getY());
    ros::param::set("ue4_rotation_z", worldToUE4.getRotation().getZ());
    ros::param::set("ue4_rotation_w", worldToUE4.getRotation().getW());



    ros::param::set("rs_bs_offset_translation_x", 0);
    ros::param::set("rs_bs_offset_translation_y", 0);
    ros::param::set("rs_bs_offset_translation_z", 0);

//    ros::param::set("rs_bs_rotation_x", 0);
//    ros::param::set("rs_bs_rotation_y", 0);
//    ros::param::set("rs_bs_rotation_z", 0);
//    ros::param::set("rs_bs_rotation_w", 0);

    ros::param::set("rs_bs_pitch", 0);
    ros::param::set("rs_bs_yaw", 0);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec) {
    rs::StopWatch clock;
    outInfo("Fetching latest ue4 transform");

    float trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w;

    ros::param::get("ue4_translation_x", trans_x);
    ros::param::get("ue4_translation_y", trans_y);
    ros::param::get("ue4_translation_z", trans_z);

    ros::param::get("ue4_rotation_x", rot_x);
    ros::param::get("ue4_rotation_y", rot_y);
    ros::param::get("ue4_rotation_z", rot_z);
    ros::param::get("ue4_rotation_w", rot_w);
    BeliefStateCommunication::getInstance().updateTransformBetweenWorldAndUE4(
        trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w);

    outInfo("Updating camera '" << bs_camera_id_ << "' in Belief State");

    float bs_camera_offset_x, bs_camera_offset_y, bs_camera_offset_z;
    ros::param::get("rs_bs_offset_translation_x", bs_camera_offset_x);
    ros::param::get("rs_bs_offset_translation_y", bs_camera_offset_y);
    ros::param::get("rs_bs_offset_translation_z", bs_camera_offset_z);

    float bs_pitch, bs_yaw;
    ros::param::get("rs_bs_pitch", bs_pitch);
    ros::param::get("rs_bs_yaw", bs_yaw);

    uima::CAS *other_cas;
    other_cas = rs::CASConsumerContext::getInstance().getCAS(other_cas_id);
    if (!other_cas) {

      outWarn("Couldn't fetch CAS identified by '"
              << other_cas_id
              << "'. Make sure you have loaded an AAE with that name and "
              << " that you've set 'otherCASId' in this config");
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }

    rs::SceneCas other_cas_scene_cas(*other_cas);
    rs::Scene other_cas_scene = other_cas_scene_cas.getScene();

    tf::StampedTransform camToWorld, worldToCam;
    tf::StampedTransform ue4ToWorld;

    camToWorld.setIdentity();
    if (other_cas_scene.viewPoint.has()) {
      rs::conversion::from(other_cas_scene.viewPoint.get(), camToWorld);
    } else {
      outWarn("No camera to world transformation, no further processing!");
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }

    // Please note that the data from the Robot kinect is usually recorded in the rgb optical frame. The UE4 camera equivalent is the rgb LINK frame. So we have to respect this transform also.
    tf::Transform kinect_optical_to_rgb_link;
    tf::Vector3 kinect_optical_to_rgb_link_loc(0, 0, 0);
    tf::Quaternion kinect_optical_to_rgb_link_rot(-0.5, 0.5, -0.5, 0.5);
    kinect_optical_to_rgb_link.setOrigin(kinect_optical_to_rgb_link_loc);
    kinect_optical_to_rgb_link.setRotation(kinect_optical_to_rgb_link_rot);

    tf::Transform worldToUE4 = BeliefStateCommunication::getInstance()
                                   .getTransformBetweenWorldAndUE4();

    // Transform from RGB optical frame coordinates to ue4 world coordinates
    tf::Transform ue4_cam_pose =
        worldToUE4 * camToWorld * kinect_optical_to_rgb_link.inverse();

    tf::Transform &ue4trans = ue4_cam_pose;

    geometry_msgs::Pose p;
    p.position.x = ue4trans.getOrigin().getX();
    p.position.y = ue4trans.getOrigin().getY();
    p.position.z = ue4trans.getOrigin().getZ();
    p.orientation.x = ue4trans.getRotation().getX();
    p.orientation.y = ue4trans.getRotation().getY();
    p.orientation.z = ue4trans.getRotation().getZ();
    p.orientation.w = ue4trans.getRotation().getW();

    // Check if a query is available that is supposed to change the belief state
    // camera pose.
    // Please note that the values of the query are also written onto the ROS
    // parameter server for the corresponding variables.
    if(query_aware_)
    {

      rs::Query query = rs::create<rs::Query>(tcas);
      rs::SceneCas cas(tcas);
      cas.getFS("BS_QUERY", query);
      std::string query_string = query.query();

      outInfo("Read query from BS_QUERY: " << query_string);
      if(query_string != "") {

        rapidjson::Document jsonDoc;
        jsonDoc.Parse(query_string.c_str());

        if (jsonDoc.HasMember("camera")) {

          rapidjson::Value::ConstMemberIterator cameraMember =
              jsonDoc.FindMember("camera");
          if(
              cameraMember->value.HasMember("x") &&
              cameraMember->value.HasMember("y") &&
              cameraMember->value.HasMember("z") &&
              cameraMember->value.HasMember("pitch") &&
              cameraMember->value.HasMember("yaw")
              ){
            bs_camera_offset_x = cameraMember->value["x"].GetFloat();
            bs_camera_offset_y = cameraMember->value["y"].GetFloat();
            bs_camera_offset_z = cameraMember->value["z"].GetFloat();
            bs_pitch = cameraMember->value["pitch"].GetFloat();
            bs_yaw = cameraMember->value["yaw"].GetFloat();

            ros::param::set("rs_bs_offset_translation_x", bs_camera_offset_x);
            ros::param::set("rs_bs_offset_translation_y", bs_camera_offset_y);
            ros::param::set("rs_bs_offset_translation_z", bs_camera_offset_z);

            ros::param::set("rs_bs_pitch", bs_pitch);
            ros::param::set("rs_bs_yaw", bs_yaw);
          }
          else
          {
            outWarn("Incomplete 'camera' query received. One of the values is missing");
          }
        }
      }
    }

    if( bs_camera_offset_x != 0 ||
        bs_camera_offset_y != 0 ||
        bs_camera_offset_z != 0)
    {
      outInfo("Applying offset to Belief State Camera");
      p.position.x += bs_camera_offset_x;
      p.position.y += bs_camera_offset_y;
      p.position.z += bs_camera_offset_z;
    }


    // Read parameter for head angle changes
    if (bs_pitch != 0 || bs_yaw != 0)
    {
      tf::Quaternion q_scene(ue4trans.getRotation().getX(),
                             ue4trans.getRotation().getY(),
                             ue4trans.getRotation().getZ(),
                             ue4trans.getRotation().getW());
      tf::Quaternion q_pitch;
      q_pitch.setRPY(0,bs_pitch,0);
      tf::Quaternion q_yaw;
      q_yaw.setRPY(0,0,bs_yaw);

      tf::Quaternion q_new_orientation = q_scene * q_yaw * q_pitch;
      p.orientation.x = q_new_orientation.getX();
      p.orientation.y = q_new_orientation.getY();
      p.orientation.z = q_new_orientation.getZ();
      p.orientation.w = q_new_orientation.getW();
    }

    // UE4 world coordinates when standing at the kitchen counter sink:
    // Z left/right
    // Y up/down
    // X tilt
    outInfo("Setting UE4 camera pose to: " << p << "in ue4 world coordinates (with ROS coordinate conventions");
    BeliefStateCommunication::getInstance().setCameraPose(p);

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BeliefStateCameraUpdater)