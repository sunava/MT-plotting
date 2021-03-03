//
// Created by pmania
//

#ifndef SRC_BELIEFSTATECOMMUNICATION_H
#define SRC_BELIEFSTATECOMMUNICATION_H

#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>

#include "world_control_msgs/SpawnModel.h"
#include "world_control_msgs/SetModelPose.h"
#include "unreal_interface/object.h"


/**
 * This class is an interface for modifications of the belief state environment
 * in a game engine. It should abstract the actions we have to do on the belief
 * state (updating camera poses, updating object poses, deleting objects,
 * keeping track of spawned objects etc.) from the actual perception
 * implementation.
 */
class BeliefStateCommunication
{
public:
  static BeliefStateCommunication& getInstance()
  {
    // Since it's a static variable, if the class has already been created,
    // it won't be created again.
    // And it **is** thread-safe in C++11.
    static BeliefStateCommunication bscInstance;

    // Return a reference to our instance.
    return bscInstance;
  }

  // delete copy and move ctors and assign operators
  BeliefStateCommunication(BeliefStateCommunication const&) = delete;
  BeliefStateCommunication(BeliefStateCommunication&&) = delete;
  BeliefStateCommunication& operator=(BeliefStateCommunication const&) = delete;
  BeliefStateCommunication& operator=(BeliefStateCommunication&&) = delete;

protected:
  BeliefStateCommunication();
  ~BeliefStateCommunication() = default;

  UnrealInterface::Objects uio;

  std::string camera_id_="urobovision_camera";

  tf::Vector3 worldToUE4_loc;
  tf::Quaternion worldToUE4_rot;


public:
    geometry_msgs::Pose savedPose_;

  std::map<std::string, std::string> rs_obj_ids_to_uio_ids;

  std::string mapsToString();

  bool setCameraPose(geometry_msgs::Pose p);

  /**
   * Sets the 'domain' of the ros world actor control.
   * The given parameter will be used as the prefix of the RosWorldControl
   * service calls.
   *
   * @param domain The prefix for the RWC service calls
   */
  void setDomainName(std::string domain)
  {
    uio.SetUROSWorldControlDomain(domain);
  }

  void setCameraID(std::string camera_id)
  {
    camera_id_ = camera_id;
  }


  /**
   * This transform is used if you want to calculate the pose of the real world camera
   * to UE4 or if you want to transform the object poses from the RW detections into
   * UE4 coordinates.
   * This transform is related to the RobCog UE4 map of our kitchen.
   *
   * @return
   */
  tf::Transform getTransformBetweenWorldAndUE4() {
    tf::Transform worldToUE4;
    worldToUE4.setOrigin(worldToUE4_loc);
    worldToUE4.setRotation(worldToUE4_rot);

    return worldToUE4;
  }

  void updateTransformBetweenWorldAndUE4(float trans_x, float trans_y, float trans_z,
                                         float rot_x, float rot_y, float rot_z, float rot_w){
      worldToUE4_loc = tf::Vector3(trans_x, trans_y, trans_z);
      worldToUE4_rot = tf::Quaternion(rot_x, rot_y, rot_z, rot_w);
  }

  /**
   * Low-level function to interface the belief state and spawn a single object.
   * This function DOES NOT implement sanity checks, but just publishes
   * the given model and keeps track of it in UnrealInterfaceObject
   *
   * @param model The model that should be spawned in the Belief State.
   *        id_of_spawned_objects If a pointer is given and the service call is successful,
   *                   the method will place the returned id of the spawned object in thatpointer.
   * @return false if an error has occured, true otherwise.
   */
  bool spawnObject(world_control_msgs::SpawnModel model, std::string *id_of_spawned_object);

  bool deleteObject(std::string id);

  bool setObjectPose(world_control_msgs::SetModelPose);

  /**
   * Returns the last published pose from UE4 which should be published
   * in regular intervals by UnrealObjectInfoPublisher.
   * If you really need the *current* pose, please use getObjectPose instead.
   *
   * @param rs_id RoboSherlock Object ID of the object of interest
   * @return The desired pose.
   *             Only set if atleast one pose update was received.
   *             The pose is in the UE4 world frame.
   */
  geometry_msgs::Pose getLatestObjectPose(std::string rs_id);
  geometry_msgs::Pose returnSavePose();


  /**
   *
   * @param rs_id RoboSherlock Object ID of the object of interest
   * @param pose The pose is in the UE4 world frame.
   * @return true if service call was successful
   */
  bool getObjectPose(std::string rs_id, geometry_msgs::Pose &pose);

  void deleteAllSpawnedObjects();

  /**
   * Clear objects in the simulator.
   * This will also clear objects that have not been spawned in *this* instance
   * of the BeliefStateCommunication (e.g. after you restart RoboSherlock and
   * objects are left in UE4)
   */
  void clearSpawnedObjects();


};

#endif // SRC_BELIEFSTATECOMMUNICATION_H
