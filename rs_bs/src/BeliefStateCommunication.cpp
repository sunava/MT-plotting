#include "rs_bs/BeliefStateCommunication.h"

#include "world_control_msgs/SetModelPose.h"
#include "world_control_msgs/DeleteModel.h"

BeliefStateCommunication::BeliefStateCommunication()
{
  ROS_INFO_STREAM("Initializing BeliefStateCommunication ...");
  worldToUE4_loc = tf::Vector3(0.892, -1.75, 0);
  worldToUE4_rot = tf::Quaternion(0, 0, 1, 0);
  // Michaels RobCog Map matches the semantic map
  // worldToUE4_loc = tf::Vector3(0, 0, 0);
  // worldToUE4_rot = tf::Quaternion(0, 0, 0, 1);
}


bool BeliefStateCommunication::setCameraPose(geometry_msgs::Pose p)
{
  // This assumes that the camera has been already created by the given ID
  // either by putting it manually in the level or spawning it.
  return uio.SetObjectPose(static_cast<UnrealInterface::Object::Id>(camera_id_), p);
}

bool BeliefStateCommunication::spawnObject(world_control_msgs::SpawnModel model, std::string *name_of_spawned_object = nullptr)
{
  std::string rs_ob_id = model.request.id;
  UnrealInterface::Object::Id id_of_spawned_object;
  if(!uio.SpawnObject(model, &id_of_spawned_object))
  {
      return false;
  }

  rs_obj_ids_to_uio_ids[rs_ob_id] = id_of_spawned_object;

  std::string name = uio.GetObjectInfo(id_of_spawned_object).actor_name_;

  //print the ID of the spawned hypothesis
  ROS_INFO_STREAM("BeliefStateCommunication::spawnObject: Object spawned with ID " << id_of_spawned_object << " and final actor name: " << name << ". RS Obj ID was: " << rs_ob_id);

  if(name_of_spawned_object)
  {
    *name_of_spawned_object = name;
  }

  return true;
}

bool BeliefStateCommunication::deleteObject(std::string rs_id)
{
  std::cout << "delete object " << rs_id << std::endl;
  UnrealInterface::Object::Id id = rs_obj_ids_to_uio_ids[rs_id];

  if(uio.DeleteObject(static_cast<UnrealInterface::Object::Id>(id)))
  {
    rs_obj_ids_to_uio_ids.erase(rs_id);
    return true;
  }
  return false;
}

geometry_msgs::Pose BeliefStateCommunication::getLatestObjectPose(std::string rs_id)
{
  UnrealInterface::Object::Id id = rs_obj_ids_to_uio_ids[rs_id];

  UnrealInterface::Object::ObjectInfo oi = uio.GetObjectInfo(id);
  return oi.pose_;
}


bool BeliefStateCommunication::getObjectPose(std::string rs_id, geometry_msgs::Pose &pose)
{
    UnrealInterface::Object::Id id = rs_obj_ids_to_uio_ids[rs_id];
    bool miauw = uio.GetObjectPose(id, pose);
    std::cout << "static destructor\n" << pose << std::endl;

    savedPose_ = pose;
    return miauw;
}
geometry_msgs::Pose BeliefStateCommunication::returnSavePose()
{
    return savedPose_;
}



void BeliefStateCommunication::deleteAllSpawnedObjects()
{
  ROS_INFO_STREAM("Deleting all previously spawned objects via UnrealInterface");
//  uio.DeleteAllSpawnedObjects(); // delete all objects iteratively
  uio.DeleteAllSpawnedObjectsByTag(); // delete all objects by tag

  // TODO
  // Check which objects have been really deleted
  // For now, we just clear everything
  rs_obj_ids_to_uio_ids.clear();
}
void BeliefStateCommunication::clearSpawnedObjects()
{
  ROS_INFO_STREAM("Clear all previously spawned objects via UnrealInterface");
  uio.DeleteAllSpawnedObjectsByTag(); // delete all objects by tag
}

bool BeliefStateCommunication::setObjectPose(world_control_msgs::SetModelPose setmodelposesrv) {
  ROS_INFO_STREAM("BeliefStateCommunication::setObjectPose: Send object pose update to UE4");
  UnrealInterface::Object::Id id = setmodelposesrv.request.id;
  geometry_msgs::Pose p = setmodelposesrv.request.pose;

  return uio.SetObjectPose(id,p);
}

std::string BeliefStateCommunication::mapsToString()
{
  std::stringstream s;
  s << "BSC::mapsToString()" << std::endl;
  s << "rs_obj_ids_to_uio_ids: " << std::endl;
  for(auto kvp : rs_obj_ids_to_uio_ids)
  {
    s << kvp.first << "->" << kvp.second << std::endl;
  }

  return s.str();
}
