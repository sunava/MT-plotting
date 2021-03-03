#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <rs_bs/BeliefStateCommunication.h>
#include <rs_bs/BeliefStateRepresentation.h>

using namespace uima;


/**
 * This class will create the connection between the (symbolic) belief
 * and the actual instantiation of it in the UE4/game engine belief state.
 * It offers multiple synchronization strategies to offer flexible synchronization
 * behavior.
 */
class BeliefStateObjectSynchronization : public Annotator {

public:
  enum SynchronizationType {
    DELETE_ALL_AND_RESPAWN,
    SYNCHRONIZATION
  } synchronization_type_;

  // This identifier is used to reference the CAS of another AAE that ran before
  // *this* AAE and holds the viewpoint information
  std::string other_cas_id_;
  float delay_in_seconds_after_sync_ = 0.6;
  bool clear_objects_in_simulation_in_init_ = false;

  TyErrorId initialize(AnnotatorContext &ctx) {
    outInfo("initialize");
    ctx.extractValue("otherCASId", other_cas_id_);
    ctx.extractValue("delayInSecondsAfterSync", delay_in_seconds_after_sync_);
    ctx.extractValue("clearObjectsInSimulationInInit", clear_objects_in_simulation_in_init_);
    synchronization_type_ = SYNCHRONIZATION;
//     synchronization_type_ = DELETE_ALL_AND_RESPAWN;

    if(clear_objects_in_simulation_in_init_)
      BeliefStateCommunication::getInstance().clearSpawnedObjects();

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy() {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  geometry_msgs::Pose rotatePoseAroundAxis(const geometry_msgs::Pose &q, double radian)
  {
    tf::Quaternion r(q.orientation.x,q.orientation.y,q.orientation.z,q.orientation.w);
    r.setRotation(r.getAxis(),r.getAngle()-radian);

    geometry_msgs::Pose result;
    result.position.x = q.position.x;
    result.position.y = q.position.y;
    result.position.z = q.position.z;
    result.orientation.x = r.getX();
    result.orientation.y = r.getY();
    result.orientation.z = r.getZ();
    result.orientation.w = r.getW();

    return result;
  }


  /**
   * This function applies pose corrections based on the class name given in object_name
   * Use this only deliberately if 3d model poses are off or something that's hard to fix.
   * Better rely on fixing your models/outputs of the other models if you can.
   */
  void completePose(geometry_msgs::Pose &p, std::string object_name) {
     if (object_name == "PfannerPfirsichIcetea")
     {
       p = rotatePoseAroundAxis(p, M_PI);
       return;
     }
      if (object_name == "PfannerGruneIcetea")
      {
          p = rotatePoseAroundAxis(p, M_PI);
          return;
      }
//      if (object_name == "VollMilch")
//      {
//          p = rotatePoseAroundAxis(p, M_PI);
//          return;
//      }
    if (object_name == "KoellnMuesliKnusperHonigNuss")
    {
      p = rotatePoseAroundAxis(p, M_PI);
      return;
    }

    if (object_name == "KnusperSchokoKeks")
    {
      p = rotatePoseAroundAxis(p, M_PI);
      return;
    }

      if (object_name == "KoellnMuesliCranberry")
      {
          p = rotatePoseAroundAxis(p, M_PI);
          return;
      }

      if (object_name == "AlbiHimbeerJuice")
      {
          p = rotatePoseAroundAxis(p, M_PI);
          return;
      }

      if (object_name == "VollMilch")
      {
          p = rotatePoseAroundAxis(p, M_PI);
          return;
      }

  }
  geometry_msgs::Pose convertFromRSCameraPose(rs::PoseAnnotation &pose)
  {
    geometry_msgs::Pose p;
    p.position.x = pose.camera.get().translation.get()[0];
    p.position.y = pose.camera.get().translation.get()[1];
    p.position.z = pose.camera.get().translation.get()[2];
    p.orientation.x = pose.camera.get().rotation.get()[0];
    p.orientation.y = pose.camera.get().rotation.get()[1];
    p.orientation.z = pose.camera.get().rotation.get()[2];
    p.orientation.w = pose.camera.get().rotation.get()[3];
    return p;
  }
  /**
   * Apply specific fixes and or completions related to object models
   * before spawning. This could be the addition of necessary information
   * or fixes such as class label rewritings in case they do not match
   * between RS and UE4.
   * @param model The SpawnModel that should be completed
   */
  void completeModelBeforeSpawning(world_control_msgs::SpawnModel &model)
  {
    if(model.request.name=="PfannerGruneIcetea"){
      model.request.material_names={"PfannerGruneIcetea"};
      model.request.material_paths={"/Models/IAIKitchen/Items/PfannerGruneIcetea"};

      return;
    }

    if(model.request.name=="SpitzenReis"){
      model.request.material_names={"SpitzenReis"};
      model.request.material_paths={"/Models/IAIKitchen/Items/SpitzenReis"};
      return;
    }

    if(model.request.name=="KoellnMuesliKnusperHonigNuss"){
      model.request.material_names={"KoellnMuesliKnusperHonigNuss"};
      model.request.material_paths={"/Models/IAIKitchen/Items/KoellnMuesliKnusperHonigNuss"};
      return;
    }

    if(model.request.name=="KelloggsCornFlakes"){
      model.request.material_names={"KelloggsCornFlakes"};
      model.request.material_paths={"/Models/IAIKitchen/Items/KelloggsCornFlakes"};
      return;
    }

    if(model.request.name=="VollMilch"){
      model.request.material_names={"VollMilch"};
      model.request.material_paths={"/Models/IAIKitchen/Items/VollMilch"};
      return;
    }

    if(model.request.name=="ReineButterMilch"){
      model.request.material_names={"ReineButterMilch"};
      model.request.material_paths={"/Models/IAIKitchen/Items/ReineButterMilch"};
      return;
    }
      if(model.request.name=="MuellerFruchtButterMilchMultiVitamin"){
          model.request.material_names={"MuellerFruchtButterMilchMultiVitamin"};
          model.request.material_paths={"/Models/IAIKitchen/Items/MuellerFruchtButterMilchMultiVitamin"};
          return;
      }
    if(model.request.name=="MeerSalz") {
      model.request.material_names = {"MeerSalz"};
      model.request.material_paths = {"/Models/IAIKitchen/Items/MeerSalz"};
    }

    if(model.request.name=="PfannerPfirsichIcetea"){
      model.request.name= "PfannerPfirschIcetea"; // note the typo..
      model.request.material_names={"PfannerPfirschIcetea"};
      model.request.material_paths={"/Models/IAIKitchen/Items/PfannerPfirschIcetea"};
      return;
    }


      if(model.request.name=="HohesCOrange"){
          model.request.name= "HohesCOrange"; // note the typo..
          model.request.material_names={"HohesCOrange"};
          model.request.material_paths={"/Models/IAIKitchen/Items/HohesCOrange"};
          return;
      }

    if(model.request.name=="WasaDelicateCrispRosemary"){
          model.request.name= "WasaDelicateCrispRosemary"; // note the typo..
          model.request.material_names={"WasaDelicateCrispRosemary"};
          model.request.material_paths={"/Models/IAIKitchen/Items/WasaDelicateCrispRosemary"};
          return;
      }

      if(model.request.name=="ComdoCappuccinoClassico"){
          model.request.name= "Cappuccino"; // note the typo..
          model.request.material_names={"Cappuccino"};
          model.request.material_paths={"/Models/IAIKitchen/Items/Cappuccino"};
          return;
      }


      if(model.request.name=="RedPlasticKnife"){
      model.request.name= "Messer";
      model.request.material_names={"Plastic_Red"};
      model.request.material_paths={"/Items/Materials"};
      return;
    }


    if(model.request.name=="BluePlasticKnife"){
      model.request.name= "Messer";
      model.request.material_names={"Material_001"};
      model.request.material_paths={"/Items/Messer"};
      return;
    }
    if(model.request.name=="BluePlasticFork"){
      model.request.name= "DessertFork";
      model.request.material_names={"Plastic_Blue"};
      model.request.material_paths={"/Items/Materials"};
      return;
    }

    if(model.request.name=="RedMetalPlateWhiteSpeckles"){
      model.request.name= "RedSpottedPlate1";
      model.request.material_names={"Ceramic_Red"};
      model.request.material_paths={"/Items/Materials"};
      return;
    }

    if(model.request.name=="BlueMetalPlateWhiteSpeckles"){
      model.request.name= "BlueSpottedPlate_1";
      model.request.material_names={"BlueSpottedPlate"};
      model.request.material_paths={"/Items/Materials"};
      return;
    }

    if(model.request.name=="SojaMilch") {
      model.request.material_names = {"SojaMilch"};
      model.request.material_paths = {"/Models/IAIKitchen/Items/SojaMilch"};
      return;
    }
    if(model.request.name=="ElBrygCoffee"){
      model.request.name= "CoffeeElBryg";
      model.request.material_names={"CoffeeElBryg"};
      model.request.material_paths={"/Models/IAIKitchen/Items/CoffeeElBryg"};
      return;
    }

      if(model.request.name=="ElBrygCoffee"){
          model.request.name= "CoffeeElBryg";
          model.request.material_names={"CoffeeElBryg"};
          model.request.material_paths={"/Models/IAIKitchen/Items/CoffeeElBryg"};
          return;
      }

    if(model.request.name=="SiggBottle"){
      model.request.name= "SiggBottle";
      model.request.material_names={"SiggBottle"};
      model.request.material_paths={"/Models/IAIKitchen/PerceptionExperimentContent/SiggBottle"};
      return;
    }

    if(model.request.name=="CupEcoOrange") {  // TODO REMOVE
      model.request.name = "CupEcoOrange";
      model.request.material_names = {"CupEcoOrange"};
      model.request.material_paths = {"/Models/IAIKitchen/PerceptionExperimentContent/CupEcoOrange"};
      return;
    }

      if(model.request.name=="Mondamin"){
          model.request.name= "Mondamin";
          model.request.material_names={"Mondamin"};
          model.request.material_paths={"/Models/IAIKitchen/Items/Mondamin"};
          return;
      }

    if(model.request.name=="AlbiHimbeerJuice"){
      model.request.name = "AlbiHimbeerJuice";
      model.request.material_names={"AlbiHimbeerJuice"};
      model.request.material_paths={"/Models/IAIKitchen/Items/AlbiHimbeerJuice"};
      return;
    }




    // TODO - Send red block or something if object can't be found?

  }

  geometry_msgs::Pose transformObjectPoseInCamToUE(geometry_msgs::Pose pose,
//  geometry_msgs::Pose transformObjectPoseInCamToUE(rs::PoseAnnotation &pose,
                                                   tf::StampedTransform &camToWorld)
  {
    tf::Transform worldToUE4 = BeliefStateCommunication::getInstance().getTransformBetweenWorldAndUE4();

    geometry_msgs::Pose p,q;
    p = pose;
//    p = convertFromRSCameraPose(pose);

    tf::Transform object_pose;
    tf::Vector3 object_pose_loc(p.position.x, p.position.y, p.position.z);
    tf::Quaternion object_pose_rot (
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
    object_pose.setOrigin(object_pose_loc);
    object_pose.setRotation(object_pose_rot);

    tf::Transform object_transform_in_ue4 = worldToUE4 * camToWorld * object_pose;

    q.position.x = object_transform_in_ue4.getOrigin().getX();
    q.position.y = object_transform_in_ue4.getOrigin().getY();
    q.position.z = object_transform_in_ue4.getOrigin().getZ();

    q.orientation.x = object_transform_in_ue4.getRotation().getX();
    q.orientation.y = object_transform_in_ue4.getRotation().getY();
    q.orientation.z = object_transform_in_ue4.getRotation().getZ();
    q.orientation.w = object_transform_in_ue4.getRotation().getW();


    tf::Quaternion r(q.orientation.x,q.orientation.y,q.orientation.z,q.orientation.w);

    geometry_msgs::Pose result;
    result.position.x = q.position.x;
    result.position.y = q.position.y;
    result.position.z = q.position.z;
    result.orientation.x = r.getX();
    result.orientation.y = r.getY();
    result.orientation.z = r.getZ();
    result.orientation.w = r.getW();

    return result;
  }

//  void transformAndSetObjectTo(rs::PoseAnnotation &pose,
  void transformAndSetObjectTo(geometry_msgs::Pose pose,
      tf::StampedTransform &camToWorld,
      world_control_msgs::SpawnModel &srv)
  {
    geometry_msgs::Pose p = transformObjectPoseInCamToUE(pose,camToWorld);

    srv.request.pose = p;
  }

//  void transformAndSetObjectTo(rs::PoseAnnotation &pose,
  void transformAndSetObjectTo(geometry_msgs::Pose &pose,
                               tf::StampedTransform &camToWorld,
                               world_control_msgs::SetModelPose &srv)
  {
    geometry_msgs::Pose p = transformObjectPoseInCamToUE(pose,camToWorld);

    srv.request.pose = p;
  }

  void spawnObjectInBeliefState(
      tf::StampedTransform &camToWorld,
      geometry_msgs::Pose &pose,
//      rs::PoseAnnotation &pose,
      const std::string &class_name,
      const std::string &object_id,
      std::string &object_name_in_game_engine,
      double offset_angle_z_axis = 0.0) { // declare a message for spawning service

    outInfo("BSOS::spawnObjectInBeliefState");
    world_control_msgs::SpawnModel spawn_model_srv;

    // set the ID and name of the hypothesis to spawn or automatic generation of
// ID
    //TODO here you can change the class to spawn
    spawn_model_srv.request.id = object_id;
    spawn_model_srv.request.name = class_name;
    transformAndSetObjectTo(pose, camToWorld, spawn_model_srv);

//    spawn_model_srv.request.physics_properties.mobility = 0;
    spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.DYNAMIC;
//    spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.STATIONARY;


    // Set a tag so we can uniquely identify the spawned objects in UE4
    world_control_msgs::Tag tag;
    tag.type = "BeliefStateInfo";
    tag.key = "spawned";
    tag.value = "TODO-date";
    spawn_model_srv.request.tags.push_back(tag);
    // Assigning the label that is also used as a reference in the object map
    // This must be unique!
    spawn_model_srv.request.actor_label =
        spawn_model_srv.request.name + "_" + object_id;


    completePose(spawn_model_srv.request.pose, spawn_model_srv.request.name);
    if(offset_angle_z_axis != 0.0)
    {
      outInfo("  Applying caller-defined offset angle");
      spawn_model_srv.request.pose =
          rotatePoseAroundAxis(spawn_model_srv.request.pose, offset_angle_z_axis);
    }
    completeModelBeforeSpawning(spawn_model_srv);

    // Last step. Spawn the actual model.
    std::string final_actor_name_of_object_in_game_engine;
    BeliefStateCommunication::getInstance().spawnObject(spawn_model_srv, &final_actor_name_of_object_in_game_engine);

    outInfo("  Object " << spawn_model_srv.request.actor_label
                        << " spawned with Actor ID: "
                        << final_actor_name_of_object_in_game_engine);

    object_name_in_game_engine = final_actor_name_of_object_in_game_engine;
  }


  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);


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

    // Look up camera pose from other CAS
    // Look up camera position to transform object poses into ue4 coordinates
    tf::StampedTransform camToWorld, worldToCam;

    camToWorld.setIdentity();
    if (other_cas_scene.viewPoint.has())
    {
      rs::conversion::from(other_cas_scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outWarn("No camera to world transformation, no further processing!");
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }

    // Please note that every code block for the different synchronization types must return;
    // TODO If we use this more extensively, we might rather
    //   create child classes for the different synchronization approaches.
    if(synchronization_type_ == DELETE_ALL_AND_RESPAWN)
    {
      BeliefStateCommunication::getInstance().deleteAllSpawnedObjects();
      rs_bs::BeliefStateRepresentation::getInstance().clearBeliefStateEntries();

      // TODO Let the deletion settle. Otherwise the spawn IDs are not unique.
      // TODO Maybe the matching should be a little different to cope with that.
      ros::Duration(delay_in_seconds_after_sync_ + 0.3 ).sleep();

      std::vector<rs::Object> hyps;
      other_cas_scene_cas.get(VIEW_OBJECTS, hyps);
      outInfo("Found "<<hyps.size()<<" object hypotheses");


      int cluster_id = -1;
      for (auto h:hyps) {
        cluster_id++;
        outInfo("OBJ CLUSTER ID: " << cluster_id
                                   << " has identifiable id: " << h.id.get());

        //get the pose, class, shape and color of each hypothesis
        std::vector<rs::PoseAnnotation>  poses;
        std::vector<rs::Classification>  classes;

        // Belief State Tests:
//        std::vector<rs::ClassConfidence> previous_belief_state_results;
//        std::vector<rs::Shape> shapes;
//        std::vector<rs::SemanticColor> colors;
        float confidence = 0.0;
//        h.annotations.filter(shapes);
//        h.annotations.filter(colors);
        h.annotations.filter(poses);
        h.annotations.filter(classes);
//        h.annotations.filter(previous_belief_state_results);

        std::string class_name;

        if(classes.size() > 0)
        {
          confidence = classes[0].confidences.get()[0].score.get();
          class_name = classes[0].classname.get();
          outInfo("  Object class and confidence ++++++++++++: " << class_name << "(" << confidence << ")");
        }else{
          outInfo("  No classification on this cluster_id: " << cluster_id);
          outInfo("  Ignoring it in spawning phase");
          continue;
        }

        // A classification label has been found and set. Complete the rest
        std::string object_id = h.id.get();
        std::string name_of_object_in_game_engine;
        geometry_msgs::Pose pose = convertFromRSCameraPose(poses[0]);
        spawnObjectInBeliefState(camToWorld, pose, class_name, object_id, name_of_object_in_game_engine);

        // Add to Belief State Representation
        rs_bs::BeliefStateEntry bse;
        bse.id = object_id;
        bse.class_name = class_name;
        bse.class_confidence = confidence;
        bse.name_of_object_in_game_engine = name_of_object_in_game_engine;
        rs_bs::BeliefStateRepresentation::getInstance().addToBeliefStateRepresentation(object_id, bse);
      }

      // Sleep for x seconds to allow the camera to change to the desired pose
      // and return the data from the new pose and the new scene
      ros::Duration(delay_in_seconds_after_sync_).sleep();

      outInfo("took: " << clock.getTime() << " ms.");
      return UIMA_ERR_NONE;
    }

    if(synchronization_type_ == SYNCHRONIZATION)
    {
      outInfo("Starting BSOS with Sync Type=SYNCHRONIZATION");
      outInfo("Current BeliefState Representation:");
      outInfo(rs_bs::BeliefStateRepresentation::getInstance().toString());

      std::vector<rs::Object> hyps;
      other_cas_scene_cas.get(VIEW_OBJECTS, hyps);
      outInfo("Found "<<hyps.size()<<" object hypotheses");


      int cluster_id = -1;
      for (auto h:hyps) {
        cluster_id++;
        outInfo("OBJ CLUSTER ID: " << cluster_id
                                   << " has identifiable id: " << h.id.get());


        // get the pose, class, shape and color of each hypothesis
        std::vector<rs::PoseAnnotation> poses;
        std::vector<rs::Classification> classes;

        float confidence = 0.0;
        h.annotations.filter(poses);
        h.annotations.filter(classes);

        std::string class_name;
//        bool class_name_found = false;
        bool inView = false;
        bool disappeared = false;

        if(classes.size() > 0)
        {
          confidence = classes[0].confidences.get()[0].score.get();
          class_name = classes[0].classname.get();
          inView = h.inView.get();
          disappeared = h.disappeared.get();

          outInfo("  Object class (confidence) ; inView ; disappeared ++++++++++++: " << class_name << "(" << confidence << ") inView:" << inView << " disappeared: " << disappeared);
        }else{
          outInfo("  No classification on this cluster_id: " << cluster_id);
          outInfo("  Skipping Sync Iteration");
          continue;
        }

        // A classification label has been found and set. Complete the rest
        std::string object_id = h.id.get();

        if(rs_bs::BeliefStateRepresentation::getInstance().isIDInBeliefState(object_id))
        {
          rs_bs::BeliefStateEntry &bse =
              rs_bs::BeliefStateRepresentation::getInstance().getBeliefStateEntryWithID(object_id);

          if(h.inView.get() && h.disappeared.get() ){
            // Object should be visible but is disappeared
            // Delete from BS
            outInfo("  Object should be in view but is disappeared. Deleting from BS and skipping rest of iteration...");
            BeliefStateCommunication::getInstance().deleteObject(object_id);
            rs_bs::BeliefStateRepresentation::getInstance().deleteObjInRepresentation(object_id);
            continue;
          }


          // TODO this was in the old experiment code. idk when this really triggered.
          // for reference...
          //if( !h.inView.get() || h.disappeared.get())
          //  continue;

          // Don't handle objects that can't be seen currently.
          if(!inView)
            continue;

//          if(bse.verification_actions.size() > 0)
          if(bse.verification_requests.size() > 0)
          {
            outWarn("  ***** Verification request available for this object *****");
            bse.under_verification = true; // This must be set to false by the verifying action

            // If we have to verify, we'll just delete the object and re-set it
            // according to the desired verification request
            // TODO How brings back the 'actual' belief afterwards?
            BeliefStateCommunication::getInstance().deleteObject(object_id);
            ros::Duration(delay_in_seconds_after_sync_).sleep();

            std::string name_of_object_in_game_engine;
            rs_bs::VerificationRequest req = bse.verification_requests.back();
            outWarn(req.toString());
            // Decide what we have to do to set this verification request
            // up in the UE4 BS
            geometry_msgs::Pose pose = convertFromRSCameraPose(poses[0]);
            double pose_offset = 0.0;

            // If we should work on the pose,
            // we'll assume that we want to try a 180 deg turn on the up-axis
            // of the object, because this is where RS detection can be shaky.
            if(req.type == rs_bs::VerificationRequest::POSE)
            {
              pose_offset= M_PI;
            }
            std::string object_name = req.class_name;

            spawnObjectInBeliefState(camToWorld, pose, object_name, object_id,
                                     name_of_object_in_game_engine, pose_offset);
            bse.name_of_object_in_game_engine = name_of_object_in_game_engine;
          }
          else if(bse.force_respawn_in_game_engine)
          {
            outInfo("  FORCE RESPAWN FLAG SET ON THIS OBJECT. Respawning object in GE based on current BSE.");
            BeliefStateCommunication::getInstance().deleteObject(object_id);
            ros::Duration(delay_in_seconds_after_sync_).sleep();

            std::string name_of_object_in_game_engine;
            geometry_msgs::Pose pose = convertFromRSCameraPose(poses[0]);
            spawnObjectInBeliefState(camToWorld, pose, class_name, object_id, name_of_object_in_game_engine);
            bse.name_of_object_in_game_engine = name_of_object_in_game_engine;
            // Some other component wants us to re-setup this object in the GE
            bse.force_respawn_in_game_engine = false;
          }
          else if(confidence > 0.55) // actual update process
          {
            // TODO - What happens when the classification label changes?

            outInfo("  Object confidence higher than 0.55. Updating pose.");
            //Simple update model right now. Only update BS entry when confidence is higher than 0.5"

            bse.class_confidence = confidence;

            // Prepare the actual spawning
            world_control_msgs::SetModelPose set_model_pose_srv;

            set_model_pose_srv.request.id = object_id;
            geometry_msgs::Pose pose = convertFromRSCameraPose(poses[0]);
            transformAndSetObjectTo(pose, camToWorld, set_model_pose_srv);
//            transformAndSetObjectTo(poses[0], camToWorld, set_model_pose_srv);
            completePose(set_model_pose_srv.request.pose, class_name);

            BeliefStateCommunication::getInstance().setObjectPose(set_model_pose_srv);
            // Update the

            bse.latest_pose.set(
                poses[0].world.get().translation.get()[0],
                poses[0].world.get().translation.get()[1],
                poses[0].world.get().translation.get()[2],
                poses[0].world.get().rotation.get()[0],
                poses[0].world.get().rotation.get()[1],
                poses[0].world.get().rotation.get()[2],
                poses[0].world.get().rotation.get()[3]);

            // Get the latest Belief State Pose and write it into the Belief State Representation
            geometry_msgs::Pose latest_bs_pose = BeliefStateCommunication::getInstance().getLatestObjectPose(object_id);
            std::cout << "UPDATING POSE FROM BS "<< std::endl;
            std::cout << "    LATEST POSE WAS: " << latest_bs_pose;
            bse.latest_bs_pose.set(
                latest_bs_pose.position.x,
                latest_bs_pose.position.y,
                latest_bs_pose.position.z,
                latest_bs_pose.orientation.x,
                latest_bs_pose.orientation.y,
                latest_bs_pose.orientation.z,
                latest_bs_pose.orientation.w);
          } // end of confidence check

          outInfo("  Object " << object_id << "is already known in BS Representation");
        }
        else
        {
          outInfo("  Object " << object_id << "is not yet in BS Representation");

          std::string name_of_object_in_game_engine;
          geometry_msgs::Pose pose = convertFromRSCameraPose(poses[0]);
          spawnObjectInBeliefState(camToWorld, pose, class_name, object_id, name_of_object_in_game_engine);
          // TODO add to BS
          rs_bs::BeliefStateEntry bse;
          bse.id = object_id;
          bse.class_name = class_name;
          bse.class_confidence = confidence;
          bse.name_of_object_in_game_engine = name_of_object_in_game_engine;
          bse.latest_pose.set(
              poses[0].world.get().translation.get()[0],
              poses[0].world.get().translation.get()[1],
              poses[0].world.get().translation.get()[2],
              poses[0].world.get().rotation.get()[0],
              poses[0].world.get().rotation.get()[1],
              poses[0].world.get().rotation.get()[2],
              poses[0].world.get().rotation.get()[3]);

          rs_bs::BeliefStateRepresentation::getInstance().addToBeliefStateRepresentation(object_id, bse);

        }

        // TODO At the end we should check if there are still objects in the belief
        //      where we don't have any information left about them. These should be cleared.
      }

      ros::Duration(delay_in_seconds_after_sync_).sleep();
      return UIMA_ERR_NONE;
    }

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_ANNOTATOR_COULD_NOT_CREATE;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BeliefStateObjectSynchronization)
