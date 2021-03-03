#ifndef SRC_BELIEFSTATEREPRESENTATION_H
#define SRC_BELIEFSTATEREPRESENTATION_H

#include <string>
#include <sstream>
#include <map>
#include <set>
#include <vector>
#include <iomanip>

//#include <geometry_msgs/Pose.h> // TODO use geometry_msgs.
                // Current problem: Can't link against CATKIN_LIBRARIES in the cmake without getting a weird long linking build time.

namespace rs_bs{

// Workaround until we can include geometry_msgs::Pose
class BSEPose{
  double position[3]; // xyz
  double rotation[4]; // xyzw

public:
  void set(double x, double y, double z,
    double rotx, double roty, double rotz, double rotw)
  {
    position[0] = x;
    position[1] = y;
    position[2] = z;

    rotation[0] = rotx;
    rotation[1] = roty;
    rotation[2] = rotz;
    rotation[3] = rotw;
  }

  void get(std::vector<double> &loc, std::vector<double> &rot)
  {
    loc.clear();
    rot.clear();
    loc.insert(loc.begin(), std::begin(position), std::end(position));
    rot.insert(rot.begin(), std::begin(rotation), std::end(rotation));
  }


  std::string toString()
  {
    std::stringstream s;
    s << std::setprecision(4);
    s << position[0] << ","
      << position[1] << ","
      << position[2] << " Rot: "
      << rotation[0] << ","
      << rotation[1] << ","
      << rotation[2] << ","
      << rotation[3] << "";
    return s.str();
  }
};

// This might be only an intermediate solution until a more sophisticated
// diff-based belief state sync tool comes up
class VerificationRequest{
public:
  // What could be wrong?
  enum verification_type
  {
    POSE = 0,
    CLASS = 1
  } type;
  // How should the alternative Belief State look like?
  std::string hint;
  std::string class_name;
  std::string id; // TODO assign unique ids

  // TODO static?
  std::map<verification_type, std::string> type_map_ =
      {
          {POSE, "POSE"},
          {CLASS, "CLASS"},
  };

//  VerificationRequest() = default;
//
//  VerificationRequest(verification_type type, std::string hint) : type(type), hint(hint){
////    id = id_counter++;
//  }

  std::string toString()
  {
    std::stringstream s;
    s << "VerificationRequest (" << id << "): ";
    s << "type: " << type_map_[type] << " hint: " << hint;
    s << std::endl;

    return s.str();
  }
};


class BeliefStateEntry{
public:
  std::string id; // RS ID!
  std::string class_name;
  float class_confidence;
  int disappeared_counter;
  std::string name_of_object_in_game_engine; // This is required for segmentation
  bool force_respawn_in_game_engine = false; // Indicate that this object should be readded to the GE based on the current values.
  bool under_verification = false;
//  std::vector<std::string> verification_actions;
  std::vector<VerificationRequest> verification_requests;

  BSEPose latest_pose; // world coordinates
  BSEPose latest_bs_pose; // ue4 coordinates

  std::string toString()
  {
    std::stringstream s;
    s << "BS Entry ID: " << id << " class_name=" << class_name <<
      " class_confidence=" << class_confidence << " dis-ctr=" <<
        disappeared_counter << " name_of_object_in_ge=" << name_of_object_in_game_engine<< std::endl;
    s << "  Latest Belief State Pose: " <<
      latest_bs_pose.toString() << std::endl;
    s << "  Latest Pose Estimation from Object Hypothesis: " <<
      latest_pose.toString() << std::endl
      << std::endl;
    return s.str();
  }
};

class BeliefStateRepresentation{
public:
  static BeliefStateRepresentation& getInstance()
  {
    // Since it's a static variable, if the class has already been created,
    // it won't be created again.
    // And it **is** thread-safe in C++11.
    static BeliefStateRepresentation bsrInstance;

    // Return a reference to our instance.
    return bsrInstance;
  }
  // delete copy and move ctors and assign operators
  BeliefStateRepresentation(BeliefStateRepresentation const&) = delete;
  BeliefStateRepresentation(BeliefStateRepresentation&&) = delete;
  BeliefStateRepresentation& operator=(BeliefStateRepresentation const&) = delete;
  BeliefStateRepresentation& operator=(BeliefStateRepresentation&&) = delete;

protected:
  BeliefStateRepresentation();
  ~BeliefStateRepresentation() = default;


  // RS Object ID -> BeliefStateEntry
  std::map<std::string, BeliefStateEntry> belief_state_entries_;


public:
  // Properties
  // Maps class to properties of that class
  static std::map<std::string, std::vector<std::string>> object_properties_;

  static uint belief_state_iteration_id_;

  std::map<std::string, std::vector<std::string>> open_verification_actions_;
  std::set<std::string> active_verifications_for_obj_id_;

  const static uint getBeliefStateIterationID();

  static void resetBeliefStateIterationID();

  /**
   * Indicate that the next iteration ID should be set internally.
   */
  static void nextBeliefStateIterationID();

  void clearBeliefStateEntries();

  bool isIDInBeliefState(std::string id);

  // True if found, else otherwise
  bool deleteObjInRepresentation(std::string id);

  // Make sure that the ID exists in the BS before accessing.
  BeliefStateEntry& getBeliefStateEntryWithID(std::string id);

  // Issues a warning and returns false if ID is already in BSR
  bool addToBeliefStateRepresentation(std::string id, BeliefStateEntry &bse);

  // Iterates over all BSEntries and calls toString() on them
  std::string toString();

  /**
   * STUBS
   */

  // Returns a list of RS Object IDs
  std::vector<std::string> has_property(std::string value);

  bool getObjectNameInGameEngine(std::string id, std::string &name_in_game_engine);

  void setLatestPose(std::string id, double x, double y, double z,
                         double rotx, double roty, double rotz, double rotw);

  void setLatestBSPose(std::string id, double x, double y, double z,
                     double rotx, double roty, double rotz, double rotw);

  BSEPose getLatestPose(std::string id);
  BSEPose getLatestBSPose(std::string id);

};

} // end of namespace rs_bs

#endif // SRC_BELIEFSTATEREPRESENTATION_H
