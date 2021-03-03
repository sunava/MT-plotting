#include "rs_bs/BeliefStateRepresentation.h"
#include <iostream>
#include <algorithm>

std::map<std::string, std::vector<std::string>> rs_bs::BeliefStateRepresentation::object_properties_ =
    {
      {"AlbiHimbeerJuice", { "box", "container","drink" }},
      {"KoellnMuesliKnusperHonigNuss", { "box", "container","food" }},
      {"KnusperSchokoKeks", { "box", "container","food" }},
      {"SpitzenReis", { "box", "container","food" }},
      {"PfannerGruneIcetea", { "box", "container","drink" }},
      {"PfannerPfirsichIcetea", { "box", "container","drink" }}
};

uint rs_bs::BeliefStateRepresentation::belief_state_iteration_id_ = 0;

const uint rs_bs::BeliefStateRepresentation::getBeliefStateIterationID()
{
  return rs_bs::BeliefStateRepresentation::belief_state_iteration_id_;
}

void rs_bs::BeliefStateRepresentation::resetBeliefStateIterationID()
{
  rs_bs::BeliefStateRepresentation::belief_state_iteration_id_ = 0;
}

void rs_bs::BeliefStateRepresentation::nextBeliefStateIterationID()
{
  rs_bs::BeliefStateRepresentation::belief_state_iteration_id_++;
}

rs_bs::BeliefStateRepresentation::BeliefStateRepresentation()
{

}

bool rs_bs::BeliefStateRepresentation::isIDInBeliefState(std::string id)
{
  return belief_state_entries_.count(id) > 0;
}
bool rs_bs::BeliefStateRepresentation::deleteObjInRepresentation(
    std::string id) {

  std::cout << "Trying to delete OBJ " << id << " from BS Representation" << std::endl;
  auto it = belief_state_entries_.find(id);

  if(it == belief_state_entries_.end())
    return false;

  belief_state_entries_.erase(it);
  return true;
}
void rs_bs::BeliefStateRepresentation::clearBeliefStateEntries() {
  std::cout << "Clearing BS Representation" << std::endl;
  belief_state_entries_.clear();
}



rs_bs::BeliefStateEntry &
rs_bs::BeliefStateRepresentation::getBeliefStateEntryWithID(std::string id) {
  return belief_state_entries_[id];
}

bool rs_bs::BeliefStateRepresentation::addToBeliefStateRepresentation(
    std::string id, rs_bs::BeliefStateEntry &bse) {
  if(isIDInBeliefState(id))
    return false;

  belief_state_entries_[id] = bse;
  return true;
}

std::string rs_bs::BeliefStateRepresentation::toString() {
  std::stringstream s;

  for(auto kvp : belief_state_entries_)
  {
    s << kvp.second.toString();
  }

  return s.str();
}

std::vector<std::string> rs_bs::BeliefStateRepresentation::has_property(std::string value)
{
  std::vector<std::string> result;
  for(auto kvp : belief_state_entries_)
  {
    const BeliefStateEntry &bse = kvp.second;
    auto list_of_properties_for_class = object_properties_[bse.class_name];
    bool found = (std::find(list_of_properties_for_class.begin(),
                            list_of_properties_for_class.end(), value)
                  != list_of_properties_for_class.end());
    if(found)
    {
      result.push_back( kvp.second.id );
    }

  }

  return result;
}

bool rs_bs::BeliefStateRepresentation::getObjectNameInGameEngine(
    std::string id, std::string &name_in_game_engine) {
  if(!isIDInBeliefState(id))
  {
    std::cerr << "Tried to get Object Name in Game Engine on non-existing RS Object ID" << id << std::endl;
    return false;
  }
  rs_bs::BeliefStateEntry bse =
    rs_bs::BeliefStateRepresentation::getInstance().getBeliefStateEntryWithID(id);
  name_in_game_engine = bse.name_of_object_in_game_engine;
  return true;
}
