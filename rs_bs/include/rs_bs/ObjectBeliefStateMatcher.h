#ifndef SRC_OBJECTBELIEFSTATEMATCHER_H
#define SRC_OBJECTBELIEFSTATEMATCHER_H

/*
 * This class implements a mapping between RS clusters from real sensor data and RS clusters from the Belief State.
 * A common action for example is, that one has a RS clusters from the real world
 * and needs the corresponding object in the Belief State.
 */
namespace rs_bs{
class ObjectBeliefStateMatcher{
public:

  // TODO: Set a matching strategy on startup if there are more in the future
  ObjectBeliefStateMatcher() = default;


  // Use something templatey / generic / lambdas
  bool match(rs_bs::BeliefStateObject& bs_obj, rs::ObjectHypothesis& other_cas_obj){
    std::cout << "Comparing " << bs_obj.rsObjectId.get() << " with " << other_cas_obj.id.get();
    return bs_obj.rsObjectId.get() == other_cas_obj.id.get();
  }

//
//  /**
//   * Return a pointer to a BeliefStateObject in scene_to_look_in, which
//   * is the corresponding object for other_cas_obj.
//   * Please note that a raw pointer will be returned.
//   * As soon as the CAS is reset or is otherwise throwing away ObjectHypotheses,
//   * this pointer shouldn't be used anymore!
//   *
//   * @param bs_clusters A vector of all the current belief state items
//   * @param other_cas_obj A reference to an ObjectHypothesis from another CAS
//   * @return Pointer if a matching BS object has been found for other_cas_obj. nullptr otherwise.
//   */
//  rs_bs::BeliefStateObject* getBeliefStateObject(
//      std::vector<rs_bs::BeliefStateObject>& bs_clusters,
//      rs::ObjectHypothesis &other_cas_obj) {
////    std::vector<rs_bs::BeliefStateObject> bs_clusters;
////    scene_to_look_in.identifiables.filter(bs_clusters);
//
//    auto it = std::find_if(bs_clusters.begin(), bs_clusters.end(),
//                           [&](rs_bs::BeliefStateObject &bso) {
//                             return match(bso, other_cas_obj);
//                           });
//    if (it == bs_clusters.end())
//      return nullptr;
//
//    return nullptr;
////    return std::addressof(*it);
//  }
//    return

//    for (auto bs_cluster: bs_clusters) {
//      if(match(bs_cluster, other_cas_obj))
//        return &bs_cluster;
//    }


//    return nullptr;
//  }


};
}

#endif // SRC_OBJECTBELIEFSTATEMATCHER_H
