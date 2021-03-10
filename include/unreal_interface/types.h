#ifndef UNREAL_INTERFACE_TYPES_H
#define UNREAL_INTERFACE_TYPES_H

#include <string>
#include <iostream>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

namespace UnrealInterface::Object
{
  // typedefs
  typedef std::string Id;

  // Complex types
  class ObjectInfo
  {
  public:
    Id id_;

    // This is the unique Name of the Actor that you wil get when calling GetName() on an UE4 Actor
    std::string actor_name_;

    // In UE4 world coordinates
    geometry_msgs::Pose pose_;

    void print(){
        std::cout << "Object " << id_ << ": actor_name_ " << actor_name_ << std::endl;
    }
  };

  class CollisionInfo
  {
      bool in_collision_;
      std::vector<std::string> in_collision_with_; // list of actor names
  };

  class StabilityInfo
    {
        bool was_stable_after_spawn_;
        bool is_currently_stable_;
    };

  enum ObjectEventType
  {
      OBJECT_SPAWNED,
      OBJECT_DELETED,
      OBJECT_MODIFIED,
      SAVED_STATE_OF_OBJECT,
      SAVED_STATE_OF_ALL_SPAWNED_OBJECTS,
      RESTORE_STATE_OF_OBJECT,
      RESTORE_STATE_OF_ALL_SPAWNED_OBJECTS,

  };
} // end of namespace
#endif //UNREAL_INTERFACE_TYPES_H
