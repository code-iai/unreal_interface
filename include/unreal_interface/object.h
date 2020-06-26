#ifndef UNREAL_INTERFACE_OBJECT_H
#define UNREAL_INTERFACE_OBJECT_H

#include <string>
#include <unreal_interface/types.h>
#include <ros/ros.h>
#include <world_control_msgs/SpawnModel.h>
#include <world_control_msgs/SetModelPose.h>
#include <world_control_msgs/DeleteModel.h>

namespace UnrealInterface
{
class Objects{
public:
    Objects()
    {
        Init();
    }
    Objects(ros::NodeHandle n) : n_(n)
    {
        Init();
    }
private:
    // Variables
    ros::NodeHandle n_;
    ros::ServiceClient spawn_client_;
    ros::ServiceClient delete_client_;
    ros::ServiceClient set_pose_client_;

    std::string urosworldcontrol_domain_ = "pie_rwc";

//    std::vector<UnrealInterface::Object::Id> spawned_objects_;
    std::map<UnrealInterface::Object::Id, UnrealInterface::Object::ObjectInfo> spawned_objects_;

public:
    // Methods
    void SetUROSWorldControlDomain(std::string s)
    {
        urosworldcontrol_domain_ = s;
    }

    /**
     * Spawn a single object in UE4
     * This function DOES NOT implement sanity checks, but just publishes
     * the given model and keeps track of that in spawned_objects_.
     *
     * @param model The model that should be spawned in the Belief State.
     *        id_of_spawned_objects If a pointer is given and the service call is successful,
     *                   the method will place the returned id of the spawned object in thatpointer.
     * @return false if an error has occured, true otherwise.
     */
    bool SpawnObject(world_control_msgs::SpawnModel model, UnrealInterface::Object::Id *id_of_spawned_object);

    /** Similar to SpawnObjects, but you can spawn multiple objects at once
    * STILL TODO
    * We need to extend UROSWorldControl for that.
    * TODO Have a seperate type that has lists of success responses and IDs
    */
    bool SpawnObjects(std::vector<world_control_msgs::SpawnModel> models, std::vector<UnrealInterface::Object::Id> *ids_of_spawned_object);


    bool DeleteObject(UnrealInterface::Object::Id id);
    bool DeleteObjects(std::vector<UnrealInterface::Object::Id> ids);

    bool SetObjectPose(world_control_msgs::SetModelPose pose);

    /**
     * Iterate over this.spawned_objects_ and delete all of them sequentially
     */
    void DeleteAllSpawnedObjects();

    UnrealInterface::Object::ObjectInfo& GetObjectInfo(UnrealInterface::Object::Id id);

    void PrintAllObjectInfo();

    /**
     * This returns the size of the internal spawned objects representation.
     * It is not yet guaranteed that it returns the actual number of objects that really have been
     * spawned.
     */
    int SpawnedObjectCount();

    void SaveStateOfObject(UnrealInterface::Object::Id id);
    void SaveStateOfAllSpawnedObjects();
    void RestoreStateOfObject(UnrealInterface::Object::Id id);
    void RestoreStateOfAllSpawnedObjects();

    // TODO
    // Add EventData Type as parameter for callback
    void BindToEvent(UnrealInterface::Object::ObjectEventType event, std::function<void(UnrealInterface::Object::ObjectEventType)> fun);

private:
    void Init();

};
} // end of namespace
#endif //UNREAL_INTERFACE_OBJECT_H
