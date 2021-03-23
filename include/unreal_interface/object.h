#ifndef UNREAL_INTERFACE_OBJECT_H
#define UNREAL_INTERFACE_OBJECT_H

#include <string>
#include <unreal_interface/types.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <world_control_msgs/SpawnModel.h>
#include <world_control_msgs/GetModelPose.h>
#include <world_control_msgs/GetModelSocketPose.h>
#include <world_control_msgs/SetModelPose.h>
#include <world_control_msgs/DeleteAll.h>
#include <world_control_msgs/DeleteModel.h>

#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <tf/tfMessage.h>

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
    // Constants
    const std::string DEFAULT_SPAWN_TAG_TYPE = "UnrealInterface";
    const std::string DEFAULT_SPAWN_TAG_KEY = "spawned";
    const std::string ERROR_SPAWN_ID_NOT_UNIQUE = "1";
    const std::string ERROR_SPAWN_OBSTRUCTED = "2";



    // Variables
    ros::NodeHandle n_;
    ros::ServiceClient spawn_client_;
    ros::ServiceClient delete_client_;
    ros::ServiceClient set_pose_client_;
    ros::ServiceClient get_pose_client_;
    ros::ServiceClient get_socket_pose_client_;
    ros::ServiceClient delete_all_client_;
    ros::ServiceClient enable_check_touch_;
    ros::Subscriber pose_update_subscriber_;
    ros::Subscriber state_update_subscriber_;
    ros::Subscriber touch_subscriber_;

    std::string urosworldcontrol_domain_ = "pie_rwc";

    // Object info might be updated asynchronously to be faster
    // Avoid collisions
    std::mutex object_info_mutex_;

    int retry_count_ = 5;
    float retry_delay_ = 0.4;

    std::map<UnrealInterface::Object::Id, UnrealInterface::Object::ObjectInfo> spawned_objects_;

    // Nt beautiful way of doing it for now.
    std::string InputStateStream;
    // Even less beautiful way.
    std::string TouchHappenings;

public:
    // Methods
    void SetUROSWorldControlDomain(std::string s)
    {
        urosworldcontrol_domain_ = s;
    }

    void SetRetryCount(int c)
    {
        retry_count_ = c;
    }

    void SetRetryDelay(float d)
    {
        retry_delay_ = d;
    }

    /**
     * Use this method to do a low-level functional check of the transport capabilities.
     * In the current version of this library, this method is checking if the necessary
     * services/topics are available.
     */
    bool TransportAvailable();

    /**
     * Spawn a single object in UE4
     * This function DOES NOT implement sanity checks, but just publishes
     * the given model and keeps track of that in spawned_objects_.
     *
     * @param model The model that should be spawned in the Belief State.
     *        id_of_spawned_objects If a pointer is given and the service call is successful,
     *                   the method will place the returned id of the spawned object in thatpointer.
     * @return 0 if there was no error, -1 if the service can't be called. 1 if there was an Id Problem, 2 if there was an obstruction problem
     */
    int SpawnObject(world_control_msgs::SpawnModel model, UnrealInterface::Object::Id *id_of_spawned_object);

    /**
     * Similar to SpawnObjects, but you can spawn multiple objects at once
     * STILL TODO
     * We need to extend UROSWorldControl for that.
     * TODO Have a seperate type that has lists of success responses and IDs
     */
    bool SpawnObjects(std::vector<world_control_msgs::SpawnModel> models,
                      std::vector<UnrealInterface::Object::Id> *ids_of_spawned_object);


    bool DeleteObject(UnrealInterface::Object::Id id);
    bool DeleteObjects(std::vector<UnrealInterface::Object::Id> ids);
    void CleanSpawnOnDelete(UnrealInterface::Object::Id id);

    /**
     * Sets the pose of a previously spawned object with the given UnrealInterface::Object::Id.
     *
     * @return true if successful, false otherwise.
     */
    bool SetObjectPose(UnrealInterface::Object::Id id, geometry_msgs::Pose pose);

    /**
     * Gets the pose of a previously spawned object with the given UnrealInterface::Object::Id.
     * This will retrieve the pose IMMEDIATELY from UE4 and put it into outPose.
     *
     * @return true if object can be found by 'Id'. false otherwise.
     */
    bool GetObjectPose(UnrealInterface::Object::Id id, geometry_msgs::Pose &outPose);

    /**
     * Gets the socket pose of a previously spawned object with the given UnrealInterface::Object::Id nd socket name as a string.
     * This will retrieve the socket pose IMMEDIATELY from UE4 and put it into outPose in UE-World frame.
     *
     * @return true if object can be found by 'Id'. false otherwise.
     */
    bool GetObjectSocketPose(UnrealInterface::Object::Id id, UnrealInterface::Object::Socket socket, geometry_msgs::Pose &outPose);

    /**
     * Iterate over this.spawned_objects_ and delete all of them sequentially
     *
     * @return false, if atleast one Delete operation failed. true otherwise.
     */

    bool DeleteAllSpawnedObjects();

    /**
    * Send a single service request to delete all objects that
    * have been spawned by this module so far.
    * Internally, this happens by deleting all objects in UE4 that
    * match the semlog tag key&type of spawned objects.
    *
    * @return false, if service call failed. true otherwise.
    */
    bool DeleteAllSpawnedObjectsByTag();

    /**
    * Sends a single service request to check for touching objects.
    */
    bool EnableTouchChecker();

    /**
     * Gets an data class that includes all the available information about an object.
     * Please note, that some of the attributes are filled asynchronously to speed up the execution.
     * So you might not have the most up2date data, especially if your update frequency in the UE4-side
     * plugins is low.
     *
     * @throws if Id is not existing (e.g. it has been spawned through this library before)
     */
    UnrealInterface::Object::ObjectInfo GetObjectInfo(UnrealInterface::Object::Id id);

    std::string GetStateString();
    std::string GetTouchString();

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

protected:
    /**
     * Low-level Set Model Pose function. Users of this API should use SetObjectPose instead of this function.
     * SetObjectPose might handle workarounds and quirks that occur with the low-level-interface.
     * @return true if successful, false otherwise.
     */
    bool SetModelPose(world_control_msgs::SetModelPose);

    /**
     * Low-level Set Model Pose function. Users of this API should use SetObjectPose instead of this function.
     * SetObjectPose might handle workarounds and quirks that occur with the low-level-interface.
     * @return true if successful, false otherwise.
     */
    bool DeleteModel(world_control_msgs::DeleteModel);

    //void PoseUpdateCallback(const geometry_msgs::PoseStamped&);

    void TFUpdateCallback(const tf::tfMessage&);

    void TFStateUpdateCallback(const tf::tfMessage&);

    void StringUpdateCallback(const std_msgs::String&);
};
} // end of namespace
#endif //UNREAL_INTERFACE_OBJECT_H
