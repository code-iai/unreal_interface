#include "unreal_interface/object.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <stdexcept>


void UnrealInterface::Objects::Init()
{
    ROS_INFO_STREAM("Initializing BeliefStateCommunication ...");
    //parameter initialization
    //client for model spawning
    spawn_client_ = n_.serviceClient<world_control_msgs::SpawnModel>(urosworldcontrol_domain_+"/spawn_model");
    //client for model deletion
    delete_client_ = n_.serviceClient<world_control_msgs::DeleteModel>(urosworldcontrol_domain_+"/delete_model");

    set_pose_client_ = n_.serviceClient<world_control_msgs::SetModelPose>(urosworldcontrol_domain_ + "/set_model_pose");
}


bool UnrealInterface::Objects::SpawnObject(world_control_msgs::SpawnModel model, UnrealInterface::Object::Id *id_of_spawned_object = nullptr)
{

    // Add additional information to the request that is necessary for the functionality
    // of UnrealInterface.

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    // Set a tag so we can uniquely identify the spawned objects in UE4
    world_control_msgs::Tag tag;
    tag.type = "UnrealInterface";
    tag.key = "spawned";
    tag.value = std::ctime(&now_time);
    model.request.tags.push_back(tag);


    //check whether or not the spawning service server was reached
    if (!spawn_client_.call(model))
    {
        ROS_ERROR("Failed to call service spawn");
        return false;
    }

    //check the status of the respond from the server
    if (!model.response.success)
    {
        ROS_ERROR("Spawn Service call returned false");
        return false;
    }

    //save object in object map

    UnrealInterface::Object::ObjectInfo object_info;
    object_info.id_ = model.response.id;
    object_info.actor_name_ = model.response.name;


    this->spawned_objects_[model.response.id] = object_info;

    //print the ID of the spawned hypothesis
    ROS_INFO_STREAM("Object spawned with ID " << model.response.id << " and final actor name: " << model.response.name);

    if(id_of_spawned_object)
    {
        *id_of_spawned_object = model.response.id;
    }

    return true;
}


bool UnrealInterface::Objects::DeleteObject(UnrealInterface::Object::Id id)
{
    std::string string_id = static_cast<std::string>(id);

    std::cout << "delete object " << string_id << std::endl;

    world_control_msgs::DeleteModel model;
    model.request.id = string_id;

    if(!DeleteModel(model))
    {
        // TODO Remove or fix Bug in UROSWorldControl.
        // This is just a method to update objects even though UROSWorldControl can't find them temporarily
        for(int i = 0; i < retry_count_; i++)
        {
            ros::Duration(retry_delay_).sleep();
            std::cout << "Retrying to delete object " << string_id << std::endl;
            if(DeleteModel(model))
            {
                std::cout << "Deleted Object " << string_id << " successfully in iteration " << i << std::endl;
                return true;
            }
        }

        std::cout << "Retrying to update pose failed. Giving up..."  << std::endl;
        return false;
    }

    return true;

}

bool UnrealInterface::Objects::DeleteModel(world_control_msgs::DeleteModel model)
{
    //check whether or not the spawning service server was reached
    if (!delete_client_.call(model))
    {
        ROS_ERROR("Failed to call service delete");
        return false;

    }

    // When is a good time to delete stuff?
    // Only if the service call suceeds?
    // But on the other hand, it will also return false if actors are already gone...
    spawned_objects_.erase(model.request.id);

    //check the status of the respond from the server
    if (!model.response.success)
    {
        ROS_ERROR("Service call returned success = false");
        return false;
    }


    return true;
}

UnrealInterface::Object::ObjectInfo& UnrealInterface::Objects::GetObjectInfo(UnrealInterface::Object::Id id)
{
    if(spawned_objects_.count(id)==0)
        throw std::invalid_argument("The given ID is not in the spawned object representation");

    return spawned_objects_[id];
}

void UnrealInterface::Objects::PrintAllObjectInfo()
{
    std::cout << "** UnrealInterface::Objects::spawned_objects_ contains **" << std::endl;
    for(const auto &key_value_pair : spawned_objects_)
    {
        GetObjectInfo(key_value_pair.first).print();
    }
}

int UnrealInterface::Objects::SpawnedObjectCount()
{
    return spawned_objects_.size();
}

bool UnrealInterface::Objects::SetObjectPose(UnrealInterface::Object::Id id, geometry_msgs::Pose pose) {
    ROS_INFO_STREAM("Send object pose update to UE4");

    world_control_msgs::SetModelPose setmodelpose_srv;

    // We assume that the UnrealInterface::Object::Id is equal to the semlog Id atm
    setmodelpose_srv.request.id = id;
    setmodelpose_srv.request.pose = pose;

    if(!SetModelPose(setmodelpose_srv))
    {
        // TODO Remove or fix Bug in UROSWorldControl.
        // This is just a method to update objects even though UROSWorldControl can't find them temporarily
        for(int i = 0; i < retry_count_; i++)
        {
            ros::Duration(retry_delay_).sleep();
            std::cout << "Retrying to update pose"  << std::endl;
            if(SetModelPose(setmodelpose_srv))
            {
                std::cout << "Updated pose successfully in iteration " << i << std::endl;
                return true;
            }
        }

        std::cout << "Retrying to update pose failed. Giving up..."  << std::endl;
        return false;
    }

    return true;
}
bool UnrealInterface::Objects::SetModelPose(world_control_msgs::SetModelPose setmodelposesrv) {
    if (!set_pose_client_.call(setmodelposesrv))
    {
        ROS_ERROR("Failed to call service client for SetModelPose");
        return false;
    }

    if (!setmodelposesrv.response.success)
    {
        ROS_ERROR("SetModelPose Service received non-success response during update");

        return false;
    }

    return true;
}

bool UnrealInterface::Objects::DeleteAllSpawnedObjects()
{
    ROS_INFO_STREAM("Deleting all previously spawned objects (" << spawned_objects_.size() << ")");
    bool return_value = true;

    for(auto const pair : spawned_objects_)
    {
        UnrealInterface::Object::Id obj_id = pair.first;
        if(!DeleteObject(obj_id))
        {
            // If you've reached this part, even after retrying we couldn't delete the desired object.
            // This might happen due to bad housekeeping in this->spawned_objects_ or a communication error
            // with UROSWorldControl.
            //
            // Try to delete the rest, but report false at the end.
            return_value = false;
        }
    }

    return return_value;
}