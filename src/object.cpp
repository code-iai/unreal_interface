#include "unreal_interface/object.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <stdexcept>


void UnrealInterface::Objects::Init()
{
    ROS_INFO_STREAM("Initializing UnrealInterface::Objects...");
    //parameter initialization
    //client for model spawning
    spawn_client_ =
            n_.serviceClient<world_control_msgs::SpawnModel>(urosworldcontrol_domain_+"/spawn_model");
    //client for model deletion
    delete_client_ =
            n_.serviceClient<world_control_msgs::DeleteModel>(urosworldcontrol_domain_+"/delete_model");
    delete_all_client_ =
      n_.serviceClient<world_control_msgs::DeleteAll>(urosworldcontrol_domain_+"/delete_all");

    set_pose_client_ =
            n_.serviceClient<world_control_msgs::SetModelPose>(urosworldcontrol_domain_ + "/set_model_pose");
    get_pose_client_ =
            n_.serviceClient<world_control_msgs::GetModelPose>(urosworldcontrol_domain_ + "/get_model_pose");
    check_touch_ =
            n_.serviceClient<std_srvs::SetBool>("/unreal_interface/call_touching");

    pose_update_subscriber_ = n_.subscribe("/unreal_interface/object_poses",
            10,
            &UnrealInterface::Objects::TFUpdateCallback,
            this);
    state_update_subscriber_ = n_.subscribe("/unreal_interface/state_publisher",
            10,
            &UnrealInterface::Objects::StringUpdateCallback,
            this);
    touch_subscriber_ = n_.subscribe("/unreal_interface/physics_contact", 10, 
        &UnrealInterface::Objects::TFStateUpdateCallback, this);
}

bool UnrealInterface::Objects::TransportAvailable()
{
    return (
            spawn_client_.exists() &&
            delete_client_.exists() &&
            set_pose_client_.exists() &&
            get_pose_client_.exists() &&
            delete_all_client_.exists() &&
            check_touch_.exists()
    );
}


bool UnrealInterface::Objects::SpawnObject(world_control_msgs::SpawnModel model, UnrealInterface::Object::Id *id_of_spawned_object = nullptr)
{
    // Add additional information to the request that is necessary for the functionality
    // of UnrealInterface.

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    // Set a tag so we can uniquely identify the spawned objects in UE4
    world_control_msgs::Tag tag;
    tag.type = DEFAULT_SPAWN_TAG_TYPE;
    tag.key = DEFAULT_SPAWN_TAG_KEY;
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
        if (model.response.etype == "1")
        {
            ROS_ERROR("Spawnmodel Error01: Object is not unique and wasn't spawned.");
        }
        else if (model.response.etype == "2")
        {
            ROS_ERROR("Spawnmodel Error02: Object Spawnlocation is obstructed and wasn't spawned.");
        }
        return false;
    }

    //save object in object map
    UnrealInterface::Object::ObjectInfo object_info;
    object_info.id_ = model.response.id;
    object_info.actor_name_ = model.response.name;

    // TODO - Use mutex here too?
    this->spawned_objects_[model.response.id] = object_info;

    //print the ID of the spawned hypothesis
    ROS_INFO_STREAM("UnrealInterface::Objects::SpawnObject: Object spawned with ID " << model.response.id << " and final actor name: " << model.response.name);

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
    if(spawned_objects_.count(model.request.id) == 0)
    {
      ROS_ERROR_STREAM("Couldn't find " << model.request.id <<
                       " in the spawned object mapping while calling DeleteModel." <<
                         "Show UIO::PrintAllObjectInfo to debug.");
      PrintAllObjectInfo();
      return false;
    }
    ROS_INFO("Erasing ID from UIO Representation");
    spawned_objects_.erase(model.request.id);
    ROS_INFO("DONE Erasing ID from UIO Representation");

    //check the status of the respond from the server
    if (!model.response.success)
    {
        ROS_ERROR("Service call returned success = false");
        return false;
    }


    return true;
}

void UnrealInterface::Objects::CleanSpawnOnDelete(UnrealInterface::Object::Id id)
{
        // Case in case it's not possible to delete but it's available in the spawned array. Seek for better way
        spawned_objects_.erase(id);
        std::cout << "Cleaning up..."  << std::endl;

}

UnrealInterface::Object::ObjectInfo UnrealInterface::Objects::GetObjectInfo(UnrealInterface::Object::Id id)
{
    if(spawned_objects_.count(id)==0)
        throw std::invalid_argument("The given ID is not in the spawned object representation");

    std::lock_guard<std::mutex> guard(object_info_mutex_);
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
    ROS_INFO_STREAM("UnrealInterface::Objects::SetObjectPose :: Send object pose update to UE4");

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

bool UnrealInterface::Objects::GetObjectPose(UnrealInterface::Object::Id id, geometry_msgs::Pose &pose)
{
    world_control_msgs::GetModelPose getmodelpose_srv;

    // We assume that the UnrealInterface::Object::Id is equal to the semlog Id atm
    getmodelpose_srv.request.id = id;

    if (!get_pose_client_.call(getmodelpose_srv))
    {
        ROS_ERROR("Failed to call service client for GetModelPose");
        return false;
    }

    if (!getmodelpose_srv.response.success)
    {
        ROS_ERROR("GetModelPose Service received non-success response during update");

        return false;
    }

    pose.position = getmodelpose_srv.response.pose.position;
    pose.orientation = getmodelpose_srv.response.pose.orientation;

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

    // We make a copy of the spawned objects list first.
    // This is due to the removal of some of the objects
    // in DeleteObject which happens during the execution.
    // This might cause problems.
    // TODO: Make this more elegant without having to copy this map
    std::map<UnrealInterface::Object::Id, UnrealInterface::Object::ObjectInfo>
        copy_of_spawned_objects_ = spawned_objects_;

    for(auto const pair : copy_of_spawned_objects_)
    {
      UnrealInterface::Object::Id obj_id = pair.first;
      if(!DeleteObject(obj_id))
      {
        ROS_INFO("DeleteObject returned false in DeleteAllSpawnedObjects");
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


bool UnrealInterface::Objects::DeleteAllSpawnedObjectsByTag()
{
  ROS_INFO_STREAM("Deleting all objects with key " << DEFAULT_SPAWN_TAG_KEY << " and type " << DEFAULT_SPAWN_TAG_TYPE);

  world_control_msgs::DeleteAll srv;
  srv.request.type_to_delete = DEFAULT_SPAWN_TAG_TYPE;
  srv.request.key_to_delete = DEFAULT_SPAWN_TAG_KEY;
  srv.request.ignore_value = true;
  if(!delete_all_client_.call(srv)) {
    ROS_INFO("DeleteAll service returned false in DeleteAllSpawnedObjectsByTag");
    return false;
  }

  // Delete all objects from our representation
  spawned_objects_.clear();

  return true;
}

bool UnrealInterface::Objects::CheckTouchOnObjects()
{
    ROS_INFO_STREAM("Activating the Touchbool.");

    std_srvs::SetBool srv;
    srv.request.data = true;
    if(!check_touch_.call(srv)) 
    {
        ROS_INFO("CheckTouch service Returned false");
        return false;
    }

    return true;
}

/*
void UnrealInterface::Objects::PoseUpdateCallback(const geometry_msgs::PoseStamped& pose_stamped_msg)
{
    std::string object_id = pose_stamped_msg.header.frame_id;

    if(spawned_objects_.count(object_id)==0)
    {
        ROS_INFO_STREAM("The given ID is not in the spawned object representation. Ignoring pose update");
        return;
    }

    std::lock_guard<std::mutex> guard(object_info_mutex_);
    spawned_objects_[object_id].pose_ = pose_stamped_msg.pose;
}
*/

void UnrealInterface::Objects::TFUpdateCallback(const tf::tfMessage& tf_message)
{
    for (geometry_msgs::TransformStamped varTransform : tf_message.transforms)
    {
        std::string object_id = varTransform.header.frame_id;

        if(spawned_objects_.count(object_id)==0)
        {
            ROS_INFO_STREAM("The given ID is not in the spawned object representation. Ignoring transform update");
            return;
        }

        std::lock_guard<std::mutex> guard(object_info_mutex_);
        spawned_objects_[object_id].transform_ = varTransform.transform;
    }
}

void UnrealInterface::Objects::TFStateUpdateCallback(const tf::tfMessage& tf_message)
{
    for (geometry_msgs::TransformStamped varTransform : tf_message.transforms)
    {
        TouchHappenings = varTransform.child_frame_id;
    }
}

std::string UnrealInterface::Objects::GetTouchString()
{
    return TouchHappenings;
}

std::string UnrealInterface::Objects::GetStateString()
{
    return InputStateStream;
}

void UnrealInterface::Objects::StringUpdateCallback(const std_msgs::String& string_message)
{
    std::string data = string_message.data;
    InputStateStream = data;
}