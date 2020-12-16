#include <gtest/gtest.h>
#include <ros/ros.h>
#include <chrono>
#include <thread>

#include <unreal_interface/object.h>

/**
 * This series of unit tests assumes that it is run into a UE4 instance with the necessary Plugins
 * mentioned in the README.md AND some meshes/models that are usually shipped with
 * RobCog (https://github.com/robcog-iai/RobCog).
 */

std::shared_ptr<UnrealInterface::Objects> uio;

TEST(TestSuite, transportShouldBeAvailable)
{
    ASSERT_TRUE(uio->TransportAvailable());
}
// Declare a test
TEST(TestSuite, SpawnObject)
{
    world_control_msgs::SpawnModel spawn_model_srv;

    spawn_model_srv.request.name = "AlbiHimbeerJuice"; // This is used to lookup the actual model

    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    spawn_model_srv.request.pose.position.x = -0.60;
    spawn_model_srv.request.pose.position.y = -2.40;
    spawn_model_srv.request.pose.position.z = 1.00;
    spawn_model_srv.request.pose.orientation.x = 0;
    spawn_model_srv.request.pose.orientation.y = 0;
    spawn_model_srv.request.pose.orientation.z = 0;
    spawn_model_srv.request.pose.orientation.w = 1;

    spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.STATIONARY;

    // Assigning the label that is also used as a reference in the object map
    // This must be unique!
    spawn_model_srv.request.actor_label = "TestObjectLabel";

    // Last step. Spawn the actual model.
    UnrealInterface::Object::Id id_of_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_object_in_unreal));

    ASSERT_EQ(uio->SpawnedObjectCount(), 1);
    uio->PrintAllObjectInfo();

    ros::Duration(1.5).sleep();

    // Try to delete the same object again after a couple of secs
    ASSERT_TRUE(uio->DeleteObject(id_of_object_in_unreal));
    ros::Duration(0.5).sleep();
}

// Declare another test
TEST(TestSuite, GetNonExistingObjectInfo)
{
    ASSERT_ANY_THROW(uio->GetObjectInfo("foobar"));
}

TEST(TestSuite, SetObjectPose)
{
    world_control_msgs::SpawnModel spawn_model_srv;

    spawn_model_srv.request.name = "AlbiHimbeerJuice"; // This is used to lookup the actual model

    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    spawn_model_srv.request.pose.position.x = -0.60;
    spawn_model_srv.request.pose.position.y = -2.40;
    spawn_model_srv.request.pose.position.z = 1.00;
    spawn_model_srv.request.pose.orientation.x = 0;
    spawn_model_srv.request.pose.orientation.y = 0;
    spawn_model_srv.request.pose.orientation.z = 0;
    spawn_model_srv.request.pose.orientation.w = 1;

    spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.DYNAMIC;

    // Assigning the label that is also used as a reference in the object map
    // This must be unique!
    spawn_model_srv.request.actor_label = "SetObjectPoseLabel";

    // Last step. Spawn the actual model.
    UnrealInterface::Object::Id id_of_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_object_in_unreal));

    ASSERT_EQ(uio->SpawnedObjectCount(), 1);

    ros::Duration(0.5).sleep();

    // Actual Set Model Pose stuff
    geometry_msgs::Pose pose;
    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    // Make it a little higher (z axis) than before.
    pose.position.x = -0.60;
    pose.position.y = -2.40;
    pose.position.z = 1.50;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    ASSERT_TRUE(uio->SetObjectPose(id_of_object_in_unreal, pose));

    ASSERT_EQ(uio->SpawnedObjectCount(), 1);

    ros::Duration(1.0).sleep();
    // Try to delete the same object again after a couple of secs
    ASSERT_TRUE(uio->DeleteObject(id_of_object_in_unreal));
    ros::Duration(1.0).sleep();

    // After deleting, calling SetObjectPose again should fail
    // and no spawned objects should be left in the data management
    // of UnrealInterface::Objects.
    ASSERT_EQ(uio->SpawnedObjectCount(), 0);
    ASSERT_FALSE(uio->SetObjectPose(id_of_object_in_unreal, pose));
    ros::Duration(1.0).sleep();
}



TEST(TestSuite, DeleteAllSpawnedObjects)
{
    world_control_msgs::SpawnModel spawn_model_srv;

    spawn_model_srv.request.name = "AlbiHimbeerJuice"; // This is used to lookup the actual model

    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    spawn_model_srv.request.pose.position.x = -0.60;
    spawn_model_srv.request.pose.position.y = -2.40;
    spawn_model_srv.request.pose.position.z = 1.00;
    spawn_model_srv.request.pose.orientation.x = 0;
    spawn_model_srv.request.pose.orientation.y = 0;
    spawn_model_srv.request.pose.orientation.z = 0;
    spawn_model_srv.request.pose.orientation.w = 1;

    spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.STATIONARY;

    // Assigning the label that is also used as a reference in the object map
    // This must be unique!
    spawn_model_srv.request.actor_label = "TestObject1Label";

    // Last step. Spawn the actual model.
    UnrealInterface::Object::Id id_of_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_object_in_unreal));

    ////////////////// Second Object //////////////////
    ///////////////////////////////////////////////////
    world_control_msgs::SpawnModel spawn_model_srv2;

    spawn_model_srv2.request.name = "KoellnMuesliKnusperHonigNuss"; // This is used to lookup the actual model

    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    spawn_model_srv2.request.pose.position.x = -0.60;
    spawn_model_srv2.request.pose.position.y = -2.10;
    spawn_model_srv2.request.pose.position.z = 1.00;
    spawn_model_srv2.request.pose.orientation.x = 0;
    spawn_model_srv2.request.pose.orientation.y = 0;
    spawn_model_srv2.request.pose.orientation.z = 0;
    spawn_model_srv2.request.pose.orientation.w = 1;

    spawn_model_srv2.request.physics_properties.mobility = spawn_model_srv2.request.physics_properties.STATIONARY;

    // Assigning the label that is also used as a reference in the object map
    // This must be unique!
    spawn_model_srv2.request.actor_label = "TestObject2Label";

    // Last step. Spawn the actual model.
    UnrealInterface::Object::Id id_of_object2_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv2, &id_of_object2_in_unreal));

    ASSERT_EQ(uio->SpawnedObjectCount(), 2);
    uio->PrintAllObjectInfo();
    ros::Duration(1.0).sleep();

    // Try to delete the same object again after a couple of secs
    ASSERT_TRUE(uio->DeleteAllSpawnedObjects());
    ASSERT_EQ(uio->SpawnedObjectCount(), 0);
    ros::Duration(0.5).sleep();
}


TEST(TestSuite, DeleteAllSpawnedObjectsByTag)
{
  world_control_msgs::SpawnModel spawn_model_srv;

  spawn_model_srv.request.name = "AlbiHimbeerJuice"; // This is used to lookup the actual model

  // Set pose in the UE4 world frame, but with ROS coordinate conventions
  spawn_model_srv.request.pose.position.x = -0.60;
  spawn_model_srv.request.pose.position.y = -2.40;
  spawn_model_srv.request.pose.position.z = 1.00;
  spawn_model_srv.request.pose.orientation.x = 0;
  spawn_model_srv.request.pose.orientation.y = 0;
  spawn_model_srv.request.pose.orientation.z = 0;
  spawn_model_srv.request.pose.orientation.w = 1;

  spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.STATIONARY;

  // Assigning the label that is also used as a reference in the object map
  // This must be unique!
  spawn_model_srv.request.actor_label = "TestObject1Label";

  // Last step. Spawn the actual model.
  UnrealInterface::Object::Id id_of_object_in_unreal;
  ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_object_in_unreal));

  ////////////////// Second Object //////////////////
  ///////////////////////////////////////////////////
  world_control_msgs::SpawnModel spawn_model_srv2;

  spawn_model_srv2.request.name = "KoellnMuesliKnusperHonigNuss"; // This is used to lookup the actual model

  // Set pose in the UE4 world frame, but with ROS coordinate conventions
  spawn_model_srv2.request.pose.position.x = -0.60;
  spawn_model_srv2.request.pose.position.y = -2.10;
  spawn_model_srv2.request.pose.position.z = 1.00;
  spawn_model_srv2.request.pose.orientation.x = 0;
  spawn_model_srv2.request.pose.orientation.y = 0;
  spawn_model_srv2.request.pose.orientation.z = 0;
  spawn_model_srv2.request.pose.orientation.w = 1;

  spawn_model_srv2.request.physics_properties.mobility = spawn_model_srv2.request.physics_properties.STATIONARY;

  // Assigning the label that is also used as a reference in the object map
  // This must be unique!
  spawn_model_srv2.request.actor_label = "TestObject2Label";

  // Last step. Spawn the actual model.
  UnrealInterface::Object::Id id_of_object2_in_unreal;
  ASSERT_TRUE(uio->SpawnObject(spawn_model_srv2, &id_of_object2_in_unreal));

  ASSERT_EQ(uio->SpawnedObjectCount(), 2);
  uio->PrintAllObjectInfo();
  ros::Duration(1.0).sleep();

  // Delete All Objects
  ASSERT_TRUE(uio->DeleteAllSpawnedObjectsByTag());
  ASSERT_EQ(uio->SpawnedObjectCount(), 0);
  ros::Duration(0.5).sleep();
}

TEST(TestSuite, GetObjectPose)
{
    world_control_msgs::SpawnModel spawn_model_srv;

    spawn_model_srv.request.name = "AlbiHimbeerJuice"; // This is used to lookup the actual model

    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    spawn_model_srv.request.pose.position.x = -0.60;
    spawn_model_srv.request.pose.position.y = -2.40;
    spawn_model_srv.request.pose.position.z = 1.00;
    spawn_model_srv.request.pose.orientation.x = 0;
    spawn_model_srv.request.pose.orientation.y = 0;
    spawn_model_srv.request.pose.orientation.z = 0;
    spawn_model_srv.request.pose.orientation.w = 1;

    spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.STATIONARY;

    // Assigning the label that is also used as a reference in the object map
    // This must be unique!
    spawn_model_srv.request.actor_label = "TestObjectLabel";

    // Last step. Spawn the actual model.
    UnrealInterface::Object::Id id_of_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_object_in_unreal));
    ASSERT_EQ(uio->SpawnedObjectCount(), 1);

    ros::Duration(1.0).sleep();

    geometry_msgs::Pose result_pose;
    ASSERT_TRUE(uio->GetObjectPose(id_of_object_in_unreal,result_pose));
    ASSERT_FLOAT_EQ(result_pose.position.x, spawn_model_srv.request.pose.position.x);
    ASSERT_FLOAT_EQ(result_pose.position.y, spawn_model_srv.request.pose.position.y);
    ASSERT_FLOAT_EQ(result_pose.position.z, spawn_model_srv.request.pose.position.z);
    ASSERT_FLOAT_EQ(result_pose.orientation.x, spawn_model_srv.request.pose.orientation.x);
    ASSERT_FLOAT_EQ(result_pose.orientation.y, spawn_model_srv.request.pose.orientation.y);
    ASSERT_FLOAT_EQ(result_pose.orientation.z, spawn_model_srv.request.pose.orientation.z);
    ASSERT_FLOAT_EQ(result_pose.orientation.w, spawn_model_srv.request.pose.orientation.w);

    // Try to delete the same object again after a couple of secs
    ASSERT_TRUE(uio->DeleteObject(id_of_object_in_unreal));
    ros::Duration(0.5).sleep();
}


TEST(TestSuite, GetObjectPoseAsynchronously)
{
    // Instead of requesting the ObjectPose directly and synchronously,
    // we'll test the async update of UnrealInterface::Objects::GetObjectInfo(UnrealInterface::Object::Id id);

    world_control_msgs::SpawnModel spawn_model_srv;

    spawn_model_srv.request.name = "AlbiHimbeerJuice"; // This is used to lookup the actual model

    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    spawn_model_srv.request.pose.position.x = -0.60;
    spawn_model_srv.request.pose.position.y = -2.40;
    spawn_model_srv.request.pose.position.z = 1.00;
    spawn_model_srv.request.pose.orientation.x = 0;
    spawn_model_srv.request.pose.orientation.y = 0;
    spawn_model_srv.request.pose.orientation.z = 0;
    spawn_model_srv.request.pose.orientation.w = 1;

    spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.DYNAMIC;

    // Assigning the label that is also used as a reference in the object map
    // This must be unique!
    spawn_model_srv.request.actor_label = "GetObjectPoseAsynchronously";

    // Last step. Spawn the actual model.
    UnrealInterface::Object::Id id_of_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_object_in_unreal));
    ASSERT_EQ(uio->SpawnedObjectCount(), 1);

    // Sleep for a while so we definitely get an update even with the slow default update rate
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();

    // Request the pose info
    UnrealInterface::Object::ObjectInfo info = uio->GetObjectInfo(id_of_object_in_unreal);
    ASSERT_FLOAT_EQ(spawn_model_srv.request.pose.position.x, info.transform_.translation.x);
    ASSERT_FLOAT_EQ(spawn_model_srv.request.pose.position.y, info.transform_.translation.y);
    ASSERT_FLOAT_EQ(spawn_model_srv.request.pose.position.z, info.transform_.translation.z);
    ASSERT_FLOAT_EQ(spawn_model_srv.request.pose.orientation.x, info.transform_.rotation.x);
    ASSERT_FLOAT_EQ(spawn_model_srv.request.pose.orientation.y, info.transform_.rotation.y);
    ASSERT_FLOAT_EQ(spawn_model_srv.request.pose.orientation.z, info.transform_.rotation.z);
    ASSERT_FLOAT_EQ(spawn_model_srv.request.pose.orientation.w, info.transform_.rotation.w);


    // Actual Set Model Pose stuff
    geometry_msgs::Pose pose;
    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    // Make it a little higher (z axis) than before.
    pose.position.x = -0.60;
    pose.position.y = -2.40;
    pose.position.z = 1.50;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    ASSERT_TRUE(uio->SetObjectPose(id_of_object_in_unreal, pose));

    ASSERT_EQ(uio->SpawnedObjectCount(), 1);

    // Sleep for a while so we definitely get an update even with the slow default update rate
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();

    // Request the pose info again and check if it changed after the SetObjectPose
    UnrealInterface::Object::ObjectInfo info2 = uio->GetObjectInfo(id_of_object_in_unreal);
    ASSERT_FLOAT_EQ(pose.position.x, info2.transform_.translation.x);
    ASSERT_FLOAT_EQ(pose.position.y, info2.transform_.translation.y);
    ASSERT_FLOAT_EQ(pose.position.z, info2.transform_.translation.z);
    ASSERT_FLOAT_EQ(pose.orientation.x, info2.transform_.rotation.x);
    ASSERT_FLOAT_EQ(pose.orientation.y, info2.transform_.rotation.y);
    ASSERT_FLOAT_EQ(pose.orientation.z, info2.transform_.rotation.z);
    ASSERT_FLOAT_EQ(pose.orientation.w, info2.transform_.rotation.w);

    // Try to delete the same object again after a couple of secs
    ASSERT_TRUE(uio->DeleteObject(id_of_object_in_unreal));
    ros::Duration(1.0).sleep();
}

TEST(TestSuite, DeleteAllSpawnedObjects2)
{
    // Spawn Three Objects
    world_control_msgs::SpawnModel spawn_model_srv;

    spawn_model_srv.request.name = "AlbiHimbeerJuice"; // This is used to lookup the actual model

    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    spawn_model_srv.request.pose.position.x = -0.60;
    spawn_model_srv.request.pose.position.y = -2.40;
    spawn_model_srv.request.pose.position.z = 1.00;
    spawn_model_srv.request.pose.orientation.x = 0;
    spawn_model_srv.request.pose.orientation.y = 0;
    spawn_model_srv.request.pose.orientation.z = 0;
    spawn_model_srv.request.pose.orientation.w = 1;

    spawn_model_srv.request.actor_label = "DeleteObjet1";

    UnrealInterface::Object::Id id_of_first_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_first_object_in_unreal));

    spawn_model_srv.request.pose.position.y = -2.60;
    spawn_model_srv.request.actor_label = "DeleteObjet2";
    UnrealInterface::Object::Id id_of_second_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_second_object_in_unreal));

    spawn_model_srv.request.pose.position.y = -2.10;
    spawn_model_srv.request.actor_label = "DeleteObjet3";
    UnrealInterface::Object::Id id_of_third_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_third_object_in_unreal));

    ASSERT_EQ(uio->SpawnedObjectCount(), 3);

    ASSERT_TRUE(uio->DeleteAllSpawnedObjects());
    ASSERT_EQ(uio->SpawnedObjectCount(), 0);
}

TEST(TestSuite, DeleteOnSpawn)
{
    // Spawn Three Objects
    world_control_msgs::SpawnModel spawn_model_srv;

    spawn_model_srv.request.name = "AlbiHimbeerJuice"; // This is used to lookup the actual model

    // Set pose in the UE4 world frame, but with ROS coordinate conventions
    spawn_model_srv.request.pose.position.x = -0.60;
    spawn_model_srv.request.pose.position.y = -2.40;
    spawn_model_srv.request.pose.position.z = 1.00;
    spawn_model_srv.request.pose.orientation.x = 0;
    spawn_model_srv.request.pose.orientation.y = 0;
    spawn_model_srv.request.pose.orientation.z = 0;
    spawn_model_srv.request.pose.orientation.w = 1;

    spawn_model_srv.request.actor_label = "DeleteObjet1";

    UnrealInterface::Object::Id id_of_first_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_first_object_in_unreal));

    spawn_model_srv.request.actor_label = "DeleteObjet2";
    UnrealInterface::Object::Id id_of_second_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_second_object_in_unreal));
    uio->DeleteObject(id_of_second_object_in_unreal);

    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    ros::Duration(0.25).sleep();
    ros::spinOnce();

    ASSERT_EQ(uio->SpawnedObjectCount(), 1);
    ASSERT_TRUE(uio->DeleteAllSpawnedObjects());
    ASSERT_EQ(uio->SpawnedObjectCount(), 0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "unreal_interface_tester");
    ros::NodeHandle nh;
    uio = std::make_shared<UnrealInterface::Objects>(nh);
    return RUN_ALL_TESTS();
}