// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <chrono>
#include <thread>

#include <unreal_interface/object.h>

std::shared_ptr<UnrealInterface::Objects> uio;

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
    spawn_model_srv.request.pose.orientation.w = 0;

    spawn_model_srv.request.physics_properties.mobility = spawn_model_srv.request.physics_properties.STATIONARY;

    // Assigning the label that is also used as a reference in the object map
    // This must be unique!
    spawn_model_srv.request.actor_label = "TestObjectLabel";

    // Last step. Spawn the actual model.
    UnrealInterface::Object::Id id_of_object_in_unreal;
    ASSERT_TRUE(uio->SpawnObject(spawn_model_srv, &id_of_object_in_unreal));

    ASSERT_EQ(uio->SpawnedObjectCount(), 1);
    uio->PrintAllObjectInfo();

    ros::Duration(2.5).sleep();

    // Try to delete the same object again after a couple of secs
    ASSERT_TRUE(uio->DeleteObject(id_of_object_in_unreal));
}

// Declare another test
TEST(TestSuite, getNonExistingObjectInfo)
{
    ASSERT_ANY_THROW(uio->GetObjectInfo("foobar"));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "unreal_interface_tester");
    ros::NodeHandle nh;
    uio = std::make_shared<UnrealInterface::Objects>(nh);
    return RUN_ALL_TESTS();
}