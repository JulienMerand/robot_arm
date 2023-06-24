#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

std::string frame_id;

auto const create_collision_box(std::string id, float dim_x, float dim_Y, float dim_Z, float pos_X, float pos_Y, float pos_Z)
{   
    // Create collision object for the robot to avoid
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = dim_x;
    primitive.dimensions[primitive.BOX_Y] = dim_Y;
    primitive.dimensions[primitive.BOX_Z] = dim_Z;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = pos_X;
    box_pose.position.y = pos_Y;
    box_pose.position.z = pos_Z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}


int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "robot_arm");
    frame_id = move_group_interface.getPlanningFrame();
    
    auto const ground = create_collision_box("ground", 0.8, 0.4, 0.06, 0.1, 0.0, -0.03);
    auto const cmd_box = create_collision_box("cmd_box", 0.15, 0.4, 0.15, -0.225, 0.0, 0.075);

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(ground);
    planning_scene_interface.applyCollisionObject(cmd_box);


    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}

