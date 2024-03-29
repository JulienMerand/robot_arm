#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iterator>  
#include <sstream>   
#include <algorithm> 
#include <math.h>

using std::placeholders::_1;

class Move : public rclcpp::Node
{
  public:
    Move(); 

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gripper_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Créer un listener pour recevoir les transformations entre les repères
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Create a ROS logger
    // auto const this->logger = rclcpp::get_logger("move");

    // Vecteur contenant toutes les positions reçues sur le topic '/pose_xyz_speed'
    std::vector<geometry_msgs::msg::Pose> all_poses;

    // Transformation between link6 and effector_final
    Eigen::Matrix4d TF_link6_effector_final;

  private:

    Eigen::Matrix4d get_transformation_matrix(const geometry_msgs::msg::TransformStamped transform);
    void pose_callback(const std_msgs::msg::Float32MultiArray msg0); 
    void gripper_callback(const std_msgs::msg::Int32 data);   
    void timer_callback();

};

Move::Move() : Node("move", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
               move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "robot_arm"),
               gripper_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "hand"),
               visual_tools_(std::shared_ptr<rclcpp::Node>(std::move(this)), "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_.getRobotModel()),
               tf_buffer_(this->get_clock()),
               tf_listener_(tf_buffer_)
{
    pose_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/pose_xyz_speed", 10, std::bind(&Move::pose_callback, this, _1));
    gripper_subscription_ = this->create_subscription<std_msgs::msg::Int32>("/pose_gripper", 10, std::bind(&Move::gripper_callback, this, _1));

    // Création d'un timer avec une période de 1 seconde
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000) , std::bind(&Move::timer_callback, this));

    /*---------------------------*/
    /* Transformation de repères */
    /*---------------------------*/

    // Attendre la disponibilité des transformations entre les repères
    std::string source_frame = "link6_1"; // Remplacez par le nom du repère source
    std::string target_frame = "frame_effector"; // Remplacez par le nom du repère cible
    try {
        while (!tf_buffer_.canTransform(source_frame, target_frame, tf2::TimePointZero)) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            RCLCPP_INFO(this->get_logger(), "Waiting for the frame : 'frame_effector' ...");
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Erreur lors de la recherche de la transformation : %s", ex.what());
    }

    // Obtenir la transformation entre les repères
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(source_frame, target_frame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Erreur lors de l'obtention de la transformation : %s", ex.what());
    }

    // Obtenir la matric de transformation entre l'axe 6 et l'effecteur final
    TF_link6_effector_final = get_transformation_matrix(transform);

    // // Afficher la matrice de transformation
    // std::cout << "Matrice de transformation entre les repères " << source_frame << " et " << target_frame << ":" << std::endl;
    // std::cout << transformation_matrix.matrix() << std::endl;

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

Eigen::Matrix4d Move::get_transformation_matrix(const geometry_msgs::msg::TransformStamped transform)
{
    tf2::Matrix3x3 rot;
    tf2::Quaternion quaternion;
    tf2::convert(transform.transform.rotation, quaternion);
    rot.setRotation(quaternion);

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix <<  rot[0][0], rot[0][1], rot[0][2],
                        rot[1][0], rot[1][1], rot[1][2],
                        rot[2][0], rot[2][1], rot[2][2];

    Eigen::Vector3d translation_vector(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity(); // Initialiser une matrice 4x4 à l'identité

    // Insérer la matrice de rotation 3x3 dans les coins supérieurs gauche de la matrice 4x4
    transformation_matrix.block(0, 0, 3, 3) = rotation_matrix;

    // Insérer le vecteur de translation 3x1 dans la dernière colonne de la matrice 4x4
    transformation_matrix.col(3).head(3) = translation_vector;

    return transformation_matrix;
}

void Move::gripper_callback(const std_msgs::msg::Int32 data)
{
    /*-----------------*/
    /* Display message */
    /*-----------------*/

    RCLCPP_INFO(this->get_logger(), "[GRIPPER] I heard: '%d'", data.data);

    /*-------------------------------------------------------*/
    /* On ajoute une position dans le vecteur all_poses      */
    /* pour prendre en compte le moment de bouger le gripper */
    /* On met tout à 0 sauf la composant w du quaternion     */
    /*-------------------------------------------------------*/

    if (data.data >= 90 || data.data <= 180)
    {
        auto const target_pose = [this, theta = data.data] {

            geometry_msgs::msg::Pose msg;

            msg.orientation.x = 0.0;
            msg.orientation.y = 0.0;
            msg.orientation.z = 0.0;
            msg.orientation.w = theta;
            msg.position.x = 0.0;
            msg.position.y = 0.0;
            msg.position.z = 0.0;
            
            return msg;
        }();
    
        all_poses.push_back(target_pose);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "[GRIPPER] Must be between 90 (open) and 180 (close)");
    }
    
}

void Move::pose_callback(const std_msgs::msg::Float32MultiArray msg0)
{
    /*-----------------*/
    /* Display message */
    /*-----------------*/

    std::ostringstream stream;
    if(!msg0.data.empty()){
        std::copy(msg0.data.begin(), msg0.data.end()-1, std::ostream_iterator<float>(stream,", "));
        stream << msg0.data.back();
    }
    RCLCPP_INFO(this->get_logger(), "[POSE] I heard: '%s'", stream.str().c_str());

    /*-------------------*/
    /* Set a target Pose */
    /*-------------------*/

    auto const target_pose = [this, pos = msg0, TF_link6_effector_final = this->TF_link6_effector_final] {

        geometry_msgs::msg::TransformStamped t;
        tf2::Quaternion quaternion;
        quaternion.setRPY(pos.data[5]*M_PI/180, pos.data[4]*M_PI/180, pos.data[3]*M_PI/180);
        // t.header.stamp = this->get_clock()->now();
        // t.header.frame_id = "base_link";
        // t.child_frame_id = "frame_effector";
        t.transform.translation.x = pos.data[0]/1000;
        t.transform.translation.y = pos.data[1]/1000;
        t.transform.translation.z = pos.data[2]/1000;
        t.transform.rotation.x = quaternion.getX();
        t.transform.rotation.y = quaternion.getY();
        t.transform.rotation.z = quaternion.getZ();
        t.transform.rotation.w = quaternion.getW();

        Eigen::Matrix4d TF_base_effector_final = get_transformation_matrix(t);
        Eigen::Matrix4d TF_base_link6 = TF_base_effector_final*TF_link6_effector_final.inverse();
        Eigen::Matrix3d rot_matrix = TF_base_link6.block(0,0,3,3);
        Eigen::Vector3d trans_matrix = TF_base_link6.col(3).head(3);
        geometry_msgs::msg::Pose msg;
        tf2::Quaternion q;
        Eigen::Vector3d euler_angles = rot_matrix.eulerAngles(2, 1, 0);
        q.setRPY(euler_angles[2], euler_angles[1], euler_angles[0]);

        msg.orientation.x = q.getX();
        msg.orientation.y = q.getY();
        msg.orientation.z = q.getZ();
        msg.orientation.w = q.getW();
        msg.position.x = trans_matrix[0];
        msg.position.y = trans_matrix[1];
        msg.position.z = trans_matrix[2];
        
        return msg;
    }();
    
    all_poses.push_back(target_pose);
}

void Move::timer_callback() 
{
    if (!all_poses.empty())
    {
        
        /*-----------------------------------*/
        /* Create closures for visualization */
        /*-----------------------------------*/
        visual_tools_.deleteAllMarkers();
        visual_tools_.loadRemoteControl();

        // move_group_.setPlanningTime(10.0);
        move_group_.allowReplanning(true);

        auto const draw_title = [visual_tools_ = &this->visual_tools_](auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;  // Place text 1m above the base link
            return msg;
        }();
        visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        };
        // auto const prompt = [visual_tools_ = &this->visual_tools_](auto text) {
        //     visual_tools_->prompt(text);
        // };
        auto const draw_trajectory_tool_path = 
        [visual_tools_ = &this->visual_tools_, jmg = this->move_group_.getRobotModel()->getJointModelGroup("robot_arm")](auto const trajectory) {
            visual_tools_->publishTrajectoryLine(trajectory, jmg);
        };


        

        geometry_msgs::msg::Pose const target_pose = all_poses.front();
        all_poses.erase(all_poses.begin());
        
        /*-------------------*/
        /* Move the Gripper  */
        /*-------------------*/

        if (target_pose.position.x == 0.0 && target_pose.position.y == 0.0 && target_pose.position.z == 0.0 
            && target_pose.orientation.x == 0.0 && target_pose.orientation.y == 0.0 && target_pose.orientation.z == 0.0) 
        {
            int theta = static_cast<int>(target_pose.orientation.w);

            if (theta < 135)
            {
                gripper_group_.setNamedTarget("open");
                RCLCPP_INFO(this->get_logger(), "[GRIPPER] Ouverture du gripper.");
            }
            else 
            {
                gripper_group_.setNamedTarget("close");
                RCLCPP_INFO(this->get_logger(), "[GRIPPER] Fermeture du gripper.");
            }
            
            /*-----------------------------------*/
            /* Create a plan to that target pose */
            /*-----------------------------------*/

            // prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
            draw_title("Planning");
            visual_tools_.trigger();
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            auto const ok = static_cast<bool>(gripper_group_.plan(my_plan));
            auto const [success, plan] = std::make_pair(ok, my_plan);


            /*------------------*/
            /* Execute the plan */
            /*------------------*/
            
            if (success) {
                draw_trajectory_tool_path(plan.trajectory_);
                visual_tools_.trigger();
                // prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
                draw_title("Executing");
                visual_tools_.trigger();
                gripper_group_.execute(plan);
            } else {
                draw_title("Planning Failed!");
                visual_tools_.trigger();
                RCLCPP_ERROR(this->get_logger(), "Planning failed!");
                // RCLCPP_ERROR(this->logger, "Planning failed!");
            }
            rclcpp::sleep_for(std::chrono::seconds(2));
        }

        /*-------------------*/
        /* Set a target Pose */
        /*-------------------*/
        
        else 
        {
            move_group_.setPoseTarget(target_pose);
        
            /*-----------------------------------*/
            /* Create a plan to that target pose */
            /*-----------------------------------*/

            // prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
            draw_title("Planning");
            visual_tools_.trigger();
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            auto const ok = static_cast<bool>(move_group_.plan(my_plan));
            auto const [success, plan] = std::make_pair(ok, my_plan);


            /*------------------*/
            /* Execute the plan */
            /*------------------*/
            
            if (success) {
                draw_trajectory_tool_path(plan.trajectory_);
                visual_tools_.trigger();
                // prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
                draw_title("Executing");
                visual_tools_.trigger();
                move_group_.execute(plan);
            } else {
                draw_title("Planning Failed!");
                visual_tools_.trigger();
                RCLCPP_ERROR(this->get_logger(), "Planning failed!");
                // RCLCPP_ERROR(this->logger, "Planning failed!");
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        
    }
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<Move>());

    auto node = std::make_shared<Move>();

    // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // auto spinner = std::thread([&executor]() { executor.spin(); });
    
    executor.spin();

    rclcpp::shutdown();
    // spinner.join();
    return 0;
}