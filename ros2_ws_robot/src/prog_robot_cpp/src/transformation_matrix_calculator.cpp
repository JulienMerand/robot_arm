#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("transformation_matrix_calculator");

    // Create a listener to receive the transformations between the frames
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Wait for the availability of the transformations between the frames
    std::string source_frame = "link6_1"; // Remplacer par le nom du repère source
    std::string target_frame = "frame_effector"; // Remplacer par le nom du repère cible
    try {
        while (!tf_buffer.canTransform(target_frame, source_frame, tf2::TimePointZero)) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            RCLCPP_INFO(node->get_logger(), "Waiting for the frame...");
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node->get_logger(), "Erreur lors de la recherche de la transformation : %s", ex.what());
        return 1;
    }

    // Get the transformation between the frames
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node->get_logger(), "Erreur lors de l'obtention de la transformation : %s", ex.what());
        return 1;
    }

    // Convert the transformation to an Eigen::Isometry3d matrix
    // Eigen::Isometry3d transformation_matrix;
    // tf2::convert(transform, transformation_matrix);
    transform.transform.translation.x = 0;
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

    // Display the transformation matrix
    std::cout << "Matrice de rotation entre les repères " << source_frame << " et " << target_frame << ":" << std::endl;
    std::cout << transformation_matrix.matrix() << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}