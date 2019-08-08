//
// Created by sdhm on 8/8/19.
//

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char **argv) {

    /// ***************************** 四元数、旋转矩阵、欧拉角 ***************************** ///

    // 初始化四元数,注意eigen Quaterniond类四元数初始化参数顺序为w,x,y,z
    Eigen::Quaterniond w2c_quat(-0.111, 0.702, -0.694, 0.115);
    w2c_quat = w2c_quat.normalized(); // 归一化
    printf("Eigen 四元数初始化: x:%f y:%f z:%f w:%f\n", w2c_quat.x(), w2c_quat.y(),  w2c_quat.z(),  w2c_quat.w());

    // 初始化旋转矩阵
    Eigen::Matrix3d w2c_rot;
//    w2c_rot << 0.707107, -0.707107, 0, 0.707107, 0.707107, 0, 0, 0, 1;

    // 初始化欧拉角(YPR, 先绕z轴yaw, 再绕y轴pitch, 最后绕x轴roll)
    Eigen::Vector3d w2c_ea(-1.561, -0.007, -2.822);

    // 欧拉角转四元数
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(w2c_ea[0], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(w2c_ea[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(w2c_ea[2], Eigen::Vector3d::UnitX());
    printf("Eigen 欧拉角转四元数: x:%f y:%f z:%f w:%f\n", quat.x(), quat.y(),  quat.z(),  quat.w());

    // 四元数转欧拉角
    Eigen::Vector3d eulerAngle = w2c_quat.matrix().eulerAngles(2,1,0);
    printf("Eigen 四元数转欧拉角: r:%f p:%f y:%f\n", eulerAngle.z(), eulerAngle.y(), eulerAngle.x()); // rpy-zyx
    cout << "Eigen 四元数转欧拉角: yaw(z) pitch(y) roll(x) = " << eulerAngle.transpose() << endl;

    // 四元数转旋转矩阵
    w2c_rot = w2c_quat.matrix();
    cout << "Eigen 四元数转旋转矩阵 =\n" << w2c_rot << endl;

    // tf 欧拉角转四元数
    tf::Quaternion tf_q;
    tf_q.setRPY(-2.822, -0.007, -1.561);
    printf("TF 欧拉角转四元数: x:%f y:%f z:%f w:%f\n", tf_q.getX(), tf_q.getY(), tf_q.getZ(), tf_q.getW());

    // tf 四元数转欧拉角
    tf::Matrix3x3 mat_rot(tf_q);
    double roll, pitch, yaw;
    mat_rot.getRPY(roll, pitch, yaw);
    printf("TF 四元数转欧拉角: r:%f p:%f y:%f\n", roll, pitch, yaw);

    // tf 四元数转旋转矩阵
    printf("TF 四元数转旋转矩阵: \n%f %f %f \n%f %f %f \n%f %f %f\n",
            mat_rot.getRow(0).getX(), mat_rot.getRow(0).getY(), mat_rot.getRow(0).getZ(),
            mat_rot.getRow(1).getX(), mat_rot.getRow(1).getY(), mat_rot.getRow(1).getZ(),
            mat_rot.getRow(2).getX(), mat_rot.getRow(2).getY(), mat_rot.getRow(2).getZ());

    // Eigen四元数转tf四元数
    tf::Quaternion tf_quat;
    tf::quaternionEigenToTF(w2c_quat, tf_quat);
    printf("Eigen四元数转tf四元数: x:%f y:%f z:%f w:%f\n", tf_quat.getX(), tf_quat.getY(), tf_quat.getZ(), tf_quat.getW());

    /// *********************************** 平移向量 *********************************** ///

    // Eigen平移向量初始化
    Eigen::Vector3d w2c_trans (0.266, 0.045, 0.586);
    // Eigen平移向量转tf平移向量
    tf::Vector3 tf_trans;
    tf::vectorEigenToTF(w2c_trans, tf_trans);
    printf("Eigen平移向量转tf平移向量: x:%f y:%f z:%f\n", tf_trans.getX(), tf_trans.getY(), tf_trans.getZ());

    /// *********************************** 坐标系转换 *********************************** ///
    Eigen::Vector3d c2o_trans (-0.084833547473, -0.0389130488038, 0.503000020981); // 相机坐标系到物体坐标系平移矩阵 NOTE: trans(相机->物体)

    Eigen::Vector3d c2o_ea(90 * M_PI/180.0, 0, 0); // YPR, 先绕z轴yaw, 再绕y轴pitch, 最后绕x轴roll
    Eigen::Quaterniond c2o_quat;
    c2o_quat = Eigen::AngleAxisd(c2o_ea[0], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(c2o_ea[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(c2o_ea[2], Eigen::Vector3d::UnitX());

    Eigen::Isometry3d T_c2o = Eigen::Isometry3d::Identity(); // 相机坐标系到物体坐标系齐次变换矩阵 NOTE: T(世界->相机)
    T_c2o.rotate(c2o_quat);  // 按照旋转矩阵进行旋转
    T_c2o.pretranslate(c2o_trans);  // 按照平移矩阵进行平移

    // 旋转矩阵表示旋转
//    Eigen::Vector3d w2o_rotated = w2c_rot * c2o_trans;
//    cout << "obj_loc after rotation: " << w2o_rotated.transpose() << endl;
//    // 平移矩阵表示平移
//    Eigen::Vector3d w2o_transed = c2o_trans + w2c_trans;
//    cout << "obj_loc after trans: " << w2o_transed.transpose() << endl;
//    // 旋转加平移矩阵表示旋转和平移
//    Eigen::Vector3d w2o_rot_trans = w2o_rotated + w2c_trans;
//    cout << "obj_loc after rotation and trans: " << w2o_rot_trans.transpose() << endl;

    // 齐次变换矩阵表示旋转和平移
    Eigen::Isometry3d T_w2c = Eigen::Isometry3d::Identity(); // 世界坐标系到相机坐标系齐次变换矩阵 NOTE: T(世界->相机)
    T_w2c.rotate(w2c_rot);  // 按照旋转矩阵进行旋转
    T_w2c.pretranslate(w2c_trans);  // 按照平移矩阵进行平移

//    Eigen::Isometry3d T_c2b = T_w2c.inverse(); // 相机坐标系到世界坐标系齐次变换矩阵

    /// 世界坐标系下物体坐标 = T(世界->相机)*trans(相机->物体)
//    Eigen::Vector3d w2o_trans = T_w2c*c2o_trans; // 相当于R*v+t
//
//    cout << "obj_w2c(correct):\n" << w2o_trans << endl;

    /// 世界坐标系下物体姿态(物体坐标系) = T(世界->相机)*T(相机->物体)
    Eigen::Isometry3d T_w2o = T_w2c*T_c2o; // 相当于R*v+t
    Eigen::Vector3d w2o_trans = T_w2o.translation();
    Eigen::Matrix3d w2o_rot = T_w2o.rotation();
    Eigen::Quaterniond w2o_quat(w2o_rot);

    cout << "w2o_trans:\n" << w2o_trans << "\nw2o_rot:\n"<< w2o_rot;

    /// 物体tf
    tf::Vector3 w2o_tf_trans;
    tf::vectorEigenToTF(w2o_trans, w2o_tf_trans);
    tf::Quaternion w2o_tf_quat;
    tf::quaternionEigenToTF(w2o_quat, w2o_tf_quat);

    /// *********************************** 发布tf *********************************** ///
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle node;
    static tf::TransformBroadcaster camera_br, obj_br;

    // 相机坐标系
    tf::Transform transform;
    transform.setOrigin(tf_trans);
    transform.setRotation(tf_quat);

    // 物体位置
    tf::Transform transform_obj;
    transform_obj.setOrigin(w2o_tf_trans);
    transform_obj.setRotation(w2o_tf_quat);

    ros::Rate rate(10.0);
    while (node.ok()) {
        camera_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera"));
        obj_br.sendTransform(tf::StampedTransform(transform_obj, ros::Time::now(), "world", "obj"));
        rate.sleep();
    }

    return 0;


}

//机器人基坐标系到相机坐标系：base_link -> camera_color_optical_frame
//- Translation: [0.266, 0.045, 0.586]
//- Rotation: in Quaternion [0.702, -0.694, 0.115, -0.111]  /// ROS中quat为[x,y,z,w]
//in RPY (radian) [-2.822, -0.007, -1.561]
//in RPY (degree) [-161.674, -0.428, -89.429]
//
//物体位置：
//相机坐标系下：
//xyz[ -0.084833547473 -0.0389130488038 0.503000020981 ]
//Angle: -29.6498641968
//
//机器人基坐标系下：
//Pose:
//x: 0.461560861676
//y: 0.128692270563
//z: 0.120652332092
//rotation_matrix:
//[[ 8.69064725e-01 -4.94698396e-01  6.05830807e-17]
//[-4.94698396e-01 -8.69064725e-01  1.06429733e-16]
//[ 0.00000000e+00 -1.22464680e-16 -1.00000000e+00]]