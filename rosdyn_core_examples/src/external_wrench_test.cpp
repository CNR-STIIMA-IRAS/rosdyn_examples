#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <rosdyn_core/urdf_parser.h>
#include <subscription_notifier/subscription_notifier.h>
int main(int argc, char **argv){
  ros::init(argc, argv, "inverse_dynamics");
  
  ros::NodeHandle nh;
  ros::Rate rate(125);
  
  std::string base_frame = "base_link";
  std::string tool_frame = "tool0";                 //"ee_link";

  urdf::Model model;
  model.initParam("robot_description");
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  rosdyn::ChainPtr chain = rosdyn::createChain(model,base_frame,tool_frame,grav);

  Eigen::VectorXd q(6);
  q.setZero();
  Eigen::VectorXd q2(6);
  q2.setZero();
  Eigen::VectorXd Dq(6);
  Dq.setZero();
  Eigen::VectorXd DDq(6);
  DDq.setZero();
  Eigen::VectorXd DDDq(6);
  DDDq.setZero();
  Eigen::VectorXd tau(6);
  Eigen::VectorXd tau_model(6);
  tau.setZero();

  rosdyn::VectorOfVector6d vec;
  Eigen::Vector6d zeros;
  zeros.setZero();
  for (unsigned int iAx=0;iAx<chain->getLinksNumber();iAx++)
    vec.push_back(zeros);
  vec.back().setConstant(1000.0);

  for (int idx=0;idx<10;idx++)
  {
    q.setRandom();
    Eigen::VectorXd torque_no_force  = chain->getJointTorque(q,Dq,DDq);
    //Eigen::VectorXd torque_no_force2 = chain->getJointTorque(q2,Dq,DDq);
    Eigen::VectorXd torque           = chain->getJointTorque(q,Dq,DDq,vec);

    ROS_INFO_STREAM("torque no force = " << torque_no_force.transpose());
    ROS_INFO_STREAM("torque          = " << torque.transpose());
  }
  return 0;
}
