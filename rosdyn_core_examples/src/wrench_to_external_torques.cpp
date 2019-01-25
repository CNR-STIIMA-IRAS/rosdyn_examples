#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

#include <rosdyn_core/urdf_parser.h>
#include <rosdyn_core/frame_distance.h>

#include <subscription_notifier/subscription_notifier.h>

Eigen::VectorXd q;
Eigen::VectorXd wrench;
sensor_msgs::JointState model_js;
sensor_msgs::JointState extra_js;

void jointStateCallback( const sensor_msgs::JointStateConstPtr& msg )
{
  if( model_js.name.size() <=0)
    return;

  if( q.rows() != model_js.name.size() )
  {
    q.resize(model_js.name.size());
    q.setZero();
  }
  for( unsigned int i=0; i < model_js.name.size(); i++  )
  {
    for( unsigned int j=0; j < msg->name.size(); j++  )
    {
      if( !strcmp(model_js.name.at(i).c_str(), msg->name.at(j).c_str() ) )
      {
       q(i)= msg->position.at(j);
      }
    }
  }
;
}

void externalWrenchCallback( const geometry_msgs::WrenchStampedConstPtr& msg )
{
  wrench.resize(6);
  wrench(0) = msg->wrench.force.x;
  wrench(1) = msg->wrench.force.y;
  wrench(2) = msg->wrench.force.z;
  wrench(3) = msg->wrench.torque.x;
  wrench(4) = msg->wrench.torque.y;
  wrench(5) = msg->wrench.torque.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wrench_to_external_torques");
  
  ros::NodeHandle nh;
  ros::Rate rate(125);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  std::string base_frame = "ur5_base_link";
  std::string tool_frame = "ur5_ee_link";                 //"ee_link";
  
  model_js.name.resize(6);
  model_js.position.clear();
  model_js.velocity.clear();
  model_js.effort.clear();
  model_js.name.at(0) = "ur5_shoulder_pan_joint";
  model_js.name.at(1) = "ur5_shoulder_lift_joint";
  model_js.name.at(2) = "ur5_elbow_joint";
  model_js.name.at(3) = "ur5_wrist_1_joint";
  model_js.name.at(4) = "ur5_wrist_2_joint";
  model_js.name.at(5) = "ur5_wrist_3_joint";

  
  urdf::Model model;
  model.initParam("robot_description");
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  boost::shared_ptr<rosdyn::Chain> chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
  chain->setInputJointsName(model_js.name);

  ros::Publisher external_torque_pub = nh.advertise<sensor_msgs::JointState>("external_torques",1);
  ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped> wrench_sub(nh,"wrench",1);
  wrench_sub.setAdvancedCallback(externalWrenchCallback);

  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> joint_state_sub(nh,"/joint_states",1);
  joint_state_sub.setAdvancedCallback(jointStateCallback);

  wrench_sub.waitForANewData();
  joint_state_sub.waitForANewData();

  while( ros::ok() )
  {

    rate.sleep();
    if( q.rows() <= 0 )
    {
      continue;
    }

    Eigen::MatrixXd jacobian_of_a_in_b = chain->getJacobian( q );
    Eigen::JacobiSVD<Eigen::MatrixXd> pinv_J(jacobian_of_a_in_b,  Eigen::ComputeThinU | Eigen::ComputeThinV);

    double no_singularity=1;
    if ( (pinv_J.singularValues()(pinv_J.cols()-1)==0) || (pinv_J.singularValues()(0)/pinv_J.singularValues()(pinv_J.cols()-1) > 1e2))
    {
      no_singularity=0;

      ROS_WARN_STREAM_THROTTLE(5,"SINGULARITY POINT (ellispoid deformed)");
      ROS_WARN_STREAM_THROTTLE(5,"  q        : " << q.transpose() );
      ROS_WARN_STREAM_THROTTLE(5,"  det      : " << jacobian_of_a_in_b.determinant() );
      ROS_WARN_STREAM_THROTTLE(5,"  sin. val.: " << pinv_J.singularValues().transpose() );

    }

    Eigen::VectorXd external_torque = 1 * jacobian_of_a_in_b.transpose() * wrench;

    sensor_msgs::JointState external_torque_msg;

    external_torque_msg.name = model_js.name;
    external_torque_msg.effort.resize(model_js.name.size());
    external_torque_msg.position.resize(model_js.name.size());
    for(unsigned int i=0; i<external_torque_msg.effort.size();i++)
    {
      external_torque_msg.position.at(i) = q(i);
      external_torque_msg.effort.at(i) = no_singularity*external_torque(i);
    }

    external_torque_pub.publish( external_torque_msg );

  }
  return 0;
}
