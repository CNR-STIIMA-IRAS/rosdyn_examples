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
  
  std::string base_frame = "ur10_base_link";
  std::string tool_frame = "ur10_ee_link";                 //"ee_link";
  
  sensor_msgs::JointState model_js;
  sensor_msgs::JointState extra_js;
  model_js.name.resize(6);
  model_js.position.resize(6);
  model_js.velocity.resize(6);
  model_js.name.at(0) = "ur10_shoulder_pan_joint";
  model_js.name.at(1) = "ur10_shoulder_lift_joint";
  model_js.name.at(2) = "ur10_elbow_joint";
  model_js.name.at(3) = "ur10_wrist_1_joint";
  model_js.name.at(4) = "ur10_wrist_2_joint";
  model_js.name.at(5) = "ur10_wrist_3_joint";
  
  urdf::Model model;
  model.initParam("robot_description");
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  boost::shared_ptr<rosdyn::Chain> chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
  
  ROS_INFO("%s",model.getName().c_str());
  chain->setInputJointsName(model_js.name);
  std::vector<rosdyn::ComponentPtr> m_components;
  
  try
  {
    for (unsigned int idx = 0;idx <model_js.name.size();idx++)
    {
      std::string component_type;
      if (nh.getParam( model.getName()+"/"+model_js.name.at(idx)+"/spring/type", component_type))
      {
        if (!component_type.compare("Ideal"))
        {
          ROS_INFO("JOINT '%s' has a spring component", model_js.name.at(idx).c_str());
          m_components.push_back(rosdyn::ComponentPtr(new rosdyn::IdealSpring(model_js.name.at(idx), model.getName(), nh )));
        }
      }
      
      if (nh.getParam( model.getName()+"/"+model_js.name.at(idx)+"/friction/type", component_type))
      {
        if (!component_type.compare("Polynomial1"))
        {
          ROS_INFO("JOINT '%s' has a Polynomial1 component", model_js.name.at(idx).c_str());
          m_components.push_back( rosdyn::ComponentPtr(new rosdyn::FirstOrderPolynomialFriction( model_js.name.at(idx), model.getName(), nh ) ));
        } 
        else if (!component_type.compare("Polynomial2"))
        {
          ROS_INFO("JOINT '%s' has a Polynomial2 component", model_js.name.at(idx).c_str());
          m_components.push_back(rosdyn::ComponentPtr(new rosdyn::SecondOrderPolynomialFriction(model_js.name.at(idx), model.getName(), nh) ));
        } 
      }      
    }
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Exception: %s",e.what());
  }
  
  Eigen::VectorXd q(6);
  q.setZero();
  Eigen::VectorXd Dq(6);
  Dq.setZero();
  Eigen::VectorXd DDq(6);
  DDq.setZero();
  Eigen::VectorXd DDDq(6);
  DDDq.setZero();
  Eigen::VectorXd tau(6);
  Eigen::VectorXd tau_model(6);
  tau.setZero();
  
  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_rec(nh,"/ur10/joint_states",1);
  ros::Publisher model_js_pub=nh.advertise<sensor_msgs::JointState>("/ur10/model/joint_states",1);
  ros::Publisher extra_torque_pub=nh.advertise<sensor_msgs::JointState>("/ur10/extra/joint_states",1);
  ros::Publisher wrench_pub=nh.advertise<geometry_msgs::WrenchStamped>("/ur10/virtual_wrench",1);
  
  DDq.setZero();

  ROS_INFO("waiting for topic");
  if (!js_rec.waitForANewData(ros::Duration(1000)))
    return 0;
  ROS_INFO("start");
  while (ros::ok())
  {
    
    
    for (unsigned int idx=0;idx<model_js.name.size();idx++)
    {
      q(idx)=js_rec.getData().position.at(idx);
      Dq(idx)=js_rec.getData().velocity.at(idx);
      tau(idx)=js_rec.getData().effort.at(idx);
      model_js=js_rec.getData();
      extra_js=model_js;
    }
    
    Eigen::VectorXd tau_model = chain->getJointTorque(q, Dq, DDq);
    for ( size_t iComponent = 0; iComponent<m_components.size(); iComponent++ )
    {
      tau_model+=m_components.at(iComponent)->getAdditiveTorque(q,Dq,DDq);
    }
    
    Eigen::VectorXd extra_tau=tau-tau_model;
    for ( size_t iComponent = 0; iComponent<m_components.size(); iComponent++ )
    {
      extra_tau=m_components.at(iComponent)->getNonAdditiveTorque(q,Dq,DDq,extra_tau);
    }
    
    for (unsigned int idx=0;idx<model_js.name.size();idx++)
    {
      model_js.effort.at(idx)=tau_model(idx);
      extra_js.effort.at(idx)=-extra_tau(idx);
    }
    
    Eigen::MatrixXd J = chain->getJacobian(q);
    Eigen::JacobiSVD<Eigen::MatrixXd> pinv_J(J.transpose(),  Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    Eigen::VectorXd wrench_of_t_in_b=-pinv_J.solve(tau-tau_model);
    
    Eigen::Affine3d T_bt = chain->getTransformation(q);
    
    
    Eigen::VectorXd wrench_of_t_in_t = rosdyn::spatialRotation(wrench_of_t_in_b,T_bt.linear().inverse());
    //external_wrench = rosdyn::spatialDualTranformation(wrench_of_t_in_t,m_Tft.inverse()); 
    
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp=model_js.header.stamp;
    wrench_msg.header.frame_id=tool_frame;
    
    wrench_msg.wrench.force.x=wrench_of_t_in_t(0);
    wrench_msg.wrench.force.y=wrench_of_t_in_t(1);
    wrench_msg.wrench.force.z=wrench_of_t_in_t(2);
    wrench_msg.wrench.torque.x=wrench_of_t_in_t(3);
    wrench_msg.wrench.torque.y=wrench_of_t_in_t(4);
    wrench_msg.wrench.torque.z=wrench_of_t_in_t(5);
    
    model_js_pub.publish(model_js);
    extra_torque_pub.publish(extra_js);
    wrench_pub.publish(wrench_msg);
    
    rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}
