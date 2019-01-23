#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <rosdyn_core/urdf_parser.h>
#include <rosdyn_core/frame_distance.h>

#include <subscription_notifier/subscription_notifier.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_kinematics");
  
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
  
  bool use_svd=false;
  bool use_FullPivLU=true;
  
  double gain=1.0;
  
  boost::shared_ptr<rosdyn::Chain> chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
  chain->setInputJointsName(model_js.name);
  
  for (unsigned int itrial = 0;itrial<10;itrial++)
  {
    Eigen::VectorXd seed(6);
    seed.setRandom();
    
    Eigen::VectorXd sol=seed;
//     sol(0)+=0.05;
//     sol(1)+=0.01;
//     sol(2)+=0.01;
//     sol(3)-=0.01;
//     sol(4)+=0.01;
    sol(5)+=0.2;

    
    Eigen::Affine3d Tbt=chain->getTransformation(sol); // base <- target
    
    Eigen::VectorXd q=seed;
    
    ros::Time t0=ros::Time::now();
    unsigned int idx=0;
    Eigen::VectorXd cart_error_in_b;
      
    for (idx=0;idx<100;idx++)
    {
      Eigen::Affine3d Tba=chain->getTransformation(q); // base <- actual
      
      rosdyn::getFrameDistance(Tbt,Tba,cart_error_in_b);

      if (cart_error_in_b.norm()<1e-5)
      {
        ROS_INFO("success");
        break;
      }
      
      Eigen::MatrixXd jacobian_of_a_in_b = chain->getJacobian(q);
    
      Eigen::VectorXd joint_error;
      
      if (use_svd)
      {
        Eigen::JacobiSVD<Eigen::MatrixXd> pinv_J(jacobian_of_a_in_b,  Eigen::ComputeThinU | Eigen::ComputeThinV);
        
        Eigen::VectorXd joint_error=pinv_J.solve(cart_error_in_b);
        if (pinv_J.singularValues()(pinv_J.cols()-1)==0)
        {
          ROS_WARN("SINGULARITY POINT");
          break;
        }
        else if (pinv_J.singularValues()(0)/pinv_J.singularValues()(pinv_J.cols()-1) > 1e2)
        {
          ROS_WARN("SINGULARITY POINT");
          break;
        }
      }
      else if (use_FullPivLU)
      {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(jacobian_of_a_in_b);
        joint_error=lu.solve(cart_error_in_b);
      }
      else 
        joint_error=jacobian_of_a_in_b.inverse()*cart_error_in_b;
      
      ROS_INFO_STREAM("joint_error = " << joint_error.transpose());
      if (joint_error.norm()>0.1)
        joint_error*=(0.1/joint_error.norm());
      q+=gain*joint_error;
      
    
    }
    ros::Time t1=ros::Time::now();
    
    if (cart_error_in_b.norm()>=1e-5)
      ROS_WARN("FAIL");

    ROS_INFO_STREAM("sol= " << sol.transpose());
    ROS_INFO_STREAM("q = " << q.transpose());
    ROS_INFO("iteration = %u, time = %f", idx,(t1-t0).toSec());
  }
  return 0;
}
