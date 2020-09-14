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
  
  std::string base_frame = "world";
  std::string tool_frame = "ur5_ee_link";                 //"ee_link";
  
  sensor_msgs::JointState model_js;
  sensor_msgs::JointState extra_js;
  model_js.name.resize(7);
  model_js.position.resize(7);
  model_js.velocity.resize(7);
  model_js.name.at(0) = "linear_motor_cursor_joint";
  model_js.name.at(1) = "ur5_shoulder_pan_joint";
  model_js.name.at(2) = "ur5_shoulder_lift_joint";
  model_js.name.at(3) = "ur5_elbow_joint";
  model_js.name.at(4) = "ur5_wrist_1_joint";
  model_js.name.at(5) = "ur5_wrist_2_joint";
  model_js.name.at(6) = "ur5_wrist_3_joint";
  
  urdf::Model model;
  model.initParam("robot_description");
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  bool use_svd=false;
  bool use_FullPivLU=true;
  
  double gain=1;
  
  rosdyn::ChainPtr chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
  chain->setInputJointsName(model_js.name);
  
  
  double ntrial=1000;
  double fail=0;
  double success=0;
  double avg_time=0;
  double avg_iter=0;
  for (double itrial = 0;itrial<ntrial;itrial++)
  {
    Eigen::VectorXd seed(chain->getActiveJointsNumber());
    seed.setRandom();
    
    Eigen::VectorXd sol=seed;
    sol.setRandom();
    
    Eigen::Affine3d Tbt=chain->getTransformation(sol); // base <- target
    
    Eigen::VectorXd q=seed;
    
    ros::Time t0=ros::Time::now();
    unsigned int idx=0;
    Eigen::VectorXd cart_error_in_b;

    for (idx=0;idx<30;idx++)
    {
      Eigen::Affine3d Tba=chain->getTransformation(q); // base <- actual
      
      rosdyn::getFrameDistance(Tbt,Tba,cart_error_in_b);
//       rosdyn::getFrameDistanceQuat(Tbt,Tba,cart_error_in_b);


      
      if (cart_error_in_b.norm()<1e-5)
      {
        break;
      }
      
      cart_error_in_b=cart_error_in_b*gain;
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
      
      if (joint_error.norm()>0.4)
        joint_error*=(0.4/joint_error.norm());
      q+=joint_error;


    }
    ros::Time t1=ros::Time::now();
    
    if (cart_error_in_b.norm()>=1e-5)
      fail++;
    else
    {
      avg_time=(avg_time*success+(t1-t0).toSec())/(success+1.0);
      avg_iter=(avg_iter*success+idx)/(success+1.0);
      
      success++;
    }

  }

  ROS_INFO("solve rate = %f %%",(100.0*success/ntrial));
  ROS_INFO("Avg Time = %f [ms]",avg_time*1000);
  ROS_INFO("Avg Iterations = %f",avg_iter);
  
  return 0;
}
