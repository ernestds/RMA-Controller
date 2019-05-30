#ifndef RMA_CONTROLLER_RMA_CONTROLLER
#define RMA_CONTROLLER_RMA_CONTROLLER

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp> //talvez n precise
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>

//TODO add jacobian stuff
//TODO remove qr_,dqr_,ddqr_ and add reference pose
//TODO add Float64MultiArray msgs

namespace effort_controllers
{
	class RMAController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		ros::NodeHandle node_;	
			
		hardware_interface::EffortJointInterface *hw_;
		std::vector<hardware_interface::JointHandle> joints_;
		int nJoints_;

		ros::Subscriber sub_command_;
		
		KDL::Tree tree_;
		KDL::Chain chain_;	
		KDL::ChainIdSolver_RNE *idsolver_;
		KDL::ChainFkSolverPos_recursive *fkSolverPos_;
		KDL::ChainFkSolverVel_recursive *fkSolverVel_;
		KDL::ChainJntToJacSolver *jacSolver_;
		KDL::ChainJntToJacDotSolver *jacDotSolver_;
		KDL::ChainIkSolverVel_pinv *jacInvSolver_;
		KDL::ChainIkSolverAcc *ikSolverJacAcc_;
		KDL::ChainFkSolverAcc *fkSolverJacAcc_;

		KDL::Jacobian djac_;
		KDL::Jacobian jac_;
		KDL::Jacobian djacanal_;
		KDL::Jacobian jacanal_;
		KDL::Jacobian jacanalInv_;

		KDL::JntArray q_;
		KDL::JntArray dq_;
		KDL::JntArray v_;

		//remover
		KDL::JntArray qr_;
		KDL::JntArray dqr_;
		KDL::JntArray ddqr_;
	

		KDL::JntArrayVel qvel_;
		KDL::JntArrayAcc qacc_;
		
		KDL::Frame x_;
		
		KDL::Frame xr_;
		KDL::FrameVel dx_;
		KDL::FrameVel dxr_;
		KDL::FrameAcc ddxr_;
		KDL::Twist vi_;	

		KDL::JntArray torque_;
		
		KDL::Wrenches fext_;
		
		Eigen::Matrix<double,6,6> Kp_;
		Eigen::Matrix<double,6,6> Kd_;
		Eigen::Matrix<double,6,1> xrpy_;
		Eigen::Matrix<double,6,1> dxrpy_;
		Eigen::Matrix<double,6,1> xo_;
		Eigen::MatrixXd Mr;
		Eigen::MatrixXd Mo;
		double gamma_, alpha_, beta_;
		double dgamma_, dalpha_, dbeta_;

		Eigen::Matrix<double,6,1> ddxr, dxr, xr;
		//TODO CB function
		void commandCB(const geometry_msgs::Pose::ConstPtr &command);

		public:
		RMAController(void);
		~RMAController(void);

		bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time,const ros::Duration& duration);
		template <class T>
		Eigen::MatrixXd invertMatrixSVD(T tempM);
	};
}
#endif
