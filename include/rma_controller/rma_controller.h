#ifndef RMA_CONTROLLER_RMA_CONTROLLER
#define RMA_CONTROLLER_RMA_CONTROLLER

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

#include <std_msgs/Float64MultiArray.h> //n usado
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

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

namespace effort_controllers
{
	class RMAController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		ros::NodeHandle node_;	
			
		hardware_interface::EffortJointInterface *hw_;
		std::vector<hardware_interface::JointHandle> joints_;
		int nJoints_;
		joint_limits_interface::JointLimits jointLimits;

		ros::Subscriber sub_command_;
		ros::Publisher pub_erpy_;
		ros::Publisher pub_x_;
		ros::Publisher pub_ref_;

		geometry_msgs::TwistStamped erroRPY; //erro de posicao
		geometry_msgs::PoseStamped msgX;
		geometry_msgs::PoseStamped msgR;
		
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
		Eigen::MatrixXd J;
		Eigen::MatrixXd dJ;
		Eigen::MatrixXd Ja;


		KDL::JntArray q_;
		KDL::JntArray dq_;

		Eigen::VectorXd qLimMax;
		Eigen::VectorXd qLimMin;
	
		KDL::JntArrayVel qvel_;
		KDL::JntArrayAcc qacc_;
		
		KDL::Frame x_;
		
		KDL::JntArray torque_;
		
		KDL::Wrenches fext_;

		Eigen::MatrixXd Gv;
		double gainv;
		
		Eigen::Matrix<double,6,6> Kp_;
		Eigen::Matrix<double,6,6> Kd_;
		Eigen::Matrix<double,6,1> xrpy_;
		Eigen::Matrix<double,6,1> dxrpy_;
		Eigen::Matrix<double,6,1> xo_;
		Eigen::MatrixXd Mr;
		double gamma_, alpha_, beta_;
		double dgamma_, dalpha_, dbeta_;

		//coisas inversa
		Eigen::VectorXd hGradient;
		Eigen::VectorXd hGradientLast;

		Eigen::Matrix<double,6,1> ddxr, dxr, xr;
		//TODO CB function
		void commandCB(const geometry_msgs::Pose::ConstPtr &command);

		public:
		RMAController(void);
		~RMAController(void); //lembrar de colocar shutdowns la

		bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time,const ros::Duration& duration);
		template <class T>
		Eigen::MatrixXd invertMatrixSVD(T tempM, double limit);
		void mRotation2Matrix(KDL::Rotation rot, Eigen::MatrixXd &matrix);
		void inverseJL(Eigen::MatrixXd jaco, Eigen::MatrixXd &invJaco);
	};
}
#endif
