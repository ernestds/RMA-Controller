#include <sys/mman.h>

#include <rma_controller/rma_controller.h>
#include <pluginlib/class_list_macros.h>
#define DEBUG
#ifdef DEBUG
#define D(x)                                      \
	std::cout << "Na linha " << __LINE__ << "\n"; \
	x
#else
#define D(x)
#endif

#include <iostream>
#include <math.h>
using std::cout;
using std::endl;
using namespace Eigen;

namespace effort_controllers
{

RMAController::RMAController(void) : q_(0), dq_(0), torque_(0), fext_(0), jac_(0), djac_(0)
{
}
RMAController::~RMAController(void)
{
	sub_command_.shutdown();
	pub_erpy_.shutdown();
	pub_ref_.shutdown();
	pub_x_.shutdown();
}

bool RMAController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
{
	node_ = n;
	hw_ = hw;

	std::vector<std::string> joint_names;
	if (!node_.getParam("joints", joint_names))
	{
		ROS_ERROR("No 'joints' in controller. (namespace: %s)",
				  node_.getNamespace().c_str());
		return false;
	}
	nJoints_ = joint_names.size();
	
	for (int i = 0; i < nJoints_; i++)
	{
		try
		{
			joints_.push_back(hw->getHandle(joint_names[i]));
		}
		catch (const hardware_interface::HardwareInterfaceException &e)
		{
			ROS_ERROR_STREAM("Exception thrown: " << e.what());
			return false;
		}
	}

	qLimMin.resize(nJoints_);
	qLimMax.resize(nJoints_);
	for (int i=0; i<nJoints_;i++)
	{
		const bool rosparam_limits_ok = getJointLimits(joint_names[i], node_, jointLimits);
		qLimMin(i)=jointLimits.min_position;
		qLimMax(i)=jointLimits.max_position;
	}
	D(cout<<"LIMITES: \n Min:"<<qLimMin<<"\n Max: "<<qLimMax<<"\n");

	sub_command_ = node_.subscribe("command", 1, &RMAController::commandCB, this);
	pub_erpy_ = node_.advertise<geometry_msgs::TwistStamped>("/erro_rpy",1);
	pub_x_ = node_.advertise<geometry_msgs::PoseStamped>("/pose_tip",1);
	pub_ref_ = node_.advertise<geometry_msgs::PoseStamped>("/pose_reference",1);

	std::string robot_desc_string;
	if (!node_.getParam("/robot_description", robot_desc_string))
	{
		ROS_ERROR("Could not find '/robot_description'.");
		return false;
	}

	if (!kdl_parser::treeFromString(robot_desc_string, tree_))
	{
		ROS_ERROR("Failed to construct KDL tree.");
		return false;
	}

	std::string chainRoot;
	if (!node_.getParam("chain/root", chainRoot))
	{
		ROS_ERROR("Could not find 'chain_root' parameter.");
		return false;
	}

	std::string chainTip;
	if (!node_.getParam("chain/tip", chainTip))
	{
		ROS_ERROR("Could not find 'chain/tip' parameter.");
		return false;
	}

	if (!tree_.getChain(chainRoot, chainTip, chain_))
	{
		ROS_ERROR("Failed to get chain from KDL tree.");
		return false;
	}

	KDL::Vector g;
	node_.param("gravity/x", g[0], 0.0);
	node_.param("gravity/y", g[1], 0.0);
	node_.param("gravity/z", g[2], -9.8);

	if ((idsolver_ = new KDL::ChainIdSolver_RNE(chain_, g)) == NULL)
	{
		ROS_ERROR("Failed to create ChainIDSolver_RNE.");
		return false;
	}

	if ((fkSolverPos_ = new KDL::ChainFkSolverPos_recursive(chain_)) == NULL)
	{
		ROS_ERROR("Failed to create ChainFkSolver_recursive");
		return false;
	}

	if ((fkSolverVel_ = new KDL::ChainFkSolverVel_recursive(chain_)) == NULL)
	{
		ROS_ERROR("Failed to create ChainFkSolverVel_recursive");
		return false;
	}

	if ((jacSolver_ = new KDL::ChainJntToJacSolver(chain_)) == NULL)
	{
		ROS_ERROR("Failed to create ChainJntToJacDotSolver");
		return false;
	}

	if ((jacDotSolver_ = new KDL::ChainJntToJacDotSolver(chain_)) == NULL)
	{
		ROS_ERROR("Failed to create ChainJntToJacDotSolver");
		return false;
	}

	if ((jacInvSolver_ = new KDL::ChainIkSolverVel_pinv(chain_)) == NULL)
	{
		ROS_ERROR("Failed to create ChainIkSolverVel_pinv");
		return false;
	}

	jac_.resize(nJoints_);
	djac_.resize(nJoints_);

	q_.resize(nJoints_);
	dq_.resize(nJoints_);
	qvel_.resize(nJoints_);

	hGradient.resize(nJoints_);
	hGradientLast.resize(nJoints_);

	torque_.resize(nJoints_);

	fext_.resize(chain_.getNrOfSegments());
	
	std::vector<double> KpVec;
	if (!node_.getParam("Kp", KpVec))
	{
		ROS_ERROR("No 'Kp' in controller %s.", node_.getNamespace().c_str());
		return false;
	}
	Kp_ = Eigen::Map<Eigen::Matrix<double, 6, 6>>(KpVec.data()).transpose();

	std::vector<double> KdVec;
	if (!node_.getParam("Kd", KdVec))
	{
		ROS_ERROR("No 'Kd' in controller %s.", node_.getNamespace().c_str());
		return false;
	}
	Kd_ = Eigen::Map<Eigen::Matrix<double, 6, 6>>(KdVec.data()).transpose();

	if (!node_.getParam("Gv", gainv))
	{
		ROS_ERROR("No 'Gv' in controller %s.", node_.getNamespace().c_str());
		return false;
	}
	D(cout<<"Gv: "<<gainv<<"\n");

	return true;
}

void RMAController::starting(const ros::Time &time)
{
	for (unsigned int i = 0; i < nJoints_; i++)
	{
		q_(i) = joints_[i].getPosition();
		dq_(i) = joints_[i].getVelocity();
	}

	for(int i=0;i<nJoints_;i++){
		hGradientLast(i)=(pow(qLimMax(i)-qLimMin(i),2)*(2*q_(i)-qLimMax(i)-qLimMin(i)))
			/(4*pow(qLimMax(i)-q_(i),2)*pow(q_(i)-qLimMin(i),2));

	}

	for(int i=0;i<nJoints_;i++){
		hGradientLast(i)=(pow(qLimMax(i)-qLimMin(i),2)*(2*q_(i)-qLimMax(i)-qLimMin(i)))
			/(4*pow(qLimMax(i)-q_(i),2)*pow(q_(i)-qLimMin(i),2));

	}

	if ((fkSolverPos_->JntToCart(q_, x_)) < 0)
	{
		ROS_ERROR("KDL forward kinematics solver failed.");
	}

	if ((fkSolverPos_->JntToCart(q_, x_, -1)) < 0)
	{
		ROS_ERROR("KDL forward kinematics solver failed.");
	}

	for(int i=0;i<6;i++){
		ddxr(i)=0;
		dxr(i)=0;
	}

	xr.head(3) << x_.p.data[0], x_.p.data[1], x_.p.data[2];
	Mr = (Eigen::MatrixXd(3, 3) << x_.M.data[0], x_.M.data[1], x_.M.data[2],
						  x_.M.data[3], x_.M.data[4], x_.M.data[5],
						  x_.M.data[6], x_.M.data[7], x_.M.data[8]).finished();

	struct sched_param param;
	if (!node_.getParam("priority", param.sched_priority))
	{
		ROS_WARN("No 'priority' configured for controller %s. Using highest possible priority.", node_.getNamespace().c_str());
		param.sched_priority = sched_get_priority_max(SCHED_FIFO);
	}
	if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
	{
		ROS_WARN("Failed to set real-time scheduler.");
		return;
	}
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
		ROS_WARN("Failed to lock memory.");
	
}

void RMAController::update(const ros::Time &time, const ros::Duration &duration)
{
	//usar solvers para calcular jacobiano e posicao cartesiana 
	for (unsigned int i = 0; i < nJoints_; i++)
	{
		q_(i) = joints_[i].getPosition();
		dq_(i) = joints_[i].getVelocity();
	}
	qvel_.q = q_;
	qvel_.qdot = dq_;

	for (unsigned int i = 0; i < fext_.size(); i++)
		fext_[i].Zero();

	if (jacSolver_->JntToJac(q_, jac_, -1) != 0)
	{
		ROS_ERROR("KDL Jacobian solver failed");
	}

	if (jacDotSolver_->JntToJacDot(qvel_, djac_, -1) != 0)
	{
		ROS_ERROR("KDL Jacobian dot solver failed");
	}

	if ((fkSolverPos_->JntToCart(q_, x_, -1)) < 0)
	{
		ROS_ERROR("KDL forward kinematics solver failed.");
	}
	//print para debug
	double ax,ay,az,aw;
	double axr,ayr,azr,awr;
    KDL::Rotation tempMr;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            tempMr.data[(i*3) + j]=Mr(i,j);
        }
    } 
    tempMr.GetQuaternion(axr,ayr,azr,awr);
	x_.M.GetQuaternion(ax,ay,az,aw);
    D(cout<<"Xr: \n"<<xr.head(3) << endl <<"Quaternio ref: "<<axr<<" "<<ayr<<" "<<azr<<" "<<awr<<"\n");
	D(cout<<"X: "<<x_.p.data[0]<<" "<<x_.p.data[1]<<" "<<x_.p.data[2]<<"\n Quaternio x: "<<ax<<" "<<ay<<" "<<az<<" "<<aw<<"\n");

	J = jac_.data; 
	dJ = djac_.data; 
	// matriz de rotacao
	Eigen::MatrixXd M = (Eigen::MatrixXd(3, 3) << 
						 x_.M.data[0], x_.M.data[1], x_.M.data[2],
						 x_.M.data[3], x_.M.data[4], x_.M.data[5],
						 x_.M.data[6], x_.M.data[7], x_.M.data[8]).finished();

	x_.M.GetRPY(alpha_, beta_, gamma_);
	xrpy_ << x_.p.data[0], x_.p.data[1], x_.p.data[2], alpha_, beta_, gamma_;
	
	Eigen::MatrixXd T(3, 3), TaInv;
	T << std::cos(gamma_) * std::cos(beta_), -std::sin(gamma_), 0,
		std::sin(gamma_) * std::cos(beta_), std::cos(gamma_), 0,
		-std::sin(gamma_), 0, 1;

	TaInv = Eigen::MatrixXd::Zero(6, 6);
	Eigen::MatrixXd dTa = TaInv;
	TaInv.bottomRightCorner(3, 3) = T.inverse();
	TaInv.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
	
	// jacobiano analitico
	Eigen::MatrixXd Ja = TaInv * J;
	Eigen::VectorXd dqtemp_ = (Eigen::VectorXd(7) << dq_(0), dq_(1), dq_(2), dq_(3), dq_(4), dq_(5), dq_(6)).finished();

	dxrpy_ = Ja * dqtemp_;
	dalpha_ = dxrpy_(3);
	dbeta_ = dxrpy_(4);
	dgamma_ = dxrpy_(5);

	dTa.bottomRightCorner(3, 3).topLeftCorner(2, 2) = 
	(Eigen::MatrixXd(2, 2) << -std::sin(gamma_) * std::cos(beta_) * dgamma_ - std::cos(gamma_) * std::sin(beta_) * dbeta_, -std::cos(gamma_) * dgamma_,
	std::cos(gamma_) * std::cos(beta_) * dgamma_ - std::sin(gamma_) * std::sin(beta_) * dbeta_, -std::sin(gamma_) * dgamma_).finished();
	
	dTa.bottomRightCorner(3, 3)(2, 0) = -std::cos(gamma_) * dgamma_;
	Eigen::MatrixXd dJa = TaInv * (dJ - dTa * Ja);
	
	KDL::JntArray qll;
	qll.resize(nJoints_);

	//calculo do erro
	Eigen::VectorXd erpy = VectorXd::Zero(6);
	erpy.head(3) = xr.head(3) - xrpy_.head(3);
	
	Eigen::Vector3d m = M.col(0);
	Eigen::Vector3d s = M.col(1);
	Eigen::Vector3d a = M.col(2);
	Eigen::Vector3d mr = Mr.col(0);
	Eigen::Vector3d sr = Mr.col(1);
	Eigen::Vector3d ar = Mr.col(2);
	erpy.tail(3) = 0.5*(m.cross(mr) + s.cross(sr) + a.cross(ar));
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(Ja);

	
	double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

	D(cout<<"Erro rpy: "<<erpy<<"\n");

	MatrixXd inverseJa;
	inverseJL(Ja,inverseJa);
	
	Gv = MatrixXd::Zero(nJoints_,nJoints_);
	for(int i=0;i<nJoints_;i++)
		Gv.diagonal()(i) = gainv; 
	//calculo das variaveis vistas no artigo
	VectorXd dqc_ = inverseJa*(dxr + Kp_*erpy); //dqc=J'(dxr+Kp*e)
	VectorXd csi_ = Gv*(dqc_-dqtemp_); 
	MatrixXd N =  Matrix<double, 7, 7>::Identity() - inverseJa*Ja;
	VectorXd av = inverseJa*(ddxr + Kd_ * (dxr - dxrpy_) + Kp_ * erpy - dJa*dqtemp_)+N*csi_;
	VectorXd avc = av + Gv*(dqc_ - dqtemp_);
	
	//coloca o resultado final em um joint array
	qll.data = avc;
	
	//calculo do torque
	if (idsolver_->CartToJnt(q_, dq_, qll, fext_, torque_) < 0)
		ROS_ERROR("KDL inverse dynamics solver failed.");

	for (unsigned int i = 0; i < nJoints_; i++)
		joints_[i].setCommand(torque_(i));
	
	//publicam nos topicos para fins graficos
	erroRPY.header.stamp=ros::Time::now();
	erroRPY.twist.linear.x=erpy(0);
	erroRPY.twist.linear.y=erpy(1);
	erroRPY.twist.linear.z=erpy(2);
	erroRPY.twist.angular.x=erpy(3);
	erroRPY.twist.angular.y=erpy(4);
	erroRPY.twist.angular.z=erpy(5);
	pub_erpy_.publish(erroRPY);

	msgX.header.stamp=ros::Time::now();
	msgX.pose.position.x=x_.p.data[0];
	msgX.pose.position.y=x_.p.data[1];
	msgX.pose.position.z=x_.p.data[2];
	msgX.pose.orientation.x=ax;
	msgX.pose.orientation.y=ay;
	msgX.pose.orientation.z=az;
	msgX.pose.orientation.w=aw;
	pub_x_.publish(msgX);
	
	msgR.header.stamp=ros::Time::now();
	msgR.pose.position.x=xr(0);
	msgR.pose.position.y=xr(1);
	msgR.pose.position.z=xr(2);
	msgR.pose.orientation.x=axr;
	msgR.pose.orientation.y=ayr;
	msgR.pose.orientation.z=azr;
	msgR.pose.orientation.w=awr;
	pub_ref_.publish(msgR);
}

//pseudo inversa com SVD
template <class T>
Eigen::MatrixXd RMAController::invertMatrixSVD(T tempM, double limit)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(tempM, Eigen::ComputeFullV | Eigen::ComputeFullU | Eigen::FullPivHouseholderQRPreconditioner);

	Eigen::MatrixXd U = svd.matrixU();
	Eigen::MatrixXd V = svd.matrixV();
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(tempM.rows(), tempM.cols());
	S.diagonal() = svd.singularValues();
	Eigen::MatrixXd Sl = S.transpose();

	for (int i = 0; i < Sl.rows(); i++)
		for (int j = 0; j < Sl.cols(); j++)
			if ((i == j) && (S(i, j) > limit)) //limite para contornar singularidades
				Sl(j, i) = 1 / S(i, j);

	Eigen::MatrixXd invJ_ = V * Sl * U.transpose();
	return invJ_;
}

// calculo da inversa direita de J como vista no artigo
void RMAController::inverseJL(Eigen::MatrixXd jaco, Eigen::MatrixXd &invJaco){
	Eigen::MatrixXd wJL = MatrixXd::Zero(nJoints_,nJoints_);
	for(int i=0;i<nJoints_;i++){
		hGradient(i)=(pow(qLimMax(i)-qLimMin(i),2)*(2*q_(i)-qLimMax(i)-qLimMin(i)))
			/(4*pow(qLimMax(i)-q_(i),2)*pow(q_(i)-qLimMin(i),2));
		if((abs(hGradient(i))-abs(hGradientLast(i)))>=0)
			wJL.diagonal()(i)=1.0+abs(hGradient(i));
		else
			wJL.diagonal()(i)=1.0;
		hGradientLast(i)=hGradient(i);
	}

	//invJaco=wJL.inverse()*jaco.transpose()*(jaco*wJL.inverse()*jaco.transpose()).inverse();
	invJaco=invertMatrixSVD(wJL,0.99)*jaco.transpose()*invertMatrixSVD(jaco*wJL.inverse()*jaco.transpose(),0.005);
}

void RMAController::mRotation2Matrix(KDL::Rotation rot, Eigen::MatrixXd &matrix){
		matrix = (Eigen::MatrixXd(3,3) << 
		rot.data[0], rot.data[1], rot.data[2], 
		rot.data[3], rot.data[4], rot.data[5], 
		rot.data[6], rot.data[7], rot.data[8]).finished();
	}

void RMAController::commandCB(const geometry_msgs::Pose::ConstPtr &command)
{
	double px, py, pz;
	double ox, oy, oz, ow;
	KDL::Rotation refM;
	MatrixXd temp;
	px= command->position.x;
	py= command->position.y;
	pz= command->position.z;
	ox = command->orientation.x;
	oy= command->orientation.y;
	oz= command->orientation.z;
	ow= command->orientation.w;

	Eigen::Quaterniond q;
	q.x() = ox;
	q.y() = oy;
	q.z() = oz;
	q.w() = ow; 

	Mr = q.normalized().toRotationMatrix();
	
	xr.head(3) << px, py, pz;
	
}

} // namespace effort_controllers
PLUGINLIB_EXPORT_CLASS(effort_controllers::RMAController,
					   controller_interface::ControllerBase)
//eof
