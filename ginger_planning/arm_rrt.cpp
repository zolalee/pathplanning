/*
***************************************************************************************************
                                    Cloudminds (Shenzhen) Ltd.

                        (c) Coyright 2019, Cloudminds Ltd.,

* All rights reserved. Cloudminds's  source code is an unpublished work and the use of a copyright
* notice does not imply otherwise. This source code contains confidential, trade secret material of
* Cloudminds Ltd. Any attempt or participation in deciphering, decoding, reverse engineering
* or in any way altering the source code is strictly prohibited, unless the prior written consent
* of Cloudminds Ltd. is obtained.
*
* Filename:       	arm_rrt.cpp
* Programmer:       li zola
* Created:        	July 30th, 2019
* Description:		XR1 motion planning
* Mail:             18754535@qq.com
* Note:				August 15th, 2019, version: 0.9
*
***************************************************************************************************
*/

#include "arm_rrt.h"

#include <fstream>
#include <ostream>
#include "cubic.h"

using namespace fcl;
using namespace std;
using std::cout;
using std::endl;
using std::vector;

typedef boost::shared_ptr<Cylinder> CylinderPtr;
typedef boost::shared_ptr<CollisionObject> CollisionPtr;

static const std::string PACKAGE = "xr1motionplanning";

Joint operator-(const Joint &v1, const Joint &v2)
{
    Joint result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] - v2[i];
    }
    return result;
}

Joint operator+(const Joint &v1, const Joint &v2)
{
    Joint result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] + v2[i];
    }
    return result;
}

Joint operator*(const Joint &v1, double scale)
{
    Joint result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] * scale;
    }
    return result;
}

std::ostream& operator<<(std::ostream& os, const Joint &vec)
{
    for(int i=0; i<vec.size(); ++i)
        os << vec[i] << "\n";

    return os;
}

bool vectorEqual(const Joint &a, const Joint &b)
{
    assert(a.size() == b.size() );
    vector<double> delta = a-b;
    double sum=0;
    for(int n=0; n<a.size(); ++n)
        sum += pow(delta[n], 2);
    if(sqrt(sum) < 1e-5)
        return true;
    return false;
}

Point forwardKinematics(const Joint& joint_)
{
    Eigen::MatrixXd pose(4,4);
    Eigen::MatrixXd A0(4,4),A1(4,4), A2(4,4), A3(4,4), A4(4,4), A5(4,4), A6(4,4), A7(4,4),A8(4,4),A9(4,4),A10(4,4);
    /*--------for arm 7 joint plannning-------------*/
    // A1<<cos(joint_[0]),0,-sin(joint_[0]),0,
    //     sin(joint_[0]),0,-cos(joint_[0]),0,
    //     0,1,0,0,
    //     0,0,0,1;
    // A2<<cos(joint_[1]),0,sin(joint_[1]),-0.0492*cos(joint_[1]),
    //     sin(joint_[1]),0,-cos(joint_[1]),0.0492*sin(joint_[1]),
    //     0,1,0,0,
    //     0,0,0,1;
    // A3<<cos(joint_[2]),0,sin(joint_[2]),0,
    //     sin(joint_[2]),0,-cos(joint_[2]),0,
    //     0,-1,0,0.224520,
    //     0,0,0,1;
    // A4<<cos(joint_[3]),0,-sin(joint_[3]),0,
    //     sin(joint_[3]),0,cos(joint_[3]),0,
    //     0,-1,0,0,
    //     0,0,0,1;
    // A5<<cos(joint_[4]),0,sin(joint_[4]),0,
    //     sin(joint_[4]),0,-cos(joint_[4]),0,
    //     0,1,0,0.219230,
    //     0,0,0,1;
    // A6<<cos(joint_[5]),0,-sin(joint_[5]),0,
    //     sin(joint_[5]),0,cos(joint_[5]),0,
    //     0,-1,0,0,
    //     0,0,0,1;
    // A7<<cos(joint_[6]),-sin(joint_[6]),0,0,
    //     sin(joint_[6]),cos(joint_[6]),0,0,
    //     0,0,1,0,
    //     0,0,0,1;
    /*---------end for 7 joint planning------------*/

    /*---------end for 10 joint planning------------*/
    A0<<cos(-1.92),0,sin(-1.92),0.167643,
        sin(-1.92),0,-cos(-1.92),-0.25743,
        0,1,0,0.01632,
        0,0,0,1;
    A1<<cos(joint_[0]),-sin(joint_[0]),0,0,
        sin(joint_[0]),cos(joint_[0]),0,0,
        0,0,1,0,
        0,0,0,1; 
    A2<<cos(joint_[1]+1.919841),-sin(joint_[1]+1.919841),0,0,
        0,0,1,0,
        -sin(joint_[1]+1.919841),-cos(joint_[1]+1.919841),0,0,
        0,0,0,1;
    A3<<cos(joint_[2]+M_PI/2),-sin(joint_[2]+M_PI/2),0,0.049124788,
        0,0,1,0.224655826,
        -sin(joint_[2]+M_PI/2),-cos(joint_[2]+M_PI/2),0,0,
        0,0,0,1;
    A4<<cos(joint_[3]),-sin(joint_[3]),0,0,
        0,0,-1,0,
        sin(joint_[3]),cos(joint_[3]),0,0,
        0,0,0,1;
    A5<<cos(joint_[4]),-sin(joint_[4]),0,0,
        0,0,-1,0.209557803,
        -sin(joint_[4]),-cos(joint_[4]),0,0,
        0,0,0,1;
    A6<<cos(joint_[5]+M_PI/2),-sin(joint_[5]+M_PI/2),0,0,
        0,0,-1,0,
        sin(joint_[5]+M_PI/2),cos(joint_[5]+M_PI/2),0,0,
        0,0,0,1;
    A7<<cos(joint_[6]+M_PI/2),-sin(joint_[6]+M_PI/2),0,0,
        0,0,-1,0,
        sin(joint_[6]+M_PI/2),cos(joint_[6]+M_PI/2),0,0,
        0,0,0,1;      
    A8<<cos(joint_[7]+M_PI/2),-sin(joint_[7]+M_PI/2),0,0.105,
        0,0,-1,0,
        sin(joint_[7]+M_PI/2),cos(joint_[7]+M_PI/2),0,0,
        0,0,0,1;
    A9<<cos(joint_[8]+M_PI/2),-sin(joint_[8]+M_PI/2),0,0,
        0,0,-1,0,
        sin(joint_[8]+M_PI/2),cos(joint_[8]+M_PI/2),0,0,
        0,0,0,1;  
    A10<<cos(joint_[9]),-sin(joint_[9]),0,0,
        0,0,1,0.459273874,
        -sin(joint_[9]),-cos(joint_[9]),0,0,
        0,0,0,1;        

    //pose = A10*A9*A8*A1*A2*A3*A4*A5*A6*A7;
    pose = A1*A2*A3*A4*A5*A6*A7;
    Point point;
    for(unsigned int i=0; i<3; ++i)
        point.push_back(pose(i,3));

    return point;
}

Node::Node()
{
    joint_.push_back((rand() % 628 -314) / 100.0);
    joint_.push_back((rand() % 400 -200) / 100.0);
    joint_.push_back((rand() % 628 -314) / 100.0);
    joint_.push_back((rand() % 400 -200) / 100.0);
    joint_.push_back((rand() % 628 -314) / 100.0);
    joint_.push_back((rand() % 400 -200) / 100.0);
    joint_.push_back((rand() % 628 -314) / 100.0);
    //joint_.push_back((rand() % 400 -200) / 100.0);
    //joint_.push_back((rand() % 628 -314) / 100.0);
    //joint_.push_back((rand() % 400 -200) / 100.0);

    parent_ = 0;
}

Node::Node(const Joint &vec)
{
    joint_ = vec;
    parent_ = -1;
}

Node::Node(const Joint &vec, int parent)
{
    joint_ = vec;
    parent_ = parent;
}

Joint Node::getJoint() const
{
    return joint_;
}

Point Node::getPoint()
{
    if(point_.empty())
        point_ = forwardKinematics(joint_);

    return point_;
}

int Node::getParent() const
{
    return parent_;
}

void Node::setParent(int parent)
{
    parent_ = parent;
}

RRT::RRT(const NodePtr init, const NodePtr goal)
{
    initialize(init, goal);
}

RRT::~RRT()
{
    cleanup();
}

void RRT::initialize(const NodePtr init, const NodePtr goal)
{
    rrtTree_.push_back(init);
    bestDelta = 100000;
    bestID = 0;
    count = 0;

    init_ = init;
    goal_ = goal;
    kdTree = kd_create(N_DIM);
    addNode(init);

    state = STEP_PROGRESS;

    min_.push_back(-M_PI);
    min_.push_back(-0.00517*M_PI);
    min_.push_back(-M_PI/2);
    min_.push_back(-2.09);
    min_.push_back(-M_PI/2);
    min_.push_back(-M_PI/6);
    min_.push_back(-M_PI*0.235);

    max_.push_back(M_PI);
    max_.push_back(0.5509*M_PI);
    max_.push_back(M_PI/2);
    max_.push_back(0.017500);
    max_.push_back(M_PI/2);
    max_.push_back(M_PI/6);
    max_.push_back(M_PI*0.235);

    //初始碰撞次数�?
    collision_n = 0;
    collision_op = 0;
    collision_joint.resize(N_DIM);
    for(int i=0; i<min_.size(); ++i)
        collision_joint[i] = 0;
}

boost::shared_ptr<ShapeBase> RRT::getObstacle(int index) const 
{
    return collisionGroup[index];
}

/*
void RRT::setObstacle(const vector<Joint>& obstacle_)
{
    obstacle = obstacle_;
}
*/

NodePtr RRT::getGoal() const
{
    return goal_;
}

void RRT::addObstacle_sphere(const boost::shared_ptr<Sphere> object, const Transform3f& tf)
{
    collisionGroup2.push_back(object);
    collisionTFGroup2.push_back(tf);
}
void RRT::addObstacle_box(const boost::shared_ptr<Box> object, const Transform3f& tf)
{
    collisionGroup1.push_back(object);
    collisionTFGroup1.push_back(tf);
}
void RRT::addObstacle_cylinder(const boost::shared_ptr<Cylinder> object, const Transform3f& tf)
{
    collisionGroup.push_back(object);
    collisionTFGroup.push_back(tf);
}

void RRT::clearObstacle()
{
    obstacle.clear();
    obstacle.resize(0);
}

void RRT::cleanup()
{
    kd_free(kdTree);

    rrtTree_.clear();
    rrtTree_.resize(0);

    path.clear();
    path.resize(0);

    traj.clear();
    traj.resize(0);
    
    obstacle.clear();
    obstacle.resize(0);
}

void RRT::addNode(const NodePtr &qnew)
{
    // Update graph vectors
    rrtTree_.push_back(qnew);

    uintptr_t id = rrtTree_.size() - 1;
    kd_insert(kdTree, qnew->getJoint().data(), (void*) id);
}

void RRT::run(bool isWrite = true)
{
    srand((unsigned)time(NULL));
    while(state != STEP_REACHED && count < max_count)
    {
        if(rand() / double(RAND_MAX) < 0.1)
            randomStep();
        else
        {
            directStep();
            int try_count = 0;
            while(state == STEP_COLLISION && try_count++ < N_DIM)
                bestRandomStep();
        }
        ++count;
    }
    if(state == STEP_REACHED )
    {
        printTrajectory();
        printModifiedPath();

        if(traj.size() < max_depth)
            //ROS_INFO("Success, final tree has %d nodes!", int(traj.size()));
            cout << "Success, final tree has"<< int(traj.size()) <<" nodes "<< "\n";

        else
            //ROS_INFO("The final depth of RRT tree exceeds the Max depth !");
            cout << "The final depth of RRT tree exceeds the Max depth !"<< "\n";
        if(isWrite)
        {
            writeFile();
            //ROS_INFO("the path information is writed !");
            cout << "the path information is writed ! !"<< "\n";
            // printTree();
        }

    }
    else
        //ROS_INFO("RRT PLANNING Fail !");
        cout << "RRT PLANNING Fail !"<< "\n";
}

void RRT::randomStep()
{
    NodePtr randpoint(new Node());
    int nearID = getNearestNeighbor(randpoint);

    state = tryStep(randpoint, nearID);
}

void RRT::bestRandomStep()
{
    NodePtr randpoint(new Node());
    state = tryStep(randpoint, bestID);
}

int RRT::getBestID()
{
    return bestID;
}

bool RRT::getState()
{
    return state == STEP_REACHED;
}

/*
void RRT::randomDirectStep()
{
    int nearID = getNearestNeighbor(goal_);
    while(state == STEP_PROGRESS)
    {
        state = tryStep(goal_, nearID);
    }
}
*/

void RRT::directStep()
{
    while(state == STEP_PROGRESS)
    {
        int nearID = getNearestNeighbor(goal_);
        state = tryStep(goal_, nearID);
    }
}

void RRT::normalize(Joint &vec)
{
    //向量范数计算
    double norm = 0;
    for(int i=0; i<vec.size(); ++i)
       norm += pow(vec[i], 2);
    norm = sqrt(norm);

    for(int i=0; i<vec.size(); ++i)
        vec[i] = vec[i] / norm;
}

RRT::StepResult RRT::tryStep(const NodePtr randpoint, int nearID)
{
    NodePtr &nearpoint = rrtTree_[nearID];
    Joint diff = randpoint->getJoint() - nearpoint->getJoint();
    normalize(diff);

    // 取样点筛�?//    double temp = atan2(diff[1], diff[0]);
//    if(fabs(temp - nearpoint->getJoint()[2]) > 0.8)
//        return STEP_COLLISION;

//    double step = RANDNM(0.0, stepsize);
    Joint qnew = nearpoint->getJoint() + diff * stepsize;
    //std::cout<<"the qnew is "<<qnew<<std::endl;
    if(!checkCollisions(qnew) && !checkJointLimits(qnew))
    {
        NodePtr newpoint(new Node(qnew, nearID));
        addNode(newpoint);

        //rewireJoint(newpoint);

        double delta = deltaNode(newpoint, goal_);
        if(delta < bestDelta)
        {
            bestDelta = delta;
            bestID = rrtTree_.size()-1;
        }

        //std::cout<<bestConf<<"\t"<<limit<<std::endl;
        if(bestDelta < limit)
        {
            return STEP_REACHED;
        }
        //std::cout<<"OK---"<<std::endl;
        return STEP_PROGRESS;
    }
    else
    {
        return STEP_COLLISION;
    }
}

void RRT::rewireJoint(const NodePtr& ptr)
{
    NodePtr &parent_node = rrtTree_[ptr->getParent()];
    if(parent_node->getParent() > 0)
    {
        Joint joint_pp = rrtTree_[parent_node->getParent()]->getJoint();
        Joint joint_p = parent_node->getJoint();
        Joint vec_pp = joint_p - joint_pp;
        Joint vec_p = ptr->getJoint() - joint_pp;

        double vec_sum1 = 0, vec_sum2 = 0;
        for(unsigned int k=0; k<2; ++k)
        {
            vec_sum1 = pow(vec_pp[k], 2);
            vec_sum2 = pow(vec_p[k], 2);
        }
        if(vec_sum2 <= vec_sum1)
        {
//            cout <<" ***************************** " << endl;
            ptr->setParent(parent_node->getParent());
        }
    }
}

double RRT::deltaNode(const NodePtr node1, const NodePtr node2)
{
    double delta = 0;

    for(int j=0; j<node1->getJoint().size(); ++j)
        delta += pow(node1->getJoint()[j] - node2->getJoint()[j], 2);

    return sqrt(delta);
}

bool RRT::checkCollisions(const Joint &joints)
{
    ++collision_n;
    static Eigen::VectorXd radius(N_DIM);
    static Eigen::VectorXd length(N_DIM);
    radius << 0.06, 0.08, 0.06, 0.06, 0.05, 0.05, 0.04;
    length << 0.18, 0.05, 0.20, 0.05, 0.35, 0.05, 0.05;
    //cout << " start to check collison " << "\n" << endl;
    Eigen::MatrixXd A_(4,4),A0(4,4),A1(4,4),A2(4,4),A3(4,4),A4(4,4),A5(4,4),A6(4,4),A7(4,4),A8(4,4),A9(4,4),A10(4,4),A(4,4);
/*-----------for 7 joints planning-------------------*/
    // A1<<cos(joints[0]),0,-sin(joints[0]),0,sin(joints[0]),0,
    //     -cos(joints[0]),0,0,1,0,length(0),0,0,0,1;
    // A2<<cos(joints[1]),0,sin(joints[1]),-0.0492*cos(joints[1]),sin(joints[1]),0,
    //     -cos(joints[1]),0.0492*sin(joints[1]),0,1,0,0,0,0,0,1;
    // A3<<cos(joints[2]),0,sin(joints[2]),0,sin(joints[2]),0,
    //     -cos(joints[2]),0,0,-1,0,length(2),0,0,0,1;
    // A4<<cos(joints[3]),0,-sin(joints[3]),0,sin(joints[3]),0,
    //     cos(joints[3]),0,0,-1,0,0,0,0,0,1;
    // A5<<cos(joints[4]),0,sin(joints[4]),0,sin(joints[4]),0,
    //     -cos(joints[4]),0,0,1,0,length(4),0,0,0,1;
    // A6<<cos(joints[5]),0,-sin(joints[5]),0,sin(joints[5]),0,
    //     cos(joints[5]),0,0,-1,0,0,0,0,0,1;
    // A7<<cos(joints[6]),-sin(joints[6]),0,0,sin(joints[6]),
    //     cos(joints[6]),0,0,0,0,1,length(6),0,0,0,1;
/*-----------for 10 joints planning----------------*/
    // A_<<cos(joints),-sin(joints),0,a,
    //     sin(joints)*cos(alp),cos(joints)*cos(alp),-sin(alp),-sin(alp)*d,
    //     sin(joints)*sin(alp),cos(joints)*sin(alp),cos(alp),cos(alp)*d,
    //     0,0,0,1;
    //cout << " joint 0 is " <<  joints[0] << "\n" << endl;
    //cout << " joint 1 is " <<  joints[1] << "\n" << endl;
    //cout << " joint 2 is " <<  joints[2] << "\n" << endl;
    A0<<cos(-1.92),0,sin(-1.92),0.167643,
        sin(-1.92),0,-cos(-1.92),-0.25743,
        0,1,0,0.01632,
        0,0,0,1;
    A1<<cos(-joints[0]),-sin(-joints[0]),0,0,
        sin(-joints[0]),cos(-joints[0]),0,0,
        0,0,1,0,
        0,0,0,1;   
    A2<<cos(-joints[1]+1.919841),-sin(-joints[1]+1.919841),0,0,
        0,0,1,0,
        -sin(-joints[1]+1.919841),-cos(-joints[1]+1.919841),0,0,
        0,0,0,1;
    A3<<cos(joints[2]+M_PI/2),-sin(joints[2]+M_PI/2),0,0.049124788,
        0,0,1,0.224655826,
        -sin(joints[2]+M_PI/2),-cos(joints[2]+M_PI/2),0,0,
        0,0,0,1;
    A4<<cos(joints[3]),-sin(joints[3]),0,0,
        0,0,-1,0,
        sin(joints[3]),cos(joints[3]),0,0,
        0,0,0,1;
    A5<<cos(joints[4]),-sin(joints[4]),0,0,
        0,0,1,0.219557803,
        -sin(joints[4]),-cos(joints[4]),0,0,
        0,0,0,1;
    A6<<cos(joints[5]+M_PI/2),-sin(joints[5]+M_PI/2),0,0,
        0,0,-1,0,
        sin(joints[5]+M_PI/2),cos(joints[5]+M_PI/2),0,0,
        0,0,0,1;
    A7<<cos(joints[6]+M_PI/2),-sin(joints[6]+M_PI/2),0,0,
        0,0,-1,0,
        sin(joints[6]+M_PI/2),cos(joints[6]+M_PI/2),0,0,
        0,0,0,1;      
    A8<<cos(joints[7]+M_PI/2),-sin(joints[7]+M_PI/2),0,0.105,
        0,0,-1,0,
        sin(joints[7]+M_PI/2),cos(joints[7]+M_PI/2),0,0,
        0,0,0,1;
    A9<<cos(joints[8]+M_PI/2),-sin(joints[8]+M_PI/2),0,0,
        0,0,-1,0,
        sin(joints[8]+M_PI/2),cos(joints[8]+M_PI/2),0,0,
        0,0,0,1;  
    A10<<cos(joints[9]),-sin(joints[9]),0,0.02584,
        0,0,1,0.459273874,
        -sin(joints[9]),-cos(joints[9]),0,0,
        0,0,0,1;
    //构造10个用来检测碰撞的圆柱体tf
    vector<Transform3f> tfGroup;
    vector<CylinderPtr> cylinderGroup;

    for(unsigned int i=0; i<N_DIM; ++i)
        cylinderGroup.push_back(CylinderPtr(new Cylinder(radius(i),length(i))));

    A = A0*A1;
    Vec3f shoulder(A(0,3),A(1,3),A(2,3));
    Vec3f z_1(A(0,2),A(1,2),A(2,2));
    //每个圆柱体的旋转矩阵
    fcl::Matrix3f z1(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    z1.setIdentity();
    tfGroup.push_back(Transform3f(z1, shoulder * 0.5));
    //cout << " A0 is " << "\n" << A0 << endl;
    A = A*A2;
    fcl::Matrix3f z2(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    tfGroup.push_back(Transform3f(z2, shoulder));
    //cout << " A1 is " << "\n" << A1 << endl;
    //cout << " A2 is " << "\n" << A2 << endl;
    //cout << " A3 is " << "\n" << A3 << endl;
    cout << " A0A1A2 is " << "\n" << A << endl;
    

    //cout << "shoulder/2 tf is " << shoulder*0.5 << endl;
    //cout << "shoulder tf is " << shoulder << endl;
   
    A = A*A3;
    //A = A2*A;
    fcl::Matrix3f z3(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    cout << " A0A1A2A3 is " << "\n" << A << endl;
    A = A*A4;
    //A = A3*A;
    Vec3f elbow(A(0,3),A(1,3),A(2,3));
    fcl::Matrix3f z4(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    tfGroup.push_back(Transform3f(z3, (elbow + shoulder) * 0.5));
    tfGroup.push_back(Transform3f(z4, elbow));
    //cout << "elbow + shoulder tf is " << (elbow + shoulder) * 0.5 << endl;
    //cout << "elbow tf is " << elbow << endl;
    
    cout << " A0A1A2A3A4 is " << "\n" << A << endl;
    //cout << " A3 is " << "\n" << A3 << endl;

    A = A*A5;
    //A = A4*A;
    fcl::Matrix3f z5(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    cout << " A0A1A2A3A4A5 is " << "\n" << A << endl;
    
    //A = A5*A;
    A = A*A6;
    cout << " A0A1A2A3A4A5A6 is " << "\n" << A << endl;
    //Vec3f z_5(A(0,2),A(1,2),A(2,2));
    Vec3f wrist(A(0,3),A(1,3),A(2,3));
    fcl::Matrix3f z6(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    tfGroup.push_back(Transform3f(z5,  (wrist + elbow) * 0.5));
    tfGroup.push_back(Transform3f(z6, wrist));
    //cout << "wrist +elbow tf is " << (wrist + elbow) * 0.5 << endl;
    //cout << "wrist tf is " << wrist << endl;
    
    //A = A6*A;
    A = A*A7;
    fcl::Matrix3f z7(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));

    cout << " A0A1A2A3A4A5A6A7 is " << "\n" << A << endl;
    //A = A7*A;
    Vec3f hand(A(0,3),A(1,3),A(2,3));
    tfGroup.push_back(Transform3f(z7, hand));
    //cout << "hand tf is " << hand << endl;

    //      GJKSolver_indep solver;
    GJKSolver_libccd solver;
    //cout << "collisiongroup size is  " <<collisionGroup1.size() <<endl;
    for(unsigned int k=0; k<collisionGroup.size(); ++k)
    {
        for(int i = N_DIM -1; i >= 0; --i){
            Vec3f contact_points;
            FCL_REAL penetration_depth;
            Vec3f normal;

            bool res = solver.shapeIntersect(*cylinderGroup[i], tfGroup[i], *(collisionGroup[k]), collisionTFGroup[k], &contact_points, &penetration_depth, &normal);
            
            //cout << "contact points: " << contact_points << endl;
            //cout << "pen depth: " << penetration_depth << endl;
            //cout << "normal: " << normal << endl;
            if(res)
            {
                ++collision_joint[i];
                //ROS_INFO("check cylinder Collision in Joint : %d", i+1);
                cout << "check cylinder Collision in Joint :"<< i+1<<"\n";
                return true;
            }
//        else
//            ROS_INFO("No CAllision!");
        }
    }
    /*------check the ----------*/
    for(unsigned int k=0; k<collisionGroup1.size(); ++k)
    {
        for(int i = N_DIM -1; i >= 0; --i){
            Vec3f contact_points;
            FCL_REAL penetration_depth;
            Vec3f normal;

            bool res = solver.shapeIntersect(*cylinderGroup[i], tfGroup[i], *(collisionGroup1[k]), collisionTFGroup1[k], &contact_points, &penetration_depth, &normal);
            
            //cout << "contact points: " << contact_points << endl;
            //cout << "pen depth: " << penetration_depth << endl;
            //cout << "normal: " << normal << endl;
            if(res)
            {
                ++collision_joint[i];
                //ROS_INFO("check the box Collision in Joint : %d", i+1);
                cout << "check the box Collision in Joint : "<< i+1<<"\n";
                return true;
            }
//        else
//            ROS_INFO("No CAllision!");
        }
    }

    return false;
}

bool RRT::checkJointLimits(const Joint &joints)
{
    for(int i=0; i<N_DIM; ++i)
    {
        if(joints[i]>max_[i] || joints[i]<min_[i] )
        {
            return true;
            //break;
		}
    }
    return false;
}

int RRT::getNearestNeighbor(const NodePtr node)
{
    struct kdres* result = kd_nearest(kdTree, node->getJoint().data());
    uintptr_t nearest = (uintptr_t)kd_res_item_data(result);
    return nearest;
}

void RRT::printPath()
{
    path.push_back(goal_->getPoint());
    NodePtr new_node = rrtTree_[bestID];
    while(new_node->getParent() > 0)
    {
        path.push_back(new_node->getPoint());
        new_node = rrtTree_[new_node->getParent()];
    }
//    path.push_back(init_->getPoint());
}

void RRT::printModifiedPath()
{
    unsigned int size_j = traj.size();
    for(unsigned int i=0; i< size_j; ++i)
    {
        mpath.push_back(forwardKinematics(traj[i]));
    }
}

void RRT::writeSmoothPath()
{
    int size_s = straj.size();
    for(int i=0; i< size_s; ++i)
    {
        spath.push_back(forwardKinematics(straj[i]));
    }
    //std::string filepath = ros::package::getPath(PACKAGE);

    std::ofstream rrtSPath("/home/nvidia/.ros/smooth_points.dat");

    int size_p = spath.size();
    for(int i=0; i<size_p; ++i)
    {
        for(int j=0; j<spath[0].size(); ++j)
        {
            rrtSPath<<spath[size_p - i - 1][j]<<"\t";
        }
        rrtSPath<<"\n";
    }
    rrtSPath.close();
}

void RRT::printTrajectory()
{
    traj.push_back(goal_->getJoint());
    NodePtr new_node = rrtTree_[bestID];
    while(new_node->getParent() > 0)
    {
        traj.push_back(new_node->getJoint());
        new_node = rrtTree_[new_node->getParent()];
    }
    traj.push_back(init_->getJoint());
    //cout <<"the traj is " << traj << "\n";; 
    directCut();
}

void RRT::printTree()
{
    //std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtTree("/home/nvidia/.ros/rrt_tree.dat");

    size_t size_p = rrtTree_.size();
    for(size_t i=0; i<size_p; ++i)
    {
        for(size_t j=0; j<rrtTree_[0]->getJoint().size(); ++j)
        {
            rrtTree<<rrtTree_[i]->getJoint()[j]<<"\t";
        }
        rrtTree<<"\n";
    }

    rrtTree.close();
}

void RRT::writeFile()
{
    printPath();

    //std::string filepath = ros::package::getPath(PACKAGE);
    //cout << "the file path is"<< filepath << " ";
    std::ofstream rrtJoints("/home/nvidia/.ros/joints.dat");
    std::ofstream rrtPoints("/home/nvidia/.ros/points.dat");
    std::ofstream rrtMPoints("/home/nvidia/.ros/mpoints.dat");

    size_t size_p = traj.size();
    for(size_t i=0; i<size_p; ++i)
    {   
        cout <<"the step"<<i<<"is :"<<" \n";
        for(size_t j=0; j<traj[0].size(); ++j)
        {
            rrtJoints<<traj[size_p - i - 1][j]<<"\t";
            cout << traj[size_p - i - 1][j] << " ";
        }
        cout <<" \n";
        rrtJoints<<"\n";
    }

    size_t size_j = path.size();
    for(size_t i=0; i<size_j; ++i)
    {
        for(size_t j=0; j<path[0].size(); ++j)
        {
            rrtPoints<<path[size_j - i - 1][j]<<"\t";
        }
        rrtPoints<<"\n";
    }

    size_t size_m = mpath.size();
    for(size_t i=0; i<size_m; ++i)
    {
        for(size_t j=0; j<mpath[0].size(); ++j)
        {
            rrtMPoints<<mpath[size_m - i - 1][j]<<"\t";
        }
        rrtMPoints<<"\n";
    }

    rrtJoints.close();
    rrtPoints.close();
    rrtMPoints.close();
}

void RRT::writeSmoothTraj()
{
    //std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream smoothFile("/home/nvidia/.ros/smooth_joints.dat");

    size_t size_p = straj.size();
    for(size_t i=0; i<size_p; ++i)
    {
        for(size_t j=0; j<straj[0].size(); ++j)
        {
            smoothFile <<straj[size_p - i - 1][j]<<"\t";
        }
        smoothFile<<"\n";
    }
}

void RRT::smooth(const vector<double> &init_vec, const vector<double> &end_vec, bool need, double s_time)
{
    int gap = 3;
    double time_node = 0.04;

    //RRT算法两点间的运行时间
    vector<double> v_start = (traj[gap+1]-traj[gap]) * (1/time_node) ;

//    smoothPart(gap,0, v_start, end_vec, 0.3);
    smoothPart(0,gap,end_vec, v_start, 0.3);

    vector<double> vec_gap = traj[gap+1]-traj[gap];
    int break_point = gap;

    //改善动态ＲＲＴ算法的性能突破�?    
    int smooth_p = need ? (traj.size()-4) : (traj.size());
    for(; break_point<smooth_p; ++break_point)
    {
        vector<double> vec_test = traj[break_point+1]-traj[break_point];
        if(vectorEqual(vec_gap, vec_test))
        {
            nodeSmooth(break_point, time_node);
        }
        else
        {
            cout << "Knot point" << endl;
            //去掉最后四个点
            straj.erase(straj.end()-5, straj.end());

            vector<double> next_gap = traj[break_point+2]-traj[break_point+1];
            vector<double> next_vec_gap = next_gap*(1/time_node);
            vector<double> last_vec_gap = vec_gap*(1/time_node);
            smoothPart(break_point-1,break_point+1,last_vec_gap, next_vec_gap, 0.2);
            vec_gap = next_gap;
//            ++break_point;
        }

    }

//    straj.erase(straj.begin(), straj.begin()+5);

//    straj.erase(straj.end()-1, straj.end());

    //    smoothPart(break_point,break_point+1,last_vec_gap, next_vec_gap, 0.2);

//    nodeSmooth(gap+1, time_node);
//    nodeSmooth(gap, time_node);



//    int p = traj.size();
//    v_start = (traj[p-2]-traj[p-1])*10;
//    smoothPart(p-1, p-2, init_vec, v_start);

    straj.erase(straj.end()-1, straj.end());
    int p = traj.size();
    v_start = (traj[p-1-gap]-traj[p-2-gap]) * (1/time_node) ;
    if(need)
        smoothPart(p-1-gap,p-1,v_start, init_vec, s_time);
    else
    {
    }

    writeSmoothTraj();
}

void RRT::nodeSmooth(int start, double time)
{
//    cout << traj[start+1] << endl;
//    cout << traj[start] << endl;
    int piece = (int)(time*Herz);

    vector<double> v_start = (traj[start+1]-traj[start]) * (1.0/piece);

    for(int i=0; i<piece; ++i)
    {
        vector<double> temp = traj[start] + v_start*(i+1);

        straj.push_back(temp);
    }
}

void RRT::smoothPart(int start, int end, const vector<double> &start_vec, const vector<double> &end_vec, double time)
{
    vector<CubicSpline> cubic_joint;
    for(int i=0; i<N_DIM; ++i)
    {
        CubicSpline cubic_temp;
        cubic_temp = cubicSolve(traj[start][i], traj[end][i], start_vec[i], end_vec[i], time);
        cubic_joint.push_back(cubic_temp);
    }

    for(int i=0; i<=time*Herz; ++i)
    {
        vector<double> temp;
        //cout<<i/Herz<<endl;
        for(int j=0; j<N_DIM; ++j)
        {
            temp.push_back( cubicValue(cubic_joint[j], i/Herz) );
        }
        straj.push_back(temp);
    }
}

void RRT::directCut()
{
    unsigned int max_part = 1;
    for(unsigned int i=0; i<traj.size()-2; ++i)
    {
        max_part = max_part > i ? max_part : i+1;
        for(unsigned int j=traj.size()-1; j>max_part; --j)
        {
            if(checkCollisionAndModify(i, j))
            {
                max_part = j;
                break;
            }
        }
    }
}

bool RRT::checkCollisionAndModify(unsigned int start, unsigned int end)
{
    static double ss = 0.02;
    Joint diff = traj[end] - traj[start];
    unsigned int piece = sqrt(pow(diff[0],2) + pow(diff[1],2))/ss;

    vector<Joint> part_traj;
    for(unsigned int i=1; i<=piece; ++i)
    {
        //统计优化时的碰撞检测次�? 
        ++collision_op;
        Joint temp = traj[start] + diff * (1.0/piece) * i;
        part_traj.push_back(temp);
        if(checkCollisions(temp))
        {
//            cout << "Direct Line Collision!" << endl;
            return false;
        }
    }

    traj.erase(traj.begin()+start+1, traj.begin()+end);
    //    cout << "start: " << start << endl;
    //    cout << traj.size() << endl;
    traj.insert(traj.begin()+start+1, part_traj.begin(), part_traj.end());
//    traj.erase(traj.end(), traj.end()+1);
    return true;
}
