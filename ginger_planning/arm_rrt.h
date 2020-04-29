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
* Filename:       	arm_rrt.h
* Programmer:       li zola
* Created:        	July 30th, 2019
* Description:		XR1 motion planning
*
* Note:				August 15th, 2019, version: 0.9
*
***************************************************************************************************
*/

#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
//#include <ros/ros.h>
//#include <ros/package.h>
#include <boost/shared_ptr.hpp>

#include <eigen3/Eigen/Dense>
extern "C"{
#include "kdtree.h"
}
//FCL�
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/collision.h>

using std::vector;

#define RANDNM(N,M) N + ((M-N) * (rand() / ((double)RAND_MAX + 1))) // random # between N&M

static int max_count = 4000;
static double stepsize = 0.05;
//static double stepsize = 0.1;
static double limit = 0.05;
static int max_depth = 200;
static unsigned int N_DIM = 7;
static double Herz = 100.0;
//typedef Eigen::Matrix<double,3,1> Point;
typedef vector<double> Joint;//Joint
typedef vector<double> Point;//Point
typedef vector<Joint> Trajectory;
typedef vector<Point> Path;

Joint operator-(const Joint &v1, const Joint &v2);
Joint operator+(const Joint &v1, const Joint &v2);
Joint operator*(const Joint &v1, double scale);
std::ostream& operator<<(std::ostream& os, const Joint &vec);
bool vectorEqual(const Joint &a, const Joint &b);

class Node
{
public:
    Node();
	  ~Node();

    Node(const Joint &vec);
    Node(const Joint &vec, int parent);

    Joint getJoint() const;
    Point getPoint();
    int getParent() const;
    void setParent(int parent);

private:
    Joint joint_;
    Point point_;
    int parent_;//父节点;
};
typedef boost::shared_ptr<Node> NodePtr;

class RRT
{
public:

    typedef enum
    {
        STEP_COLLISION, /**< Collided with obstacle. No added */
        STEP_REACHED,    /**< The Joint that we grow to is less than stepSize away from node we grow from. No node added */
        STEP_PROGRESS    /**< One node added */
    }  StepResult;

    RRT(const NodePtr init, const NodePtr goal);
    ~RRT();
    void run(bool isWrite);
    void addNode(const NodePtr &qnew);
    double deltaNode(const NodePtr node1, const NodePtr node2);

    void setPathSize(int size);
    int getPathSize() const;
    void addObstacle_box(const boost::shared_ptr<fcl::Box> object, const fcl::Transform3f& tf);
    void addObstacle_sphere(const boost::shared_ptr<fcl::Sphere> object, const fcl::Transform3f& tf);
    void addObstacle_cylinder(const boost::shared_ptr<fcl::Cylinder> object, const fcl::Transform3f& tf);
    void clearObstacle();
    bool checkCollisions(const Joint &joints);
    NodePtr getGoal() const;
    int getBestID();
    bool getState();
    //轨迹的平�?    
    void smooth(const vector<double> &init_vec, const vector<double> &end_vec, bool need = true, double s_time = 0.3);
    void writeSmoothPath();

    boost::shared_ptr<fcl::ShapeBase> getObstacle(int index) const;

    //RRT树所有节�?    
    vector<NodePtr> rrtTree_;

    //关节轨迹的向量，维数＝７
    Trajectory traj;
    //优化后的轨迹
    Trajectory straj;

    //末端轨迹的向量，维数�?
    Path path;
    Path mpath;
    Path spath;

	int cPos_[2];
	bool isCollision_;

    //关节角度限制
    Joint min_, max_;

    vector<boost::shared_ptr<fcl::Sphere> > collisionGroup2;
    vector<boost::shared_ptr<fcl::Box> > collisionGroup1;
    vector<boost::shared_ptr<fcl::Cylinder> > collisionGroup;
    vector<fcl::Transform3f> collisionTFGroup;
    vector<fcl::Transform3f> collisionTFGroup1;
    vector<fcl::Transform3f> collisionTFGroup2;
    //The collision check count
    int collision_n;
    int collision_op;
    vector<int> collision_joint;

private:

    void initialize(const NodePtr init, const NodePtr goal);
    StepResult tryStep(const NodePtr randpoint, int nearID);
    void randomStep();
    void directStep();
    void bestRandomStep();
    void cleanup();

    void rewireJoint(const NodePtr& ptr);
    void normalize(Joint &vec);
    void printPath();
    void printModifiedPath();
    void printTrajectory();
    void printTree();
    int getNearestNeighbor(const NodePtr node);
    bool checkJointLimits(const Joint &joints);
    void writeFile();
    void directCut();
    bool checkCollisionAndModify(unsigned int start, unsigned int end);

    void smoothPart(int start, int end, const vector<double> &start_vec, const vector<double> &end_vec, double time);
    void writeSmoothTraj();
    void nodeSmooth(int start, double time);

    NodePtr init_;
    NodePtr goal_;
    int count;
    double bestDelta;
    int bestID;
    StepResult state;
    struct kdtree *kdTree;
    vector<Joint> obstacle;

};

typedef boost::shared_ptr<RRT> RRTPtr;
