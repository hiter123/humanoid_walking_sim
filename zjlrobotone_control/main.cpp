#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
using namespace std;
using namespace KDL;
int main() {
    Tree my_tree;
    bool flag;
    flag = kdl_parser::treeFromFile("/home/zhejianglab/catkin_space/src/zjlrobotone/urdf/zjlrobotone.urdf",my_tree);
    if (!flag)
    {
        cout<<"Load zjlrobotone.urdf failed!"<<endl;
    } else
    {
        cout<<"Load zjlrobotone.urdf succeed!"<<endl;
    }
    Chain chain_left_leg , chain_right_leg , chain_leg;
    bool left_parse_success,right_parse_success , leg_parse_success;
    left_parse_success = my_tree.getChain("base","left_foot_Link",chain_left_leg);
    if (!left_parse_success)
    {
        cout<<("Get chain_left_leg failed!")<<endl;
    } else
    {
        cout<<"Get chain_left_leg succeed!"<<endl;
    }
    right_parse_success = my_tree.getChain("base","right_foot_Link",chain_right_leg);
    if (!right_parse_success)
    {
        cout<<"Get chain_right_leg failed!"<<endl;
    } else
    {
        cout<<"Get chain_left_leg succeed!"<<endl;
    }
    leg_parse_success = my_tree.getChain("left_foot_Link","right_foot_Link",chain_leg);
    int Num_whole_joint = chain_leg.getNrOfJoints();
    if (!leg_parse_success)
    {
        cout<<"Get chain_leg failed!"<<endl;
    } else
    {
        cout<<"Get chain_left_leg succeed!"<<endl;
        cout<<"The number of both leg is"<<Num_whole_joint<<endl;
    }
    /// inverse kinetics of leg
    int Num_joint = chain_right_leg.getNrOfJoints();
    ChainIkSolverPos_LMA solver_left(chain_right_leg);
    ChainIkSolverPos_LMA solver_right(chain_right_leg);
    JntArray q(Num_joint);
    JntArray q_init(Num_joint) , q_init_left(Num_joint) , q_init_right(Num_joint);
    JntArray joint_left(Num_joint),joint_right(Num_joint);
    JntArray q_sol_left(Num_joint) , q_sol_right(Num_joint) , q_sol_right_last(Num_joint) , q_sol_left_last(Num_joint);
    JntArray q_leg(Num_whole_joint);
    Frame pos_goal_left,pos_goal_right;
    pos_goal_left=Frame(Rotation().RPY(0,0,0),Vector(0.00106573234120409, 0.08999999999999, 0.1));
    pos_goal_right=Frame(Rotation().RPY(0,0,0),Vector(0.00106573234120409, -0.08999999999999, 0.1));
    q_init.data = -0.2*q_init.data.setRandom();
    q_init.data = joint_left.data - q_init.data;
    q_init_left = q_init;
    q_init_right = q_init;
    solver_left.CartToJnt(q_init,pos_goal_left,q_sol_left);
    solver_right.CartToJnt(q_init,pos_goal_right,q_sol_right);
    cout<<"q_sol_left: "<<q_sol_left.data<<endl;
    cout<<"q_sol_right: "<<q_sol_right.data<<endl;

    return 0;
}
