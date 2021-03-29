
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "chessboard_chesspieces/chessboard_chesspieces.h"
#include "chessboard_chesspieces/board_state_manager.hpp"

#include "marlin_serial/MoveActionAction.h"
#include "marlin_serial/ResetAction.h"
#include "marlin_motion_planner/PieceMoveAction.h"
#include "marlin_motion_planner/PieceDefeatedAction.h"


namespace chessboard
{
    ChessPiece::ChessPiece(uint64_t guid, Sides side, PieceTypes type, int id, int pos_X, int pos_Y) :
     _side(side), _type(type), _tf_id(id), x_pos(pos_X), y_pos(pos_Y), isActive(false), guid(guid)
    {
        //!TODO: same format is used in xacro. Possible to read?
        std::stringstream ss;
        ss << PieceTypeNameStr[type];
        ss << "_" << _tf_id;
        ss << "_" << (_side == SIDE_WHITE ? "white" : "black");

        _tf_name = ss.str();
    }
}

using namespace chessboard;


class MoveAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<marlin_serial::MoveActionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
        actionlib::SimpleActionClient<marlin_motion_planner::PieceMoveAction> ac_move_;
        actionlib::SimpleActionClient<marlin_motion_planner::PieceDefeatedAction> ac_def_;
        std::string action_name_;

        marlin_serial::MoveActionActionFeedback feedback_;
        marlin_serial::MoveActionActionResult result_;

        BoardStateManager & bsm;

    public:
        MoveAction(std::string name, BoardStateManager & manager) : as_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1), false),
                                            action_name_(name), ac_move_("planner_move", true), ac_def_("planner_defeated", true), bsm(manager)
        {
            as_.start();
        }

        ~MoveAction(void)
        {
        }

        void executeCB(const marlin_serial::MoveActionGoalConstPtr &goal)
        {
            int srcX = goal->target[0];
            int srcY = goal->target[1];
            int destX = goal->target[2];
            int destY = goal->target[3];

            const ChessPiece * active = bsm.GetPieceAtPos(srcX, srcY);
            const ChessPiece * target = bsm.GetPieceAtPos(destX, destY);

            bool success = false;

            if (active != NULL)
            {
                if (target != NULL)
                {
                    marlin_motion_planner::PieceDefeatedGoal defGoal;
                    defGoal.source = {target->x_pos, target->y_pos};
                    defGoal.state = bsm.GetState(true);
                    bsm.MovePieceDefeated(target->GetGUID());
                    assert(target != NULL);
                    defGoal.target = {target->x_pos, target->y_pos};
                    ac_def_.sendGoal(defGoal);
                    ac_def_.waitForResult();
                }

                marlin_motion_planner::PieceMoveGoal mpGoal;
                mpGoal.source = {active->x_pos, active->y_pos};
                mpGoal.target = {destX, destY};
                mpGoal.state = bsm.GetState(false);
                if (active->GetType() == chessboard::ChessPiece::PieceTypes::CP_KNIGHT)
                {
                    ROS_ERROR("KNIGHT MOVEMENT: %u %u -> %u %u", active->x_pos, active->y_pos, destX, destY);
                }
                bsm.MovePieceTo(active->GetGUID(), destX, destY);
                ac_move_.sendGoal(mpGoal);
                ac_move_.waitForResult();
                success = true;
            }
            else
            {
                ROS_ERROR_STREAM("Active piece not found at " << goal->target[0] << " " << goal->target[1]);
            }
            

            if (success)
            {
                //result_.sequence = feedback_.sequence;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // set the action state to succeeded
                as_.setSucceeded();
            }
            else
                as_.setAborted();
        }
};

class ResetAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<marlin_serial::ResetAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
        std::string action_name_;

        marlin_serial::ResetActionFeedback feedback_;
        marlin_serial::ResetActionResult result_;
        BoardStateManager & bsm;

    public:
        ResetAction(std::string name, BoardStateManager & manager) : as_(nh_, name, boost::bind(&ResetAction::executeCB, this, _1), false),
                                            action_name_(name), bsm(manager)
        {
            as_.start();
        }

        ~ResetAction(void)
        {
        }

        void executeCB(const marlin_serial::ResetGoalConstPtr &goal)
        {
            ros::Rate r();
            bool success = true;
            // srcx srcy destx desty

            bsm.Reset();

            if (success)
            {
                ROS_WARN("ResetAction called!");
                //result_.sequence = feedback_.sequence;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // set the action state to succeeded
                as_.setSucceeded();
            }
            else
                as_.setAborted();
        }
};

geometry_msgs::Vector3 GetPieceTransform(const ChessPiece * piece)
{
    //!TODO: param!
    float board_square_size = 0.03f; // m
    geometry_msgs::Vector3 translation;
    // XZ plane
    translation.x = (piece->x_pos + 0.5f) * board_square_size;
    translation.z = (piece->y_pos + 0.5f) * board_square_size;
    translation.y = 0.040; // end_effector_1 height

    return translation;
}

int main (int argc, char ** argv)
{
    ros::init(argc, argv, "chessboard_chesspieces_node");

    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(10.0);


    //for (chessboard::ChessPiece & itr : pieces)
    //    ROS_INFO(itr.GetTFName().c_str());

    ChessPiece * activePiece = NULL;

    std::vector<geometry_msgs::TransformStamped> pieceTransforms;

    BoardStateManager bsm;
    bsm.Init();

    MoveAction move_action("chessboard_chesspieces_node", bsm);
    ResetAction reset_action("chessboard_chesspieces_reset", bsm);

    while (ros::ok())
    {
        ros::Time stamp = ros::Time::now();
        for (auto piece : bsm.GetPieces())
        {
            geometry_msgs::TransformStamped pieceTrans;
            pieceTrans.header.stamp = stamp;
            pieceTrans.child_frame_id = piece.GetTFName();

            // fixed rotation
            pieceTrans.transform.rotation = 
            tf::createQuaternionMsgFromRollPitchYaw(0.0, piece.GetSide() == ChessPiece::SIDE_BLACK ? M_PI : 0.0, 0.0);

            if (piece.IsActive())
            {
                pieceTrans.header.frame_id = "end_effector_1";
            }
            else
            {
                pieceTrans.header.frame_id = "home_link";
                pieceTrans.transform.translation = GetPieceTransform(&piece);
            }

            pieceTransforms.push_back(pieceTrans);

        }

        broadcaster.sendTransform(pieceTransforms);
        pieceTransforms.clear();
        

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}