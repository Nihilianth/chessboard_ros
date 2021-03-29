#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "chessboard_chesspieces/chessboard_chesspieces.h"
#include "marlin_serial/MoveActionAction.h"
#include "marlin_serial/ResetAction.h"
#include "marlin_motion_planner/PieceMoveAction.h"

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

std::vector<ChessPiece> InitChessPieces()
{
    static const ChessPiece::PieceTypes initLocation[8] =
    {
        ChessPiece::CP_ROOK,
        ChessPiece::CP_KNIGHT,
        ChessPiece::CP_BISHOP,
        ChessPiece::CP_QUEEN,
        ChessPiece::CP_KING,
        ChessPiece::CP_BISHOP,
        ChessPiece::CP_KNIGHT,
        ChessPiece::CP_ROOK
    };

    std::vector<ChessPiece> pieces;

    //for (const auto side : {ChessPiece::SIDE_WHITE, ChessPiece::SIDE_BLACK})
    {
        for (int i = 0; i < 8; ++i)
            pieces.push_back(ChessPiece(0,ChessPiece::SIDE_WHITE, initLocation[i], i >= 5, i, 0));

        for (int i = 0; i < 8; ++i)
            pieces.push_back(ChessPiece(0,ChessPiece::SIDE_WHITE, ChessPiece::CP_PAWN, i, i, 1));
    }
    {
        for (int i = 0; i < 8; ++i)
            pieces.push_back(ChessPiece(0,ChessPiece::SIDE_BLACK, initLocation[i], i >= 5, i, 7));

        for (int i = 0; i < 8; ++i)
            pieces.push_back(ChessPiece(0,ChessPiece::SIDE_BLACK, ChessPiece::CP_PAWN, i, i, 6));
    }

    return pieces;
}

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

class MoveAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<marlin_serial::MoveActionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
        actionlib::SimpleActionClient<marlin_motion_planner::PieceMoveAction> ac_;
        std::string action_name_;

        marlin_serial::MoveActionActionFeedback feedback_;
        marlin_serial::MoveActionActionResult result_;
        std::vector<ChessPiece> &pieces_;

    public:
        MoveAction(std::string name, std::vector<ChessPiece> &pieces) : as_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1), false),
                                            action_name_(name), pieces_(pieces), ac_("planner_move", true)
        {
            as_.start();
        }

        ~MoveAction(void)
        {
        }

        void executeCB(const marlin_serial::MoveActionGoalConstPtr &goal)
        {
            //!TODO: param!
            float board_square_size = 0.03f; // m
            ros::Rate r(10);
            bool success = true;
            // srcx srcy destx desty

            auto activePiece = std::find_if(pieces_.begin(), pieces_.end(), [&goal](const chessboard::ChessPiece &piece) {
                return piece.x_pos == goal->target[0] &&
                piece.y_pos == goal->target[1];
            });
            auto targetPiece = std::find_if(pieces_.begin(), pieces_.end(), [&goal](const chessboard::ChessPiece &piece) {
                return piece.x_pos == goal->target[2] &&
                piece.y_pos == goal->target[3];
            });

            if (activePiece == pieces_.end())
            {
                ROS_ERROR_STREAM("Active piece not found at " << goal->target[0] << " " << goal->target[1]);
                success = false;
            }


            if (targetPiece != pieces_.end())
            {
                // hit another piece
                float offset = (targetPiece->GetSide() == chessboard::ChessPiece::SIDE_WHITE) ? -30.f : 30*9.f;

                if (success)
                {
                    // TODO: find free slot, move it there
                    marlin_motion_planner::PieceMoveGoal mpGoal;
                    //std::vector<float> src = {targetPiece->x_pos * board_square_size, targetPiece->y_pos * board_square_size};
                    //std::vector<float> tgt = {board_square_size * 9, 0};
                    mpGoal.source = {targetPiece->x_pos * board_square_size, targetPiece->y_pos * board_square_size};
                    mpGoal.target = {board_square_size * 9, 0};
                    ac_.sendGoal(mpGoal);
                }

                targetPiece->x_pos = offset;
                targetPiece->y_pos = 30.0f;
            }


            if (success)
            {

                marlin_motion_planner::PieceMoveGoal mpGoal;
                mpGoal.source = {activePiece->x_pos * board_square_size, activePiece->y_pos * board_square_size};

                activePiece->x_pos = goal->target[2];
                activePiece->y_pos = goal->target[3];

                mpGoal.target = {activePiece->x_pos * board_square_size, activePiece->y_pos * board_square_size};
                ac_.sendGoal(mpGoal);
                ac_.waitForResult();

                //result_.sequence = feedback_.sequence;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // set the action state to succeeded
                as_.setSucceeded();
            }
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
        std::vector<ChessPiece> &pieces_;

    public:
        ResetAction(std::string name, std::vector<ChessPiece> &pieces) : as_(nh_, name, boost::bind(&ResetAction::executeCB, this, _1), false),
                                            action_name_(name), pieces_(pieces)
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

            pieces_ = InitChessPieces();

            if (success)
            {
                //result_.sequence = feedback_.sequence;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // set the action state to succeeded
                as_.setSucceeded();
            }
        }
};


int main (int argc, char ** argv)
{
    ros::init(argc, argv, "chessboard_chesspieces_node");

    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(60.0);

    // init chess pieces
    
    std::vector<ChessPiece> pieces = InitChessPieces();


    for (chessboard::ChessPiece & itr : pieces)
        ROS_INFO(itr.GetTFName().c_str());

    ChessPiece * activePiece = NULL;

    auto firstPawn = std::find_if(pieces.begin(), pieces.end(), [] (const chessboard::ChessPiece & piece) 
    {
        return piece.GetSide() == ChessPiece::SIDE_WHITE &&
                piece.GetType() == ChessPiece::CP_PAWN &&
                piece.GetId() == 0;
    });

    // test
    //if (firstPawn != pieces.end())
    //    firstPawn->SetActive(true);


    std::vector<geometry_msgs::TransformStamped> pieceTransforms;

    MoveAction move_action("chessboard_chesspieces_node", pieces);
    ResetAction reset_action("chessboard_chesspieces_reset", pieces);

    while (ros::ok())
    {
        ros::Time stamp = ros::Time::now();
        for (const auto piece : pieces)
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

        //broadcaster.sendTransform(pieceTrans);
            //ROS_INFO(pieceTrans);
        }

        broadcaster.sendTransform(pieceTransforms);
        pieceTransforms.clear();
        

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}