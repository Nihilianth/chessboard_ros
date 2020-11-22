#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "chessboard_chesspieces/chessboard_chesspieces.h"

namespace chessboard
{
    ChessPiece::ChessPiece(Sides side, PieceTypes type, int id, int pos_X, int pos_Y) :
     _side(side), _type(type), _tf_id(id), x_pos(pos_X), y_pos(pos_Y), isActive(false)
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
            pieces.push_back(ChessPiece(ChessPiece::SIDE_WHITE, initLocation[i], i >= 5, i, 0));

        for (int i = 0; i < 8; ++i)
            pieces.push_back(ChessPiece(ChessPiece::SIDE_WHITE, ChessPiece::CP_PAWN, i, i, 1));
    }
    {
        for (int i = 0; i < 8; ++i)
            pieces.push_back(ChessPiece(ChessPiece::SIDE_BLACK, initLocation[i], i >= 5, i, 7));

        for (int i = 0; i < 8; ++i)
            pieces.push_back(ChessPiece(ChessPiece::SIDE_BLACK, ChessPiece::CP_PAWN, i, i, 6));
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
    if (firstPawn != pieces.end())
        firstPawn->SetActive(true);


    std::vector<geometry_msgs::TransformStamped> pieceTransforms;

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
        

        loop_rate.sleep();
    }
    return 1;
}