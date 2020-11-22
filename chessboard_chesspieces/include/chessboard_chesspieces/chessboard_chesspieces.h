#pragma once
#include <string>

namespace chessboard
{
    

    class ChessPiece
    {


    public:
        enum Sides
        {
            SIDE_WHITE,
            SIDE_BLACK
        };

        enum PieceTypes
        {
            CP_PAWN,
            CP_ROOK,
            CP_QUEEN,
            CP_KING,
            CP_BISHOP,
            CP_KNIGHT,
            CP_TOTAL
        };

        ChessPiece(Sides side, PieceTypes type, int id, int pos_X, int pos_Y);

        ~ChessPiece() {}

        const Sides GetSide() const { return _side; }
        const PieceTypes GetType() const { return _type; }
        const std::string GetTFName() const { return _tf_name; }
        const int GetId() const { return _tf_id; }
        const bool IsActive() const { return isActive; }
        void SetActive(bool newState) { isActive = newState; }

        int x_pos;
        int y_pos;
    protected:

        //!TODO: Make xacro and this node fill the names from config
        const char * PieceTypeNameStr[CP_TOTAL]{
            "pawn",
            "rook",
            "queen",
            "king",
            "bishop",
            "knight"};

        Sides _side;
        PieceTypes _type;
        int _tf_id;
        std::string _tf_name;
        bool isActive;

    };

}