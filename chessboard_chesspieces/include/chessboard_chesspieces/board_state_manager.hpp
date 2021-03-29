#include <vector>
#include "chessboard_chesspieces.h"
#include "marlin_motion_planner/BoardState.h"

namespace chessboard
{
    class BoardStateManager
    {
    protected:
        const ChessPiece::PieceTypes initLocation[8] =
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

        std::vector<chessboard::ChessPiece> _chesspieces;
        bool _defeat_slots[2][8];
        const uint8_t BOARD_SIZE_X = 10;
        const uint8_t BOARD_SIZE_Y = 8;

    public:
        BoardStateManager();
        ~BoardStateManager();

        void Init();
        void Reset();

        chessboard::ChessPiece * GetPieceByGUID(uint64_t guid);
        const chessboard::ChessPiece * GetPieceAtPos(int x, int y) const;
        const chessboard::ChessPiece * MovePieceDefeated(uint64_t guid);
        void MovePieceTo(uint64_t guid, int x, int y);
        const std::vector<ChessPiece> & GetPieces() const { return _chesspieces; }
        const marlin_motion_planner::BoardState GetState(bool full = false);
    };
} // namespace chessboard