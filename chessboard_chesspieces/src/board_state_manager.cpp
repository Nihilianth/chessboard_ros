#include <algorithm>
#include <ros/ros.h>
#include "chessboard_chesspieces/board_state_manager.hpp"
#include "chessboard_chesspieces/chessboard_chesspieces.h"

namespace chessboard
{
    BoardStateManager::BoardStateManager()
    {
    }

    BoardStateManager::~BoardStateManager()
    {
    }

    void BoardStateManager::Init()
    {
        uint64_t guid = 1;
        for (int i = 0; i < 8; ++i)
        {
            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_WHITE, initLocation[i], i >= 5, i, 0));
            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_WHITE, ChessPiece::CP_PAWN, i, i, 1));

            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_BLACK, initLocation[i], i >= 5, i, 7));
            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_BLACK, ChessPiece::CP_PAWN, i, i, 6));

            _defeat_slots[0][i] = false;
        }
    }

    void BoardStateManager::Reset()
    {
        _chesspieces.clear();
        Init();
    }

    chessboard::ChessPiece *BoardStateManager::GetPieceByGUID(uint64_t guid)
    {
        auto piece = std::find_if(_chesspieces.begin(), _chesspieces.end(), [&guid](const chessboard::ChessPiece &piece) {
                return piece.GetGUID() == guid;
            });
        
        if (piece != _chesspieces.end())
            return &(*piece);

        return NULL;
    }

    const chessboard::ChessPiece *BoardStateManager::GetPieceAtPos(int x, int y) const
    {
        auto piece = std::find_if(_chesspieces.begin(), _chesspieces.end(), [&x, &y](const chessboard::ChessPiece &piece) {
                return piece.x_pos == x && piece.y_pos == y;
            });
        
        if (piece != _chesspieces.end())
            return &(*piece);

        return NULL;
    }

    const chessboard::ChessPiece *BoardStateManager::MovePieceDefeated(uint64_t guid)
    {
        if (chessboard::ChessPiece * cp = GetPieceByGUID(guid))
        {
            int left = cp->x_pos < 5;
            int freeIdx = 0;
            bool success = false;
            // FIXME: only works for 16 figures atm, also needs to utilise both sides
            for (freeIdx = 0; freeIdx < 8; ++freeIdx)
                if (!_defeat_slots[left][freeIdx])
                {
                    success = true;
                    _defeat_slots[left][freeIdx] = true;
                    break;
                }
            
            if (success)
            {
                cp->x_pos = left ? -1 : 8;
                cp->y_pos = freeIdx;
                cp->SetDefeated(true);
                return cp;
            }
        }
        return NULL;
    }

    void BoardStateManager::MovePieceTo(uint64_t guid, int x, int y)
    {
       if (chessboard::ChessPiece * cp = GetPieceByGUID(guid))
        {
            cp->x_pos = x,
            cp->y_pos = y;
        }
    }

    const marlin_motion_planner::BoardState BoardStateManager::GetState()
    {
        std::vector<uint8_t> states(128);


        for (auto itr : _chesspieces)
        {
            if ((itr.x_pos * 8 + itr.y_pos) < 0)
                ROS_ERROR("idx lower than 0");

            states[itr.x_pos * 8 + itr.y_pos] = 1;
        }

        marlin_motion_planner::BoardState board_state;
        board_state.states = states;

        return board_state;
    }


} // namespace chessboard