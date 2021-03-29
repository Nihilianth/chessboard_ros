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
        for (int i = 0; i < 8; ++i) // iterate over x
        {
            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_WHITE, initLocation[i], i >= 5, i, 0));
            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_WHITE, ChessPiece::CP_PAWN, i, i, 1));

            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_BLACK, initLocation[i], i >= 5, i, 7));
            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_BLACK, ChessPiece::CP_PAWN, i, i, 6));
            _defeat_slots[0][i] = false;
        }

        /*for (int i = 0; i < BOARD_SIZE_X; ++i) // iterate over x
        {
            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_WHITE, ChessPiece::CP_DEFEAT_PH, i, 0, i));
            _chesspieces.push_back(ChessPiece(++guid, ChessPiece::SIDE_BLACK, ChessPiece::CP_DEFEAT_PH, i, 9, i));
            _defeat_slots[0][i] = false;
        }*/
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
            bool side = cp->GetSide();
            int slot = -1;
            for (int i = 0; i < 8; ++i)
                if (!_defeat_slots[side][i])
                {
                    slot = i;
                    break;
                }

            if (slot == -1)
            {
                ROS_ERROR("no free slot found");
                return NULL;
            }

            _defeat_slots[side][slot] = true;
            cp->x_pos = side ? 8: -1;
            cp->y_pos = slot;
            cp->SetDefeated(true);
            GetState(true);
            return cp;

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

    const marlin_motion_planner::BoardState BoardStateManager::GetState(bool full)
    {
        uint8_t piece_cnt = full ? 10*10 : 8*8;//_chesspieces.size();
        std::vector<uint8_t> cur_state(piece_cnt);
        marlin_motion_planner::BoardState board_state;

        //std::sort(_chesspieces.begin(), _chesspieces.end(), [] (const ChessPiece & a, const ChessPiece & b) { return a.x_pos < b.x_pos && a.y_pos < b.y_pos; });
        for (auto itr : _chesspieces)
        {
            uint8_t idx = 0;

            if (!full)
            {
                if (itr.x_pos < 0 || itr.x_pos > 7) // FIXME: better way
                    continue;

                idx = itr.x_pos * 8 + itr.y_pos;
            }
            else
            {
                if (itr.x_pos == -1)
                    idx = itr.y_pos;
                else
                    idx = (itr.x_pos + 1) * 10 + itr.y_pos;
            }

            /*if (full)
            {
                ROS_WARN_STREAM("" << itr.x_pos << " " << itr.y_pos << " -> " << (int) idx);
            }*/

            if (idx >= piece_cnt)
                ROS_ERROR_STREAM("index larger than piece_cnt: " << (int) idx << " x " << itr.x_pos << " y " << itr.y_pos);
            cur_state[idx] = 1;
        }

        ROS_WARN("GetState called!");

        if (full)
        {
            std::stringstream ss;

            ss << "\n";
            for (int y = 0; y < 8; ++y)
            {
                for (int x = 0; x < (full ? 10 : 8); ++x)
                {
                    //ROS_WARN_STREAM("" << x << " " << y);
                    ss << (int)cur_state[x * (full ? 10 : 8) + y] << " ";
                }
                ss << "\n";
            }

            ROS_WARN(ss.str().c_str());
        }

        //ROS_WARN_STREAM("idx " << (int)idx << " x " << itr.x_pos << " y " << itr.y_pos << " cnt " << (int)piece_cnt);

        board_state.states = cur_state;
        return board_state;
    }


} // namespace chessboard