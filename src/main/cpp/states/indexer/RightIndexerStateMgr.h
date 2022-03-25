
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

// C++ Includes

// FRC includes
#include <frc/Timer.h>

// Team 302 includes
#include <states/indexer/IndexerStates.h>
#include <states/StateStruc.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/Shooter.h>
#include <states/Intake/RightIntakeStateMgr.h>
#include <subsys/Indexer.h>



// Third Party Includes

class RightIndexerStateMgr : public IndexerStates
{
    public:
		/// @brief  Find or create the state manmanager
		/// @return RightIntakeStateMgr* pointer to the state manager
		static RightIndexerStateMgr* GetInstance();
        void CheckForStateTransition() override;

        RightIndexerStateMgr();
        ~RightIndexerStateMgr() = default;

    protected:
        const StateStruc  m_offState = {INDEXER_STATE::OFF, StateType::RIGHT_INDEXER, true};
        const StateStruc  m_indexState = {INDEXER_STATE::INDEX, StateType::RIGHT_INDEXER, false};
        const StateStruc  m_expelState = {INDEXER_STATE::EXPEL, StateType::RIGHT_INDEXER, false};


    private:
        void DelayForLeftIndexer();

		static RightIndexerStateMgr*	m_instance;
        Indexer*                        m_indexer;
        ShooterStateMgr*                m_shooterStateMgr;
        Shooter*                        m_shooter;
        RightIntakeStateMgr*            m_rightIntakeStateMgr;

        frc::Timer*                      m_timer;
        bool                            m_delay;
};
