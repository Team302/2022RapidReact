
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

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/Indexer.h>
#include <subsys/Shooter.h>



// Third Party Includes

class IndexerStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Intake
        enum INDEXER_STATE
        {
            OFF,
            INDEX_LEFT,
            INDEX_RIGHT,
            EXPEL_LEFT,
            EXPEL_RIGHT,
            INDEX_BOTH
        };

        const std::string m_indexerOffXmlString = "INDEXER_OFF";
        const std::string m_indexerIndexLeftXmlString = "INDEX_LEFT";
        const std::string m_indexerIndexRightXmlString = "INDEX_RIGHT";
        const std::string m_indexerExpelLeftXmlString = "EXPEL_LEFT";
        const std::string m_indexerExpelRightXmlString = "EXPEL_RIGHT";
        const std::string m_indexerIndexBothXmlString = "INDEX_BOTH";

        
        const std::map<const std::string, INDEXER_STATE> m_indexerXmlStringToStateEnumMap
        {   {m_indexerOffXmlString, INDEXER_STATE::OFF},
            {m_indexerIndexLeftXmlString, INDEXER_STATE::INDEX_LEFT},
            {m_indexerIndexRightXmlString, INDEXER_STATE::INDEX_RIGHT},
            {m_indexerExpelLeftXmlString, INDEXER_STATE::EXPEL_LEFT},
            {m_indexerExpelRightXmlString, INDEXER_STATE::EXPEL_RIGHT},
            {m_indexerIndexBothXmlString, INDEXER_STATE::INDEX_BOTH},
        };

		/// @brief  Find or create the state manmanager
		/// @return IndexerStateMgr* pointer to the state manager
		static IndexerStateMgr* GetInstance();

        void CheckForStateTransition() override;
        bool IsBallPresent() const;

    protected:
        const StateStruc  m_offState = {INDEXER_STATE::OFF, StateType::INDEXER, true};
        const StateStruc  m_indexLeftState = {INDEXER_STATE::INDEX_LEFT, StateType::INDEXER, false};
        const StateStruc  m_indexRightState = {INDEXER_STATE::INDEX_RIGHT, StateType::INDEXER, false};
        const StateStruc  m_expelLeftState = {INDEXER_STATE::EXPEL_LEFT, StateType::INDEXER, false};
        const StateStruc  m_expelRightState = {INDEXER_STATE::EXPEL_RIGHT, StateType::INDEXER, false};
        const StateStruc  m_indexBothState = {INDEXER_STATE::INDEX_BOTH, StateType::INDEXER, false};

    private:
		static IndexerStateMgr*	m_instance;

        IndexerStateMgr();
        ~IndexerStateMgr() = default;

        Indexer*            m_indexer;
        ShooterStateMgr*    m_shooterStateMgr;
	    INDEXER_STATE                   m_prevIndexState;
        int                             m_loopsWithBallPresent;
        int                             m_loopsToCenterBall;
        bool                            m_keepCurrentState;

        const int NUM_LOOPS_TO_CENTER_BALL = 10;

        bool IsIntakingLeft() const;
        bool IsIntakingRight() const;

};
