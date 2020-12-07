#include "Raven_ReinforcementPathPlanner.h"
#include "../Raven_Game.h"
#include "misc/utils.h"
#include "graph/GraphAlgorithms.h"
#include "misc/Cgdi.h"
#include "../Raven_Learning_Bot.h"
#include "../constants.h"
#include "../Raven_UserOptions.h"
#include "pathmanager.h"
#include "SearchTerminationPolicies.h"
#include "../lua/Raven_Scriptor.h"
#include "misc/CellSpacePartition.h"
#include "../Raven_Messages.h"
#include "Messaging/MessageDispatcher.h"
#include "graph/NodeTypeEnumerations.h"
#include "game/EntityManager.h"


#include "Debug/DebugConsole.h"
//#define SHOW_NAVINFO
#include <cassert>

//---------------------------- ctor -------------------------------------------
//-----------------------------------------------------------------------------
Raven_ReinforcementPathPlanner::Raven_ReinforcementPathPlanner(Raven_Learning_Bot* owner) :
	Raven_PathPlanner(owner)
{
}

//-------------------------- dtor ---------------------------------------------
//-----------------------------------------------------------------------------
Raven_ReinforcementPathPlanner::~Raven_ReinforcementPathPlanner()
{
}



//----------------------------- GetPath ------------------------------------
//
//  called by an agent after it has been notified that a search has terminated
//  successfully. The method extracts the path from m_pCurrentSearch, adds
//  additional edges appropriate to the search type and returns it as a 
//  PathEdges.
//-----------------------------------------------------------------------------
Raven_ReinforcementPathPlanner::Path Raven_ReinforcementPathPlanner::GetPath()
{
	return path;
}


//--------------------------- RequestPathToPosition ------------------------------
//
//  Given a target, this method first determines if nodes can be reached from 
//  the  bot's current position and the target position. If either end point
//  is unreachable the method returns false. 
//
//  If nodes are reachable from both positions then an instance of the time-
//  sliced A* search is created and registered with the search manager. the
//  method then returns true.
//        
//-----------------------------------------------------------------------------
bool Raven_ReinforcementPathPlanner::RequestPathToPosition(Vector2D TargetPos)
{
#ifdef SHOW_NAVINFO
	debug_con << "------------------------------------------------" << "";
#endif
	GetReadyForNewSearch();
	
	//if the target is walkable from the bot's position a path does not need to
	//be calculated, the bot can go straight to the position by ARRIVING at
	//the current waypoint
	/*if (m_pOwner->canWalkTo(TargetPos))
	{
		theNewPath.push_back(PathEdge(m_pOwner->Pos(), TargetPos, NavGraphEdge::normal));
		path = theNewPath;
		return true;
	}*/

	int indexOfClosestNodeToBot = GetIndexOfClosestNodeToPosition(m_pOwner->Pos());
	if (indexOfClosestNodeToBot == no_closest_node_found)
	{
#ifdef SHOW_NAVINFO
		debug_con << "No closest node to bot found!" << "";
#endif
		return false;
	}
	int indexOfclosestNodeToTarget = GetIndexOfClosestNodeToPosition(TargetPos);
	if (indexOfclosestNodeToTarget == no_closest_node_found)
	{
#ifdef SHOW_NAVINFO
		debug_con << "No closest node to target (" << indexOfclosestNodeToTarget << ") found!" << "";
#endif
		return false;
	}


	//search in the transition matrix the nodes we'll use
	path = m_pOwner->GetWorld()->GetMap()->GetReinforcementLearning()->GetPathFromPosToPos(m_pOwner->Pos(), TargetPos, this);
	
	if (path.size() == 0)
	{
		return false;
	}

	//smooth paths if required
	/*if (UserOptions->m_bSmoothPathsQuick)
	{
		SmoothPathEdgesQuick(theNewPath);
	}

	if (UserOptions->m_bSmoothPathsPrecise)
	{
		SmoothPathEdgesPrecise(theNewPath);
	}*/

	Dispatcher->DispatchMsg(SEND_MSG_IMMEDIATELY,
		SENDER_ID_IRRELEVANT,
		m_pOwner->ID(),
		Msg_PathReady,
		NO_ADDITIONAL_INFO);
	
	return true;
}


//------------------------------ RequestPathToItem -----------------------------
//
// Given an item type, this method determines the closest reachable graph node
// to the bot's position and then creates a instance of the time-sliced 
// Dijkstra's algorithm, which it registers with the search manager
//
//-----------------------------------------------------------------------------
bool Raven_ReinforcementPathPlanner::RequestPathToItem(unsigned int ItemType)
{
	int nodeIndexWithShortestDistance = -1;
	double shortestDistanceFound = 1000000;
	for (int nodeIndex = 0; nodeIndex < m_NavGraph.NumNodes(); nodeIndex++)
	{
		Raven_Map::NavGraph::NodeType node = m_NavGraph.GetNode(nodeIndex);
		if (node.ExtraInfo() != NULL &&
			node.ExtraInfo()->isActive() &&
			node.ExtraInfo()->EntityType() == ItemType)
		{	//verifying the item is here
			double xDistance = node.Pos().x - m_pOwner->Pos().x;
			double yDistance = node.Pos().y - m_pOwner->Pos().y;
			double distance = xDistance * xDistance + yDistance * yDistance;
			if (distance < shortestDistanceFound)
			{
				nodeIndexWithShortestDistance = nodeIndex;
				shortestDistanceFound = distance;
			}
		}
	}
	if (nodeIndexWithShortestDistance == -1)
	{
		debug_con << "no active item of that type, return" << "";
		return false;
	}
	bool toReturn = RequestPathToPosition(m_NavGraph.GetNode(nodeIndexWithShortestDistance).Pos());

	if (toReturn)
	{
		void* pTrigger =
			m_NavGraph.GetNode(ItemType).ExtraInfo();

		Dispatcher->DispatchMsg(SEND_MSG_IMMEDIATELY,
			SENDER_ID_IRRELEVANT,
			m_pOwner->ID(),
			Msg_PathReady,
			pTrigger);
	}

	return toReturn;
}
