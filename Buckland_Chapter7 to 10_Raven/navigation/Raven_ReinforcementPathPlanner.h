#pragma once
#ifndef REINFORCEMENT_PATHPLANNER_H
#define REINFORCEMENT_PATHPLANNER_H
#pragma warning (disable:4786)
//-----------------------------------------------------------------------------
//
//  Name:   Raven_ReinforcementPathPlanner.h
//
//  Author: Mat Buckland (www.ai-junkie.com)
//
//  Desc:   class to handle the creation of paths through a navigation graph
//-----------------------------------------------------------------------------
#include <list>
#include <vector>
#include "TimeSlicedGraphAlgorithms.h"
#include "Graph/GraphAlgorithms.h"
#include "Graph/SparseGraph.h"
#include "PathEdge.h"
#include "../Raven_Map.h"
#include "Raven_PathPlanner.h"

class Raven_Learning_Bot;


class Raven_ReinforcementPathPlanner : public Raven_PathPlanner
{
private:
	Path path;
public:

	virtual ~Raven_ReinforcementPathPlanner();

	Raven_ReinforcementPathPlanner(Raven_Learning_Bot* owner);

	//creates an instance of the A* time-sliced search and registers it with
	//the path manager
	virtual bool       RequestPathToItem(unsigned int ItemType);

	//creates an instance of the Dijkstra's time-sliced search and registers 
	//it with the path manager
	virtual bool       RequestPathToPosition(Vector2D TargetPos);

	//called by an agent after it has been notified that a search has terminated
	//successfully. The method extracts the path from m_pCurrentSearch, adds
	//additional edges appropriate to the search type and returns it as a list of
	//PathEdges.
	Path       GetPath();
};


#endif

