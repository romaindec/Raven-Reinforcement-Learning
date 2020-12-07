#pragma once
#include <vector>
#include <list>
#include <cassert>
#include <string>
#include <iostream>
#include <map>
#include <fstream>  
#include <stdio.h>

#include "2D/Vector2D.h"
#include "../Common/Graph/GraphNodeTypes.h"
#include "../Common/Graph/GraphEdgeTypes.h"
#include "misc/utils.h" 
#include "graph/NodeTypeEnumerations.h"
#include "triggers/Trigger.h"
#include "Raven_Bot.h"

#include "debug/DebugConsole.h"
#include "../Buckland_Chapter7 to 10_Raven/navigation/PathEdge.h"

class ReinforcementLearning
{

protected:

	typedef NavGraphEdge				                EdgeType;
	typedef NavGraphNode<Trigger<Raven_Bot>*>           NodeType;
	typedef std::vector<NodeType>						NodeVector;
	typedef std::list<EdgeType>							EdgeList;
	typedef std::vector<EdgeList>						EdgeListVector;

	//First value is the consideted edge
	//Second is the quality associated to that edge
	//Third value is the number of times that edge was taken
	typedef std::tuple<EdgeType, double, int> ReinforcementValue;

	//the matrix with the values necessary for the reinforcement learning for moving
	//first node : starting/current node
	//second node : arrival node desired
	//third : reinforcement values for the edges around first node
	typedef std::map<int, std::map<int, std::vector<ReinforcementValue>>> TransitionMatrix;

	//The matrix containing all data for transitions between nodes and for the reinforcement learning
	TransitionMatrix m_transitionMatrix;
	NodeVector      m_Nodes;
	EdgeListVector  m_Edges;

	double alpha;	//learning rate : 0 = no learning from new experience, 1 = no memory of last episodes
	double gamma;	//discount factor : 0 = search for fast rewards, 1 = explore more

public:
	ReinforcementLearning(NodeVector nodes, EdgeListVector edges);

	//TransitionMatrix* GetReinforcementMatrix();

	//method to create from scratch a new transition matrix used for reinforcement learning
	void BuildTransitionMatrix();

	//method to get a path from startingPos to endingPos using the transition matrix
	std::list<PathEdge> GetPathFromPosToPos(Vector2D startingPos, Vector2D endingPos, Raven_PathPlanner* pathPlanner);

	//returns the vector of values used for reinforcement learning for the required nodes
	std::vector<ReinforcementValue> GetValuesForNode(int startingNode, int endingNode);

	//method to calculate the reward to give from the ratio (best distance - distance made)/ best distance
	//low ratio implies great path used and so big reward
	double CalculateRewardFromRatio(double ratio);

	//method to give the reward once a path is completed
	void giveRewardForPathCompleted(std::list<PathEdge> m_completePath, Raven_PathPlanner* pathPlanner);

	//method to update the quality of the specified edge in the transition matrix
	void UpdateValue(int startingNode, int endingNode, int edgeIndex, double newQuality);

	//method to train the reinforcement learning
	void TrainEpisode(Raven_PathPlanner* pathPlanner, int distance, bool importantNodes);

	//method to train the reinforcement learning
	void Train(Raven_PathPlanner* pathPlanner);

	//method used to save the transition matrix in a file for future execution
	void SaveTransitionMatrixInFile();

	//method used to restore the transition matrix from previous execution
	void ReadTransitionMatrixInFile();
};