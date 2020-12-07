#pragma once

#include "ReinforcementLearning.h"
#include "navigation/Raven_PathPlanner.h"


ReinforcementLearning::ReinforcementLearning(NodeVector nodes, EdgeListVector edges)
{
	m_Nodes = nodes;
	m_Edges = edges;
	alpha = 0.1;
	gamma = 0.5;
}


//method to create from scratch a new transition matrix used for reinforcement learning
void ReinforcementLearning::BuildTransitionMatrix()
{
	//TransitionMatrix tm;
	for (NodeVector::iterator startingNode = m_Nodes.begin(); startingNode != m_Nodes.end(); ++startingNode)
	{
		std::map<int, std::vector<ReinforcementValue>> map1;
		for (NodeVector::iterator endingNode = m_Nodes.begin(); endingNode != m_Nodes.end(); ++endingNode)
		{
			std::vector<ReinforcementValue> v;
			EdgeList edges = m_Edges[startingNode->Index()];
			for (EdgeList::iterator edge = edges.begin(); edge != edges.end(); ++edge)
			{
				ReinforcementValue val = std::make_tuple(*edge, 1, 0);
				v.push_back(val);
			}
			map1.insert(std::pair<int, std::vector<ReinforcementValue>>(endingNode->Index(), v));
		}
		m_transitionMatrix.insert(std::pair<int, std::map<int, std::vector<ReinforcementValue>>>(startingNode->Index(), map1));
	}
}

//returns the vector of values used for reinforcement learning for the required nodes
std::vector<ReinforcementLearning::ReinforcementValue> ReinforcementLearning::GetValuesForNode(int startingNode, int endingNode)
{
	return m_transitionMatrix.at(startingNode).at(endingNode);
}


//method to get a path from startingPos to endingPos using the transition matrix
std::list<PathEdge> ReinforcementLearning::GetPathFromPosToPos(Vector2D startingPos, Vector2D targetPos, Raven_PathPlanner* pathPlanner)
{
	Raven_Map::NavGraph::NodeType closestNodeToStart = pathPlanner->GetClosestNodeToPosition(startingPos);
	Raven_Map::NavGraph::NodeType closestNodeToTarget = pathPlanner->GetClosestNodeToPosition(targetPos);
	std::list<PathEdge> returnPath;
	bool satisfyingPath = false;

	double xDistance = targetPos.x - startingPos.x;
	double yDistance = targetPos.y - startingPos.y;
	double distanceStartToEnd = sqrt(xDistance * xDistance + yDistance * yDistance);

	int numberOfTries = 0;
	int numberOfTriesLimit = 50;
	while (!satisfyingPath && numberOfTries < numberOfTriesLimit)
	{	//no satisfaying path found -> try again
		numberOfTries++;
		satisfyingPath = true;

		returnPath.clear();
		//add the first part of the path, from starting position to first node
		returnPath.push_back(PathEdge(startingPos, closestNodeToStart.Pos(), NavGraphEdge::normal));

		int nodesUsedLimit = 400;
		int numberOfNodesUsed = 0;	//how many nodes were tried until reaching target
		double pathDistance = 0;
		NodeType currentNode = closestNodeToStart;

		while (currentNode.Index() != closestNodeToTarget.Index() && satisfyingPath)
		{	//if we got to the target -> out of this while
			//if the path is no more satisfying -> out of this while
			//else, keep searching 

			//randomly choose one edge to use between those available, until we reach the last node
			numberOfNodesUsed++;
			std::vector<ReinforcementValue> availableEdgesAndTheirValues = GetValuesForNode(currentNode.Index(), closestNodeToTarget.Index());

			double sumOfQualityCoef = 0;
			for (std::vector<ReinforcementValue>::iterator currentEdge = availableEdgesAndTheirValues.begin();
				currentEdge != availableEdgesAndTheirValues.end();
				currentEdge++)
			{	//parse available edges to sum up their quality
				sumOfQualityCoef += max(0, std::get<1>(*currentEdge));
			}


			if (sumOfQualityCoef != 0)
			{	//if all are nodes do not have negative qualities
				double random = ((double)rand()) / RAND_MAX;
				double currentSumOfValues = 0;
				for (std::vector<ReinforcementValue>::iterator currentEdge = availableEdgesAndTheirValues.begin();
					currentEdge != availableEdgesAndTheirValues.end();
					currentEdge++)
				{	//choose one edge between those available, randomly but weighted with the edge's estimated quality
					//(higher quality => higher chance to be taken)
					if (max(0,std::get<1>(*currentEdge)) / sumOfQualityCoef + currentSumOfValues >= random)
					{	//we choose that edge
						EdgeType edge = std::get<0>(*currentEdge);
						NodeType nextNode = pathPlanner->GetNavGraph().GetNode(edge.To());

						returnPath.push_back(PathEdge(currentNode.Pos(),
							nextNode.Pos(),
							NavGraphEdge::normal));
						xDistance = currentNode.Pos().x - nextNode.Pos().x;
						yDistance = currentNode.Pos().y - nextNode.Pos().y;
						pathDistance += sqrt(xDistance * xDistance + yDistance * yDistance);
						currentNode = nextNode;
						break;
					}
					else
					{
						currentSumOfValues += max(0,std::get<1>(*currentEdge)) / sumOfQualityCoef;
					}
				}
			}
			else
			{	//if all nodes have negative quality, then take one randomly
				int nbOfEdges = availableEdgesAndTheirValues.size();
				double random = ((double)rand()) / RAND_MAX;
				double currentSumOfValues = 0;
				for (std::vector<ReinforcementValue>::iterator currentEdge = availableEdgesAndTheirValues.begin();
					currentEdge != availableEdgesAndTheirValues.end();
					currentEdge++)
				{
					if (1.0 / nbOfEdges + currentSumOfValues >= random)
					{	//then we choose that edge
						EdgeType edge = std::get<0>(*currentEdge);
						NodeType nextNode = pathPlanner->GetNavGraph().GetNode(edge.To());

						returnPath.push_back(PathEdge(currentNode.Pos(),
							nextNode.Pos(),
							NavGraphEdge::normal));
						xDistance = currentNode.Pos().x - nextNode.Pos().x;
						yDistance = currentNode.Pos().y - nextNode.Pos().y;
						pathDistance += sqrt(xDistance * xDistance + yDistance * yDistance);
						currentNode = nextNode;
						break;
					}
					else
					{
						currentSumOfValues += 1.0 / nbOfEdges;
					}
				}
			}
			satisfyingPath = /*(numberOfNodesUsed <= nodesUsedLimit) &&*/ (pathDistance <= 10 * distanceStartToEnd);
		}
		//debug_con << "path distance found : " << pathDistance << " , distance S to E : " << distanceStartToEnd << "";
		//add the last part of the path, from last node to target position
		returnPath.push_back(PathEdge(currentNode.Pos(), targetPos, NavGraphEdge::normal));
	}
	if (numberOfTries >= numberOfTriesLimit)
	{
		debug_con << "request path failed" << "";
		returnPath.clear();
		return returnPath;
	}
	debug_con << "end of request path" /*<< " tries : " << numberOfTries <<*/ " , path size : " << returnPath.size()-2 << "";
	return returnPath;
}

//method to calculate the reward to give from the ratio (best distance - distance made)/ best distance
//low ratio implies great path used and so big reward
double ReinforcementLearning::CalculateRewardFromRatio(double ratio)
{
	if (ratio == 0)
	{	//best possible distance
		return 1000000;
	}
	else if (ratio > -0.25)
	{
		return 64 * (ratio + 1.25);
	}
	else if (ratio > -0.5)
	{
		return 32 * (ratio + 1.5);
	}
	else if (ratio > -1)	
	{   
		return 16 * (ratio + 2);
	}
	else if (ratio > -1.5)
	{
		return 8 * (ratio + 2.5);
	}
	else if (ratio > -2)
	{
		return 4 * (ratio + 3);
	}
	else if (ratio > -3)
	{
		return 2 * (ratio + 4);
	}
	else if (ratio > -4)
	{
		return ratio + 5;
	}
	else if (ratio <= -4)
	{	//that was a bad path -> small negative reward
		return max(-1, ratio + 4);
	}
	//reward = 10 * ((6 * bestDistance - distanceTravelled) / bestDistance);
}


//method to give the reward once a path is completed
void ReinforcementLearning::giveRewardForPathCompleted(std::list<PathEdge> m_completePath, Raven_PathPlanner* pathPlanner)
{
	if (m_completePath.size() == 0)
	{
		return;
	}
	//calculate the distance between start and end of path
	//possible improvement : start a a* search to actually get the best distance, but takes long time
	double x = (++m_completePath.begin())->Source().x - (--m_completePath.end())->Source().x;
	double y = (++m_completePath.begin())->Source().y - (--m_completePath.end())->Source().y;
	double bestDistance = sqrt(x * x + y * y);

	//calculate the effective travelled distance
	double distanceTravelled = 0;
	for (std::list<PathEdge>::iterator currentEdge = ++m_completePath.begin(); currentEdge != --m_completePath.end() && currentEdge != m_completePath.end(); currentEdge++)
	{
		x = currentEdge->Source().x - currentEdge->Destination().x;
		y = currentEdge->Source().y - currentEdge->Destination().y;
		distanceTravelled += sqrt(x * x + y * y);
	}

	//calculate the reward
	double reward;
	if (bestDistance > 10)
	{		//if the distance is not super short
		double ratio = (bestDistance - distanceTravelled) / bestDistance;	//ratio close to 0 = best possible way
		reward = CalculateRewardFromRatio(ratio);
	}
	else
	{
		return;
	}
	debug_con << "calculated reward = " << reward << "";

	Vector2D endNodePosition = (--m_completePath.end())->Source();
	NavGraphNode<Trigger<Raven_Bot>*> endingNode = pathPlanner->GetClosestNodeToPosition(endNodePosition);
	for (std::list<PathEdge>::iterator currentEdge = ++m_completePath.begin();
		currentEdge != m_completePath.end() && currentEdge != --m_completePath.end();
		currentEdge++)
	{	  //update the matrix for the all nodes used on that path
		NodeType startingNode = pathPlanner->GetClosestNodeToPosition(currentEdge->Source());
		NodeType followingNode = pathPlanner->GetClosestNodeToPosition(currentEdge->Destination());
		std::vector<ReinforcementValue> vectorOfValues = GetValuesForNode(startingNode.Index(), endingNode.Index());
		int indexOfTheFollowingNodeInVector = 0;
		for (std::vector<ReinforcementLearning::ReinforcementValue>::iterator value = vectorOfValues.begin();
			value != vectorOfValues.end();
			value++)
		{	//search in the edges the one we just used
			if (std::get<0>(*value).To() == followingNode.Index())
			{
				//search the highest quality of the edges available after the current edge
				double maxQualityFound = 0;
				double totalQuality = 0;
				if (followingNode.Index() == endingNode.Index())
				{
					maxQualityFound = 1;
					totalQuality = 1;
				}
				else
				{
					std::vector<ReinforcementLearning::ReinforcementValue> vectorOfNextValues = GetValuesForNode(followingNode.Index(), endingNode.Index());
					//std::vector<Raven_Map::NavGraph::ReinforcementValue> vectorOfNextValues = transitionMatrix->at(followingNode).at(endingNode);
					for (std::vector<ReinforcementLearning::ReinforcementValue>::iterator valueOfNextNodes = vectorOfNextValues.begin();
						valueOfNextNodes != vectorOfNextValues.end();
						valueOfNextNodes++)
					{
						double quality = std::get<1>(*valueOfNextNodes);
						totalQuality += quality;
						if (quality > maxQualityFound)
						{
							maxQualityFound = quality;
						}
					}
				}
				//now use the formula to update the transitionMatrix
				double newQuality = std::get<1>(*value) + alpha * (reward + gamma * maxQualityFound /* / totalQuality*/ - std::get<1>(*value));
				UpdateValue(startingNode.Index(), endingNode.Index(), indexOfTheFollowingNodeInVector, newQuality);
				break;
			}
			indexOfTheFollowingNodeInVector++;
		}
	}
}

//method to update the quality of the specified edge in the transition matrix
void ReinforcementLearning::UpdateValue(int startingNode, int endingNode, int edgeIndex, double newQuality)
{
	std::get<1>(m_transitionMatrix.at(startingNode).at(endingNode)[edgeIndex]) = newQuality;
	std::get<2>(m_transitionMatrix.at(startingNode).at(endingNode)[edgeIndex]) += 1;
}

void ReinforcementLearning::TrainEpisode(Raven_PathPlanner* pathPlanner, int distance, bool importantNodes)
{
	int nbOfPossibleNodes = m_Nodes.size();
	if (importantNodes)
	{	//train to go to important nodes
		std::vector<int> importantNodes{ 332, 333, 334, 335, 336, 337, 338, 339 };
		int nbOfPossibleImportantNodes = importantNodes.size();
		int node1 = importantNodes[rand() % nbOfPossibleImportantNodes];
		if (distance > 0)
		{	//part to train reinforcement with a specific distance between a node with an item and another node

			int nbOfPossibleEdges = m_Edges[node1].size();
			auto lastEdge = m_Edges[node1].begin();
			int rndAdvance = rand() % nbOfPossibleEdges;
			for (int i = 0; i < rndAdvance; i++)
			{
				lastEdge++;
			}

			int currentDistance = distance;
			while (currentDistance > 1)
			{
				int intermediaryNode = lastEdge->To();
				int nbOfPossibleEdges2 = m_Edges[intermediaryNode].size();
				lastEdge = m_Edges[intermediaryNode].begin();
				int rndAdvance2 = rand() % nbOfPossibleEdges2;
				for (int i = 0; i < rndAdvance2; i++)
				{
					lastEdge++;
				}
				currentDistance -= 1;
			}
			int node2 = lastEdge->To();

			if (node1 != node2)
			{
				giveRewardForPathCompleted(GetPathFromPosToPos(
					m_Nodes[node2].Pos(),
					m_Nodes[node1].Pos(),
					pathPlanner), pathPlanner);
			}
		}
		if (distance == 0)
		{	//train from random nodes to nodes with items
			int node2 = rand() % nbOfPossibleNodes;
			while (node2 == node1)
			{
				node2 = rand() % nbOfPossibleNodes;
			}
			giveRewardForPathCompleted(GetPathFromPosToPos(
				m_Nodes[node2].Pos(),
				m_Nodes[node1].Pos(),
				pathPlanner), pathPlanner);
		}
	}
	else
	{	//not on important nodes
		if (distance > 0)
		{	//train reinforcement with a specific distance between random nodes
			int currentDistance = distance;
			int randomStartingNode = rand() % nbOfPossibleNodes;
			int nbOfPossibleEdges = m_Edges[randomStartingNode].size();
			auto lastEdge = m_Edges[randomStartingNode].begin();
			int rndAdvance = rand() % nbOfPossibleEdges;
			for (int i = 0; i < rndAdvance; i++)
			{
				lastEdge++;
			}
			int node1 = lastEdge->From();


			while (currentDistance > 1)
			{
				int intermediaryNode = lastEdge->To();
				int nbOfPossibleEdges2 = m_Edges[intermediaryNode].size();
				lastEdge = m_Edges[intermediaryNode].begin();
				int rndAdvance2 = rand() % nbOfPossibleEdges2;
				for (int i = 0; i < rndAdvance2; i++)
				{
					lastEdge++;
				}
				currentDistance -= 1;
			}

			int node2 = lastEdge->To();
			if (node1 != node2)
			{
				giveRewardForPathCompleted(GetPathFromPosToPos(
					m_Nodes[node1].Pos(),
					m_Nodes[node2].Pos(),
					pathPlanner), pathPlanner);
			}
		}
		else
		{	//train on totally random nodes
			int node1 = rand() % nbOfPossibleNodes;
			int node2 = node1;
			while (node2 == node1)
			{
				node2 = rand() % nbOfPossibleNodes;
			}
			giveRewardForPathCompleted(GetPathFromPosToPos(
				m_Nodes[node1].Pos(),
				m_Nodes[node2].Pos(),
				pathPlanner), pathPlanner);
		}
	}
}

void ReinforcementLearning::Train(Raven_PathPlanner* pathPlanner)
{
	//train on small distances first
	int maxDistanceToTrain = 51;
	int nbOfTrainings = 100;
	for (int currentDistance = 1; currentDistance < maxDistanceToTrain; currentDistance++)
	{
		for (int i = 0; i < nbOfTrainings; i++)
		{
			TrainEpisode(pathPlanner, currentDistance, false);
			Sleep(1);
		}
		if (currentDistance % 10 == 0)
		{
			SaveTransitionMatrixInFile();
		}
	}

	//now train from a radom node at a specific distance to an important node (with item)
	nbOfTrainings = 100;
	maxDistanceToTrain = 151;
	for (int currentDistance = 1; currentDistance < maxDistanceToTrain; currentDistance++)
	{
		for (int i = 0; i < nbOfTrainings; i++)
		{
			TrainEpisode(pathPlanner, currentDistance, true);
			Sleep(1);
		}
		if (currentDistance % 10 == 0)
		{
			SaveTransitionMatrixInFile();
		}
	}

	//now train on important nodes (with items), from random node
	/*nbOfTrainings = 1000;
	for (int i = 0; i < nbOfTrainings; i++)
	{
		TrainEpisode(pathPlanner, 0, true);
		Sleep(1);
	}
	SaveTransitionMatrixInFile();*/

	//now train on long distances
	/*maxDistanceToTrain = 151;
	nbOfTrainings = 50;
	for (int currentDistance = 51; currentDistance < maxDistanceToTrain; currentDistance++)
	{
		for (int i = 0; i < nbOfTrainings; i++)
		{
			TrainEpisode(pathPlanner, currentDistance, false);
			Sleep(1);
		}
		if (currentDistance % 10 == 0)
		{
			SaveTransitionMatrixInFile();
		}
	}*/

	//now train on totally random nodes
	nbOfTrainings = 1000;
	for (int i = 0; i < nbOfTrainings; i++)
	{
		TrainEpisode(pathPlanner, 0, false);
		Sleep(1);
	}
	SaveTransitionMatrixInFile();
}


//method used to save the transition matrix in a file for future execution
void ReinforcementLearning::SaveTransitionMatrixInFile()
{
	debug_con << "starting saving transition matrix, do not close" << "";
	char saveFile[] = "transitionMatrix.txt";

	std::ofstream file;
	file.open(saveFile, std::ofstream::trunc);
	file << std::fixed;
	int firstMapSize = m_transitionMatrix.size();
	//write first map size
	file << firstMapSize << "\n";
	for (int firstMapParser = 0; firstMapParser < firstMapSize; firstMapParser++)
	{
		std::pair<int, std::map<int, std::vector<ReinforcementValue>>> pair1 = *(m_transitionMatrix.find(firstMapParser));
		std::map<int, std::vector<ReinforcementValue>> map = pair1.second;
		//write first node index
		file << pair1.first << "\n";
		int secondMapSize = map.size();
		//write second map size
		file << secondMapSize << "\n";
		for (int secondMapParser = 0; secondMapParser < secondMapSize; secondMapParser++)
		{
			std::pair<int, std::vector<ReinforcementValue>> pair2 = *(map.find(secondMapParser));
			std::vector<ReinforcementValue> vector = pair2.second;
			//write second node index
			file << pair2.first << std::endl;
			int vectorSize = vector.size();
			//write vector size
			file << vectorSize << "\n";
			for (int vectorParser = 0; vectorParser < vectorSize; vectorParser++)
			{
				/*EdgeType e = std::get<0>(vector[vectorParser]);
				int n = e.To();*/
				int endNodeIndex = std::get<0>(vector[vectorParser]).To();
				double quality = std::get<1>(vector[vectorParser]);
				int numberOfTimesUsed = std::get<2>(vector[vectorParser]);
				//write the tuple values
				file << endNodeIndex << " " << quality << " " << numberOfTimesUsed << std::endl;
			}
		}
	}
	debug_con << "saved transition matrix successfully" << "";
}

//method used to restore the transition matrix from previous execution
void ReinforcementLearning::ReadTransitionMatrixInFile()
{
	debug_con << "starting building transition matrix" << "";
	std::ifstream file("transitionMatrix.txt");
	if (!file.good())
	{	//build the matrix if file doesn't exist yet
		BuildTransitionMatrix();
	}
	else
	{	//else read it
		int numberOfLinesMap1;
		file >> numberOfLinesMap1;
		for (int firstMapIndex = 0; firstMapIndex < numberOfLinesMap1; firstMapIndex++)
		{
			std::map<int, std::vector<ReinforcementValue>> map1;
			int index1;
			file >> index1;
			int numberOfLinesMap2;
			file >> numberOfLinesMap2;
			for (int secondMapIndex = 0; secondMapIndex < numberOfLinesMap2; secondMapIndex++)
			{
				std::vector<ReinforcementValue> v;
				int index2;
				file >> index2;
				int numberOfLinesVector;
				file >> numberOfLinesVector;
				for (int vectorIndex = 0; vectorIndex < numberOfLinesVector; vectorIndex++)
				{
					int endNodeIndex, numberOfTimesUsed;
					double quality;
					file >> endNodeIndex >> quality >> numberOfTimesUsed;
					//find the right edge
					EdgeList edges = m_Edges[index1];
					for (EdgeList::iterator edge = edges.begin(); edge != edges.end(); ++edge)
					{
						if (edge->To() == endNodeIndex)
						{
							ReinforcementValue val = std::make_tuple(*edge, 1, 0);
							v.push_back(val);
						}
					}
					//make the tuple
				}
				map1.insert(std::pair<int, std::vector<ReinforcementValue>>(index2, v));
			}
			m_transitionMatrix.insert(std::pair<int, std::map<int, std::vector<ReinforcementValue>>>(index1, map1));
		}
		std::ifstream mapFile("maps/Raven_DM1.map");
		if (mapFile.good())
		{
			int numberOfNodes;
			mapFile >> numberOfNodes;
			if (m_transitionMatrix.size() != numberOfNodes)
			{	//build the matrix from scratch if file wasn't good
				debug_con << "file wasn't good, building new matrix from scratch" << "";
				m_transitionMatrix.clear();
				BuildTransitionMatrix();
			}
		}
	}
	debug_con << "built transition matrix from file successfully" << "";
};