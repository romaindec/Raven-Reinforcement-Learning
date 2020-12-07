#include "Goal_FollowPath.h"
#include "../Raven_Bot.h"
#include "../Raven_Game.h"

#include "Goal_TraverseEdge.h"
#include "Goal_NegotiateDoor.h"
#include "misc/cgdi.h"
#include "../Buckland_Chapter7 to 10_Raven/ReinforcementLearning.h"




//------------------------------ ctor -----------------------------------------
//-----------------------------------------------------------------------------
Goal_FollowPath::
Goal_FollowPath(Raven_Bot*          pBot,
                std::list<PathEdge> path):Goal_Composite<Raven_Bot>(pBot, goal_follow_path),
                                                  m_Path(path), m_completePath(path)
{
}


//------------------------------ Activate -------------------------------------
//-----------------------------------------------------------------------------
void Goal_FollowPath::Activate()
{
  m_iStatus = active;
  
  //get a reference to the next edge
  PathEdge edge = m_Path.front();

  //remove the edge from the path
  m_Path.pop_front(); 

  //some edges specify that the bot should use a specific behavior when
  //following them. This switch statement queries the edge behavior flag and
  //adds the appropriate goals/s to the subgoal list.
  switch(edge.Behavior())
  {
  case NavGraphEdge::normal:
    {
      AddSubgoal(new Goal_TraverseEdge(m_pOwner, edge, m_Path.empty()));
    }

    break;

  case NavGraphEdge::goes_through_door:
    {

      //also add a goal that is able to handle opening the door
      AddSubgoal(new Goal_NegotiateDoor(m_pOwner, edge, m_Path.empty()));
    }

    break;

  case NavGraphEdge::jump:
    {
      //add subgoal to jump along the edge
    }

    break;

  case NavGraphEdge::grapple:
    {
      //add subgoal to grapple along the edge
    }

    break;

  default:

    throw std::runtime_error("<Goal_FollowPath::Activate>: Unrecognized edge type");
  }
}


//-------------------------- Process ------------------------------------------
//-----------------------------------------------------------------------------
int Goal_FollowPath::Process()
{
  //if status is inactive, call Activate()
  ActivateIfInactive();

  m_iStatus = ProcessSubgoals();

  
  
  if (m_pOwner->GetReinforcement() && m_iStatus == completed && m_Path.size()==2)	//size of 2, because if we use size <= 1,
	  //then when arriving onto an item, the item is picked off before the bot actually arrives to the position (thanks to its size), 
	  //removing the goal aiming to take that item, and so leading to never calling that part of the code
  {	  
	  //if no edges left in the path, and the bot uses reinforcement learning, calculate the reward and update the transition matrix
	  ReinforcementLearning* reinforcement = m_pOwner->GetWorld()->GetMap()->GetReinforcementLearning();
	  Raven_PathPlanner* pathPlanner = m_pOwner->GetPathPlanner();
	  reinforcement->giveRewardForPathCompleted(m_completePath, pathPlanner);
  }
  if (m_iStatus == completed && !m_Path.empty())
  {	  //if there are no subgoals present check to see if the path still has edges.
	  //remaining. If it does then call activate to grab the next edge.
    Activate(); 
  }

  return m_iStatus;
}
 
//---------------------------- Render -----------------------------------------
//-----------------------------------------------------------------------------
void Goal_FollowPath::Render()
{ 
  //render all the path waypoints remaining on the path list
  std::list<PathEdge>::iterator it;
  for (it = m_Path.begin(); it != m_Path.end(); ++it)
  {  
    gdi->BlackPen();
    gdi->LineWithArrow(it->Source(), it->Destination(), 5);
    
    gdi->RedBrush();
    gdi->BlackPen();
    gdi->Circle(it->Destination(), 3);
  }

  //forward the request to the subgoals
  Goal_Composite<Raven_Bot>::Render();
}
  




