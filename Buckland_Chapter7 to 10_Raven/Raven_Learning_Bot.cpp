#include "Raven_Learning_Bot.h"
#include "misc/Cgdi.h"
#include "misc/utils.h"
#include "2D/Transformations.h"
#include "2D/Geometry.h"
#include "lua/Raven_Scriptor.h"
#include "Raven_Game.h"
#include "navigation/Raven_PathPlanner.h"
#include "Raven_SteeringBehaviors.h"
#include "Raven_UserOptions.h"
#include "time/Regulator.h"
#include "Raven_WeaponSystem.h"
#include "Raven_SensoryMemory.h"

#include "Messaging/Telegram.h"
#include "Raven_Messages.h"
#include "Messaging/MessageDispatcher.h"

#include "goals/Raven_Goal_Types.h"
#include "goals/Goal_Think.h"


#include "Debug/DebugConsole.h"

//-------------------------- ctor ---------------------------------------------
Raven_Learning_Bot::Raven_Learning_Bot(Raven_Game* world, Vector2D pos) :
	Raven_Bot(world, pos)
{
	SetEntityType(type_learning_bot);
	m_bReinforcementBot = true;

	//create the navigation module
	m_pReinforcementPathPlanner = new Raven_ReinforcementPathPlanner(this);

	//create the goal queue
	m_pBrain = new Goal_Think(this);
	//not sure if i should keep this

}

//-------------------------------- dtor ---------------------------------------
//-----------------------------------------------------------------------------
Raven_Learning_Bot::~Raven_Learning_Bot()
{
	delete m_pReinforcementPathPlanner;
}
