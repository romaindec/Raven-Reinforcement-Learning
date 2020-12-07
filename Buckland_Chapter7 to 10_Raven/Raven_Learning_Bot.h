#ifndef RAVEN_LBOT_H
#define RAVEN_LBOT_H
#pragma warning (disable:4786)
//-----------------------------------------------------------------------------
//
//  Name:   Raven_Learning_Bot.h
//
//  Author: Mat Buckland (www.ai-junkie.com)
//
//  Desc:
//-----------------------------------------------------------------------------
#include <vector>
#include <iosfwd>
#include <map>

#include "game/MovingEntity.h"
#include "misc/utils.h"
#include "Raven_TargetingSystem.h"
#include "navigation/Raven_ReinforcementPathPlanner.h"


class Raven_PathPlanner;
class Raven_Steering;
class Raven_Game;
class Regulator;
class Raven_Weapon;
struct Telegram;
class Raven_Learning_Bot;
class Goal_Think;
class Raven_WeaponSystem;
class Raven_SensoryMemory;




class Raven_Learning_Bot : public Raven_Bot
{
private:
	//the bot uses this to plan paths
	Raven_ReinforcementPathPlanner*                 m_pReinforcementPathPlanner;

protected:

	//bots shouldn't be copied, only created or respawned
	Raven_Learning_Bot(const Raven_Learning_Bot&);
	Raven_Learning_Bot& operator=(const Raven_Learning_Bot&);



public:

	Raven_Learning_Bot(Raven_Game* world, Vector2D pos);
	virtual ~Raven_Learning_Bot();


	Raven_PathPlanner* const           GetPathPlanner() { return m_pReinforcementPathPlanner; }

};




#endif