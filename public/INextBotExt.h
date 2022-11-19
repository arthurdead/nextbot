#ifndef INEXTBOTEXT_H
#define INEXTBOTEXT_H

#pragma once

#include <IShareSys.h>

#define SMINTERFACE_NEXTBOT_NAME "INextBotExt"
#define SMINTERFACE_NEXTBOT_VERSION 2

#if SOURCE_ENGINE == SE_TF2
enum Class_T
{
	CLASS_NONE = 0,
	CLASS_PLAYER,
	CLASS_PLAYER_ALLY,

	CLASS_TFGOAL,
	CLASS_TFGOAL_TIMER,
	CLASS_TFGOAL_ITEM,
	CLASS_TFSPAWN,
	CLASS_MACHINE,

	//CUSTOM ADDED!!

	//TF2
	CLASS_ZOMBIE,
	CLASS_HHH,
	CLASS_WIZARD,
	CLASS_EYEBALL,
	CLASS_TANK,

	//
	CLASS_CUSTOM_NPC,

	NUM_AI_CLASSES
};
#else
	#error
#endif

enum npc_type : unsigned char
{
	npc_none =    0,
	npc_any =     (1 << 0),
	npc_undead =  (1 << 1),
	npc_default = (1 << 2),
	npc_custom = (npc_any|(1 << 3)),
#if SOURCE_ENGINE == SE_TF2
	npc_zombie = (npc_default|npc_any|npc_undead|(1 << 4)),
	npc_hhh =    (npc_default|npc_any|npc_undead|(1 << 5)),
	npc_eye =    (npc_default|npc_any|npc_undead|(1 << 6)),
	npc_wizard = (npc_default|npc_any|npc_undead|(1 << 7)),
	npc_tank =   (npc_default|npc_any|(1 << 8)),
#else
	#error
#endif
};

class CBaseEntity;

class INextBotExt : public SourceMod::SMInterface
{
public:
	virtual const char *GetInterfaceName()
	{ return SMINTERFACE_NEXTBOT_NAME; }
	virtual unsigned int GetInterfaceVersion()
	{ return SMINTERFACE_NEXTBOT_VERSION; }

	virtual npc_type entity_to_npc_type(CBaseEntity *pEntity, const char *classname) = 0;
};

#endif
