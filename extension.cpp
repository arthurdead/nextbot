/**
 * vim: set ts=4 :
 * =============================================================================
 * SourceMod Sample Extension
 * Copyright (C) 2004-2008 AlliedModders LLC.  All rights reserved.
 * =============================================================================
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License, version 3.0, as published by the
 * Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, AlliedModders LLC gives you permission to link the
 * code of this program (as well as its derivative works) to "Half-Life 2," the
 * "Source Engine," the "SourcePawn JIT," and any Game MODs that run on software
 * by the Valve Corporation.  You must obey the GNU General Public License in
 * all respects for all other code used.  Additionally, AlliedModders LLC grants
 * this exception to all derivative works.  AlliedModders LLC defines further
 * exceptions, found in LICENSE.txt (as of this writing, version JULY-31-2007),
 * or <http://www.sourcemod.net/license.php>.
 *
 * Version: $Id$
 */

#define swap V_swap

#define BASEENTITY_H
#define NEXT_BOT
#define GLOWS_ENABLE
#define TF_DLL
#define USES_ECON_ITEMS
#define USE_NAV_MESH
#define RAD_TELEMETRY_DISABLED

#include "extension.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <mathlib/vmatrix.h>
#include <toolframework/itoolentity.h>
#include <ehandle.h>
#include <GameEventListener.h>
#include <eiface.h>
#include <dt_common.h>
#include <shareddefs.h>
#include <util.h>
#include <variant_t.h>
#include <predictioncopy.h>
#include <tier1/utldict.h>
#include <tier1/utlvector.h>
#include <CDetour/detours.h>

/**
 * @file extension.cpp
 * @brief Implement extension code here.
 */

Sample g_Sample{};		/**< Global singleton for extension's main interface */

SMEXT_LINK(&g_Sample);

ICvar *icvar = nullptr;
CGlobalVars *gpGlobals = nullptr;
CBaseEntityList *g_pEntityList = nullptr;

class INextBot;
class CNavArea;
class CNavLadder;
class CFuncElevator;
class CBaseCombatCharacter;
class CNavMesh;
class INextBotComponent;
class PathFollower;
class ISave;
class IRestore;

CNavMesh *TheNavMesh = nullptr;

ConVar nav_authorative("nav_authorative", "0");
ConVar path_expensive_optimize("path_expensive_optimize", "1");

ConVar *tf_nav_combat_decay_rate = nullptr;

int sizeofPath = 0;
int sizeofPathFollower = 0;
int sizeofNextBotCombatCharacter = 0;
int sizeofCTFPathFollower = 0;
int sizeofNextBotGroundLocomotion = 0;

void *PathCTOR = nullptr;
void *PathFollowerCTOR = nullptr;
void *NextBotCombatCharacterCTOR = nullptr;
void *CTFPathFollowerCTOR = nullptr;
void *NextBotGroundLocomotionCTOR = nullptr;

void *PathComputePathDetails = nullptr;
void *PathBuildTrivialPath = nullptr;
void *PathFindNextOccludedNode = nullptr;
void *PathPostProcess = nullptr;
void *CNavMeshGetGroundHeight = nullptr;
void *CNavMeshGetNearestNavArea = nullptr;
void *CBaseEntitySetAbsOrigin = nullptr;

void *NavAreaBuildPathPtr = nullptr;

int CBaseEntityMyNextBotPointer = 0;
int CBaseEntityMyCombatCharacterPointer = 0;
int CBaseCombatCharacterGetLastKnownArea = 0;
int CBaseCombatCharacterUpdateLastKnownArea = 0;
int CBaseEntityWorldSpaceCenter = 0;
int CBaseEntityEyeAngles = 0;
int CFuncNavCostGetCostMultiplier = 0;

int m_vecAbsOriginOffset = 0;
int m_iTeamNumOffset = 0;

template <typename T>
T void_to_func(void *ptr)
{
	union { T f; void *p; };
	p = ptr;
	return f;
}

extern "C"
{
__attribute__((__visibility__("default"), __cdecl__)) double __pow_finite(double a, double b)
{
	return pow(a, b);
}

__attribute__((__visibility__("default"), __cdecl__)) double __log_finite(double a)
{
	return log(a);
}

__attribute__((__visibility__("default"), __cdecl__)) double __exp_finite(double a)
{
	return exp(a);
}

__attribute__((__visibility__("default"), __cdecl__)) double __atan2_finite(double a, double b)
{
	return atan2(a, b);
}

__attribute__((__visibility__("default"), __cdecl__)) float __atan2f_finite(float a, float b)
{
	return atan2f(a, b);
}

__attribute__((__visibility__("default"), __cdecl__)) float __powf_finite(float a, float b)
{
	return powf(a, b);
}

__attribute__((__visibility__("default"), __cdecl__)) float __logf_finite(float a)
{
	return logf(a);
}

__attribute__((__visibility__("default"), __cdecl__)) float __expf_finite(float a)
{
	return expf(a);
}

__attribute__((__visibility__("default"), __cdecl__)) float __acosf_finite(float a)
{
	return acosf(a);
}

__attribute__((__visibility__("default"), __cdecl__)) double __asin_finite(double a)
{
	return asin(a);
}

__attribute__((__visibility__("default"), __cdecl__)) double __acos_finite(double a)
{
	return acos(a);
}
}

#define DECLARE_PREDICTABLE()

#include <collisionproperty.h>

class CBaseEntity : public IServerEntity
{
public:
	DECLARE_CLASS_NOBASE( CBaseEntity );
	DECLARE_SERVERCLASS();
	DECLARE_DATADESC();
	
	INextBot *MyNextBotPointer()
	{
		void **vtable = *(void ***)this;
		return (this->*void_to_func<INextBot *(CBaseEntity::*)()>(vtable[CBaseEntityMyNextBotPointer]))();
	}
	
	CBaseCombatCharacter *MyCombatCharacterPointer()
	{
		void **vtable = *(void ***)this;
		return (this->*void_to_func<CBaseCombatCharacter *(CBaseEntity::*)()>(vtable[CBaseEntityMyCombatCharacterPointer]))();
	}
	
	const Vector &GetAbsOrigin()
	{
		if(m_vecAbsOriginOffset == 0) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_vecAbsOrigin", &info);
			m_vecAbsOriginOffset = info.actual_offset;
		}
		
		return *(Vector *)(((unsigned char *)this) + m_vecAbsOriginOffset);
	}
	
	int GetTeamNumber()
	{
		return *(int *)(((unsigned char *)this) + m_iTeamNumOffset);
	}
	
	const Vector &WorldSpaceCenter( ) const
	{
		void **vtable = *(void ***)this;
		return (this->*void_to_func<const Vector & (CBaseEntity::*)() const>(vtable[CBaseEntityWorldSpaceCenter]))();
	}
	
	const QAngle &EyeAngles( void )
	{
		void **vtable = *(void ***)this;
		return (this->*void_to_func<const QAngle & (CBaseEntity::*)()>(vtable[CBaseEntityEyeAngles]))();
	}
	
	void SetAbsOrigin( const Vector& origin )
	{
		return (this->*void_to_func<void(CBaseEntity::*)(const Vector&)>(CBaseEntitySetAbsOrigin))(origin);
	}
	
	int GetHealth() { return 0; }
	int m_takedamage;
	CCollisionProperty		*CollisionProp() { return nullptr; }
	const CCollisionProperty*CollisionProp() const { return nullptr; }
	void	NetworkStateChanged() { }
	void	NetworkStateChanged( void *pVar ) {  }
	virtual bool ShouldBlockNav() const { return true; }
	int ObjectCaps() { return 0; }
	bool HasSpawnFlags(int) { return 0; }
};

inline bool FClassnameIs(CBaseEntity *pEntity, const char *szClassname)
{ 
	return false;
}

class CBaseToggle : public CBaseEntity
{
};

class CBaseCombatCharacter : public CBaseEntity
{
public:
	CNavArea *GetLastKnownArea()
	{
		void **vtable = *(void ***)this;
		return (this->*void_to_func<CNavArea *(CBaseCombatCharacter::*)()>(vtable[CBaseCombatCharacterGetLastKnownArea]))();
	}
	
	void UpdateLastKnownArea()
	{
		void **vtable = *(void ***)this;
		(this->*void_to_func<void (CBaseCombatCharacter::*)()>(vtable[CBaseCombatCharacterUpdateLastKnownArea]))();
	}
	
	void OnNavAreaRemoved(CNavArea *) {}
};

#define FCAP_USE_IN_RADIUS 0
#define FCAP_IMPULSE_USE 0

#define private public

#include <nav_mesh.h>
#include <nav_area.h>

class CFuncNavCost : public CBaseEntity
{
public:
	float GetCostMultiplier( CBaseCombatCharacter *who ) const
	{
		void **vtable = *(void ***)this;
		return (this->*void_to_func<float (CFuncNavCost::*)(CBaseCombatCharacter *) const>(vtable[CFuncNavCostGetCostMultiplier]))(who);
	}
};

// work-around since client header doesn't like inlined gpGlobals->curtime
float IntervalTimer::Now( void ) const
{
	return gpGlobals->curtime;
}

// work-around since client header doesn't like inlined gpGlobals->curtime
float CountdownTimer::Now( void ) const
{
	return gpGlobals->curtime;
}

enum
{
	TF_TEAM_RED = LAST_SHARED_TEAM+1,
	TF_TEAM_BLUE,
	TF_TEAM_COUNT
};

class CTFNavArea : public CNavArea
{
public:
	bool HasAttributeTF( int flags ) const { return ( m_attributeFlags & flags ) ? true : false; }
	float GetCombatIntensity( void ) const;
	bool IsInCombat( void ) const { return GetCombatIntensity() > 0.01f; }
	
	float m_distanceFromSpawnRoom[ TF_TEAM_COUNT ];
	CUtlVector< CTFNavArea * > m_invasionAreaVector[ TF_TEAM_COUNT ];	// use our team as index to get list of areas the enemy is invading from
	unsigned int m_invasionSearchMarker;

	unsigned int m_attributeFlags;

	CUtlVector< CHandle< CBaseCombatCharacter > > m_potentiallyVisibleActor[ TF_TEAM_COUNT ];

	float m_combatIntensity;
	IntervalTimer m_combatTimer;

	static unsigned int m_masterTFMark;
	unsigned int m_TFMark;					// this area's mark

	// Raid mode -------------------------------------------------
	int m_wanderCount;						// how many wandering defenders to populate here
	// Raid mode -------------------------------------------------

	float m_distanceToBombTarget;
};

float CTFNavArea::GetCombatIntensity( void ) const
{
	if ( !m_combatTimer.HasStarted() )
	{
		return 0.0f;
	}

	float actualIntensity = m_combatIntensity - m_combatTimer.GetElapsedTime() * tf_nav_combat_decay_rate->GetFloat();

	if ( actualIntensity < 0.0f )
	{
		actualIntensity = 0.0f;
	}

	return actualIntensity;
}

float CNavArea::ComputeFuncNavCost( CBaseCombatCharacter *who ) const
{
	float funcCost = 1.0f;

	for( int i=0; i<m_funcNavCostVector.Count(); ++i )
	{
		if ( m_funcNavCostVector[i] != NULL )
		{
			funcCost *= m_funcNavCostVector[i]->GetCostMultiplier( who );
		}
	}

	return funcCost;
}

bool CNavArea::IsConnected( const CNavArea *area, NavDirType dir ) const
{
	// we are connected to ourself
	if (area == this)
		return true;

	if (dir == NUM_DIRECTIONS)
	{
		// search all directions
		for( int d=0; d<NUM_DIRECTIONS; ++d )
		{
			FOR_EACH_VEC( m_connect[ d ], it )
			{
				if (area == m_connect[ d ][ it ].area)
					return true;
			}
		}

		// check ladder connections
		FOR_EACH_VEC( m_ladder[ CNavLadder::LADDER_UP ], it )
		{
			CNavLadder *ladder = m_ladder[ CNavLadder::LADDER_UP ][ it ].ladder;

			if (ladder->m_topBehindArea == area ||
				ladder->m_topForwardArea == area ||
				ladder->m_topLeftArea == area ||
				ladder->m_topRightArea == area)
				return true;
		}

		FOR_EACH_VEC( m_ladder[ CNavLadder::LADDER_DOWN ], dit )
		{
			CNavLadder *ladder = m_ladder[ CNavLadder::LADDER_DOWN ][ dit ].ladder;

			if (ladder->m_bottomArea == area)
				return true;
		}
	}
	else
	{
		// check specific direction
		FOR_EACH_VEC( m_connect[ dir ], it )
		{
			if (area == m_connect[ dir ][ it ].area)
				return true;
		}
	}

	return false;
}

void CNavArea::ComputePortal( const CNavArea *to, NavDirType dir, Vector *center, float *halfWidth ) const
{
	if ( dir == NORTH || dir == SOUTH )
	{
		if ( dir == NORTH )
		{
			center->y = m_nwCorner.y;
		}
		else
		{
			center->y = m_seCorner.y;
		}

		float left = MAX( m_nwCorner.x, to->m_nwCorner.x );
		float right = MIN( m_seCorner.x, to->m_seCorner.x );

		// clamp to our extent in case areas are disjoint
		if ( left < m_nwCorner.x )
		{
			left = m_nwCorner.x;
		}
		else if ( left > m_seCorner.x )
		{
			left = m_seCorner.x;
		}

		if ( right < m_nwCorner.x )
		{
			right = m_nwCorner.x;
		}
		else if ( right > m_seCorner.x )
		{
			right = m_seCorner.x;
		}

		center->x = ( left + right )/2.0f;
		*halfWidth = ( right - left )/2.0f;
	}
	else	// EAST or WEST
	{
		if ( dir == WEST )
		{
			center->x = m_nwCorner.x;
		}
		else
		{
			center->x = m_seCorner.x;
		}

		float top = MAX( m_nwCorner.y, to->m_nwCorner.y );
		float bottom = MIN( m_seCorner.y, to->m_seCorner.y );

		// clamp to our extent in case areas are disjoint
		if ( top < m_nwCorner.y )
		{
			top = m_nwCorner.y;
		}
		else if ( top > m_seCorner.y )
		{
			top = m_seCorner.y;
		}

		if ( bottom < m_nwCorner.y )
		{
			bottom = m_nwCorner.y;
		}
		else if ( bottom > m_seCorner.y )
		{
			bottom = m_seCorner.y;
		}

		center->y = (top + bottom)/2.0f;
		*halfWidth = (bottom - top)/2.0f;
	}

	center->z = GetZ( center->x, center->y );
}

float CNavArea::ComputeAdjacentConnectionHeightChange( const CNavArea *destinationArea ) const
{
	// find which side it is connected on
	int dir;
	for( dir=0; dir<NUM_DIRECTIONS; ++dir )
	{
		if ( IsConnected( destinationArea, (NavDirType)dir ) )
			break;
	}

	if ( dir == NUM_DIRECTIONS )
		return FLT_MAX;

	Vector myEdge;
	float halfWidth;
	ComputePortal( destinationArea, (NavDirType)dir, &myEdge, &halfWidth );

	Vector otherEdge;
	destinationArea->ComputePortal( this, OppositeDirection( (NavDirType)dir ), &otherEdge, &halfWidth );

	return otherEdge.z - myEdge.z;
}

float CNavArea::GetZ( float x, float y ) const RESTRICT
{
	// guard against division by zero due to degenerate areas
	if (m_invDxCorners == 0.0f || m_invDyCorners == 0.0f)
		return m_neZ;

	float u = (x - m_nwCorner.x) * m_invDxCorners;
	float v = (y - m_nwCorner.y) * m_invDyCorners;

	// clamp Z values to (x,y) volume
	
	u = fsel( u, u, 0 );			// u >= 0 ? u : 0
	u = fsel( u - 1.0f, 1.0f, u );	// u >= 1 ? 1 : u

	v = fsel( v, v, 0 );			// v >= 0 ? v : 0
	v = fsel( v - 1.0f, 1.0f, v );	// v >= 1 ? 1 : v

	float northZ = m_nwCorner.z + u * (m_neZ - m_nwCorner.z);
	float southZ = m_swZ + u * (m_seCorner.z - m_swZ);

	return northZ + v * (southZ - northZ);
}

CNavArea *CNavMesh::GetNearestNavArea( const Vector &pos, bool anyZ, float maxDist, bool checkLOS, bool checkGround, int team ) const
{
	return (this->*void_to_func<CNavArea *(CNavMesh::*)(const Vector &, bool, float, bool, bool, int) const>(CNavMeshGetNearestNavArea))(pos, anyZ, maxDist, checkLOS, checkGround, team);
}

bool CNavMesh::GetGroundHeight( const Vector &pos, float *height, Vector *normal ) const
{
	return (this->*void_to_func<bool(CNavMesh::*)(const Vector &, float *, Vector *) const>(CNavMeshGetGroundHeight))(pos, height, normal);
}

template< typename CostFunctor >
bool NavAreaBuildPath( CNavArea *startArea, CNavArea *goalArea, const Vector *goalPos, CostFunctor &costFunc, CNavArea **closestArea = NULL, float maxPathLength = 0.0f, int teamID = TEAM_ANY, bool ignoreNavBlockers = false )
{
	return (void_to_func<bool(*)(CNavArea *, CNavArea *, const Vector *, CostFunctor &, CNavArea **, float, int, bool)>(NavAreaBuildPathPtr))(startArea, goalArea, goalPos, costFunc, closestArea, maxPathLength, teamID, ignoreNavBlockers);
}

class CBaseCombatWeapon;
class Path;
struct animevent_t;
enum MoveToFailureType : int;
struct AI_Response;
using AIConcept_t = int;

class INextBotEventResponder
{
public:
	virtual ~INextBotEventResponder() {}
	
	// these methods are used by derived classes to define how events propagate
	virtual INextBotEventResponder *FirstContainedResponder( void ) const { return nullptr; }
	virtual INextBotEventResponder *NextContainedResponder( INextBotEventResponder *current ) const { return nullptr; }
	
	//
	// Events.  All events must be 'extended' by calling the derived class explicitly to ensure propagation.
	// Each event must implement its propagation in this interface class.
	//
	virtual void OnLeaveGround( CBaseEntity *ground ) {}		// invoked when bot leaves ground for any reason
	virtual void OnLandOnGround( CBaseEntity *ground ) {}		// invoked when bot lands on the ground after being in the air

	virtual void OnContact( CBaseEntity *other, CGameTrace *result = NULL ) {}	// invoked when bot touches 'other'

	virtual void OnMoveToSuccess( const Path *path ) {}		// invoked when a bot reaches the end of the given Path
	virtual void OnMoveToFailure( const Path *path, MoveToFailureType reason ) {}	// invoked when a bot fails to reach the end of the given Path
	virtual void OnStuck( void ) {}							// invoked when bot becomes stuck while trying to move
	virtual void OnUnStuck( void ) {}							// invoked when a previously stuck bot becomes un-stuck and can again move

	virtual void OnPostureChanged( void ) {}					// when bot has assumed new posture (query IBody for posture)

	virtual void OnAnimationActivityComplete( int activity ) {}	// when animation activity has finished playing
	virtual void OnAnimationActivityInterrupted( int activity ) {}// when animation activity was replaced by another animation
	virtual void OnAnimationEvent( animevent_t *event ) {}	// when a QC-file animation event is triggered by the current animation sequence

	virtual void OnIgnite( void ) {}							// when bot starts to burn
	virtual void OnInjured( const CTakeDamageInfo &info ) {}	// when bot is damaged by something
	virtual void OnKilled( const CTakeDamageInfo &info ) {}	// when the bot's health reaches zero
	virtual void OnOtherKilled( CBaseCombatCharacter *victim, const CTakeDamageInfo &info ) {}	// when someone else dies

	virtual void OnSight( CBaseEntity *subject ) {}			// when subject initially enters bot's visual awareness
	virtual void OnLostSight( CBaseEntity *subject ) {}		// when subject leaves enters bot's visual awareness

	virtual void OnSound( CBaseEntity *source, const Vector &pos, KeyValues *keys ) {}				// when an entity emits a sound. "pos" is world coordinates of sound. "keys" are from sound's GameData
	virtual void OnSpokeConcept( CBaseCombatCharacter *who, AIConcept_t concept, AI_Response *response ) {}	// when an Actor speaks a concept
	virtual void OnWeaponFired( CBaseCombatCharacter *whoFired, CBaseCombatWeapon *weapon ) {}		// when someone fires a weapon

	virtual void OnNavAreaChanged( CNavArea *newArea, CNavArea *oldArea ) {}	// when bot enters a new navigation area

	virtual void OnModelChanged( void ) {}					// when the entity's model has been changed	

	virtual void OnPickUp( CBaseEntity *item, CBaseCombatCharacter *giver ) {}	// when something is added to our inventory
	virtual void OnDrop( CBaseEntity *item ) {}									// when something is removed from our inventory
	virtual void OnActorEmoted( CBaseCombatCharacter *emoter, int emote ) {}			// when "emoter" does an "emote" (ie: manual voice command, etc)

	virtual void OnCommandAttack( CBaseEntity *victim ) {}	// attack the given entity
	virtual void OnCommandApproach( const Vector &pos, float range = 0.0f ) {}	// move to within range of the given position
	virtual void OnCommandApproach( CBaseEntity *goal ) {}	// follow the given leader
	virtual void OnCommandRetreat( CBaseEntity *threat, float range = 0.0f ) {}	// retreat from the threat at least range units away (0 == infinite)
	virtual void OnCommandPause( float duration = 0.0f ) {}	// pause for the given duration (0 == forever)
	virtual void OnCommandResume( void ) {}					// resume after a pause

	virtual void OnCommandString( const char *command ) {}	// for debugging: respond to an arbitrary string representing a generalized command

	virtual void OnShoved( CBaseEntity *pusher ) {}			// 'pusher' has shoved me
	virtual void OnBlinded( CBaseEntity *blinder ) {}			// 'blinder' has blinded me with a flash of light

	virtual void OnTerritoryContested( int territoryID ) {}	// territory has been invaded and is changing ownership
	virtual void OnTerritoryCaptured( int territoryID ) {}	// we have captured enemy territory
	virtual void OnTerritoryLost( int territoryID ) {}		// we have lost territory to the enemy

	virtual void OnWin( void ) {}
	virtual void OnLose( void ) {}
};

class ILocomotion;
class IBody;
class IIntention;
class IVision;

class INextBotReply
{
public:
	virtual void OnSuccess( INextBot *bot ) = 0;						// invoked when process completed successfully

	enum FailureReason
	{
		DENIED,
		INTERRUPTED,
		FAILED
	};
	virtual void OnFail( INextBot *bot, FailureReason reason ) = 0;		// invoked when process failed
};

class INextBotComponent : public INextBotEventResponder
{
public:
	INextBotComponent( INextBot *bot, bool reg );
	virtual ~INextBotComponent();

	virtual void Reset( void ) { m_lastUpdateTime = 0; m_curInterval = TICK_INTERVAL; }				// reset to initial state
	virtual void Update( void ) {}									// update internal state
	virtual void Upkeep( void ) {}										// lightweight update guaranteed to occur every server tick

	virtual INextBot *GetBot( void ) const { return m_bot; }
	
	float m_lastUpdateTime;
	float m_curInterval;

	INextBot *m_bot;
	INextBotComponent *m_nextComponent;									// simple linked list of components in the bot
};

class INextBot : public INextBotEventResponder
{
public:
	virtual void Reset( void ) = 0;										// (EXTEND) reset to initial state
	virtual void Update( void ) = 0;									// (EXTEND) update internal state
	virtual void Upkeep( void ) = 0;									// (EXTEND) lightweight update guaranteed to occur every server tick
	
	virtual bool IsRemovedOnReset( void ) const = 0;	// remove this bot when the NextBot manager calls Reset
	
	virtual CBaseCombatCharacter *GetEntity( void ) const	= 0;
	virtual class NextBotCombatCharacter *GetNextBotCombatCharacter( void ) const = 0;
	
	// interfaces are never NULL - return base no-op interfaces at a minimum
	virtual ILocomotion *	GetLocomotionInterface( void ) const = 0;
	virtual IBody *			GetBodyInterface( void ) const = 0;
	virtual IIntention *	GetIntentionInterface( void ) const = 0;
	virtual IVision *		GetVisionInterface( void ) const = 0;

	/**
	 * Attempt to change the bot's position. Return true if successful.
	 */
	virtual bool SetPosition( const Vector &pos ) = 0;
	virtual const Vector &GetPosition( void ) const = 0;				// get the global position of the bot
	
	void RegisterComponent( INextBotComponent *comp )
	{
		comp->m_nextComponent = m_componentList;
		m_componentList = comp;
	}
	
	void UnregisterComponent( INextBotComponent *comp )
	{
		for(INextBotComponent *cur = m_componentList, *prev = nullptr; cur != nullptr; prev = cur, cur = cur->m_nextComponent) {
			if (cur == comp) {
				if(prev != nullptr) {
					prev->m_nextComponent = cur->m_nextComponent;
				} else {
					m_componentList = cur->m_nextComponent;
				}
				cur->m_nextComponent = nullptr;
				break;
			}
		}
	}
	
	void ReplaceComponent(INextBotComponent *whom, INextBotComponent *with)
	{
		with->m_nextComponent = whom->m_nextComponent;
		
		for(INextBotComponent *cur = m_componentList, *prev = nullptr; cur != nullptr; prev = cur, cur = cur->m_nextComponent) {
			if (cur == whom) {
				if(prev != nullptr) {
					prev->m_nextComponent = with;
				} else {
					m_componentList = with;
				}
				break;
			}
		}
	}
	
	INextBotComponent *m_componentList;						// the first component

	const PathFollower *m_currentPath;						// the path we most recently followed

	int m_id;
	bool m_bFlaggedForUpdate;
	int m_tickLastUpdate;

	unsigned int m_debugType;
	mutable int m_debugDisplayLine;

	Vector m_immobileAnchor;
	CountdownTimer m_immobileCheckTimer;
	IntervalTimer m_immobileTimer;

	mutable ILocomotion *m_baseLocomotion;
	mutable IBody		*m_baseBody;
	mutable IIntention	*m_baseIntention;
	mutable IVision		*m_baseVision;
	//mutable IAttention	*m_baseAttention;

	//CUtlVector< NextBotDebugLineType * > m_debugHistory;
};

INextBotComponent::INextBotComponent( INextBot *bot, bool reg )
{
	m_curInterval = TICK_INTERVAL;
	m_lastUpdateTime = 0;
	m_bot = bot;
	
	// register this component with the bot
	if(reg) {
		bot->RegisterComponent( this );
	}
}

INextBotComponent::~INextBotComponent()
{
	m_bot->UnregisterComponent( this );
}

enum QueryResultType : int;
class CKnownEntity;

class IContextualQuery
{
public:
	virtual ~IContextualQuery() = 0;

	virtual QueryResultType			ShouldPickUp( const INextBot *me, CBaseEntity *item ) const = 0;		// if the desired item was available right now, should we pick it up?
	virtual QueryResultType			ShouldHurry( const INextBot *me ) const = 0;							// are we in a hurry?
	virtual QueryResultType			ShouldRetreat( const INextBot *me ) const = 0;							// is it time to retreat?
	virtual QueryResultType			ShouldAttack( const INextBot *me, const CKnownEntity *them ) const = 0;	// should we attack "them"?
	virtual QueryResultType			IsHindrance( const INextBot *me, CBaseEntity *blocker ) const = 0;		// return true if we should wait for 'blocker' that is across our path somewhere up ahead.

	virtual Vector					SelectTargetPoint( const INextBot *me, const CBaseCombatCharacter *subject ) const = 0;		// given a subject, return the world space position we should aim at

	/**
	 * Allow bot to approve of positions game movement tries to put him into.
	 * This is most useful for bots derived from CBasePlayer that go through
	 * the player movement system.
	 */
	virtual QueryResultType IsPositionAllowed( const INextBot *me, const Vector &pos ) const = 0;

	virtual const CKnownEntity *	SelectMoreDangerousThreat( const INextBot *me, 
															   const CBaseCombatCharacter *subject,
															   const CKnownEntity *threat1, 
															   const CKnownEntity *threat2 ) const = 0;	// return the more dangerous of the two threats to 'subject', or NULL if we have no opinion
};

class IIntention : public INextBotComponent, public IContextualQuery
{
public:
	
};

class CKnownEntity;

class IForEachKnownEntity
{
public:
	virtual bool Inspect( const CKnownEntity &known ) = 0;
};

class INextBotEntityFilter
{
public:
	// return true if the given entity passes this filter
	virtual bool IsAllowed( CBaseEntity *entity ) = 0;
};

enum FieldOfViewCheckType { USE_FOV, DISREGARD_FOV };

class CKnownEntity
{
public:
	virtual ~CKnownEntity() = 0;
	virtual void Destroy( void ) = 0;
	virtual void UpdatePosition( void ) = 0;
	virtual CBaseEntity *GetEntity( void ) const = 0;
	virtual const Vector &GetLastKnownPosition( void ) const = 0;
	virtual bool HasLastKnownPositionBeenSeen( void ) const = 0;
	virtual void MarkLastKnownPositionAsSeen( void ) = 0;
	virtual const CNavArea *GetLastKnownArea( void ) const = 0;
	virtual float GetTimeSinceLastKnown( void ) const = 0;
	virtual float GetTimeSinceBecameKnown( void ) const = 0;
	virtual void UpdateVisibilityStatus( bool visible ) = 0;
	virtual bool IsVisibleInFOVNow( void ) const = 0;
	virtual bool IsVisibleRecently( void ) const = 0;
	virtual float GetTimeSinceBecameVisible( void ) const = 0;
	virtual float GetTimeWhenBecameVisible( void ) const = 0;
	virtual float GetTimeSinceLastSeen( void ) const = 0;
	virtual bool WasEverVisible( void ) const = 0;
	virtual bool IsObsolete( void ) const = 0;
	virtual bool operator==( const CKnownEntity &other ) const = 0;
	virtual bool Is( CBaseEntity *who ) const = 0;
};

class IVision : public INextBotComponent
{
public:
	virtual ~IVision() = 0;

	virtual void Reset( void ) = 0;									// reset to initial state
	virtual void Update( void ) = 0;								// update internal state

	//-- attention/short term memory interface follows ------------------------------------------

	//
	// WARNING: Do not keep CKnownEntity pointers returned by these methods, as they can be invalidated/freed 
	//

	/**
	 * Iterate each interesting entity we are aware of.
	 * If functor returns false, stop iterating and return false.
	 * NOTE: known.GetEntity() is guaranteed to be non-NULL
	 */
	virtual bool ForEachKnownEntity( IForEachKnownEntity &func ) = 0;

	virtual void CollectKnownEntities( CUtlVector< CKnownEntity > *knownVector ) = 0;	// populate given vector with all currently known entities

	virtual const CKnownEntity *GetPrimaryKnownThreat( bool onlyVisibleThreats = false ) const = 0;	// return the biggest threat to ourselves that we are aware of
	virtual float GetTimeSinceVisible( int team ) const = 0;				// return time since we saw any member of the given team

	virtual const CKnownEntity *GetClosestKnown( int team = TEAM_ANY ) const = 0;	// return the closest known entity
	virtual int GetKnownCount( int team, bool onlyVisible = false, float rangeLimit = -1.0f ) const = 0;		// return the number of entities on the given team known to us closer than rangeLimit

	virtual const CKnownEntity *GetClosestKnown( const INextBotEntityFilter &filter ) const = 0;	// return the closest known entity that passes the given filter

	virtual const CKnownEntity *GetKnown( const CBaseEntity *entity ) const = 0;		// given an entity, return our known version of it (or NULL if we don't know of it)

	// Introduce a known entity into the system. Its position is assumed to be known
	// and will be updated, and it is assumed to not yet have been seen by us, allowing for learning
	// of known entities by being told about them, hearing them, etc.
	virtual void AddKnownEntity( CBaseEntity *entity ) = 0;

	virtual void ForgetEntity( CBaseEntity *forgetMe ) = 0;			// remove the given entity from our awareness (whether we know if it or not)
	virtual void ForgetAllKnownEntities( void ) = 0;

	//-- physical vision interface follows ------------------------------------------------------

	/**
	 * Populate "potentiallyVisible" with the set of all entities we could potentially see. 
	 * Entities in this set will be tested for visibility/recognition in IVision::Update()
	 */
	virtual void CollectPotentiallyVisibleEntities( CUtlVector< CBaseEntity * > *potentiallyVisible ) = 0;

	virtual float GetMaxVisionRange( void ) const = 0;				// return maximum distance vision can reach
	virtual float GetMinRecognizeTime( void ) const = 0;			// return VISUAL reaction time

	/**
	 * IsAbleToSee() returns true if the viewer can ACTUALLY SEE the subject or position,
	 * taking into account blindness, smoke effects, invisibility, etc.
	 * If 'visibleSpot' is non-NULL, the highest priority spot on the subject that is visible is returned.
	 */ 
	virtual bool IsAbleToSee( CBaseEntity *subject, FieldOfViewCheckType checkFOV, Vector *visibleSpot = NULL ) const = 0;
	virtual bool IsAbleToSee( const Vector &pos, FieldOfViewCheckType checkFOV ) const = 0;

	virtual bool IsIgnored( CBaseEntity *subject ) const = 0;		// return true to completely ignore this entity (may not be in sight when this is called)
	virtual bool IsVisibleEntityNoticed( CBaseEntity *subject ) const = 0;		// return true if we 'notice' the subject, even though we have LOS to it

	/**
	 * Check if 'subject' is within the viewer's field of view
	 */
	virtual bool IsInFieldOfView( const Vector &pos ) const = 0;
	virtual bool IsInFieldOfView( CBaseEntity *subject ) const = 0;
	virtual float GetDefaultFieldOfView( void ) const = 0;			// return default FOV in degrees
	virtual float GetFieldOfView( void ) const = 0;					// return FOV in degrees
	virtual void SetFieldOfView( float horizAngle ) = 0;			// angle given in degrees

	virtual bool IsLineOfSightClear( const Vector &pos ) const = 0;	// return true if the ray to the given point is unobstructed

	/**
	 * Returns true if the ray between the position and the subject is unobstructed.
	 * A visible spot on the subject is returned in 'visibleSpot'.
	 */
	virtual bool IsLineOfSightClearToEntity( const CBaseEntity *subject, Vector *visibleSpot = NULL ) const = 0;

	/// @todo: Implement LookAt system
	virtual bool IsLookingAt( const Vector &pos, float cosTolerance = 0.95f ) const = 0;					// are we looking at the given position
	virtual bool IsLookingAt( const CBaseCombatCharacter *actor, float cosTolerance = 0.95f ) const = 0;	// are we looking at the given actor
};

class ILocomotion : public INextBotComponent
{
public:
	virtual ~ILocomotion() = 0;
	
	virtual void Reset( void ) = 0;								// (EXTEND) reset to initial state
	virtual void Update( void ) = 0;							// (EXTEND) update internal state
	
	//
	// The primary locomotive method
	// Depending on the physics of the bot's motion, it may not actually
	// reach the given position precisely.
	// The 'weight' can be used to combine multiple Approach() calls within
	// a single frame into a single goal (ie: weighted average)
	//
	virtual void Approach( const Vector &goalPos, float goalWeight = 1.0f ) = 0;	// (EXTEND) move directly towards the given position

	//
	// Move the bot to the precise given position immediately, 
	// updating internal state as needed
	// Collision resolution is done to prevent interpenetration, which may prevent
	// the bot from reaching the given position. If no collisions occur, the
	// bot will be at the given position when this method returns.
	//
	virtual void DriveTo( const Vector &pos ) = 0;				// (EXTEND) Move the bot to the precise given position immediately, 

	//
	// Locomotion modifiers
	//
	virtual bool ClimbUpToLedge( const Vector &landingGoal, const Vector &landingForward, const CBaseEntity *obstacle ) = 0;	// initiate a jump to an adjacent high ledge, return false if climb can't start
	virtual void JumpAcrossGap( const Vector &landingGoal, const Vector &landingForward ) = 0;	// initiate a jump across an empty volume of space to far side
	virtual void Jump( void ) = 0;							// initiate a simple undirected jump in the air
	virtual bool IsClimbingOrJumping( void ) = 0;			// is jumping in any form
	virtual bool IsClimbingUpToLedge( void ) = 0;			// is climbing up to a high ledge
	virtual bool IsJumpingAcrossGap( void ) = 0;			// is jumping across a gap to the far side
	virtual bool IsScrambling( void ) = 0;				// is in the middle of a complex action (climbing a ladder, climbing a ledge, jumping, etc) that shouldn't be interrupted

	virtual void Run( void ) = 0;							// set desired movement speed to running
	virtual void Walk( void ) = 0;							// set desired movement speed to walking
	virtual void Stop( void ) = 0;							// set desired movement speed to stopped
	virtual bool IsRunning( void ) = 0;
	virtual void SetDesiredSpeed( float speed )  = 0;			// set desired speed for locomotor movement
	virtual float GetDesiredSpeed( void ) = 0;			// returns the current desired speed

	virtual void SetSpeedLimit( float speed ) = 0;					// set maximum speed bot can reach, regardless of desired speed
	virtual float GetSpeedLimit( void )  = 0;	// get maximum speed bot can reach, regardless of desired speed

	virtual bool IsOnGround( void ) = 0;					// return true if standing on something
	virtual void OnLeaveGround( CBaseEntity *ground ) = 0;	// invoked when bot leaves ground for any reason
	virtual void OnLandOnGround( CBaseEntity *ground ) = 0;	// invoked when bot lands on the ground after being in the air
	virtual CBaseEntity *GetGround( void ) = 0;			// return the current ground entity or NULL if not on the ground
	virtual const Vector &GetGroundNormal( void ) = 0;	// surface normal of the ground we are in contact with
	virtual float GetGroundSpeed( void ) = 0;				// return current world space speed in XY plane
	virtual const Vector &GetGroundMotionVector( void ) = 0;	// return unit vector in XY plane describing our direction of motion - even if we are currently not moving

	virtual void ClimbLadder( const CNavLadder *ladder, const CNavArea *dismountGoal ) = 0;		// climb the given ladder to the top and dismount
	virtual void DescendLadder( const CNavLadder *ladder, const CNavArea *dismountGoal ) = 0;	// descend the given ladder to the bottom and dismount
	virtual bool IsUsingLadder( void ) = 0;				// we are moving to get on, ascending/descending, and/or dismounting a ladder
	virtual bool IsAscendingOrDescendingLadder( void ) = 0;	// we are actually on the ladder right now, either climbing up or down
	virtual bool IsAbleToAutoCenterOnLadder( void )  = 0;

	virtual void FaceTowards( const Vector &target ) = 0;	// rotate body to face towards "target"

	virtual void SetDesiredLean( const QAngle &lean ) = 0;
	virtual const QAngle &GetDesiredLean( void ) = 0;
	

	//
	// Locomotion information
	//
	virtual bool IsAbleToJumpAcrossGaps( void ) = 0;		// return true if this bot can jump across gaps in its path
	virtual bool IsAbleToClimb( void ) = 0;				// return true if this bot can climb arbitrary geometry it encounters

	virtual const Vector &GetFeet( void ) = 0;			// return position of "feet" - the driving point where the bot contacts the ground

	virtual float GetStepHeight( void ) = 0;				// if delta Z is greater than this, we have to jump to get up
	virtual float GetMaxJumpHeight( void ) = 0;			// return maximum height of a jump
	virtual float GetDeathDropHeight( void ) = 0;			// distance at which we will die if we fall

	virtual float GetRunSpeed( void ) = 0;				// get maximum running speed
	virtual float GetWalkSpeed( void ) = 0;				// get maximum walking speed

	virtual float GetMaxAcceleration( void ) = 0;			// return maximum acceleration of locomotor
	virtual float GetMaxDeceleration( void ) = 0;			// return maximum deceleration of locomotor

	virtual const Vector &GetVelocity( void ) = 0;		// return current world space velocity
	virtual float GetSpeed( void ) const = 0;					// return current world space speed (magnitude of velocity)
	virtual const Vector &GetMotionVector( void ) = 0;	// return unit vector describing our direction of motion - even if we are currently not moving

	virtual bool IsAreaTraversable( const CNavArea *baseArea ) = 0;	// return true if given area can be used for navigation

	virtual float GetTraversableSlopeLimit( void ) = 0;	// return Z component of unit normal of steepest traversable slope

	// return true if the given entity can be ignored during locomotion
	enum TraverseWhenType 
	{ 
		IMMEDIATELY,		// the entity will not block our motion - we'll carry right through
		EVENTUALLY			// the entity will block us until we spend effort to open/destroy it
	};

	/**
	 * Return true if this locomotor could potentially move along the line given.
	 * If false is returned, fraction of walkable ray is returned in 'fraction'
	 */
	virtual bool IsPotentiallyTraversable( const Vector &from, const Vector &to, TraverseWhenType when = EVENTUALLY, float *fraction = NULL ) = 0;

	/**
	 * Return true if there is a possible "gap" that will need to be jumped over
	 * If true is returned, fraction of ray before gap is returned in 'fraction'
	 */
	virtual bool HasPotentialGap( const Vector &from, const Vector &to, float *fraction = NULL ) = 0;

	// return true if there is a "gap" here when moving in the given direction
	virtual bool IsGap( const Vector &pos, const Vector &forward ) = 0;

	virtual bool IsEntityTraversable( CBaseEntity *obstacle, TraverseWhenType when = EVENTUALLY ) = 0;

	//
	// Stuck state.  If the locomotor cannot make progress, it becomes "stuck" and can only leave 
	// this stuck state by successfully moving and becoming un-stuck.
	//
	virtual bool IsStuck( void ) = 0;				// return true if bot is stuck 
	virtual float GetStuckDuration( void ) = 0;	// return how long we've been stuck
	virtual void ClearStuckStatus( const char *reason = "" ) = 0;	// reset stuck status to un-stuck

	virtual bool IsAttemptingToMove( void ) = 0;	// return true if we have tried to Approach() or DriveTo() very recently

	/**
	 * Should we collide with this entity?
	 */
	virtual bool ShouldCollideWith( const CBaseEntity *object ) = 0;

	virtual void AdjustPosture( const Vector &moveGoal ) = 0;
	virtual void StuckMonitor( void ) = 0;
};

class IBody : public INextBotComponent
{
public:
	IBody( INextBot *bot, bool reg ) : INextBotComponent( bot, reg ) { }
	virtual ~IBody() {}

	virtual void Reset( void )  { INextBotComponent::Reset(); }			// reset to initial state
	virtual void Update( void )  {}										// update internal state

	/**
	 * Move the bot to a new position.
	 * If the body is not currently movable or if it
	 * is in a motion-controlled animation activity 
	 * the position will not be changed and false will be returned.
	 */
	virtual bool SetPosition( const Vector &pos ) = 0;

	virtual const Vector &GetEyePosition( void ) = 0;					// return the eye position of the bot in world coordinates
	virtual const Vector &GetViewVector( void ) = 0;					// return the view unit direction vector in world coordinates

	enum LookAtPriorityType
	{
		BORING,
		INTERESTING,				// last known enemy location, dangerous sound location
		IMPORTANT,					// a danger
		CRITICAL,					// an active threat to our safety
		MANDATORY					// nothing can interrupt this look at - two simultaneous look ats with this priority is an error
	};
	virtual void AimHeadTowards( const Vector &lookAtPos, 
								 LookAtPriorityType priority = BORING, 
								 float duration = 0.0f,
								 INextBotReply *replyWhenAimed = NULL,
								 const char *reason = NULL ) {
		if ( replyWhenAimed )
		{
			replyWhenAimed->OnFail( GetBot(), INextBotReply::FAILED );
		}
	}		// aim the bot's head towards the given goal
	virtual void AimHeadTowards( CBaseEntity *subject,
								 LookAtPriorityType priority = BORING, 
								 float duration = 0.0f,
								 INextBotReply *replyWhenAimed = NULL,
								 const char *reason = NULL ) {
		if ( replyWhenAimed )
		{
			replyWhenAimed->OnFail( GetBot(), INextBotReply::FAILED );
		}
	}		// continually aim the bot's head towards the given subject

	virtual bool IsHeadAimingOnTarget( void ) const { return false; }				// return true if the bot's head has achieved its most recent lookat target
	virtual bool IsHeadSteady( void ) const { return true; }						// return true if head is not rapidly turning to look somewhere else
	virtual float GetHeadSteadyDuration( void ) const { return 0.0f; }				// return the duration that the bot's head has not been rotating
	virtual float GetHeadAimSubjectLeadTime( void ) const { return 0.0f; }			// return how far into the future we should predict our moving subject's position to aim at when tracking subject look-ats
	virtual float GetHeadAimTrackingInterval( void ) const { return 0.0f; }			// return how often we should sample our target's position and velocity to update our aim tracking, to allow realistic slop in tracking
	virtual void ClearPendingAimReply( void ) {}					// clear out currently pending replyWhenAimed callback

	virtual float GetMaxHeadAngularVelocity( void ) const { return 1000.0f; }			// return max turn rate of head in degrees/second

	enum ActivityType 
	{ 
		MOTION_CONTROLLED_XY	= 0x0001,	// XY position and orientation of the bot is driven by the animation.
		MOTION_CONTROLLED_Z		= 0x0002,	// Z position of the bot is driven by the animation.
		ACTIVITY_UNINTERRUPTIBLE= 0x0004,	// activity can't be changed until animation finishes
		ACTIVITY_TRANSITORY		= 0x0008,	// a short animation that takes over from the underlying animation momentarily, resuming it upon completion
		ENTINDEX_PLAYBACK_RATE	= 0x0010,	// played back at different rates based on entindex
	};

	/**
	 * Begin an animation activity, return false if we cant do that right now.
	 */
	virtual bool StartActivity( Activity act, unsigned int flags = 0 ) { return false; }
	virtual int SelectAnimationSequence( Activity act ) const { return 0; }			// given an Activity, select and return a specific animation sequence within it

	virtual Activity GetActivity( void ) const { return ACT_INVALID; }							// return currently animating activity
	virtual bool IsActivity( Activity act ) const { return false; }						// return true if currently animating activity matches the given one
	virtual bool HasActivityType( unsigned int flags ) const { return false; }			// return true if currently animating activity has any of the given flags

	enum PostureType
	{
		STAND,
		CROUCH,
		SIT,
		CRAWL,
		LIE
	};
	virtual void SetDesiredPosture( PostureType posture )  {}			// request a posture change
	virtual PostureType GetDesiredPosture( void ) const { return IBody::STAND; }				// get posture body is trying to assume
	virtual bool IsDesiredPosture( PostureType posture ) const { return true; }			// return true if body is trying to assume this posture
	virtual bool IsInDesiredPosture( void ) const { return true; }						// return true if body's actual posture matches its desired posture

	virtual PostureType GetActualPosture( void ) const { return IBody::STAND; }					// return body's current actual posture
	virtual bool IsActualPosture( PostureType posture ) const { return true; }			// return true if body is actually in the given posture

	virtual bool IsPostureMobile( void ) const { return true; }							// return true if body's current posture allows it to move around the world
	virtual bool IsPostureChanging( void ) const { return false; }						// return true if body's posture is in the process of changing to new posture
	
	
	/**
	 * "Arousal" is the level of excitedness/arousal/anxiety of the body.
	 * Is changes instantaneously to avoid complex interactions with posture transitions.
	 */
	enum ArousalType
	{
		NEUTRAL,
		ALERT,
		INTENSE
	};
	virtual void SetArousal( ArousalType arousal ) {}					// arousal level change
	virtual ArousalType GetArousal( void ) const { return IBody::NEUTRAL; }						// get arousal level
	virtual bool IsArousal( ArousalType arousal ) const { return true; }				// return true if body is at this arousal level


	virtual float GetHullWidth( void ) { return 26.0f; }							// width of bot's collision hull in XY plane
	virtual float GetHullHeight( void ) { return 16.0f; }							// height of bot's current collision hull based on posture
	virtual float GetStandHullHeight( void ) { return 68.0f; }						// height of bot's collision hull when standing
	virtual float GetCrouchHullHeight( void ) { return 32.0f; }					// height of bot's collision hull when crouched
	virtual const Vector &GetHullMins( void ) { return vec3_origin; }					// return current collision hull minimums based on actual body posture
	virtual const Vector &GetHullMaxs( void ) { return vec3_origin; }					// return current collision hull maximums based on actual body posture

	virtual unsigned int GetSolidMask( void ) { return MASK_NPCSOLID; }					// return the bot's collision mask (hack until we get a general hull trace abstraction here or in the locomotion interface)
	virtual unsigned int GetCollisionGroup( void ) { return COLLISION_GROUP_NONE; }
};

class IBodyCustom : public IBody
{
public:
	IBodyCustom( INextBot *bot, bool reg ) : IBody( bot, reg ) {
		
		HullWidth = 26.0f;
		HullHeight = 68.0f;
		
		hullMins.x = -HullWidth/2.0f;
		hullMins.y = hullMins.x;
		hullMins.z = 0.0f;
		
		hullMaxs.x = HullWidth/2.0f;
		hullMaxs.y = hullMaxs.x;
		hullMaxs.z = HullHeight;
	}
	
	void SetHullWidth(float height)
	{
		HullWidth = height;
	}
	
	void SetHullHeight(float height)
	{
		HullHeight = height;
	}
	
	void SetHullMins(Vector &&height)
	{
		hullMins = std::move(height);
	}
	
	void SetHullMaxs(Vector &&height)
	{
		hullMaxs = std::move(height);
	}
	
	float HullWidth;
	float HullHeight;
	float StandHullHeight = 68.0f;
	float CrouchHullHeight = 32.0f;
	int SolidMask = MASK_NPCSOLID;
	int CollisionGroup = COLLISION_GROUP_NONE;
	Vector eye;
	Vector view;
	Vector hullMaxs;
	Vector hullMins;
	
	float GetHullWidth( void ) override { return HullWidth; }							// width of bot's collision hull in XY plane
	float GetHullHeight( void ) override { return HullHeight; }							// height of bot's current collision hull based on posture
	float GetStandHullHeight( void ) override { return StandHullHeight; }						// height of bot's collision hull when standing
	float GetCrouchHullHeight( void ) override { return CrouchHullHeight; }					// height of bot's collision hull when crouched
	const Vector &GetHullMins( void ) override { return hullMaxs; }					// return current collision hull minimums based on actual body posture
	const Vector &GetHullMaxs( void ) override { return hullMins; }					// return current collision hull maximums based on actual body posture

	unsigned int GetSolidMask( void ) override { return SolidMask; }					// return the bot's collision mask (hack until we get a general hull trace abstraction here or in the locomotion interface)
	unsigned int GetCollisionGroup( void ) override { return CollisionGroup; }
	
	const Vector &GetEyePosition( void ) override
	{
		eye = GetBot()->GetEntity()->WorldSpaceCenter();
		return eye;
	}
	const Vector &GetViewVector( void ) override
	{
		AngleVectors( GetBot()->GetEntity()->EyeAngles(), &view );
		return view;
	}
	
	bool SetPosition( const Vector &pos ) override
	{
		GetBot()->GetEntity()->SetAbsOrigin( pos );
		return true;
	}
};

class NextBotCombatCharacter : public CBaseCombatCharacter, INextBot
{
public:
	static NextBotCombatCharacter *create()
	{
		NextBotCombatCharacter *bytes = (NextBotCombatCharacter *)engine->PvAllocEntPrivateData(sizeofNextBotCombatCharacter);
		(bytes->*void_to_func<void(NextBotCombatCharacter::*)()>(NextBotCombatCharacterCTOR))();
		return bytes;
	}
};

class IPathCost
{
public:
	virtual float operator()( CNavArea *area, CNavArea *fromArea, const CNavLadder *ladder, const CFuncElevator *elevator, float length ) const = 0;
};

//---------------------------------------------------------------------------------------------------------------
/**
 * The interface for selecting a goal area during "open goal" pathfinding
 */
class IPathOpenGoalSelector
{
public:
	// compare "newArea" to "currentGoal" and return the area that is the better goal area
	virtual CNavArea *operator() ( CNavArea *currentGoal, CNavArea *newArea ) const = 0;
};

enum SegmentType
{
	ON_GROUND,
	DROP_DOWN,
	CLIMB_UP,
	JUMP_OVER_GAP,
	PATH_LADDER_UP,
	PATH_LADDER_DOWN,
	
	NUM_SEGMENT_TYPES
};

struct Segment
{
	CNavArea *area;									// the area along the path
	NavTraverseType how;							// how to enter this area from the previous one
	Vector pos;										// our movement goal position at this point in the path
	const CNavLadder *ladder;						// if "how" refers to a ladder, this is it
	
	SegmentType type;								// how to traverse this segment of the path
	Vector forward;									// unit vector along segment
	float length;									// length of this segment
	float distanceFromStart;						// distance of this node from the start of the path
	float curvature;								// how much the path 'curves' at this point in the XY plane (0 = none, 1 = 180 degree doubleback)

	Vector m_portalCenter;							// position of center of 'portal' between previous area and this area
	float m_portalHalfWidth;						// half width of 'portal'
};

class Path
{
public:
	virtual ~Path() = 0;
	
	static Path *create()
	{
		Path *bytes = (Path *)calloc(1, sizeofPath);
		(bytes->*void_to_func<void(Path::*)()>(PathCTOR))();
		return bytes;
	}
	
	using SegmentType = ::SegmentType;

	// @todo Allow custom Segment classes for different kinds of paths	
	using Segment = ::Segment;

	virtual float GetLength( void ) const = 0;						// return length of path from start to finish
	virtual const Vector &GetPosition( float distanceFromStart, const Segment *start = NULL ) const = 0;	// return a position on the path at the given distance from the path start
	virtual const Vector &GetClosestPosition( const Vector &pos, const Segment *start = NULL, float alongLimit = 0.0f ) const = 0;		// return the closest point on the path to the given position

	virtual const Vector &GetStartPosition( void ) const = 0;	// return the position where this path starts
	virtual const Vector &GetEndPosition( void ) const = 0;		// return the position where this path ends
	virtual CBaseCombatCharacter *GetSubject( void ) const = 0;	// return the actor this path leads to, or NULL if there is no subject

	virtual const Path::Segment *GetCurrentGoal( void ) const = 0;	// return current goal along the path we are trying to reach

	virtual float GetAge( void ) const = 0;					// return "age" of this path (time since it was built)

	enum SeekType
	{
		SEEK_ENTIRE_PATH,			// search the entire path length
		SEEK_AHEAD,					// search from current cursor position forward toward end of path
		SEEK_BEHIND					// search from current cursor position backward toward path start
	};
	virtual void MoveCursorToClosestPosition( const Vector &pos, SeekType type = SEEK_ENTIRE_PATH, float alongLimit = 0.0f ) const = 0;		// Set cursor position to closest point on path to given position
	
	enum MoveCursorType
	{
		PATH_ABSOLUTE_DISTANCE,
		PATH_RELATIVE_DISTANCE
	};
	virtual void MoveCursorToStart( void ) = 0;				// set seek cursor to start of path
	virtual void MoveCursorToEnd( void ) = 0;				// set seek cursor to end of path
	virtual void MoveCursor( float value, MoveCursorType type = PATH_ABSOLUTE_DISTANCE ) = 0;	// change seek cursor position
	virtual float GetCursorPosition( void ) const = 0;		// return position of seek cursor (distance along path)

	struct Data
	{
		Vector pos;										// the position along the path
		Vector forward;									// unit vector along path direction
		float curvature;								// how much the path 'curves' at this point in the XY plane (0 = none, 1 = 180 degree doubleback)
		const Segment *segmentPrior;					// the segment just before this position
	};
	virtual const Data &GetCursorData( void ) const = 0;	// return path state at the current cursor position

	virtual bool IsValid( void ) const = 0;
	virtual void Invalidate( void ) = 0;					// make path invalid (clear it)

	virtual void Draw( const Path::Segment *start = NULL ) const = 0;	// draw the path for debugging
	virtual void DrawInterpolated( float from, float to ) = 0;	// draw the path for debugging - MODIFIES cursor position

	virtual const Segment *FirstSegment( void ) const = 0;	// return first segment of path
	virtual const Segment *NextSegment( const Segment *currentSegment ) const = 0;	// return next segment of path, given current one
	virtual const Segment *PriorSegment( const Segment *currentSegment ) const = 0;	// return previous segment of path, given current one
	virtual const Segment *LastSegment( void ) const = 0;	// return last segment of path

	enum ResultType
	{
		COMPLETE_PATH,
		PARTIAL_PATH,
		NO_PATH
	};
	virtual void OnPathChanged( INextBot *bot, ResultType result ) = 0;		// invoked when the path is (re)computed (path is valid at the time of this call)

	virtual void Copy( INextBot *bot, const Path &path ) = 0;	// Replace this path with the given path's data
	
	virtual bool ComputeWithOpenGoal( INextBot *bot, const IPathCost &costFunc, const IPathOpenGoalSelector &goalSelector, float maxSearchRadius = 0.0f ) = 0;
	
	virtual void ComputeAreaCrossing( INextBot *bot, const CNavArea *from, const Vector &fromPos, const CNavArea *to, NavDirType dir, Vector *crossPos ) const = 0;
	
	enum { MAX_PATH_SEGMENTS = 256 };
	Segment m_path[ MAX_PATH_SEGMENTS ];
	int m_segmentCount;
	
	mutable Vector m_pathPos;								// used by GetPosition()
	mutable Vector m_closePos;								// used by GetClosestPosition()

	mutable float m_cursorPos;					// current cursor position (distance along path)
	mutable Data m_cursorData;					// used by GetCursorData()
	mutable bool m_isCursorDataDirty;

	IntervalTimer m_ageTimer;					// how old is this path?
	CHandle< CBaseCombatCharacter > m_subject;	// the subject this path leads to
	
	enum { MAX_ADJ_AREAS = 64 };

	struct AdjInfo
	{
		CNavArea *area;
		CNavLadder *ladder;
		NavTraverseType how;		
	};

	AdjInfo m_adjAreaVector[ MAX_ADJ_AREAS ];
	int m_adjAreaIndex;
	
	template <typename CostFunctor>
	bool Compute(INextBot *bot, CBaseCombatCharacter *subject, CostFunctor &costFunc, float maxPathLength, bool includeGoalIfPathFails)
	{
		Invalidate();
		
		m_subject = subject;
		
		const Vector &start = bot->GetPosition();
		
		CNavArea *startArea = bot->GetEntity()->GetLastKnownArea();
		if ( !startArea )
		{
			OnPathChanged( bot, NO_PATH );
			return false;
		}

		CNavArea *subjectArea = subject->GetLastKnownArea();
		if ( !subjectArea )
		{
			OnPathChanged( bot, NO_PATH );
			return false;
		}

		Vector subjectPos = subject->GetAbsOrigin();
		
		// if we are already in the subject area, build trivial path
		if ( startArea == subjectArea )
		{
			BuildTrivialPath( bot, subjectPos );
			return true;
		}

		//
		// Compute shortest path to subject
		//
		CNavArea *closestArea = NULL;
		bool pathResult = NavAreaBuildPath( startArea, subjectArea, &subjectPos, costFunc, &closestArea, maxPathLength, bot->GetEntity()->GetTeamNumber() );
		
		// Failed?
		if ( closestArea == NULL ) {
			return false;
		}

		//
		// Build actual path by following parent links back from goal area
		//

		// get count
		int count = 0;
		CNavArea *area = nullptr;
		for( area = closestArea; area; area = area->GetParent() )
		{
			++count;
			
			if ( area == startArea )
			{
				// startArea can be re-evaluated during the pathfind and given a parent...
				break;
			}
			if ( count >= MAX_PATH_SEGMENTS-1 ) // save room for endpoint
				break;
		}
		
		if ( count == 1 )
		{
			BuildTrivialPath( bot, subjectPos );
			return pathResult;
		}

		// assemble path
		m_segmentCount = count;
		for( area = closestArea; count && area; area = area->GetParent() )
		{
			--count;
			m_path[ count ].area = area;
			m_path[ count ].how = area->GetParentHow();
			m_path[ count ].type = ON_GROUND;
		}

		if ( pathResult || includeGoalIfPathFails )
		{
			// append actual subject position
			m_path[ m_segmentCount ].area = closestArea;
			m_path[ m_segmentCount ].pos = subjectPos;
			m_path[ m_segmentCount ].ladder = NULL;
			m_path[ m_segmentCount ].how = NUM_TRAVERSE_TYPES;
			m_path[ m_segmentCount ].type = ON_GROUND;
			++m_segmentCount;
		}
				
		// compute path positions
		if ( ComputePathDetails( bot, start ) == false )
		{
			Invalidate();
			OnPathChanged( bot, NO_PATH );
			return false;
		}

		// remove redundant nodes and clean up path
		Optimize( bot );
		
		PostProcess();

		OnPathChanged( bot, pathResult ? COMPLETE_PATH : PARTIAL_PATH );

		return pathResult;
	}
	
	template <typename CostFunctor>
	bool Compute(INextBot *bot, const Vector &goal, CostFunctor &costFunc, float maxPathLength, bool includeGoalIfPathFails)
	{
		Invalidate();
		
		const Vector &start = bot->GetPosition();
		
		CNavArea *startArea = bot->GetEntity()->GetLastKnownArea();
		if ( !startArea )
		{
			OnPathChanged( bot, NO_PATH );
			return false;
		}

		// check line-of-sight to the goal position when finding it's nav area
		const float maxDistanceToArea = 200.0f;
		CNavArea *goalArea = TheNavMesh->GetNearestNavArea( goal, true, maxDistanceToArea, true );

		// if we are already in the goal area, build trivial path
		if ( startArea == goalArea )
		{
			BuildTrivialPath( bot, goal );
			return true;
		}

		// make sure path end position is on the ground
		Vector pathEndPosition = goal;
		if ( goalArea )
		{
			pathEndPosition.z = goalArea->GetZ( pathEndPosition );
		}
		else
		{
			TheNavMesh->GetGroundHeight( pathEndPosition, &pathEndPosition.z );
		}

		//
		// Compute shortest path to goal
		//
		CNavArea *closestArea = NULL;
		bool pathResult = NavAreaBuildPath( startArea, goalArea, &goal, costFunc, &closestArea, maxPathLength, bot->GetEntity()->GetTeamNumber() );

		// Failed?
		if ( closestArea == NULL ) {
			return false;
		}

		//
		// Build actual path by following parent links back from goal area
		//

		// get count
		int count = 0;
		CNavArea *area = nullptr;
		for( area = closestArea; area; area = area->GetParent() )
		{
			++count;

			if ( area == startArea )
			{
				// startArea can be re-evaluated during the pathfind and given a parent...
				break;
			}
			if ( count >= MAX_PATH_SEGMENTS-1 ) // save room for endpoint
				break;
		}
		
		if ( count == 1 )
		{
			BuildTrivialPath( bot, goal );
			return pathResult;
		}

		// assemble path
		m_segmentCount = count;
		for( area = closestArea; count && area; area = area->GetParent() )
		{
			--count;
			m_path[ count ].area = area;
			m_path[ count ].how = area->GetParentHow();
			m_path[ count ].type = ON_GROUND;
		}

		if ( pathResult || includeGoalIfPathFails )
		{
			// append actual goal position
			m_path[ m_segmentCount ].area = closestArea;
			m_path[ m_segmentCount ].pos = pathEndPosition;
			m_path[ m_segmentCount ].ladder = NULL;
			m_path[ m_segmentCount ].how = NUM_TRAVERSE_TYPES;
			m_path[ m_segmentCount ].type = ON_GROUND;
			++m_segmentCount;
		}
				
		// compute path positions
		if ( ComputePathDetails( bot, start ) == false )
		{
			Invalidate();
			OnPathChanged( bot, NO_PATH );
			return false;
		}

		// remove redundant nodes and clean up path
		Optimize( bot );
		
		PostProcess();

		OnPathChanged( bot, pathResult ? COMPLETE_PATH : PARTIAL_PATH );

		return pathResult;
	}
	
	bool ComputePathDetails( INextBot *bot, const Vector &start )
	{
		return (this->*void_to_func<bool(Path::*)(INextBot *, const Vector &)>(PathComputePathDetails))(bot, start);
	}
	
	bool BuildTrivialPath( INextBot *bot, const Vector &goal )
	{
		return (this->*void_to_func<bool(Path::*)(INextBot *, const Vector &)>(PathBuildTrivialPath))(bot, goal);
	}
	
	int FindNextOccludedNode( INextBot *bot, int anchorIndex )
	{
		return (this->*void_to_func<int(Path::*)(INextBot *, int)>(PathFindNextOccludedNode))(bot, anchorIndex);
	}
	
	void Optimize( INextBot *bot )
	{
		if(!path_expensive_optimize.GetBool())
			return;
		
		if (m_segmentCount < 3)
			return;

		int anchor = 0;

		while( anchor < m_segmentCount )
		{
			int occluded = FindNextOccludedNode( bot, anchor );
			int nextAnchor = occluded-1;

			if (nextAnchor > anchor)
			{
				// remove redundant nodes between anchor and nextAnchor
				int removeCount = nextAnchor - anchor - 1;
				if (removeCount > 0)
				{
					for( int i=nextAnchor; i<m_segmentCount; ++i )
					{
						m_path[i-removeCount] = m_path[i];
					}
					m_segmentCount -= removeCount;
				}
			}

			++anchor;
		}
	}

	void PostProcess()
	{
		(this->*void_to_func<void(Path::*)()>(PathPostProcess))();
	}
};

DETOUR_DECL_MEMBER1(PathOptimize, void, INextBot *, bot)
{
	((Path *)this)->Optimize(bot);
}

class PathFollower : public Path
{
public:
	static PathFollower *create()
	{
		PathFollower *bytes = (PathFollower *)calloc(1, sizeofPathFollower);
		(bytes->*void_to_func<void(PathFollower::*)()>(PathFollowerCTOR))();
		return bytes;
	}
	
	virtual void Update( INextBot *bot ) = 0;			// move bot along path
	
	virtual void SetMinLookAheadDistance( float value ) = 0;		// minimum range movement goal must be along path
	
	virtual CBaseEntity *GetHindrance( void ) const = 0;			// returns entity that is hindering our progress along the path

	virtual bool IsDiscontinuityAhead( INextBot *bot, Path::SegmentType type, float range = -1.0f ) const = 0;	// return true if there is a the given discontinuity ahead in the path within the given range (-1 = entire remaining path)
	
	const Path::Segment *m_goal;					// our current goal along the path
	float m_minLookAheadRange;

	//bool IsOnStairs( INextBot *bot ) const;		// return true if bot is standing on a stairway
	bool m_isOnStairs;

	CountdownTimer m_avoidTimer;					// do avoid check more often if we recently avoided

	CountdownTimer m_waitTimer;						// for waiting for a blocker to move off our path
	CHandle< CBaseEntity > m_hindrance;
	
	// debug display data for avoid volumes
	bool m_didAvoidCheck;
	Vector m_leftFrom;
	Vector m_leftTo;
	bool m_isLeftClear;
	Vector m_rightFrom;
	Vector m_rightTo;
	bool m_isRightClear;
	Vector m_hullMin, m_hullMax;

	float m_goalTolerance;
	
	void SetGoalTolerance(float val)
	{
		m_goalTolerance = val;
	}
	
	float GetGoalTolerance()
	{
		return m_goalTolerance;
	}
	
	float GetMinLookAheadDistance()
	{
		return m_minLookAheadRange;
	}
};

class CTFPathFollower : public PathFollower
{
public:
	static CTFPathFollower *create()
	{
		CTFPathFollower *bytes = (CTFPathFollower *)calloc(1, sizeofCTFPathFollower);
		(bytes->*void_to_func<void(CTFPathFollower::*)()>(CTFPathFollowerCTOR))();
		return bytes;
	}
	
	const Path::Segment *m_goal;					// our current goal along the path
	float m_minLookAheadRange;
	
	float GetMinLookAheadDistance()
	{
		return m_minLookAheadRange;
	}
};

class NextBotGroundLocomotion : public ILocomotion
{
public:
	virtual const Vector &GetAcceleration( void ) = 0;	// return current world space acceleration
	virtual void SetAcceleration( const Vector &accel ) = 0;	// set world space acceleration
	virtual void SetVelocity( const Vector &vel ) = 0;		// set world space velocity
	virtual float GetGravity( void ) = 0;					// return gravity force acting on bot
	virtual float GetFrictionForward( void ) = 0;			// return magnitude of forward friction
	virtual float GetFrictionSideways( void ) = 0;		// return magnitude of lateral friction
	virtual float GetMaxYawRate( void ) = 0;				// return max rate of yaw rotation
};

class NextBotGroundLocomotionCustom : public NextBotGroundLocomotion
{
public:
	struct vars_t
	{
		float step = 18.0f;
		float jump = 180.0f;
		float death = 200.0f;
		float run = 150.0f;
		float walk = 75.0f;
		float accel = 500.0f;
		float deaccel = 500.0f;
		float limit = 99999999.9f;
		float slope = 0.6f;
		
		float gravity = 1000.0f;
		float fricforward = 0.0f;
		float fricsideway = 3.0f;
		float yaw = 250.0f;
	};
	
	void dtor();
	
	float HookGetMaxJumpHeight()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().jump); }
	float HookGetStepHeight()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().step); }
	float HookGetDeathDropHeight()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().death); }
	float HookGetRunSpeed()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().run); }
	float HookGetWalkSpeed()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().walk); }
	float HookGetMaxAcceleration()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().accel); }
	float HookGetMaxDeceleration()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().deaccel); }
	float HookGetSpeedLimit()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().limit); }
	float HookGetTraversableSlopeLimit()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().slope); }
	
	float HookGetGravity()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().gravity); }
	float HookGetFrictionForward()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().fricforward); }
	float HookGetFrictionSideways()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().fricsideway); }
	float HookGetMaxYawRate()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().yaw); }
	
	unsigned char *vars_ptr()
	{ return (((unsigned char *)this) + sizeofNextBotGroundLocomotion); }
	vars_t &getvars()
	{ return *(vars_t *)vars_ptr(); }
	
	static NextBotGroundLocomotionCustom *create(INextBot *bot, bool reg);
};

SH_DECL_MANUALHOOK0_void(NextBotGroundLocomotionDtor, 0, 0, 0)

SH_DECL_HOOK0(ILocomotion, GetMaxJumpHeight, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetStepHeight, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetDeathDropHeight, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetRunSpeed, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetWalkSpeed, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetMaxAcceleration, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetMaxDeceleration, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetSpeedLimit, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetTraversableSlopeLimit, SH_NOATTRIB, 0, float);

SH_DECL_HOOK0(NextBotGroundLocomotion, GetGravity, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(NextBotGroundLocomotion, GetFrictionForward, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(NextBotGroundLocomotion, GetFrictionSideways, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(NextBotGroundLocomotion, GetMaxYawRate, SH_NOATTRIB, 0, float);

NextBotGroundLocomotionCustom *NextBotGroundLocomotionCustom::create(INextBot *bot, bool reg)
{
	NextBotGroundLocomotionCustom *bytes = (NextBotGroundLocomotionCustom *)calloc(1, sizeofNextBotGroundLocomotion + sizeof(vars_t));
	(bytes->*void_to_func<void(NextBotGroundLocomotion::*)(INextBot *)>(NextBotGroundLocomotionCTOR))(bot);
	new (bytes->vars_ptr()) vars_t();
	
	SH_ADD_MANUALHOOK(NextBotGroundLocomotionDtor, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::dtor), false);
	
	SH_ADD_HOOK(ILocomotion, GetMaxJumpHeight, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetMaxJumpHeight), false);
	SH_ADD_HOOK(ILocomotion, GetStepHeight, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetStepHeight), false);
	SH_ADD_HOOK(ILocomotion, GetDeathDropHeight, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetDeathDropHeight), false);
	SH_ADD_HOOK(ILocomotion, GetRunSpeed, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetRunSpeed), false);
	SH_ADD_HOOK(ILocomotion, GetWalkSpeed, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetWalkSpeed), false);
	SH_ADD_HOOK(ILocomotion, GetMaxAcceleration, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetMaxAcceleration), false);
	SH_ADD_HOOK(ILocomotion, GetMaxDeceleration, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetMaxDeceleration), false);
	SH_ADD_HOOK(ILocomotion, GetSpeedLimit, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetSpeedLimit), false);
	SH_ADD_HOOK(ILocomotion, GetTraversableSlopeLimit, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetTraversableSlopeLimit), false);
	
	SH_ADD_HOOK(NextBotGroundLocomotion, GetGravity, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetGravity), false);
	SH_ADD_HOOK(NextBotGroundLocomotion, GetFrictionForward, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetFrictionForward), false);
	SH_ADD_HOOK(NextBotGroundLocomotion, GetFrictionSideways, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetFrictionSideways), false);
	SH_ADD_HOOK(NextBotGroundLocomotion, GetMaxYawRate, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetMaxYawRate), false);
	
	if(!reg) {
		bot->m_componentList = bytes->m_nextComponent;
		bytes->m_nextComponent = nullptr;
	}
	
	return bytes;
}

void NextBotGroundLocomotionCustom::dtor()
{
	SH_REMOVE_MANUALHOOK(NextBotGroundLocomotionDtor, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::dtor), false);
	
	SH_REMOVE_HOOK(ILocomotion, GetMaxJumpHeight, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetMaxJumpHeight), false);
	SH_REMOVE_HOOK(ILocomotion, GetStepHeight, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetStepHeight), false);
	SH_REMOVE_HOOK(ILocomotion, GetDeathDropHeight, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetDeathDropHeight), false);
	SH_REMOVE_HOOK(ILocomotion, GetRunSpeed, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetRunSpeed), false);
	SH_REMOVE_HOOK(ILocomotion, GetWalkSpeed, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetWalkSpeed), false);
	SH_REMOVE_HOOK(ILocomotion, GetMaxAcceleration, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetMaxAcceleration), false);
	SH_REMOVE_HOOK(ILocomotion, GetMaxDeceleration, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetMaxDeceleration), false);
	SH_REMOVE_HOOK(ILocomotion, GetSpeedLimit, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetSpeedLimit), false);
	SH_REMOVE_HOOK(ILocomotion, GetTraversableSlopeLimit, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetTraversableSlopeLimit), false);
	
	SH_REMOVE_HOOK(NextBotGroundLocomotion, GetGravity, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetGravity), false);
	SH_REMOVE_HOOK(NextBotGroundLocomotion, GetFrictionForward, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetFrictionForward), false);
	SH_REMOVE_HOOK(NextBotGroundLocomotion, GetFrictionSideways, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetFrictionSideways), false);
	SH_REMOVE_HOOK(NextBotGroundLocomotion, GetMaxYawRate, this, SH_MEMBER(this, &NextBotGroundLocomotionCustom::HookGetMaxYawRate), false);
	
	getvars().~vars_t();
	
	RETURN_META(MRES_IGNORED);
}

HandleType_t PathHandleType = 0;
HandleType_t PathFollowerHandleType = 0;
HandleType_t CTFPathFollowerHandleType = 0;

cell_t PathCTORNative(IPluginContext *pContext, const cell_t *params)
{
	Path *obj = Path::create();
	return handlesys->CreateHandle(PathHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
}

cell_t PathFollowerCTORNative(IPluginContext *pContext, const cell_t *params)
{
	PathFollower *obj = PathFollower::create();
	return handlesys->CreateHandle(PathFollowerHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
}

class SPPathCost : public IPathCost
{
public:
	SPPathCost(INextBot *entity_, IPluginFunction *callback_, cell_t data_)
		: entity(entity_), callback(callback_), data(data_)
	{
		
	}
	
	float operator()( CNavArea *area, CNavArea *fromArea, const CNavLadder *ladder, const CFuncElevator *elevator, float length ) const
	{
		cell_t res;
		callback->PushCell((cell_t)entity);
		callback->PushCell((cell_t)area);
		callback->PushCell((cell_t)fromArea);
		callback->PushCell((cell_t)ladder);
		callback->PushCell((cell_t)elevator);
		callback->PushCell(sp_ftoc(length));
		callback->PushCell(data);
		callback->Execute(&res);
		return sp_ctof(res);
	}
	
	IPluginFunction *callback;
	INextBot *entity;
	cell_t data;
};

cell_t PathComputeVectorNative(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	cell_t *value = nullptr;
	pContext->LocalToPhysAddr(params[3], &value);
	
	Vector goal = Vector(sp_ctof(value[0]), sp_ctof(value[1]), sp_ctof(value[2]));
	
	INextBot *bot = (INextBot *)params[2];
	
	IPluginFunction *callback = pContext->GetFunctionById(params[4]);
	SPPathCost cost(bot, callback, params[5]);
	
	float maxPathLength = sp_ctof(params[6]);
	
	bool includeGoalIfPathFails = params[7];
	
	return obj->Compute(bot, goal, cost, maxPathLength, includeGoalIfPathFails);
}

cell_t PathComputeEntityNative(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[3]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[3]);
	}
	
	CBaseCombatCharacter *pCombat = pSubject->MyCombatCharacterPointer();
	if(!pCombat)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[3]);
	}
	
	INextBot *bot = (INextBot *)params[2];
	
	IPluginFunction *callback = pContext->GetFunctionById(params[4]);
	SPPathCost cost(bot, callback, params[5]);
	
	float maxPathLength = sp_ctof(params[6]);
	
	bool includeGoalIfPathFails = params[7];
	
	return obj->Compute(bot, pCombat, cost, maxPathLength, includeGoalIfPathFails);
}

cell_t PathFollowerUpdateNative(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	PathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	INextBot *bot = (INextBot *)params[2];
	
	obj->Update(bot);
	
	return 0;
}

cell_t PathMemoryget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return (cell_t)obj;
}

cell_t CNavAreaCostSoFarget(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return sp_ftoc(area->GetCostSoFar());
}

cell_t CNavAreaIDget(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->GetID();
}

cell_t CNavAreaGetCenter(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	const Vector &vec = area->GetCenter();
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	return 0;
}

cell_t CNavAreaComputeAdjacentConnectionHeightChange(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	CNavArea *other = (CNavArea *)params[2];
	return sp_ftoc(area->ComputeAdjacentConnectionHeightChange(other));
}

cell_t CNavAreaHasAttributes(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->HasAttributes((NavAttributeType)params[2]);
}

cell_t CNavAreaComputeFuncNavCost(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	CBaseCombatCharacter *pCombat = pSubject->MyCombatCharacterPointer();
	if(!pCombat)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return sp_ftoc(area->ComputeFuncNavCost(pCombat));
}

cell_t CNavAreaGetPlayerCount(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->GetPlayerCount(params[2]);
}

cell_t CTFNavAreaInCombatget(IPluginContext *pContext, const cell_t *params)
{
	CTFNavArea *area = (CTFNavArea *)params[1];
	return area->IsInCombat();
}

cell_t CTFNavAreaCombatIntensityget(IPluginContext *pContext, const cell_t *params)
{
	CTFNavArea *area = (CTFNavArea *)params[1];
	return sp_ftoc(area->GetCombatIntensity());
}

cell_t CTFNavAreaHasAttributeTF(IPluginContext *pContext, const cell_t *params)
{
	CTFNavArea *area = (CTFNavArea *)params[1];
	return area->HasAttributeTF((int)params[2]);
}

cell_t CNavLadderLengthget(IPluginContext *pContext, const cell_t *params)
{
	CNavLadder *area = (CNavLadder *)params[1];
	return sp_ftoc(area->m_length);
}

cell_t CNavMeshGetNearestNavAreaVector(IPluginContext *pContext, const cell_t *params)
{
	CNavMesh *area = (CNavMesh *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	return (cell_t)area->GetNearestNavArea(pos, params[3], sp_ctof(params[4]), params[5], params[6], params[7]);
}

cell_t CNavMeshGetGroundHeightNative(IPluginContext *pContext, const cell_t *params)
{
	CNavMesh *area = (CNavMesh *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	cell_t *heightaddr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	
	Vector normal{};
	
	cell_t *pNullVec = pContext->GetNullRef(SP_NULL_VECTOR);
	
	float height = 0.0f;
	bool ret = area->GetGroundHeight(pos, &height, &normal);
	
	addr = nullptr;
	pContext->LocalToPhysAddr(params[4], &addr);
	
	if(addr != pNullVec) {
		addr[0] = sp_ftoc(normal.x);
		addr[1] = sp_ftoc(normal.y);
		addr[2] = sp_ftoc(normal.z);
	}
	
	*heightaddr = sp_ftoc(height);
	
	return ret;
}

cell_t ILocomotionIsAreaTraversable(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	CNavArea *other = (CNavArea *)params[2];
	return area->IsAreaTraversable(other);
}

cell_t ILocomotionStepHeightget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetStepHeight());
}

cell_t ILocomotionMaxJumpHeightget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetMaxJumpHeight());
}

cell_t ILocomotionDeathDropHeightget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetDeathDropHeight());
}

cell_t INextBotget(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[1]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	INextBot *bot = pEntity->MyNextBotPointer();
	
	return (cell_t)bot;
}

cell_t INextBotLocomotionInterfaceget(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	return (cell_t)bot->GetLocomotionInterface();
}

cell_t INextBotVisionInterfaceget(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	return (cell_t)bot->GetVisionInterface();
}

cell_t INextBotBodyInterfaceget(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	return (cell_t)bot->GetBodyInterface();
}

cell_t CTFPathFollowerCTORNative(IPluginContext *pContext, const cell_t *params)
{
	CTFPathFollower *obj = CTFPathFollower::create();
	return handlesys->CreateHandle(CTFPathFollowerHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
}

cell_t INextBotEntityget(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	return gamehelpers->EntityToBCompatRef(bot->GetEntity());
}

cell_t INextBotAllocateCustomLocomotion(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	NextBotGroundLocomotionCustom *locomotion = NextBotGroundLocomotionCustom::create(bot, false);
	
	if(bot->m_baseLocomotion) {
		bot->ReplaceComponent(bot->m_baseLocomotion, locomotion);
		delete bot->m_baseLocomotion;
	} else {
		bot->RegisterComponent(locomotion);
	}
	
	locomotion->Reset();
	
	bot->m_baseLocomotion = locomotion;

	return (cell_t)locomotion;
}

cell_t INextBotAllocateCustomBody(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	IBodyCustom *locomotion = new IBodyCustom(bot, false);
	
	if(bot->m_baseBody) {
		bot->ReplaceComponent(bot->m_baseBody, locomotion);
		delete bot->m_baseBody;
	} else {
		bot->RegisterComponent(locomotion);
	}
	
	locomotion->Reset();
	
	bot->m_baseBody = locomotion;

	return (cell_t)locomotion;
}

cell_t PathLengthget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return sp_ftoc(obj->GetLength());
}

cell_t PathAgeget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return sp_ftoc(obj->GetAge());
}

cell_t PathIsValid(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return obj->IsValid();
}

cell_t PathFirstSegmentget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return (cell_t)obj->FirstSegment();
}

cell_t PathLastSegmentget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return (cell_t)obj->LastSegment();
}

cell_t PathNextSegment(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	Segment *seg = (Segment *)params[2];
	
	return (cell_t)obj->NextSegment(seg);
}

cell_t PathPriorSegment(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	Segment *seg = (Segment *)params[2];
	
	return (cell_t)obj->PriorSegment(seg);
}

cell_t PathGetPosition(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	float distanceFromStart = sp_ctof(params[3]);
	
	Segment *start = (Segment *)params[4];
	
	const Vector &vec = obj->GetPosition(distanceFromStart, start);
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	
	return 0;
}

cell_t PathGetClosestPosition(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	float alongLimit = sp_ctof(params[5]);
	
	Segment *start = (Segment *)params[4];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	const Vector &vec = obj->GetClosestPosition(pos, start, alongLimit);
	
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	
	return 0;
}

cell_t PathGetStartPosition(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	const Vector &vec = obj->GetStartPosition();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	
	return 0;
}

cell_t PathGetEndPosition(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	const Vector &vec = obj->GetEndPosition();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	
	return 0;
}

cell_t PathSubjectget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	CBaseCombatCharacter *subject = obj->GetSubject();
	if(!subject) {
		return -1;
	}
	
	return gamehelpers->EntityToBCompatRef(subject);
}

cell_t PathCurrentGoalget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return (cell_t)obj->GetCurrentGoal();
}

cell_t PathInvalidate(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	obj->Invalidate();
	
	return 0;
}

cell_t PathFollowerMinLookAheadDistanceget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	PathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return sp_ftoc(obj->GetMinLookAheadDistance());
}

cell_t PathFollowerMinLookAheadDistanceset(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	PathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	obj->SetMinLookAheadDistance(sp_ctof(params[2]));
	
	return 0;
}

cell_t PathFollowerGoalToleranceget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	PathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return sp_ftoc(obj->GetGoalTolerance());
}

cell_t PathFollowerGoalToleranceset(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	PathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	obj->SetGoalTolerance(sp_ctof(params[2]));
	
	return 0;
}

cell_t PathFollowerIsDiscontinuityAhead(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	PathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	INextBot *bot = (INextBot *)params[2];
	
	return obj->IsDiscontinuityAhead(bot, (SegmentType)params[2], sp_ctof(params[3]));
}

cell_t CTFPathFollowerMinLookAheadDistanceget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	CTFPathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], CTFPathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return sp_ftoc(obj->GetMinLookAheadDistance());
}

cell_t SegmentAreaget(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	return (cell_t)seg->area;
}

cell_t SegmentLadderget(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	return (cell_t)seg->ladder;
}

cell_t SegmentTypeget(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	return seg->type;
}

cell_t SegmentLengthget(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	return sp_ftoc(seg->length);
}

cell_t SegmentDistanceFromStartget(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	return sp_ftoc(seg->distanceFromStart);
}

cell_t SegmentCurvatureget(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	return sp_ftoc(seg->curvature);
}

cell_t SegmentPortalHalfWidthget(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	return sp_ftoc(seg->m_portalHalfWidth);
}

cell_t SegmentHowget(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	return seg->how;
}

cell_t SegmentGetPosition(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	const Vector &vec = seg->pos;
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	
	return 0;
}

cell_t SegmentGetPortalCenter(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	const Vector &vec = seg->m_portalCenter;
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	
	return 0;
}

cell_t SegmentGetForward(IPluginContext *pContext, const cell_t *params)
{
	Segment *seg = (Segment *)params[1];
	
	const Vector &vec = seg->forward;
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	
	return 0;
}

cell_t ILocomotionRunSpeedget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetRunSpeed());
}

cell_t ILocomotionWalkSpeedget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetWalkSpeed());
}

cell_t IBodyHullWidthget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return sp_ftoc(area->GetHullWidth());
}

cell_t IBodyHullHeightget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return sp_ftoc(area->GetHullHeight());
}

cell_t IBodyStandHullHeightget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return sp_ftoc(area->GetStandHullHeight());
}

cell_t IBodyCrouchHullHeightget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return sp_ftoc(area->GetCrouchHullHeight());
}

cell_t IBodySolidMaskget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return sp_ftoc(area->GetSolidMask());
}

cell_t IBodyCollisionGroupget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return sp_ftoc(area->GetCollisionGroup());
}

cell_t ILocomotionMaxAccelerationget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetMaxAcceleration());
}

cell_t ILocomotionMaxDecelerationget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetMaxDeceleration());
}

cell_t ILocomotionSpeedLimitget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetSpeedLimit());
}

cell_t ILocomotionTraversableSlopeLimitget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetTraversableSlopeLimit());
}

cell_t ILocomotionDesiredSpeedget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetDesiredSpeed());
}

cell_t ILocomotionGroundSpeedget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetGroundSpeed());
}

cell_t ILocomotionDesiredSpeedset(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	area->SetDesiredSpeed(sp_ctof(params[2]));
	return 0;
}

cell_t ILocomotionIsClimbingOrJumping(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsClimbingOrJumping();
}

cell_t ILocomotionIsClimbingUpToLedge(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsClimbingUpToLedge();
}

cell_t ILocomotionIsJumpingAcrossGap(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsJumpingAcrossGap();
}

cell_t ILocomotionIsScrambling(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsScrambling();
}

cell_t ILocomotionIsRunning(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsRunning();
}

cell_t ILocomotionIsStuck(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsStuck();
}

cell_t ILocomotionSetDesiredLean(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	QAngle ang(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	area->SetDesiredLean(ang);
	
	return 0;
}

cell_t IBodyCustomSetHullMins(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *area = (IBodyCustom *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector ang(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	area->SetHullMins(std::move(ang));
	
	return 0;
}

cell_t IBodyCustomSetHullMaxs(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *area = (IBodyCustom *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector ang(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	area->SetHullMaxs(std::move(ang));
	
	return 0;
}

cell_t ILocomotionGetDesiredLean(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	const QAngle &ang = area->GetDesiredLean();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(ang.x);
	addr[1] = sp_ftoc(ang.y);
	addr[2] = sp_ftoc(ang.z);
	
	return 0;
}

cell_t ILocomotionGetGroundMotionVector(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	const Vector &ang = area->GetGroundMotionVector();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(ang.x);
	addr[1] = sp_ftoc(ang.y);
	addr[2] = sp_ftoc(ang.z);
	
	return 0;
}

cell_t ILocomotionGetVelocity(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	const Vector &ang = area->GetVelocity();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(ang.x);
	addr[1] = sp_ftoc(ang.y);
	addr[2] = sp_ftoc(ang.z);
	
	return 0;
}

cell_t IBodyGetHullMins(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];

	const Vector &ang = area->GetHullMins();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(ang.x);
	addr[1] = sp_ftoc(ang.y);
	addr[2] = sp_ftoc(ang.z);
	
	return 0;
}

cell_t IBodyGetHullMaxs(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];

	const Vector &ang = area->GetHullMaxs();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(ang.x);
	addr[1] = sp_ftoc(ang.y);
	addr[2] = sp_ftoc(ang.z);
	
	return 0;
}

cell_t ILocomotionRun(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	area->Run();
	return 0;
}

cell_t ILocomotionWalk(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	area->Walk();
	return 0;
}

cell_t ILocomotionStop(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	area->Stop();
	return 0;
}

cell_t ILocomotionJump(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	area->Jump();
	return 0;
}

cell_t ILocomotionJumpAcrossGap(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector goal(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	pContext->LocalToPhysAddr(params[3], &addr);
	Vector fwrd(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	area->JumpAcrossGap(goal, fwrd);
	return 0;
}

cell_t ILocomotionClimbUpToLedge(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector goal(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	pContext->LocalToPhysAddr(params[3], &addr);
	Vector fwrd(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[4]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[4]);
	}
	
	return area->ClimbUpToLedge(goal, fwrd, pEntity);
}

cell_t ILocomotionFaceTowards(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	area->FaceTowards(vec);
	
	return 0;
}

cell_t ILocomotionApproach(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	area->Approach(vec, sp_ctof(params[3]));
	
	return 0;
}

cell_t ILocomotionDriveTo(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	area->DriveTo(vec);
	
	return 0;
}

cell_t NextBotGroundLocomotionGravityget(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotion *area = (NextBotGroundLocomotion *)params[1];
	return sp_ftoc(area->GetGravity());
}

cell_t NextBotGroundLocomotionFrictionForwardget(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotion *area = (NextBotGroundLocomotion *)params[1];
	return sp_ftoc(area->GetFrictionForward());
}

cell_t NextBotGroundLocomotionFrictionSidewaysget(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotion *area = (NextBotGroundLocomotion *)params[1];
	return sp_ftoc(area->GetFrictionSideways());
}

cell_t NextBotGroundLocomotionMaxYawRateget(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *area = (NextBotGroundLocomotionCustom *)params[1];
	return sp_ftoc(area->GetMaxYawRate());
}

cell_t NextBotGroundLocomotionCustomRunSpeedset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().run = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomWalkSpeedset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().walk = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomMaxAccelerationset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().accel = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomMaxDecelerationset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().deaccel = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomSpeedLimitset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().limit = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomTraversableSlopeLimitset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().slope = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomGravityset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().gravity = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomFrictionForwardset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().fricforward = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomFrictionSidewaysset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().fricsideway = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomMaxYawRateset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().yaw = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomStepHeightset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().step = sp_ctof(params[2]);
	return 0;
}

cell_t IBodyCustomHullWidthset(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *locomotion = (IBodyCustom *)params[1];
	locomotion->SetHullWidth(sp_ctof(params[2]));
	return 0;
}

cell_t IBodyCustomHullHeightset(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *locomotion = (IBodyCustom *)params[1];
	locomotion->SetHullHeight(sp_ctof(params[2]));
	return 0;
}

cell_t IBodyCustomStandHullHeightset(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *locomotion = (IBodyCustom *)params[1];
	locomotion->StandHullHeight = sp_ctof(params[2]);
	return 0;
}

cell_t IBodyCustomCrouchHullHeightset(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *locomotion = (IBodyCustom *)params[1];
	locomotion->CrouchHullHeight = sp_ctof(params[2]);
	return 0;
}

cell_t IBodyCustomSolidMaskset(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *locomotion = (IBodyCustom *)params[1];
	locomotion->SolidMask = params[2];
	return 0;
}

cell_t IBodyCustomCollisionGroupset(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *locomotion = (IBodyCustom *)params[1];
	locomotion->CollisionGroup = params[2];
	return 0;
}

cell_t NextBotGroundLocomotionCustomMaxJumpHeightset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().jump = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotGroundLocomotionCustomDeathDropHeightset(IPluginContext *pContext, const cell_t *params)
{
	NextBotGroundLocomotionCustom *locomotion = (NextBotGroundLocomotionCustom *)params[1];
	locomotion->getvars().death = sp_ctof(params[2]);
	return 0;
}

class SPForEachKnownEntity : public IForEachKnownEntity
{
public:
	SPForEachKnownEntity(IPluginFunction *callbacl_, cell_t data_)
		: callback(callbacl_), data(data_)
	{}
	
	virtual bool Inspect( const CKnownEntity &known )
	{
		cell_t res = 0;
		callback->PushCell((cell_t)&known);
		callback->PushCell(data);
		callback->Execute(&res);
		return res;
	}
	
	IPluginFunction *callback;
	cell_t data;
};

cell_t IVisionForEachKnownEntity(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	IPluginFunction *callback = pContext->GetFunctionById(params[2]);
	SPForEachKnownEntity each(callback, params[3]);
	return locomotion->ForEachKnownEntity(each);
}

cell_t IVisionGetPrimaryKnownThreat(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return (cell_t)locomotion->GetPrimaryKnownThreat(params[2]);
}

cell_t IVisionGetTimeSinceVisible(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return sp_ftoc(locomotion->GetTimeSinceVisible(params[2]));
}

cell_t IVisionGetClosestKnownTeam(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return (cell_t)locomotion->GetClosestKnown(params[2]);
}

class SPGetClosestKnown : public INextBotEntityFilter
{
public:
	SPGetClosestKnown(IPluginFunction *callbacl_, cell_t data_)
		: callback(callbacl_), data(data_)
	{}
	
	virtual bool IsAllowed( CBaseEntity *known )
	{
		cell_t res = 0;
		callback->PushCell(gamehelpers->EntityToBCompatRef(known));
		callback->PushCell(data);
		callback->Execute(&res);
		return res;
	}
	
	IPluginFunction *callback;
	cell_t data;
};

cell_t IVisionGetClosestKnownFilter(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	IPluginFunction *callback = pContext->GetFunctionById(params[2]);
	SPGetClosestKnown each(callback, params[3]);
	return (cell_t)locomotion->GetClosestKnown(each);
}

cell_t IVisionGetKnownCount(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return locomotion->GetKnownCount(params[2], params[3], sp_ctof(params[4]));
}

cell_t IVisionGetKnown(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return (cell_t)locomotion->GetKnown(pSubject);
}

cell_t IVisionAddKnownEntity(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	locomotion->AddKnownEntity(pSubject);
	return 0;
}

cell_t IVisionForgetEntity(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	locomotion->ForgetEntity(pSubject);
	return 0;
}

cell_t IVisionForgetAllKnownEntities(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	locomotion->ForgetAllKnownEntities();
	return 0;
}

cell_t IVisionIsAbleToSeeEntity(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	Vector vec{};
	bool ret = locomotion->IsAbleToSee(pSubject, (FieldOfViewCheckType)params[3], &vec);
	
	cell_t *pNullVec = pContext->GetNullRef(SP_NULL_VECTOR);
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[4], &addr);
	
	if(addr != pNullVec) {
		addr[0] = sp_ftoc(vec.x);
		addr[1] = sp_ftoc(vec.y);
		addr[2] = sp_ftoc(vec.z);
	}
	
	return ret;
}

cell_t IVisionIsAbleToSeeVector(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	return locomotion->IsAbleToSee(vec, (FieldOfViewCheckType)params[3]);
}

cell_t IVisionIsIgnored(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return locomotion->IsIgnored(pSubject);
}

cell_t IVisionIsVisibleEntityNoticed(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return locomotion->IsVisibleEntityNoticed(pSubject);
}

cell_t IVisionIsInFieldOfViewVector(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	return locomotion->IsInFieldOfView(vec);
}

cell_t IVisionIsInFieldOfViewEntity(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return locomotion->IsInFieldOfView(pSubject);
}

cell_t IVisionIsLineOfSightClear(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	return locomotion->IsLineOfSightClear(vec);
}

cell_t IVisionIsLineOfSightClearToEntity(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	Vector vec{};
	bool ret = locomotion->IsLineOfSightClearToEntity(pSubject, &vec);
	
	cell_t *pNullVec = pContext->GetNullRef(SP_NULL_VECTOR);
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	
	if(addr != pNullVec) {
		addr[0] = sp_ftoc(vec.x);
		addr[1] = sp_ftoc(vec.y);
		addr[2] = sp_ftoc(vec.z);
	}
	
	return ret;
}

cell_t IVisionIsLookingAtVector(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	return locomotion->IsLookingAt(vec, sp_ctof(params[3]));
}

cell_t IVisionIsLookingAtEntity(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	CBaseCombatCharacter *pCombat = pSubject->MyCombatCharacterPointer();
	if(!pCombat)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return locomotion->IsLookingAt(pCombat, sp_ctof(params[3]));
}

cell_t IVisionDefaultFieldOfViewget(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return sp_ftoc(locomotion->GetDefaultFieldOfView());
}

cell_t IVisionFieldOfViewget(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return sp_ftoc(locomotion->GetFieldOfView());
}

cell_t IVisionMaxVisionRangeget(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return sp_ftoc(locomotion->GetMaxVisionRange());
}

cell_t IVisionMinRecognizeTimeget(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return sp_ftoc(locomotion->GetMinRecognizeTime());
}

cell_t IVisionFieldOfViewset(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	locomotion->SetFieldOfView(sp_ctof(params[2]));
	return 0;
}

cell_t CKnownEntityEntityget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	
	CBaseEntity *pEntity = locomotion->GetEntity();
	if(!pEntity) {
		return -1;
	}
	
	return gamehelpers->EntityToBCompatRef(pEntity);
}

cell_t CKnownEntityLastKnownPositionBeenSeenget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return locomotion->HasLastKnownPositionBeenSeen();
}

cell_t CKnownEntityMarkLastKnownPositionAsSeen(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	locomotion->MarkLastKnownPositionAsSeen();
	return 0;
}

cell_t CKnownEntityLastKnownAreaget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return (cell_t)locomotion->GetLastKnownArea();
}

cell_t CKnownEntityTimeSinceLastKnownget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return sp_ftoc(locomotion->GetTimeSinceLastKnown());
}

cell_t CKnownEntityTimeSinceBecameKnownget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return sp_ftoc(locomotion->GetTimeSinceBecameKnown());
}

cell_t CKnownEntityTimeSinceBecameVisibleget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return sp_ftoc(locomotion->GetTimeSinceBecameVisible());
}

cell_t CKnownEntityTimeWhenBecameVisibleget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return sp_ftoc(locomotion->GetTimeWhenBecameVisible());
}

cell_t CKnownEntityTimeSinceLastSeenget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return sp_ftoc(locomotion->GetTimeSinceLastSeen());
}

cell_t CKnownEntityWasEverVisibleget(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return sp_ftoc(locomotion->WasEverVisible());
}

cell_t CKnownEntityVisibilityStatusset(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	locomotion->UpdateVisibilityStatus(params[2]);
	return 0;
}

cell_t CKnownEntityIsVisibleInFOVNow(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return locomotion->IsVisibleInFOVNow();
}

cell_t CKnownEntityIsVisibleRecently(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return locomotion->IsVisibleRecently();
}

cell_t CKnownEntityIsObsolete(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	return locomotion->IsObsolete();
}

cell_t CKnownEntityIs(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return locomotion->Is(pSubject);
}

cell_t CKnownEntityIsEqual(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	CKnownEntity *known = (CKnownEntity *)params[2];
	
	return locomotion->operator==(*known);
}

cell_t CKnownEntityDestroy(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	locomotion->Destroy();
	return 0;
}

cell_t CKnownEntityUpdatePosition(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	locomotion->UpdatePosition();
	return 0;
}

cell_t CKnownEntityGetLastKnownPosition(IPluginContext *pContext, const cell_t *params)
{
	CKnownEntity *locomotion = (CKnownEntity *)params[1];
	const Vector &vec = locomotion->GetLastKnownPosition();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	addr[0] = sp_ftoc(vec.x);
	addr[1] = sp_ftoc(vec.y);
	addr[2] = sp_ftoc(vec.z);
	
	return 0;
}

cell_t AllocateNextBotCombatCharacter(IPluginContext *pContext, const cell_t *params)
{
	return (cell_t)NextBotCombatCharacter::create();
}

cell_t EntityIsCombatCharacter(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	CBaseCombatCharacter *pCombat = pSubject->MyCombatCharacterPointer();
	
	return (pCombat != nullptr);
}

cell_t GetEntityLastKnownArea(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	CBaseCombatCharacter *pCombat = pSubject->MyCombatCharacterPointer();
	if(!pCombat)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	return (cell_t)pCombat->GetLastKnownArea();
}

cell_t UpdateEntityLastKnownArea(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	CBaseCombatCharacter *pCombat = pSubject->MyCombatCharacterPointer();
	if(!pCombat)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	pCombat->UpdateLastKnownArea();
	return 0;
}

sp_nativeinfo_t natives[] =
{
	{"Path.Path", PathCTORNative},
	{"Path.ComputeVector", PathComputeVectorNative},
	{"Path.ComputeEntity", PathComputeEntityNative},
	{"Path.Memory.get", PathMemoryget},
	{"Path.Length.get", PathLengthget},
	{"Path.Age.get", PathAgeget},
	{"Path.IsValid", PathIsValid},
	{"Path.FirstSegment.get", PathFirstSegmentget},
	{"Path.LastSegment.get", PathLastSegmentget},
	{"Path.NextSegment", PathNextSegment},
	{"Path.PriorSegment", PathPriorSegment},
	{"Path.GetPosition", PathGetPosition},
	{"Path.GetClosestPosition", PathGetClosestPosition},
	{"Path.GetStartPosition", PathGetStartPosition},
	{"Path.GetEndPosition", PathGetEndPosition},
	{"Path.Subject.get", PathSubjectget},
	{"Path.CurrentGoal.get", PathCurrentGoalget},
	{"Path.Invalidate", PathInvalidate},
	{"Segment.Area.get", SegmentAreaget},
	{"Segment.Ladder.get", SegmentLadderget},
	{"Segment.Type.get", SegmentTypeget},
	{"Segment.Length.get", SegmentLengthget},
	{"Segment.DistanceFromStart.get", SegmentDistanceFromStartget},
	{"Segment.Curvature.get", SegmentCurvatureget},
	{"Segment.PortalHalfWidth.get", SegmentPortalHalfWidthget},
	{"Segment.How.get", SegmentHowget},
	{"Segment.GetPosition", SegmentGetPosition},
	{"Segment.GetPortalCenter", SegmentGetPortalCenter},
	{"Segment.GetForward", SegmentGetForward},
	{"PathFollower.PathFollower", PathFollowerCTORNative},
	{"PathFollower.Update", PathFollowerUpdateNative},
	{"PathFollower.MinLookAheadDistance.get", PathFollowerMinLookAheadDistanceget},
	{"PathFollower.MinLookAheadDistance.set", PathFollowerMinLookAheadDistanceset},
	{"PathFollower.GoalTolerance.get", PathFollowerGoalToleranceget},
	{"PathFollower.GoalTolerance.set", PathFollowerGoalToleranceset},
	{"PathFollower.IsDiscontinuityAhead", PathFollowerIsDiscontinuityAhead},
	{"CTFPathFollower.MinLookAheadDistance.get", CTFPathFollowerMinLookAheadDistanceget},
	{"CNavArea.CostSoFar.get", CNavAreaCostSoFarget},
	{"CNavArea.ID.get", CNavAreaIDget},
	{"CNavArea.GetCenter", CNavAreaGetCenter},
	{"CNavArea.ComputeAdjacentConnectionHeightChange", CNavAreaComputeAdjacentConnectionHeightChange},
	{"CNavArea.HasAttributes", CNavAreaHasAttributes},
	{"CNavArea.ComputeFuncNavCost", CNavAreaComputeFuncNavCost},
	{"CNavArea.GetPlayerCount", CNavAreaGetPlayerCount},
	{"CTFNavArea.InCombat.get", CTFNavAreaInCombatget},
	{"CTFNavArea.CombatIntensity.get", CTFNavAreaCombatIntensityget},
	{"CTFNavArea.HasAttributeTF", CTFNavAreaHasAttributeTF},
	{"CNavMesh.GetNearestNavAreaVector", CNavMeshGetNearestNavAreaVector},
	{"CNavMesh.GetGroundHeight", CNavMeshGetGroundHeightNative},
	{"CNavLadder.Length.get", CNavLadderLengthget},
	{"ILocomotion.StepHeight.get", ILocomotionStepHeightget},
	{"ILocomotion.MaxJumpHeight.get", ILocomotionMaxJumpHeightget},
	{"ILocomotion.DeathDropHeight.get", ILocomotionDeathDropHeightget},
	{"ILocomotion.RunSpeed.get", ILocomotionRunSpeedget},
	{"ILocomotion.WalkSpeed.get", ILocomotionWalkSpeedget},
	{"ILocomotion.MaxAcceleration.get", ILocomotionMaxAccelerationget},
	{"ILocomotion.MaxDeceleration.get", ILocomotionMaxDecelerationget},
	{"ILocomotion.SpeedLimit.get", ILocomotionSpeedLimitget},
	{"ILocomotion.TraversableSlopeLimit.get", ILocomotionTraversableSlopeLimitget},
	{"ILocomotion.DesiredSpeed.get", ILocomotionDesiredSpeedget},
	{"ILocomotion.DesiredSpeed.set", ILocomotionDesiredSpeedset},
	{"ILocomotion.IsAreaTraversable", ILocomotionIsAreaTraversable},
	{"ILocomotion.IsClimbingOrJumping", ILocomotionIsClimbingOrJumping},
	{"ILocomotion.IsClimbingUpToLedge", ILocomotionIsClimbingUpToLedge},
	{"ILocomotion.IsJumpingAcrossGap", ILocomotionIsJumpingAcrossGap},
	{"ILocomotion.IsScrambling", ILocomotionIsScrambling},
	{"ILocomotion.IsRunning", ILocomotionIsRunning},
	{"ILocomotion.IsStuck", ILocomotionIsStuck},
	{"ILocomotion.SetDesiredLean", ILocomotionSetDesiredLean},
	{"ILocomotion.GetDesiredLean", ILocomotionGetDesiredLean},
	{"ILocomotion.Run", ILocomotionRun},
	{"ILocomotion.Walk", ILocomotionWalk},
	{"ILocomotion.Stop", ILocomotionStop},
	{"ILocomotion.Jump", ILocomotionJump},
	{"ILocomotion.JumpAcrossGap", ILocomotionJumpAcrossGap},
	{"ILocomotion.ClimbUpToLedge", ILocomotionClimbUpToLedge},
	{"ILocomotion.FaceTowards", ILocomotionFaceTowards},
	{"ILocomotion.Approach", ILocomotionApproach},
	{"ILocomotion.DriveTo", ILocomotionDriveTo},
	{"ILocomotion.GetDesiredLean", ILocomotionGetDesiredLean},
	{"ILocomotion.GroundSpeed.get", ILocomotionGroundSpeedget},
	{"ILocomotion.GetGroundMotionVector", ILocomotionGetGroundMotionVector},
	{"ILocomotion.GetVelocity", ILocomotionGetVelocity},
	{"INextBot.INextBot", INextBotget},
	{"INextBot.LocomotionInterface.get", INextBotLocomotionInterfaceget},
	{"INextBot.VisionInterface.get", INextBotVisionInterfaceget},
	{"INextBot.BodyInterface.get", INextBotBodyInterfaceget},
	{"INextBot.AllocateCustomLocomotion", INextBotAllocateCustomLocomotion},
	{"INextBot.AllocateCustomBody", INextBotAllocateCustomBody},
	{"INextBot.Entity.get", INextBotEntityget},
	{"CTFPathFollower.CTFPathFollower", CTFPathFollowerCTORNative},
	{"NextBotGroundLocomotion.Gravity.get", NextBotGroundLocomotionGravityget},
	{"NextBotGroundLocomotion.FrictionForward.get", NextBotGroundLocomotionFrictionForwardget},
	{"NextBotGroundLocomotion.FrictionSideways.get", NextBotGroundLocomotionFrictionSidewaysget},
	{"NextBotGroundLocomotion.MaxYawRate.get", NextBotGroundLocomotionMaxYawRateget},
	{"NextBotGoundLocomotionCustom.StepHeight.set", NextBotGroundLocomotionCustomStepHeightset},
	{"NextBotGoundLocomotionCustom.MaxJumpHeight.set", NextBotGroundLocomotionCustomMaxJumpHeightset},
	{"NextBotGoundLocomotionCustom.DeathDropHeight.set", NextBotGroundLocomotionCustomDeathDropHeightset},
	{"NextBotGoundLocomotionCustom.RunSpeed.set", NextBotGroundLocomotionCustomRunSpeedset},
	{"NextBotGoundLocomotionCustom.WalkSpeed.set", NextBotGroundLocomotionCustomWalkSpeedset},
	{"NextBotGoundLocomotionCustom.MaxAcceleration.set", NextBotGroundLocomotionCustomMaxAccelerationset},
	{"NextBotGoundLocomotionCustom.MaxDeceleration.set", NextBotGroundLocomotionCustomMaxDecelerationset},
	{"NextBotGoundLocomotionCustom.SpeedLimit.set", NextBotGroundLocomotionCustomSpeedLimitset},
	{"NextBotGoundLocomotionCustom.TraversableSlopeLimit.set", NextBotGroundLocomotionCustomTraversableSlopeLimitset},
	{"NextBotGoundLocomotionCustom.Gravity.set", NextBotGroundLocomotionCustomGravityset},
	{"NextBotGoundLocomotionCustom.FrictionForward.set", NextBotGroundLocomotionCustomFrictionForwardset},
	{"NextBotGoundLocomotionCustom.FrictionSideways.set", NextBotGroundLocomotionCustomFrictionSidewaysset},
	{"NextBotGoundLocomotionCustom.MaxYawRate.set", NextBotGroundLocomotionCustomMaxYawRateset},
	{"IVision.ForEachKnownEntity", IVisionForEachKnownEntity},
	//{"IVision.CollectKnownEntities", IVisionCollectKnownEntities},
	{"IVision.GetPrimaryKnownThreat", IVisionGetPrimaryKnownThreat},
	{"IVision.GetTimeSinceVisible", IVisionGetTimeSinceVisible},
	{"IVision.GetClosestKnownTeam", IVisionGetClosestKnownTeam},
	{"IVision.GetClosestKnownFilter", IVisionGetClosestKnownFilter},
	{"IVision.GetKnownCount", IVisionGetKnownCount},
	{"IVision.GetKnown", IVisionGetKnown},
	{"IVision.AddKnownEntity", IVisionAddKnownEntity},
	{"IVision.ForgetEntity", IVisionForgetEntity},
	{"IVision.ForgetAllKnownEntities", IVisionForgetAllKnownEntities},
	//{"IVision.CollectPotentiallyVisibleEntities", IVisionCollectPotentiallyVisibleEntities},
	{"IVision.IsAbleToSeeEntity", IVisionIsAbleToSeeEntity},
	{"IVision.IsAbleToSeeVector", IVisionIsAbleToSeeVector},
	{"IVision.IsIgnored", IVisionIsIgnored},
	{"IVision.IsVisibleEntityNoticed", IVisionIsVisibleEntityNoticed},
	{"IVision.IsInFieldOfViewVector", IVisionIsInFieldOfViewVector},
	{"IVision.IsInFieldOfViewEntity", IVisionIsInFieldOfViewEntity},
	{"IVision.IsLineOfSightClear", IVisionIsLineOfSightClear},
	{"IVision.IsLineOfSightClearToEntity", IVisionIsLineOfSightClearToEntity},
	{"IVision.IsLookingAtVector", IVisionIsLookingAtVector},
	{"IVision.IsLookingAtEntity", IVisionIsLookingAtEntity},
	{"IVision.DefaultFieldOfView.get", IVisionDefaultFieldOfViewget},
	{"IVision.FieldOfView.get", IVisionFieldOfViewget},
	{"IVision.MaxVisionRange.get", IVisionMaxVisionRangeget},
	{"IVision.MinRecognizeTime.get", IVisionMinRecognizeTimeget},
	{"IVision.FieldOfView.set", IVisionFieldOfViewset},
	{"IBody.HullWidth.get", IBodyHullWidthget},
	{"IBody.HullHeight.get", IBodyHullHeightget},
	{"IBody.StandHullHeight.get", IBodyStandHullHeightget},
	{"IBody.CrouchHullHeight.get", IBodyCrouchHullHeightget},
	{"IBody.SolidMask.get", IBodySolidMaskget},
	{"IBody.CollisionGroup.get", IBodyCollisionGroupget},
	{"IBody.GetHullMins", IBodyGetHullMins},
	{"IBody.GetHullMaxs", IBodyGetHullMaxs},
	{"IBodyCustom.HullWidth.set", IBodyCustomHullWidthset},
	{"IBodyCustom.HullHeight.set", IBodyCustomHullHeightset},
	{"IBodyCustom.StandHullHeight.set", IBodyCustomStandHullHeightset},
	{"IBodyCustom.CrouchHullHeight.set", IBodyCustomCrouchHullHeightset},
	{"IBodyCustom.SolidMask.set", IBodyCustomSolidMaskset},
	{"IBodyCustom.CollisionGroup.set", IBodyCustomCollisionGroupset},
	{"IBodyCustom.SetHullMins", IBodyCustomSetHullMins},
	{"IBodyCustom.SetHullMaxs", IBodyCustomSetHullMaxs},
	{"CKnownEntity.Entity.get", CKnownEntityEntityget},
	{"CKnownEntity.LastKnownPositionBeenSeen.get", CKnownEntityLastKnownPositionBeenSeenget},
	{"CKnownEntity.LastKnownArea.get", CKnownEntityLastKnownAreaget},
	{"CKnownEntity.TimeSinceLastKnown.get", CKnownEntityTimeSinceLastKnownget},
	{"CKnownEntity.TimeSinceBecameKnown.get", CKnownEntityTimeSinceBecameKnownget},
	{"CKnownEntity.TimeSinceBecameVisible.get", CKnownEntityTimeSinceBecameVisibleget},
	{"CKnownEntity.TimeWhenBecameVisible.get", CKnownEntityTimeWhenBecameVisibleget},
	{"CKnownEntity.TimeSinceLastSeen.get", CKnownEntityTimeSinceLastSeenget},
	{"CKnownEntity.WasEverVisible.get", CKnownEntityWasEverVisibleget},
	{"CKnownEntity.VisibilityStatus.get", CKnownEntityVisibilityStatusset},
	{"CKnownEntity.MarkLastKnownPositionAsSeen", CKnownEntityMarkLastKnownPositionAsSeen},
	{"CKnownEntity.IsVisibleInFOVNow", CKnownEntityIsVisibleInFOVNow},
	{"CKnownEntity.IsVisibleRecently", CKnownEntityIsVisibleRecently},
	{"CKnownEntity.IsObsolete", CKnownEntityIsObsolete},
	{"CKnownEntity.Is", CKnownEntityIs},
	{"CKnownEntity.IsEqual", CKnownEntityIsEqual},
	{"CKnownEntity.Destroy", CKnownEntityDestroy},
	{"CKnownEntity.UpdatePosition", CKnownEntityUpdatePosition},
	{"CKnownEntity.GetLastKnownPosition", CKnownEntityGetLastKnownPosition},
	{"AllocateNextBotCombatCharacter", AllocateNextBotCombatCharacter},
	{"EntityIsCombatCharacter", EntityIsCombatCharacter},
	{"GetEntityLastKnownArea", GetEntityLastKnownArea},
	{"UpdateEntityLastKnownArea", UpdateEntityLastKnownArea},
	{NULL, NULL}
};

bool Sample::SDK_OnMetamodLoad(ISmmAPI *ismm, char *error, size_t maxlen, bool late)
{
	gpGlobals = ismm->GetCGlobals();
	GET_V_IFACE_ANY(GetEngineFactory, engine, IVEngineServer, INTERFACEVERSION_VENGINESERVER)
	GET_V_IFACE_CURRENT(GetEngineFactory, icvar, ICvar, CVAR_INTERFACE_VERSION);
	g_pCVar = icvar;
	ConVar_Register(0, this);
	tf_nav_combat_decay_rate = g_pCVar->FindVar("tf_nav_combat_decay_rate");
	return true;
}

void Sample::OnHandleDestroy(HandleType_t type, void *object)
{
	if(type == PathHandleType) {
		Path *obj = (Path *)object;
		delete obj;
	} else if(type == PathFollowerHandleType) {
		PathFollower *obj = (PathFollower *)object;
		delete obj;
	} else if(type == CTFPathFollowerHandleType) {
		CTFPathFollower *obj = (CTFPathFollower *)object;
		delete obj;
	}
}

bool Sample::RegisterConCommandBase(ConCommandBase *pCommand)
{
	META_REGCVAR(pCommand);
	return true;
}

SH_DECL_HOOK0(CNavMesh, IsAuthoritative, const, 0, bool);

bool HookIsAuthoritative()
{
	RETURN_META_VALUE(MRES_SUPERCEDE, nav_authorative.GetBool());
}

IGameConfig *g_pGameConf = nullptr;

CDetour *pPathOptimize = nullptr;

bool Sample::SDK_OnLoad(char *error, size_t maxlen, bool late)
{
	gameconfs->LoadGameConfigFile("nextbot", &g_pGameConf, error, maxlen);
	
	g_pGameConf->GetOffset("sizeof(Path)", &sizeofPath);
	g_pGameConf->GetOffset("sizeof(PathFollower)", &sizeofPathFollower);
	g_pGameConf->GetOffset("sizeof(NextBotCombatCharacter)", &sizeofNextBotCombatCharacter);
	g_pGameConf->GetOffset("sizeof(CTFPathFollower)", &sizeofCTFPathFollower);
	g_pGameConf->GetOffset("sizeof(NextBotGroundLocomotion)", &sizeofNextBotGroundLocomotion);
	
	g_pGameConf->GetMemSig("Path::Path", &PathCTOR);
	g_pGameConf->GetMemSig("PathFollower::PathFollower", &PathFollowerCTOR);
	g_pGameConf->GetMemSig("NextBotCombatCharacter::NextBotCombatCharacter", &NextBotCombatCharacterCTOR);
	g_pGameConf->GetMemSig("CTFPathFollower::CTFPathFollower", &CTFPathFollowerCTOR);
	g_pGameConf->GetMemSig("NextBotGroundLocomotion::NextBotGroundLocomotion", &NextBotGroundLocomotionCTOR);
	
	g_pGameConf->GetMemSig("Path::ComputePathDetails", &PathComputePathDetails);
	g_pGameConf->GetMemSig("Path::BuildTrivialPath", &PathBuildTrivialPath);
	g_pGameConf->GetMemSig("Path::FindNextOccludedNode", &PathFindNextOccludedNode);
	g_pGameConf->GetMemSig("Path::PostProcess", &PathPostProcess);
	g_pGameConf->GetMemSig("CNavMesh::GetGroundHeight", &CNavMeshGetGroundHeight);
	g_pGameConf->GetMemSig("CNavMesh::GetNearestNavArea", &CNavMeshGetNearestNavArea);
	g_pGameConf->GetMemSig("CBaseEntity::SetAbsOrigin", &CBaseEntitySetAbsOrigin);
	
	g_pGameConf->GetMemSig("NavAreaBuildPath", &NavAreaBuildPathPtr);
	
	g_pGameConf->GetOffset("CBaseEntity::MyNextBotPointer", &CBaseEntityMyNextBotPointer);
	g_pGameConf->GetOffset("CBaseEntity::MyCombatCharacterPointer", &CBaseEntityMyCombatCharacterPointer);
	g_pGameConf->GetOffset("CBaseEntity::WorldSpaceCenter", &CBaseEntityWorldSpaceCenter);
	g_pGameConf->GetOffset("CBaseEntity::EyeAngles", &CBaseEntityEyeAngles);
	g_pGameConf->GetOffset("CBaseCombatCharacter::GetLastKnownArea", &CBaseCombatCharacterGetLastKnownArea);
	g_pGameConf->GetOffset("CBaseCombatCharacter::UpdateLastKnownArea", &CBaseCombatCharacterUpdateLastKnownArea);
	g_pGameConf->GetOffset("CFuncNavCost::GetCostMultiplier", &CFuncNavCostGetCostMultiplier);
	
	g_pGameConf->GetMemSig("TheNavMesh", (void **)&TheNavMesh);
	
	SH_ADD_HOOK(CNavMesh, IsAuthoritative, TheNavMesh, SH_STATIC(HookIsAuthoritative), false);
	
	CDetourManager::Init(g_pSM->GetScriptingEngine(), g_pGameConf);
	
	pPathOptimize = DETOUR_CREATE_MEMBER(PathOptimize, "Path::Optimize")
	pPathOptimize->EnableDetour();
	
	sm_sendprop_info_t info{};
	gamehelpers->FindSendPropInfo("CBaseEntity", "m_iTeamNum", &info);
	m_iTeamNumOffset = info.actual_offset;
	
	g_pEntityList = reinterpret_cast<CBaseEntityList *>(gamehelpers->GetGlobalEntityList());
	
	PathHandleType = handlesys->CreateType("Path", this, 0, nullptr, nullptr, myself->GetIdentity(), nullptr);
	PathFollowerHandleType = handlesys->CreateType("PathFollower", this, PathHandleType, nullptr, nullptr, myself->GetIdentity(), nullptr);
	CTFPathFollowerHandleType = handlesys->CreateType("CTFPathFollower", this, PathFollowerHandleType, nullptr, nullptr, myself->GetIdentity(), nullptr);
	
	plsys->AddPluginsListener(this);
	
	sharesys->RegisterLibrary(myself, "nextbot");

	return true;
}

void Sample::OnPluginLoaded(IPlugin *plugin)
{
	IPluginRuntime *runtime = plugin->GetRuntime();
	
	uint32_t idx = (uint32_t)-1;
	if(runtime->FindPubvarByName("TheNavMesh", &idx) == SP_ERROR_NONE) {
		sp_pubvar_t *TheNavMeshVar = nullptr;
		runtime->GetPubvarByIndex(idx, &TheNavMeshVar);
		*TheNavMeshVar->offs == (cell_t)TheNavMesh;
	}
}

void Sample::OnPluginUnloaded(IPlugin *plugin)
{
	
}

void Sample::SDK_OnAllLoaded()
{
	sharesys->AddNatives(myself, natives);
}

bool Sample::QueryRunning(char *error, size_t maxlength)
{
	return true;
}

bool Sample::QueryInterfaceDrop(SMInterface *pInterface)
{
	return IExtensionInterface::QueryInterfaceDrop(pInterface);
}

void Sample::NotifyInterfaceDrop(SMInterface *pInterface)
{
	
}

void Sample::SDK_OnUnload()
{
	pPathOptimize->Destroy();
	plsys->RemovePluginsListener(this);
	handlesys->RemoveType(PathHandleType, myself->GetIdentity());
	handlesys->RemoveType(PathFollowerHandleType, myself->GetIdentity());
	handlesys->RemoveType(CTFPathFollowerHandleType, myself->GetIdentity());
	gameconfs->CloseGameConfigFile(g_pGameConf);
}
