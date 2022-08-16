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

#include <string>
#include <vector>
#include <unordered_map>
#include <string_view>

using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

#if SOURCE_ENGINE == SE_TF2
	#define TF_DLL
	#define USES_ECON_ITEMS
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	#define TERROR
	#define LEFT4DEAD
#endif

#define BASEENTITY_H
#define NEXT_BOT
#define GLOWS_ENABLE
#define USE_NAV_MESH
#define RAD_TELEMETRY_DISABLED

#if SOURCE_ENGINE == SE_LEFT4DEAD2
#include <public/tier1/utlmemory.h>
#include <public/tier1/utlvector.h>
#endif

class IGameEventManager2 *gameeventmanager = nullptr;

#include "extension.h"
#include <ISDKTools.h>
#include <mathlib/vmatrix.h>
#include <toolframework/itoolentity.h>
#include <ehandle.h>
#include <GameEventListener.h>
#include <eiface.h>
#include <dt_common.h>

class IEngineTrace *enginetrace = nullptr;
class IStaticPropMgrServer *staticpropmgr = nullptr;

#include <public/networkvar.h>
#include <shared/shareddefs.h>
#include <shared/util_shared.h>

#ifndef FMTFUNCTION
#define FMTFUNCTION(...)
#endif

#include <util.h>
#include <variant_t.h>
#include <shared/predictioncopy.h>
#include <tier1/utldict.h>
#include <tier1/utlvector.h>
#include <CDetour/detours.h>

#include <takedamageinfo.h>

#ifdef __HAS_DAMAGERULES
#include <IDamageRules.h>
#include <public/damageinfo.cpp>
#endif

#ifdef __HAS_ANIMHELPERS
#include <IAnimHelpers.h>
#endif

#include <toolframework/itoolentity.h>

/**
 * @file extension.cpp
 * @brief Implement extension code here.
 */

Sample g_Sample{};		/**< Global singleton for extension's main interface */

SMEXT_LINK(&g_Sample);

ICvar *icvar = nullptr;
CGlobalVars *gpGlobals = nullptr;
CBaseEntityList *g_pEntityList = nullptr;
IServerTools *servertools = nullptr;
ISDKTools *g_pSDKTools = nullptr;

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

#if SOURCE_ENGINE == SE_TF2
ConVar nav_authorative("nav_authorative", "1");
#endif

ConVar path_expensive_optimize("path_expensive_optimize", "0");

#if SOURCE_ENGINE == SE_TF2
ConVar *tf_nav_combat_decay_rate = nullptr;
#endif

int sizeofNextBotCombatCharacter = 0;
#if SOURCE_ENGINE == SE_LEFT4DEAD2
int sizeofInfected = 0;
#endif

void *PathCTOR = nullptr;
void *PathFollowerCTOR = nullptr;
void *NextBotCombatCharacterCTOR = nullptr;
void *INextBotCTOR = nullptr;
#if SOURCE_ENGINE == SE_TF2
void *CTFPathFollowerCTOR = nullptr;
#endif
#if SOURCE_ENGINE == SE_TF2
void *NextBotGroundLocomotionCTOR = nullptr;
void *ILocomotionCTOR = nullptr;
void *IVisionCTOR = nullptr;
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
void *ZombieBotLocomotionCTOR = nullptr;
void *ZombieBotVisionCTOR = nullptr;
#endif

void *PathComputePathDetails = nullptr;
void *PathBuildTrivialPath = nullptr;
void *PathFindNextOccludedNode = nullptr;
void *PathPostProcess = nullptr;
void *CNavMeshGetGroundHeight = nullptr;
void *CNavMeshGetNearestNavArea = nullptr;
void *CBaseEntitySetAbsOrigin = nullptr;
void *INextBotBeginUpdatePtr = nullptr;
void *INextBotEndUpdatePtr = nullptr;
void *CBaseEntityCalcAbsoluteVelocity = nullptr;
#if SOURCE_ENGINE == SE_TF2
void *NextBotGroundLocomotionResolveCollision = nullptr;
#endif
void *CBaseEntitySetGroundEntity = nullptr;
void *CTraceFilterSimpleShouldHitEntity = nullptr;
#if SOURCE_ENGINE == SE_LEFT4DEAD2
void *PathComputeVector = nullptr;
void *PathComputeEntity = nullptr;
void *INextBotIsDebuggingPtr = nullptr;
void *InfectedChasePathComputeAreaCrossing = nullptr;
void *EntityFactoryDictionaryPtr = nullptr;
#endif
void *CBaseCombatCharacterEvent_Killed = nullptr;
void *CBaseEntityCalcAbsolutePosition = nullptr;
void *CBaseCombatCharacterAllocateDefaultRelationships = nullptr;
void *CBaseCombatCharacterSetDefaultRelationship = nullptr;

int CBaseCombatCharacterIsHiddenByFogVec = -1;
int CBaseCombatCharacterIsHiddenByFogEnt = -1;
int CBaseCombatCharacterIsHiddenByFogR = -1;

int CBaseCombatCharacterGetFogObscuredRatioVec = -1;
int CBaseCombatCharacterGetFogObscuredRatioEnt = -1;
int CBaseCombatCharacterGetFogObscuredRatioR = -1;

int CBaseCombatCharacterIsLookingTowardsEnt = -1;
int CBaseCombatCharacterIsLookingTowardsVec = -1;

int CBaseCombatCharacterIsInFieldOfViewEnt = -1;
int CBaseCombatCharacterIsInFieldOfViewVec = -1;

int CBaseCombatCharacterIsLineOfSightClearEnt = -1;
int CBaseCombatCharacterIsLineOfSightClearVec = -1;

int CBaseEntityFVisibleVec = -1;
int CBaseEntityFVisibleEnt = -1;

int CBaseCombatCharacterFInViewConeVec = -1;
int CBaseCombatCharacterFInViewConeEnt = -1;

int CBaseCombatCharacterFInAimConeVec = -1;
int CBaseCombatCharacterFInAimConeEnt = -1;

void *CBaseCombatCharacterIsAbleToSeeEnt = nullptr;
void *CBaseCombatCharacterIsAbleToSeeCC = nullptr;

void *NavAreaBuildPathPtr = nullptr;

int CBaseEntityMyNextBotPointer = -1;
int CBaseEntityMyCombatCharacterPointer = -1;
int CBaseEntityClassify = -1;
#if SOURCE_ENGINE == SE_TF2
int CBaseEntityIsBaseObject = -1;
#endif
int CBaseEntityIsNPC = -1;
#if SOURCE_ENGINE == SE_LEFT4DEAD2
int CBaseEntityMyInfectedPointer = -1;
#endif
int CBaseCombatCharacterGetLastKnownArea = -1;
int CBaseCombatCharacterUpdateLastKnownArea = -1;
int CBaseEntityWorldSpaceCenter = -1;
int CBaseEntityEyeAngles = -1;
#if SOURCE_ENGINE == SE_TF2
int CFuncNavCostGetCostMultiplier = -1;
#endif
#if SOURCE_ENGINE == SE_LEFT4DEAD2
int NextBotCombatCharacterGetNextBotCombatCharacter = -1;
int CBaseCombatCharacterCanBeA = -1;
#endif
int CBaseEntityEyePosition = -1;
int CBaseEntitySpawn = -1;

int m_vecAbsOriginOffset = -1;
int m_iTeamNumOffset = -1;
int m_vecAbsVelocityOffset = -1;
int m_vecVelocityOffset = -1;
int m_iEFlagsOffset = -1;
int m_fFlagsOffset = -1;
int m_hGroundEntityOffset = -1;
int m_MoveTypeOffset = -1;
#if SOURCE_ENGINE == SE_LEFT4DEAD2
int m_isGhostOffset = -1;
#endif
int m_rgflCoordinateFrameOffset = -1;
int m_hMoveParentOffset = -1;
int m_hMoveChildOffset = -1;
int m_hMovePeerOffset = -1;
int m_iParentAttachmentOffset = -1;
int m_iClassnameOffset = -1;
int m_flSimulationTimeOffset = -1;
int m_angAbsRotationOffset = -1;
int m_angRotationOffset = -1;
int m_bloodColorOffset = -1;
int m_CollisionGroupOffset = -1;

int m_bSequenceFinishedOffset = -1;
int m_flPlaybackRateOffset = -1;
int m_nSequenceOffset = -1;
int m_flAnimTimeOffset = -1;
int m_flCycleOffset = -1;
int m_flGroundSpeedOffset = -1;

ConVar *NextBotStop = nullptr;

template <typename T>
T void_to_func(void *ptr)
{
	union { T f; void *p; };
	p = ptr;
	return f;
}

template <typename R, typename T>
R func_to_func(T ptr)
{
	union { T f; R p; };
	f = ptr;
	return p;
}

template <typename T>
void *func_to_void(T ptr)
{
	union { T f; void *p; };
	f = ptr;
	return p;
}

template <typename T>
int vfunc_index(T func)
{
	SourceHook::MemFuncInfo info{};
	SourceHook::GetFuncInfo<T>(func, info);
	return info.vtblindex;
}

template <typename R, typename T, typename ...Args>
R call_mfunc(T *pThisPtr, void *offset, Args ...args)
{
	class VEmptyClass {};
	
	void **this_ptr = *reinterpret_cast<void ***>(&pThisPtr);
	
	union
	{
		R (VEmptyClass::*mfpnew)(Args...);
#ifndef PLATFORM_POSIX
		void *addr;
	} u;
	u.addr = offset;
#else
		struct  
		{
			void *addr;
			intptr_t adjustor;
		} s;
	} u;
	u.s.addr = offset;
	u.s.adjustor = 0;
#endif
	
	return (R)(reinterpret_cast<VEmptyClass *>(this_ptr)->*u.mfpnew)(args...);
}

template <typename R, typename T, typename ...Args>
R call_mfunc(const T *pThisPtr, void *offset, Args ...args)
{
	return call_mfunc<R, T, Args...>(const_cast<T *>(pThisPtr), offset, args...);
}

template <typename R, typename T, typename ...Args>
R call_vfunc(T *pThisPtr, size_t offset, Args ...args)
{
	void **vtable = *reinterpret_cast<void ***>(pThisPtr);
	void *vfunc = vtable[offset];
	
	return call_mfunc<R, T, Args...>(pThisPtr, vfunc, args...);
}

template <typename R, typename T, typename ...Args>
R call_vfunc(const T *pThisPtr, size_t offset, Args ...args)
{
	return call_vfunc<R, T, Args...>(const_cast<T *>(pThisPtr), offset, args...);
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
#include <ServerNetworkProperty.h>

void SetEdictStateChanged(CBaseEntity *pEntity, int offset);

class CBasePlayer;

enum InvalidatePhysicsBits_t
{
	POSITION_CHANGED	= 0x1,
	ANGLES_CHANGED		= 0x2,
	VELOCITY_CHANGED	= 0x4,
	ANIMATION_CHANGED	= 0x8,
};

float k_flMaxEntityEulerAngle = 360.0 * 1000.0f;

inline bool IsEntityQAngleReasonable( const QAngle &q )
{
	float r = k_flMaxEntityEulerAngle;
	return
		q.x > -r && q.x < r &&
		q.y > -r && q.y < r &&
		q.z > -r && q.z < r;
}

float UTIL_VecToYaw( const Vector &vec )
{
	if (vec.y == 0 && vec.x == 0)
		return 0;
	
	float yaw = atan2( vec.y, vec.x );

	yaw = RAD2DEG(yaw);

	if (yaw < 0)
		yaw += 360;

	return yaw;
}

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

	NUM_AI_CLASSES
};
#else
	#error
#endif

class CBaseEntity : public IServerEntity
{
public:
	DECLARE_CLASS_NOBASE( CBaseEntity );
	DECLARE_SERVERCLASS();
	DECLARE_DATADESC();
	
	int entindex()
	{
		return gamehelpers->EntityToBCompatRef(this);
	}
	
	INextBot *MyNextBotPointer()
	{
		return call_vfunc<INextBot *>(this, CBaseEntityMyNextBotPointer);
	}
	
	CBaseCombatCharacter *MyCombatCharacterPointer()
	{
		return call_vfunc<CBaseCombatCharacter *>(this, CBaseEntityMyCombatCharacterPointer);
	}

	Class_T Classify()
	{
		return call_vfunc<Class_T>(this, CBaseEntityClassify);
	}

#if SOURCE_ENGINE == SE_TF2
	bool IsBaseObject()
	{
		return call_vfunc<bool>(this, CBaseEntityIsBaseObject);
	}
#endif

	bool IsNPC()
	{
		return call_vfunc<bool>(this, CBaseEntityIsNPC);
	}

	Vector EyePosition()
	{
		return call_vfunc<Vector>(this, CBaseEntityEyePosition);
	}

	CBasePlayer *IsPlayer()
	{
		int idx = gamehelpers->EntityToBCompatRef(this);
		if(idx >= 1 && idx <= playerhelpers->GetNumPlayers()) {
			return (CBasePlayer *)this;
		} else {
			return nullptr;
		}
	}
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	CBaseCombatCharacter *MyInfectedPointer()
	{
		return call_vfunc<CBaseCombatCharacter *>(this, CBaseEntityMyInfectedPointer);
	}
#endif
	
	const Vector &GetAbsOrigin()
	{
		if(m_vecAbsOriginOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_vecAbsOrigin", &info);
			m_vecAbsOriginOffset = info.actual_offset;
		}
		
		if(GetIEFlags() & EFL_DIRTY_ABSTRANSFORM) {
			CalcAbsolutePosition();
		}
		
		return *(Vector *)(((unsigned char *)this) + m_vecAbsOriginOffset);
	}
	
	matrix3x4_t &EntityToWorldTransform()
	{
		if(m_rgflCoordinateFrameOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_rgflCoordinateFrame", &info);
			m_rgflCoordinateFrameOffset = info.actual_offset;
		}
		
		if(GetIEFlags() & EFL_DIRTY_ABSTRANSFORM) {
			CalcAbsolutePosition();
		}
		
		return *(matrix3x4_t *)((unsigned char *)this + m_rgflCoordinateFrameOffset);
	}
	
	void CalcAbsolutePosition()
	{
		call_mfunc<void, CBaseEntity>(this, CBaseEntityCalcAbsolutePosition);
	}
	
	int GetTeamNumber()
	{
		return *(int *)(((unsigned char *)this) + m_iTeamNumOffset);
	}
	
	const Vector &WorldSpaceCenter( ) const
	{
		return call_vfunc<const Vector &>(this, CBaseEntityWorldSpaceCenter);
	}
	
	const QAngle &EyeAngles( void )
	{
		return call_vfunc<const QAngle &>(this, CBaseEntityEyeAngles);
	}
	
	void SetAbsOrigin( const Vector& origin )
	{
		call_mfunc<void, CBaseEntity, const Vector &>(this, CBaseEntitySetAbsOrigin, origin);
	}

	void SetSimulationTime(float time)
	{
		if(m_flSimulationTimeOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_flSimulationTime", &info);
			m_flSimulationTimeOffset = info.actual_offset;
		}

		*(float *)((unsigned char *)this + m_flSimulationTimeOffset) = time;
		SetEdictStateChanged(this, m_flSimulationTimeOffset);
	}

	const QAngle& GetAbsAngles( void )
	{
		if(m_angAbsRotationOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_angAbsRotation", &info);
			m_angAbsRotationOffset = info.actual_offset;
		}

		if (GetIEFlags() & EFL_DIRTY_ABSTRANSFORM)
		{
			CalcAbsolutePosition();
		}

		return *(QAngle *)((unsigned char *)this + m_angAbsRotationOffset);
	}
	
	const QAngle& GetLocalAngles( void )
	{
		if(m_angRotationOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_angRotation", &info);
			m_angRotationOffset = info.actual_offset;
		}

		return *(QAngle *)((unsigned char *)this + m_angRotationOffset);
	}

	void SetLocalAngles( const QAngle& angles )
	{
		if(m_angRotationOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_angRotation", &info);
			m_angRotationOffset = info.actual_offset;
		}

		// NOTE: The angle normalize is a little expensive, but we can save
		// a bunch of time in interpolation if we don't have to invalidate everything
		// and sometimes it's off by a normalization amount

		// FIXME: The normalize caused problems in server code like momentary_rot_button that isn't
		//        handling things like +/-180 degrees properly. This should be revisited.
		//QAngle angleNormalize( AngleNormalize( angles.x ), AngleNormalize( angles.y ), AngleNormalize( angles.z ) );

		// Safety check against NaN's or really huge numbers
		if ( !IsEntityQAngleReasonable( angles ) )
		{
			return;
		}

		if (*(QAngle *)((unsigned char *)this + m_angRotationOffset) != angles)
		{
			InvalidatePhysicsRecursive( ANGLES_CHANGED );
			*(QAngle *)((unsigned char *)this + m_angRotationOffset) = angles;
			SetEdictStateChanged(this, m_angRotationOffset);
			SetSimulationTime( gpGlobals->curtime );
		}
	}

	void SetAbsAngles( const QAngle& absAngles )
	{
		if(m_angAbsRotationOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_angAbsRotation", &info);
			m_angAbsRotationOffset = info.actual_offset;
		}

		if(m_angRotationOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_angRotation", &info);
			m_angRotationOffset = info.actual_offset;
		}

		if(m_rgflCoordinateFrameOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_rgflCoordinateFrame", &info);
			m_rgflCoordinateFrameOffset = info.actual_offset;
		}

		if(m_vecAbsOriginOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_vecAbsOrigin", &info);
			m_vecAbsOriginOffset = info.actual_offset;
		}

		// This is necessary to get the other fields of m_rgflCoordinateFrame ok
		CalcAbsolutePosition();

		// FIXME: The normalize caused problems in server code like momentary_rot_button that isn't
		//        handling things like +/-180 degrees properly. This should be revisited.
		//QAngle angleNormalize( AngleNormalize( absAngles.x ), AngleNormalize( absAngles.y ), AngleNormalize( absAngles.z ) );

		if ( *(QAngle *)((unsigned char *)this + m_angAbsRotationOffset) == absAngles )
			return;

		// All children are invalid, but we are not
		InvalidatePhysicsRecursive( ANGLES_CHANGED );
		RemoveIEFlags( EFL_DIRTY_ABSTRANSFORM );

		*(QAngle *)((unsigned char *)this + m_angAbsRotationOffset) = absAngles;
		SetEdictStateChanged(this, m_angAbsRotationOffset);
		AngleMatrix( absAngles, *(matrix3x4_t *)((unsigned char *)this + m_rgflCoordinateFrameOffset) );
		MatrixSetColumn( *(Vector *)((unsigned char *)this + m_vecAbsOriginOffset), 3, *(matrix3x4_t *)((unsigned char *)this + m_rgflCoordinateFrameOffset) ); 

		QAngle angNewRotation;
		CBaseEntity *pMoveParent = GetMoveParent();
		if (!pMoveParent)
		{
			angNewRotation = absAngles;
		}
		else
		{
			if ( *(QAngle *)((unsigned char *)this + m_angAbsRotationOffset) == pMoveParent->GetAbsAngles() )
			{
				angNewRotation.Init( );
			}
			else
			{
				// Moveparent case: transform the abs transform into local space
				matrix3x4_t worldToParent, localMatrix;
				MatrixInvert( pMoveParent->EntityToWorldTransform(), worldToParent );
				ConcatTransforms( worldToParent, *(matrix3x4_t *)((unsigned char *)this + m_rgflCoordinateFrameOffset), localMatrix );
				MatrixAngles( localMatrix, angNewRotation );
			}
		}

		if (*(QAngle *)((unsigned char *)this + m_angRotationOffset) != angNewRotation)
		{
			*(QAngle *)((unsigned char *)this + m_angRotationOffset) = angNewRotation;
			SetEdictStateChanged(this, m_angRotationOffset);
			SetSimulationTime( gpGlobals->curtime );
		}
	}

	int GetIEFlags()
	{
		if(m_iEFlagsOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_iEFlags", &info);
			m_iEFlagsOffset = info.actual_offset;
		}
		
		return *(int *)((unsigned char *)this + m_iEFlagsOffset);
	}

	void DispatchUpdateTransmitState()
	{

	}

	bool ClassMatches( const char *pszClassOrWildcard )
	{
		if(m_iClassnameOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_iClassname", &info);
			m_iClassnameOffset = info.actual_offset;
		}

		if ( IDENT_STRINGS( *(string_t *)((unsigned char *)this + m_iClassnameOffset), pszClassOrWildcard ) )
			return true;

		return false;
	}

	bool ClassMatches( string_t nameStr )
	{
		if(m_iClassnameOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_iClassname", &info);
			m_iClassnameOffset = info.actual_offset;
		}

		if ( IDENT_STRINGS( *(string_t *)((unsigned char *)this + m_iClassnameOffset), nameStr ) )
			return true;

		return false;
	}

	const char* GetClassname()
	{
		if(m_iClassnameOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_iClassname", &info);
			m_iClassnameOffset = info.actual_offset;
		}

		return STRING( *(string_t *)((unsigned char *)this + m_iClassnameOffset) );
	}

	void AddIEFlags(int flags)
	{
		if(m_iEFlagsOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_iEFlags", &info);
			m_iEFlagsOffset = info.actual_offset;
		}

		*(int *)((unsigned char *)this + m_iEFlagsOffset) |= flags;

		if ( flags & ( EFL_FORCE_CHECK_TRANSMIT | EFL_IN_SKYBOX ) )
		{
			DispatchUpdateTransmitState();
		}
	}

	void RemoveIEFlags(int flags)
	{
		if(m_iEFlagsOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_iEFlags", &info);
			m_iEFlagsOffset = info.actual_offset;
		}

		*(int *)((unsigned char *)this + m_iEFlagsOffset) &= ~flags;

		if ( flags & ( EFL_FORCE_CHECK_TRANSMIT | EFL_IN_SKYBOX ) )
		{
			DispatchUpdateTransmitState();
		}
	}

	void SetMoveType(MoveType_t val)
	{
		if(m_MoveTypeOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_MoveType", &info);
			m_MoveTypeOffset = info.actual_offset;
		}
		
		*(unsigned char *)((unsigned char *)this + m_MoveTypeOffset) = val;
	}
	
	int GetFlags()
	{
		if(m_fFlagsOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_fFlags", &info);
			m_fFlagsOffset = info.actual_offset;
		}
		
		return *(int *)((unsigned char *)this + m_fFlagsOffset);
	}

	void AddFlag(int flag)
	{
		if(m_fFlagsOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_fFlags", &info);
			m_fFlagsOffset = info.actual_offset;
		}

		*(int *)((unsigned char *)this + m_fFlagsOffset) |= flag;
	}
	
	void CalcAbsoluteVelocity()
	{
		call_mfunc<void, CBaseEntity>(this, CBaseEntityCalcAbsoluteVelocity);
	}
	
	const Vector &GetAbsVelocity()
	{
		if(m_vecAbsVelocityOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_vecAbsVelocity", &info);
			m_vecAbsVelocityOffset = info.actual_offset;
		}
		
		if (GetIEFlags() & EFL_DIRTY_ABSVELOCITY)
		{
			CalcAbsoluteVelocity();
		}
		
		return *(Vector *)(((unsigned char *)this) + m_vecAbsVelocityOffset);
	}

	CBaseEntity *FirstMoveChild()
	{
		if(m_hMoveChildOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_hMoveChild", &info);
			m_hMoveChildOffset = info.actual_offset;
		}

		return (*(EHANDLE *)(((unsigned char *)this) + m_hMoveChildOffset)).Get();
	}

	CBaseEntity *GetMoveParent()
	{
		if(m_hMoveParentOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_hMoveParent", &info);
			m_hMoveParentOffset = info.actual_offset;
		}

		return (*(EHANDLE *)(((unsigned char *)this) + m_hMoveParentOffset)).Get();
	}

	CBaseEntity *NextMovePeer()
	{
		if(m_hMovePeerOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_hMovePeer", &info);
			m_hMovePeerOffset = info.actual_offset;
		}

		return (*(EHANDLE *)(((unsigned char *)this) + m_hMovePeerOffset)).Get();
	}

	int GetParentAttachment()
	{
		if(m_iParentAttachmentOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_iParentAttachment", &info);
			m_iParentAttachmentOffset = info.actual_offset;
		}

		return *(unsigned char *)(((unsigned char *)this) + m_vecAbsVelocityOffset);
	}

	void InvalidatePhysicsRecursive( int nChangeFlags )
	{
		// Main entry point for dirty flag setting for the 90% case
		// 1) If the origin changes, then we have to update abstransform, Shadow projection, PVS, KD-tree, 
		//    client-leaf system.
		// 2) If the angles change, then we have to update abstransform, Shadow projection,
		//    shadow render-to-texture, client-leaf system, and surrounding bounds. 
		//	  Children have to additionally update absvelocity, KD-tree, and PVS.
		//	  If the surrounding bounds actually update, when we also need to update the KD-tree and the PVS.
		// 3) If it's due to attachment, then all children who are attached to an attachment point
		//    are assumed to have dirty origin + angles.

		// Other stuff:
		// 1) Marking the surrounding bounds dirty will automatically mark KD tree + PVS dirty.
		
		int nDirtyFlags = 0;

		if ( nChangeFlags & VELOCITY_CHANGED )
		{
			nDirtyFlags |= EFL_DIRTY_ABSVELOCITY;
		}

		if ( nChangeFlags & POSITION_CHANGED )
		{
			nDirtyFlags |= EFL_DIRTY_ABSTRANSFORM;

	#ifndef CLIENT_DLL
			NetworkProp()->MarkPVSInformationDirty();
	#endif

			// NOTE: This will also mark shadow projection + client leaf dirty
			CollisionProp()->MarkPartitionHandleDirty();
		}

		// NOTE: This has to be done after velocity + position are changed
		// because we change the nChangeFlags for the child entities
		if ( nChangeFlags & ANGLES_CHANGED )
		{
			nDirtyFlags |= EFL_DIRTY_ABSTRANSFORM;
			if ( CollisionProp()->DoesRotationInvalidateSurroundingBox() )
			{
				// NOTE: This will handle the KD-tree, surrounding bounds, PVS
				// render-to-texture shadow, shadow projection, and client leaf dirty
				CollisionProp()->MarkSurroundingBoundsDirty();
			}
			else
			{
	#ifdef CLIENT_DLL
				MarkRenderHandleDirty();
				g_pClientShadowMgr->AddToDirtyShadowList( this );
				g_pClientShadowMgr->MarkRenderToTextureShadowDirty( GetShadowHandle() );
	#endif
			}

			// This is going to be used for all children: children
			// have position + velocity changed
			nChangeFlags |= POSITION_CHANGED | VELOCITY_CHANGED;
		}

		AddIEFlags( nDirtyFlags );

		// Set flags for children
		bool bOnlyDueToAttachment = false;
		if ( nChangeFlags & ANIMATION_CHANGED )
		{
	#ifdef CLIENT_DLL
			g_pClientShadowMgr->MarkRenderToTextureShadowDirty( GetShadowHandle() );
	#endif

			// Only set this flag if the only thing that changed us was the animation.
			// If position or something else changed us, then we must tell all children.
			if ( !( nChangeFlags & (POSITION_CHANGED | VELOCITY_CHANGED | ANGLES_CHANGED) ) )
			{
				bOnlyDueToAttachment = true;
			}

			nChangeFlags = POSITION_CHANGED | ANGLES_CHANGED | VELOCITY_CHANGED;
		}

		for (CBaseEntity *pChild = FirstMoveChild(); pChild; pChild = pChild->NextMovePeer())
		{
			// If this is due to the parent animating, only invalidate children that are parented to an attachment
			// Entities that are following also access attachments points on parents and must be invalidated.
			if ( bOnlyDueToAttachment )
			{
	#ifdef CLIENT_DLL
				if ( (pChild->GetParentAttachment() == 0) && !pChild->IsFollowingEntity() )
					continue;
	#else
				if ( pChild->GetParentAttachment() == 0 )
					continue;
	#endif
			}
			pChild->InvalidatePhysicsRecursive( nChangeFlags );
		}

		//
		// This code should really be in here, or the bone cache should not be in world space.
		// Since the bone transforms are in world space, if we move or rotate the entity, its
		// bones should be marked invalid.
		//
		// As it is, we're near ship, and don't have time to setup a good A/B test of how much
		// overhead this fix would add. We've also only got one known case where the lack of
		// this fix is screwing us, and I just fixed it, so I'm leaving this commented out for now.
		//
		// Hopefully, we'll put the bone cache in entity space and remove the need for this fix.
		//
		//#ifdef CLIENT_DLL
		//	if ( nChangeFlags & (POSITION_CHANGED | ANGLES_CHANGED | ANIMATION_CHANGED) )
		//	{
		//		C_BaseAnimating *pAnim = GetBaseAnimating();
		//		if ( pAnim )
		//			pAnim->InvalidateBoneCache();		
		//	}
		//#endif
	}

	void SetAbsVelocity( const Vector &vecAbsVelocity )
	{
		if(m_vecAbsVelocityOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_vecAbsVelocity", &info);
			m_vecAbsVelocityOffset = info.actual_offset;
		}

		if(m_vecVelocityOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_vecVelocity", &info);
			m_vecVelocityOffset = info.actual_offset;
		}

		if ( GetAbsVelocity() == vecAbsVelocity )
			return;

		// The abs velocity won't be dirty since we're setting it here
		// All children are invalid, but we are not
		InvalidatePhysicsRecursive( VELOCITY_CHANGED );
		RemoveIEFlags( EFL_DIRTY_ABSVELOCITY );

		*(Vector *)(((unsigned char *)this) + m_vecAbsVelocityOffset) = vecAbsVelocity;

		// NOTE: Do *not* do a network state change in this case.
		// m_vecVelocity is only networked for the player, which is not manual mode
		CBaseEntity *pMoveParent = GetMoveParent();
		if (!pMoveParent)
		{
			*(Vector *)(((unsigned char *)this) + m_vecVelocityOffset) = vecAbsVelocity;
			return;
		}

		// First subtract out the parent's abs velocity to get a relative
		// velocity measured in world space
		Vector relVelocity;
		VectorSubtract( vecAbsVelocity, pMoveParent->GetAbsVelocity(), relVelocity );

		// Transform relative velocity into parent space
		Vector vNew;
		VectorIRotate( relVelocity, pMoveParent->EntityToWorldTransform(), vNew );
		*(Vector *)(((unsigned char *)this) + m_vecVelocityOffset) = vNew;
	}
	
	bool IsWorld()
	{
		return gamehelpers->EntityToBCompatRef(this) == 0;
	}
	
	EHANDLE GetGroundEntity()
	{
		return *(EHANDLE *)((unsigned char *)this + m_hGroundEntityOffset);
	}
	
	void SetGroundEntity(CBaseEntity *pEntity)
	{
		call_mfunc<void, CBaseEntity, CBaseEntity *>(this, CBaseEntitySetGroundEntity, pEntity);
	}

	const Vector& WorldAlignMins( ) const
	{
		return CollisionProp()->OBBMins();
	}

	const Vector& WorldAlignMaxs( ) const
	{
		return CollisionProp()->OBBMaxs();
	}

	CCollisionProperty		*CollisionProp() { return (CCollisionProperty		*)GetCollideable(); }
	const CCollisionProperty*CollisionProp() const { return (const CCollisionProperty*)const_cast<CBaseEntity *>(this)->GetCollideable(); }

	CServerNetworkProperty *NetworkProp() { return (CServerNetworkProperty *)GetNetworkable(); }
	const CServerNetworkProperty *NetworkProp() const { return (const CServerNetworkProperty *)const_cast<CBaseEntity *>(this)->GetNetworkable(); }
	
	int GetCollisionGroup()
	{
		if(m_CollisionGroupOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_CollisionGroup", &info);
			m_CollisionGroupOffset = info.actual_offset;
		}

		return *(int *)(((unsigned char *)this) + m_CollisionGroupOffset);
	}

	bool FVisible( CBaseEntity *pEntity, int traceMask = MASK_BLOCKLOS, CBaseEntity **ppBlocker = NULL )
	{
		return call_vfunc<bool, CBaseEntity, CBaseEntity *, int, CBaseEntity **>(this, CBaseEntityFVisibleEnt, pEntity, traceMask, ppBlocker);
	}

	bool FVisible( const Vector &vecTarget, int traceMask = MASK_BLOCKLOS, CBaseEntity **ppBlocker = NULL )
	{
		return call_vfunc<bool, CBaseEntity, const Vector &, int, CBaseEntity **>(this, CBaseEntityFVisibleVec, vecTarget, traceMask, ppBlocker);
	}

	//GARBAGE
	int GetHealth() { return 0; }
	int m_takedamage;
	void	NetworkStateChanged() { }
	void	NetworkStateChanged( void *pVar ) {  }
	virtual bool ShouldBlockNav() const { return true; }
	int ObjectCaps() { return 0; }
	bool HasSpawnFlags(int) { return 0; }
};

void CCollisionProperty::MarkSurroundingBoundsDirty()
{
	GetOuter()->AddIEFlags( EFL_DIRTY_SURROUNDING_COLLISION_BOUNDS );
	MarkPartitionHandleDirty();

#ifdef CLIENT_DLL
	g_pClientShadowMgr->MarkRenderToTextureShadowDirty( GetOuter()->GetShadowHandle() );
#else
	GetOuter()->NetworkProp()->MarkPVSInformationDirty();
#endif
}

void CCollisionProperty::MarkPartitionHandleDirty()
{
	// don't bother with the world
	if ( m_pOuter->entindex() == 0 )
		return;
	
	if ( !(m_pOuter->GetIEFlags() & EFL_DIRTY_SPATIAL_PARTITION) )
	{
		m_pOuter->AddIEFlags( EFL_DIRTY_SPATIAL_PARTITION );
		//s_DirtyKDTree.AddEntity( m_pOuter );
	}

#ifdef CLIENT_DLL
	GetOuter()->MarkRenderHandleDirty();
	g_pClientShadowMgr->AddToDirtyShadowList( GetOuter() );
#endif
}

void CCollisionProperty::CollisionAABBToWorldAABB( const Vector &entityMins, 
	const Vector &entityMaxs, Vector *pWorldMins, Vector *pWorldMaxs ) const
{
	if ( !IsBoundsDefinedInEntitySpace() || (GetCollisionAngles() == vec3_angle) )
	{
		VectorAdd( entityMins, GetCollisionOrigin(), *pWorldMins );
		VectorAdd( entityMaxs, GetCollisionOrigin(), *pWorldMaxs );
	}
	else
	{
		TransformAABB( CollisionToWorldTransform(), entityMins, entityMaxs, *pWorldMins, *pWorldMaxs );
	}
}

void SetEdictStateChanged(CBaseEntity *pEntity, int offset)
{
	IServerNetworkable *pNet = pEntity->GetNetworkable();
	edict_t *edict = pNet->GetEdict();
	
	gamehelpers->SetEdictStateChanged(edict, offset);
}

inline bool FClassnameIs(CBaseEntity *pEntity, const char *szClassname)
{ 
	return pEntity->ClassMatches(szClassname);
}

#ifdef __HAS_ANIMHELPERS
IAnimHelpers *g_pAnimHelpers = nullptr;
#endif

class CBaseAnimating : public CBaseEntity
{
public:
	int SelectWeightedSequence(int activity)
	{
	#ifdef __HAS_ANIMHELPERS
		return g_pAnimHelpers ? g_pAnimHelpers->SelectWeightedSequence(this, activity) : -1;
	#else
		return -1;
	#endif
	}

	void StudioFrameAdvance()
	{
	#ifdef __HAS_ANIMHELPERS
		if(g_pAnimHelpers) {
			g_pAnimHelpers->StudioFrameAdvance(this);
		}
	#endif
	}
	
	void DispatchAnimEvents()
	{
	#ifdef __HAS_ANIMHELPERS
		if(g_pAnimHelpers) {
			g_pAnimHelpers->DispatchAnimEvents(this);
		}
	#endif
	}

	void ResetSequenceInfo()
	{
	#ifdef __HAS_ANIMHELPERS
		if(g_pAnimHelpers) {
			g_pAnimHelpers->ResetSequenceInfo(this);
		}
	#endif
	}

	bool GetSequeceFinished()
	{
		if(m_bSequenceFinishedOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_bSequenceFinished", &info);
			m_bSequenceFinishedOffset = info.actual_offset;
		}

		return *(bool *)((unsigned char *)this + m_bSequenceFinishedOffset);
	}

	void SetSequeceFinished(bool fin)
	{
		if(m_bSequenceFinishedOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_bSequenceFinished", &info);
			m_bSequenceFinishedOffset = info.actual_offset;
		}

		*(bool *)((unsigned char *)this + m_bSequenceFinishedOffset) = fin;
	}

	void SetPlaybackRate(float rate)
	{
		if(m_flPlaybackRateOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_flPlaybackRate", &info);
			m_flPlaybackRateOffset = info.actual_offset;
		}

		*(float *)((unsigned char *)this + m_flPlaybackRateOffset) = rate;
		SetEdictStateChanged(this, m_flPlaybackRateOffset);
	}

	int GetSequence()
	{
		if(m_nSequenceOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_nSequence", &info);
			m_nSequenceOffset = info.actual_offset;
		}

		return *(int *)((unsigned char *)this + m_nSequenceOffset);
	}

	void SetSequence(int seq)
	{
		if(m_nSequenceOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_nSequence", &info);
			m_nSequenceOffset = info.actual_offset;
		}

		*(int *)((unsigned char *)this + m_nSequenceOffset) = seq;
		SetEdictStateChanged(this, m_nSequenceOffset);
	}

	void SetAnimTime(float tim)
	{
		if(m_flAnimTimeOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_flAnimTime", &info);
			m_flAnimTimeOffset = info.actual_offset;
		}

		*(float *)((unsigned char *)this + m_flAnimTimeOffset) = tim;
		SetEdictStateChanged(this, m_flAnimTimeOffset);
	}

	void SetCycle(float tim)
	{
		if(m_flCycleOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_flCycle", &info);
			m_flCycleOffset = info.actual_offset;
		}

		*(float *)((unsigned char *)this + m_flCycleOffset) = tim;
		SetEdictStateChanged(this, m_flCycleOffset);
	}

	void ResetSequence(int seq)
	{
		int old = GetSequence();
		bool changed = (old != seq);
		SetSequence(seq);
		if(changed) {
			SetAnimTime(gpGlobals->curtime);
			SetCycle(0.0f);
			ResetSequenceInfo();
		}
	}

	float GetGroundSpeed()
	{
		if(m_flGroundSpeedOffset == -1) {
			sm_datatable_info_t info{};
			datamap_t *pMap = gamehelpers->GetDataMap(this);
			gamehelpers->FindDataMapInfo(pMap, "m_flGroundSpeed", &info);
			m_flGroundSpeedOffset = info.actual_offset;
		}

		return *(float *)((unsigned char *)this + m_flGroundSpeedOffset);
	}
};

class CBaseToggle : public CBaseEntity
{
};

enum Disposition_t 
{
	D_ER,		// Undefined - error
	D_HT,		// Hate
	D_FR,		// Fear
	D_LI,		// Like
	D_NU		// Neutral
};

const int DEF_RELATIONSHIP_PRIORITY = INT_MIN;

struct Relationship_t
{
	EHANDLE			entity;			// Relationship to a particular entity
	Class_T			classType;		// Relationship to a class  CLASS_NONE = not class based (Def. in baseentity.h)
	Disposition_t	disposition;	// D_HT (Hate), D_FR (Fear), D_LI (Like), D_NT (Neutral)
	int				priority;		// Relative importance of this relationship (higher numbers mean more important)
};

enum LineOfSightCheckType
{
	IGNORE_NOTHING,
	IGNORE_ACTORS
};

enum FieldOfViewCheckType { USE_FOV, DISREGARD_FOV };

#define BCC_DEFAULT_LOOK_TOWARDS_TOLERANCE 0.9f

class CBaseCombatCharacter : public CBaseAnimating
{
public:
	CNavArea *GetLastKnownArea()
	{
		return call_vfunc<CNavArea *>(this, CBaseCombatCharacterGetLastKnownArea);
	}
	
	void UpdateLastKnownArea()
	{
		call_vfunc<void>(this, CBaseCombatCharacterUpdateLastKnownArea);
	}
	
	void OnNavAreaRemoved(CNavArea *) {}
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	bool CanBeA(int cls)
	{
		return call_vfunc<bool, CBaseCombatCharacter, int>(this, CBaseCombatCharacterCanBeA, cls);
	}
#endif

	void Event_Killed(const CTakeDamageInfo &info)
	{
		call_mfunc<void, CBaseCombatCharacter, const CTakeDamageInfo &>(this, CBaseCombatCharacterEvent_Killed, info);
	}

	void SetBloodColor(int color)
	{
		if(m_bloodColorOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_bloodColor", &info);
			m_bloodColorOffset = info.actual_offset;
		}

		*(int *)(((unsigned char *)this) + m_bloodColorOffset) = color;
	}

	int GetBloodColor()
	{
		if(m_bloodColorOffset == -1) {
			datamap_t *map = gamehelpers->GetDataMap(this);
			sm_datatable_info_t info{};
			gamehelpers->FindDataMapInfo(map, "m_bloodColor", &info);
			m_bloodColorOffset = info.actual_offset;
		}

		return *(int *)(((unsigned char *)this) + m_bloodColorOffset);
	}

	static void AllocateDefaultRelationships()
	{
		((void(*)())(CBaseCombatCharacterAllocateDefaultRelationships))();
	}

	static void SetDefaultRelationship(Class_T nClass, Class_T nClassTarget, Disposition_t nDisposition, int nPriority)
	{
		((void(*)(Class_T, Class_T, Disposition_t, int))(CBaseCombatCharacterSetDefaultRelationship))(nClass, nClassTarget, nDisposition, nPriority);
	}

	bool IsHiddenByFog( const Vector &target )
	{
		return call_vfunc<bool, CBaseCombatCharacter, const Vector &>(this, CBaseCombatCharacterIsHiddenByFogVec, target);
	}

	bool IsHiddenByFog( CBaseEntity *target )
	{
		return call_vfunc<bool, CBaseCombatCharacter, CBaseEntity *>(this, CBaseCombatCharacterIsHiddenByFogEnt, target);
	}

	bool IsHiddenByFog( float range )
	{
		return call_vfunc<bool, CBaseCombatCharacter, float>(this, CBaseCombatCharacterIsHiddenByFogR, range);
	}

	float GetFogObscuredRatio( const Vector &target )
	{
		return call_vfunc<float, CBaseCombatCharacter, const Vector &>(this, CBaseCombatCharacterGetFogObscuredRatioVec, target);
	}

	float GetFogObscuredRatio( CBaseEntity *target )
	{
		return call_vfunc<float, CBaseCombatCharacter, CBaseEntity *>(this, CBaseCombatCharacterGetFogObscuredRatioEnt, target);
	}

	float GetFogObscuredRatio( float range )
	{
		return call_vfunc<float, CBaseCombatCharacter, float>(this, CBaseCombatCharacterGetFogObscuredRatioR, range);
	}

	bool IsLookingTowards( CBaseEntity *target, float cosTolerance = BCC_DEFAULT_LOOK_TOWARDS_TOLERANCE )
	{
		return call_vfunc<bool, CBaseCombatCharacter, CBaseEntity *, float>(this, CBaseCombatCharacterIsLookingTowardsEnt, target, cosTolerance);
	}

	bool IsLookingTowards( const Vector &target, float cosTolerance = BCC_DEFAULT_LOOK_TOWARDS_TOLERANCE )
	{
		return call_vfunc<bool, CBaseCombatCharacter, const Vector &, float>(this, CBaseCombatCharacterIsLookingTowardsVec, target, cosTolerance);
	}

	bool IsInFieldOfView( CBaseEntity *entity )
	{
		return call_vfunc<bool, CBaseCombatCharacter, CBaseEntity *>(this, CBaseCombatCharacterIsInFieldOfViewEnt, entity);
	}

	bool IsInFieldOfView( const Vector &pos )
	{
		return call_vfunc<bool, CBaseCombatCharacter, const Vector &>(this, CBaseCombatCharacterIsInFieldOfViewVec, pos);
	}

	bool IsLineOfSightClear( CBaseEntity *entity, LineOfSightCheckType checkType = IGNORE_NOTHING )
	{
		return call_vfunc<bool, CBaseCombatCharacter, CBaseEntity *, LineOfSightCheckType>(this, CBaseCombatCharacterIsLineOfSightClearEnt, entity, checkType);
	}

	bool IsLineOfSightClear( const Vector &pos, LineOfSightCheckType checkType = IGNORE_NOTHING, CBaseEntity *entityToIgnore = NULL )
	{
		return call_vfunc<bool, CBaseCombatCharacter, const Vector &, LineOfSightCheckType>(this, CBaseCombatCharacterIsLineOfSightClearVec, pos, checkType, entityToIgnore);
	}

	bool FInViewCone( CBaseEntity *pEntity )
	{
		return call_vfunc<bool, CBaseCombatCharacter, CBaseEntity *>(this, CBaseCombatCharacterFInViewConeEnt, pEntity);
	}

	bool FInViewCone( const Vector &vecTarget )
	{
		return call_vfunc<bool, CBaseCombatCharacter, const Vector &>(this, CBaseCombatCharacterFInViewConeVec, vecTarget);
	}

	bool FInAimCone( CBaseEntity *pEntity )
	{
		return call_vfunc<bool, CBaseCombatCharacter, CBaseEntity *>(this, CBaseCombatCharacterFInAimConeEnt, pEntity);
	}

	bool FInAimCone( const Vector &vecTarget )
	{
		return call_vfunc<bool, CBaseCombatCharacter, const Vector &>(this, CBaseCombatCharacterFInAimConeVec, vecTarget);
	}

	bool IsAbleToSee( CBaseEntity *entity, FieldOfViewCheckType checkFOV )
	{
		return call_mfunc<bool, CBaseCombatCharacter, CBaseEntity *, FieldOfViewCheckType>(this, CBaseCombatCharacterIsAbleToSeeEnt, entity, checkFOV);
	}

	bool IsAbleToSee( CBaseCombatCharacter *pBCC, FieldOfViewCheckType checkFOV )
	{
		return call_mfunc<bool, CBaseCombatCharacter, CBaseCombatCharacter *, FieldOfViewCheckType>(this, CBaseCombatCharacterIsAbleToSeeCC, pBCC, checkFOV);
	}
};

class CBasePlayer : public CBaseCombatCharacter
{
public:
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	bool IsGhost()
	{
		return *(bool *)((unsigned char *)this + m_isGhostOffset);
	}
#endif
};

#define FCAP_USE_IN_RADIUS 0
#define FCAP_IMPULSE_USE 0

#define private public
#define protected public

#include <tier0/vprof.h>

#undef VPROF_BUDGET
#define VPROF_BUDGET(...) 

#include <nav.h>
#include <nav_ladder.h>
#include <nav_area.h>
#include <nav_mesh.h>
#define Max MAX
#include <nav_pathfind.h>

NavAreaVector *TheNavAreas = nullptr;

#if SOURCE_ENGINE == SE_TF2
class CFuncNavCost : public CBaseEntity
{
public:
	float GetCostMultiplier(CBaseCombatCharacter *who) const
	{
		return call_vfunc<float, CFuncNavCost, CBaseCombatCharacter *>(this, CFuncNavCostGetCostMultiplier, who);
	}
};
#endif

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

#if SOURCE_ENGINE == SE_TF2
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
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
class TerrorNavArea : public CNavArea
{
public:
	unsigned int GetSpawnAttributes() const
	{ return m_SpawnAttributes; }
	
	unsigned int m_SpawnAttributes;
};
#endif

#if SOURCE_ENGINE == SE_TF2
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
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
bool CNavArea::IsBlocked( int teamID, bool ignoreNavBlockers ) const
{
	if ( ignoreNavBlockers && ( m_attributeFlags & NAV_MESH_NAV_BLOCKER ) )
	{
		return false;
	}
	
	#define TEAM_SURVIVOR 2
	#define NAV_PLAYERCLIP NAV_MESH_FIRST_CUSTOM

#ifdef TERROR
	if ( ( teamID == TEAM_SURVIVOR ) && ( m_attributeFlags & NAV_PLAYERCLIP ) )
		return true;
#endif

	if ( teamID == TEAM_ANY )
	{
		bool isBlocked = false;
		for ( int i=0; i<MAX_NAV_TEAMS; ++i )
		{
			isBlocked |= m_isBlocked[ i ];
		}

		return isBlocked;
	}

	int teamIdx = teamID % MAX_NAV_TEAMS;
	return m_isBlocked[ teamIdx ];
}
#endif

void CNavArea::DecayDanger( void )
{
	for( int i=0; i<MAX_NAV_TEAMS; ++i )
	{
		float deltaT = gpGlobals->curtime - m_dangerTimestamp[i];
		float decayAmount = GetDangerDecayRate() * deltaT;

		m_danger[i] -= decayAmount;
		if (m_danger[i] < 0.0f)
			m_danger[i] = 0.0f;

		// update timestamp
		m_dangerTimestamp[i] = gpGlobals->curtime;
	}
}

float CNavArea::GetDanger( int teamID )
{
	DecayDanger();

	int teamIdx = teamID % MAX_NAV_TEAMS;
	return m_danger[ teamIdx ];
}

void CNavArea::GetExtent( Extent *extent ) const
{
	extent->lo = m_nwCorner;
	extent->hi = m_seCorner;

	extent->lo.z = MIN( extent->lo.z, m_nwCorner.z );
	extent->lo.z = MIN( extent->lo.z, m_seCorner.z );
	extent->lo.z = MIN( extent->lo.z, m_neZ );
	extent->lo.z = MIN( extent->lo.z, m_swZ );

	extent->hi.z = MAX( extent->hi.z, m_nwCorner.z );
	extent->hi.z = MAX( extent->hi.z, m_seCorner.z );
	extent->hi.z = MAX( extent->hi.z, m_neZ );
	extent->hi.z = MAX( extent->hi.z, m_swZ );
}

void CNavArea::GetClosestPointOnArea( const Vector * RESTRICT pPos, Vector *close ) const RESTRICT 
{
	float x, y, z;

	// Using fsel rather than compares, as much faster on 360 [7/28/2008 tom]
	x = fsel( pPos->x - m_nwCorner.x, pPos->x, m_nwCorner.x );
	x = fsel( x - m_seCorner.x, m_seCorner.x, x );

	y = fsel( pPos->y - m_nwCorner.y, pPos->y, m_nwCorner.y );
	y = fsel( y - m_seCorner.y, m_seCorner.y, y );

	z = GetZ( x, y );

	close->Init( x, y, z );
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

Vector CNavArea::GetRandomPoint( void ) const
{
	Extent extent;
	GetExtent( &extent );

	Vector spot;
	spot.x = RandomFloat( extent.lo.x, extent.hi.x ); 
	spot.y = RandomFloat( extent.lo.y, extent.hi.y );
	spot.z = GetZ( spot.x, spot.y );

	return spot;
}

bool CNavArea::IsOverlapping( const Vector &pos, float tolerance ) const
{
	if (pos.x + tolerance >= m_nwCorner.x && pos.x - tolerance <= m_seCorner.x &&
		pos.y + tolerance >= m_nwCorner.y && pos.y - tolerance <= m_seCorner.y)
		return true;

	return false;
}

bool CNavArea::IsOverlapping( const CNavArea *area ) const
{
	if (area->m_nwCorner.x < m_seCorner.x && area->m_seCorner.x > m_nwCorner.x && 
		area->m_nwCorner.y < m_seCorner.y && area->m_seCorner.y > m_nwCorner.y)
		return true;

	return false;
}

bool CNavArea::IsOverlapping( const Extent &extent ) const
{
	return ( extent.lo.x < m_seCorner.x && extent.hi.x > m_nwCorner.x && 
			 extent.lo.y < m_seCorner.y && extent.hi.y > m_nwCorner.y );
}

bool CNavArea::IsOverlappingX( const CNavArea *area ) const
{
	if (area->m_nwCorner.x < m_seCorner.x && area->m_seCorner.x > m_nwCorner.x)
		return true;

	return false;
}

bool CNavArea::IsOverlappingY( const CNavArea *area ) const
{
	if (area->m_nwCorner.y < m_seCorner.y && area->m_seCorner.y > m_nwCorner.y)
		return true;

	return false;
}

bool CNavArea::Contains( const CNavArea *area ) const
{
	return ( ( m_nwCorner.x <= area->m_nwCorner.x ) && ( m_seCorner.x >= area->m_seCorner.x ) &&
		( m_nwCorner.y <= area->m_nwCorner.y ) && ( m_seCorner.y >= area->m_seCorner.y ) &&
		( m_nwCorner.z <= area->m_nwCorner.z ) && ( m_seCorner.z >= area->m_seCorner.z ) );
}

class COverlapCheck
{
public:
	COverlapCheck( const CNavArea *me, const Vector &pos ) : m_pos( pos )
	{
		m_me = me;
		m_myZ = me->GetZ( pos );
	}

	bool operator() ( CNavArea *area )
	{
		// skip self
		if ( area == m_me )
			return true;

		// check 2D overlap
		if ( !area->IsOverlapping( m_pos ) )
			return true;

		float theirZ = area->GetZ( m_pos );
		if ( theirZ > m_pos.z )
		{
			// they are above the point
			return true;
		}

		if ( theirZ > m_myZ )
		{
			// we are below an area that is beneath the given position
			return false;
		}

		return true;
	}

	const CNavArea *m_me;
	float m_myZ;
	const Vector &m_pos;
};

bool CNavArea::Contains( const Vector &pos ) const
{
	// check 2D overlap
	if (!IsOverlapping( pos ))
		return false;

	// the point overlaps us, check that it is above us, but not above any areas that overlap us
	float myZ = GetZ( pos );

	// if the nav area is above the given position, fail
	// allow nav area to be as much as a step height above the given position
	if (myZ - StepHeight > pos.z)
		return false;

	Extent areaExtent;
	GetExtent( &areaExtent );

	COverlapCheck overlap( this, pos );
	return TheNavMesh->ForAllAreasOverlappingExtent( overlap, areaExtent );
}

void CNavArea::ClearSearchLists( void )
{
	// effectively clears all open list pointers and closed flags
	CNavArea::MakeNewMarker();

	*m_openList = NULL;
	*m_openListTail = NULL;
}

void CNavArea::AddToOpenList( void )
{
	if ( IsOpen() )
	{
		// already on list
		return;
	}

	// mark as being on open list for quick check
	m_openMarker = *m_masterMarker;

	// if list is empty, add and return
	if ( *m_openList == NULL )
	{
		*m_openList = this;
		*m_openListTail = this;
		this->m_prevOpen = NULL;
		this->m_nextOpen = NULL;
		return;
	}

	// insert self in ascending cost order
	CNavArea *area, *last = NULL;
	for( area = *m_openList; area; area = area->m_nextOpen )
	{
		if ( GetTotalCost() < area->GetTotalCost() )
		{
			break;
		}
		last = area;
	}

	if ( area )
	{
		// insert before this area
		this->m_prevOpen = area->m_prevOpen;

		if ( this->m_prevOpen )
		{
			this->m_prevOpen->m_nextOpen = this;
		}
		else
		{
			*m_openList = this;
		}

		this->m_nextOpen = area;
		area->m_prevOpen = this;
	}
	else
	{
		// append to end of list
		last->m_nextOpen = this;
		this->m_prevOpen = last;
	
		this->m_nextOpen = NULL;

		*m_openListTail = this;
	}
}

void CNavArea::UpdateOnOpenList( void )
{
	// since value can only decrease, bubble this area up from current spot
	while( m_prevOpen && this->GetTotalCost() < m_prevOpen->GetTotalCost() )
	{
		// swap position with predecessor
		CNavArea *other = m_prevOpen;
		CNavArea *before = other->m_prevOpen;
		CNavArea *after  = this->m_nextOpen;

		this->m_nextOpen = other;
		this->m_prevOpen = before;

		other->m_prevOpen = this;
		other->m_nextOpen = after;

		if ( before )
		{
			before->m_nextOpen = this;
		}
		else
		{
			*m_openList = this;
		}

		if ( after )
		{
			after->m_prevOpen = other;
		}
		else
		{
			*m_openListTail = this;
		}
	}
}

void CNavArea::RemoveFromOpenList( void )
{
	if ( m_openMarker == 0 )
	{
		// not on the list
		return;
	}

	if ( m_prevOpen )
	{
		m_prevOpen->m_nextOpen = m_nextOpen;
	}
	else
	{
		*m_openList = m_nextOpen;
	}
	
	if ( m_nextOpen )
	{
		m_nextOpen->m_prevOpen = m_prevOpen;
	}
	else
	{
		*m_openListTail = m_prevOpen;
	}
	
	// zero is an invalid marker
	m_openMarker = 0;
}

unsigned int *CNavArea::m_masterMarker = NULL;
CNavArea **CNavArea::m_openList = NULL;
CNavArea **CNavArea::m_openListTail = NULL;

#if SOURCE_ENGINE == SE_TF2
CNavArea *CNavMesh::GetNearestNavArea( const Vector &pos, bool anyZ, float maxDist, bool checkLOS, bool checkGround, int team ) const
{
	return call_mfunc<CNavArea *, CNavMesh, const Vector &, bool, float, bool, bool, int>(this, CNavMeshGetNearestNavArea, pos, anyZ, maxDist, checkLOS, checkGround, team);
}
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
CNavArea *CNavMesh::GetNearestNavArea( const Vector &pos, bool anyZ, float maxDist, bool checkLOS, bool checkGround, bool unk ) const
{
	return call_mfunc<CNavArea *, CNavMesh, const Vector &, bool, float, bool, bool, bool>(this, CNavMeshGetNearestNavArea, pos, anyZ, maxDist, checkLOS, checkGround, unk);
}
#endif

CNavArea *CNavMesh::GetNavArea( CBaseEntity *pEntity, int nFlags, float flBeneathLimit ) const
{
	if ( !m_grid.Count() )
		return NULL;

	Vector testPos = pEntity->GetAbsOrigin();

	float flStepHeight = 1e-3;
	CBaseCombatCharacter *pBCC = pEntity->MyCombatCharacterPointer();
	if ( pBCC )
	{
		// Check if we're still in the last area
		CNavArea *pLastNavArea = pBCC->GetLastKnownArea();
		if ( pLastNavArea && pLastNavArea->IsOverlapping( testPos ) )
		{
			float flZ = pLastNavArea->GetZ( testPos );
			if ( ( flZ <= testPos.z + StepHeight ) && ( flZ >= testPos.z - StepHeight ) )
				return pLastNavArea;
		}
		flStepHeight = StepHeight;
	}

	// get list in cell that contains position
	int x = WorldToGridX( testPos.x );
	int y = WorldToGridY( testPos.y );
	NavAreaVector *areaVector = &m_grid[ x + y*m_gridSizeX ];

	// search cell list to find correct area
	CNavArea *use = NULL;
	float useZ = -99999999.9f;

	bool bSkipBlockedAreas = ( ( nFlags & GETNAVAREA_ALLOW_BLOCKED_AREAS ) == 0 );
	FOR_EACH_VEC( (*areaVector), it )
	{
		CNavArea *pArea = (*areaVector)[ it ];

		// check if position is within 2D boundaries of this area
		if ( !pArea->IsOverlapping( testPos ) )
			continue;

		// don't consider blocked areas
		if ( bSkipBlockedAreas && pArea->IsBlocked( pEntity->GetTeamNumber() ) )
			continue;

		// project position onto area to get Z
		float z = pArea->GetZ( testPos );

		// if area is above us, skip it
		if ( z > testPos.z + flStepHeight )
			continue;

		// if area is too far below us, skip it
		if ( z < testPos.z - flBeneathLimit )
			continue;

		// if area is lower than the one we have, skip it
		if ( z <= useZ )
			continue;

		use = pArea;
		useZ = z;
	}

	// Check LOS if necessary
	if ( use && ( nFlags & GETNAVAREA_CHECK_LOS ) && ( useZ < testPos.z - flStepHeight ) )
	{
		// trace directly down to see if it's below us and unobstructed
		trace_t result;
		UTIL_TraceLine( testPos, Vector( testPos.x, testPos.y, useZ ), MASK_NPCSOLID_BRUSHONLY, NULL, COLLISION_GROUP_NONE, &result );
		if ( ( result.fraction != 1.0f ) && ( fabs( result.endpos.z - useZ ) > flStepHeight ) )
			return NULL;
	}
	return use;
}

CNavArea *CNavMesh::GetNavArea( const Vector &pos, float beneathLimit ) const
{
	if ( !m_grid.Count() )
		return NULL;

	// get list in cell that contains position
	int x = WorldToGridX( pos.x );
	int y = WorldToGridY( pos.y );
	NavAreaVector *areaVector = &m_grid[ x + y*m_gridSizeX ];

	// search cell list to find correct area
	CNavArea *use = NULL;
	float useZ = -99999999.9f;
	Vector testPos = pos + Vector( 0, 0, 5 );

	FOR_EACH_VEC( (*areaVector), it )
	{
		CNavArea *area = (*areaVector)[ it ];

		// check if position is within 2D boundaries of this area
		if (area->IsOverlapping( testPos ))
		{
			// project position onto area to get Z
			float z = area->GetZ( testPos );

			// if area is above us, skip it
			if (z > testPos.z)
				continue;

			// if area is too far below us, skip it
			if (z < pos.z - beneathLimit)
				continue;

			// if area is higher than the one we have, use this instead
			if (z > useZ)
			{
				use = area;
				useZ = z;
			}
		}
	}

	return use;
}

CNavArea *CNavMesh::GetNearestNavArea( CBaseEntity *pEntity, int nFlags, float maxDist ) const
{
	if ( !m_grid.Count() )
		return NULL;

	// quick check
	CNavArea *pClose = GetNavArea( pEntity, nFlags );
	if ( pClose )
		return pClose;

	bool bCheckLOS = ( nFlags & GETNAVAREA_CHECK_LOS ) != 0;
	bool bCheckGround = ( nFlags & GETNAVAREA_CHECK_GROUND ) != 0;
#if SOURCE_ENGINE == SE_TF2
	return GetNearestNavArea( pEntity->GetAbsOrigin(), false, maxDist, bCheckLOS, bCheckGround, pEntity->GetTeamNumber() );
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	return GetNearestNavArea( pEntity->GetAbsOrigin(), false, maxDist, bCheckLOS, bCheckGround, false );
#endif
}

CNavArea *CNavMesh::GetNavAreaByID( unsigned int id ) const
{
	if (id == 0)
		return NULL;

	int key = ComputeHashKey( id );

	for( CNavArea *area = m_hashTable[key]; area; area = area->m_nextHash )
	{
		if (area->GetID() == id)
		{
			return area;
		}
	}

	return NULL;
}

bool CNavMesh::GetSimpleGroundHeight( const Vector &pos, float *height, Vector *normal ) const
{
	Vector to;
	to.x = pos.x;
	to.y = pos.y;
	to.z = pos.z - 9999.9f;

	trace_t result;

	UTIL_TraceLine( pos, to, MASK_NPCSOLID_BRUSHONLY, NULL, COLLISION_GROUP_NONE, &result );

	if (result.startsolid)
		return false;

	*height = result.endpos.z;

	if (normal)
		*normal = result.plane.normal;

	return true;
}

unsigned int CNavMesh::GetPlace( const Vector &pos ) const
{
	CNavArea *area = GetNearestNavArea( pos, true );

	if (area)
	{
		return area->GetPlace();
	}

	return UNDEFINED_PLACE;
}

const char *CNavMesh::PlaceToName( Place place ) const
{
	if (place >= 1 && place <= m_placeCount)
		return m_placeName[ (int)place - 1 ];

	return NULL;
}

Place CNavMesh::NameToPlace( const char *name ) const
{
	for( unsigned int i=0; i<m_placeCount; ++i )
	{
		if (FStrEq( m_placeName[i], name ))
			return i+1;
	}

	return UNDEFINED_PLACE;
}

bool CNavMesh::GetGroundHeight( const Vector &pos, float *height, Vector *normal ) const
{
	return call_mfunc<bool, CNavMesh, const Vector &, float *, Vector *>(this, CNavMeshGetGroundHeight, pos, height, normal);
}

#if SOURCE_ENGINE == SE_TF2
template< typename CostFunctor >
bool NavAreaBuildPath_real( CNavArea *startArea, CNavArea *goalArea, const Vector *goalPos, CostFunctor &costFunc, CNavArea **closestArea, float maxPathLength, int teamID, bool ignoreNavBlockers )
{
	return (void_to_func<bool(*)(CNavArea *, CNavArea *, const Vector *, CostFunctor &, CNavArea **, float, int, bool)>(NavAreaBuildPathPtr))(startArea, goalArea, goalPos, costFunc, closestArea, maxPathLength, teamID, ignoreNavBlockers);
}
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
template< typename CostFunctor >
bool NavAreaBuildPath_real( CNavArea *startArea, CNavArea *goalArea, const Vector *startPos, const Vector *goalPos, CostFunctor &costFunc, CNavArea **closestArea, float maxPathLength, int teamID, bool ignoreNavBlockers )
{
	return (void_to_func<bool(*)(CNavArea *, CNavArea *, const Vector *, const Vector *, CostFunctor &, CNavArea **, float, int, bool)>(NavAreaBuildPathPtr))(startArea, goalArea, startPos, goalPos, costFunc, closestArea, maxPathLength, teamID, ignoreNavBlockers);
}
#endif

class CBaseCombatWeapon;
class Path;
struct animevent_t;
enum MoveToFailureType
{
	FAIL_NO_PATH_EXISTS,
	FAIL_STUCK,
	FAIL_FELL_OFF,
};
struct AI_Response;
using AIConcept_t = int;
class CTakeDamageInfo;

class INextBotEventResponder
{
public:
	virtual ~INextBotEventResponder() {}
	
	// these methods are used by derived classes to define how events propagate
	virtual INextBotEventResponder *FirstContainedResponder( void ) const { return nullptr; }
	virtual INextBotEventResponder *NextContainedResponder( INextBotEventResponder *current ) const { return nullptr; }
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual const char *GetDebugString() const = 0;
#endif
	
	//
	// Events.  All events must be 'extended' by calling the derived class explicitly to ensure propagation.
	// Each event must implement its propagation in this interface class.
	//
	virtual void OnLeaveGround( CBaseEntity *ground )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnLeaveGround( ground );
		}
	}
	virtual void OnLandOnGround( CBaseEntity *ground )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnLandOnGround( ground );
		}
	}

	virtual void OnContact( CBaseEntity *other, CGameTrace *result = NULL )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnContact( other, result );
		}
	}

	virtual void OnMoveToSuccess( const Path *path )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnMoveToSuccess( path );
		}
	}
	virtual void OnMoveToFailure( const Path *path, MoveToFailureType reason )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnMoveToFailure( path, reason );
		}
	}
	virtual void OnStuck( void )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnStuck();
		}
	}
	virtual void OnUnStuck( void )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnUnStuck();
		}
	}

	virtual void OnPostureChanged( void )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnPostureChanged();
		}
	}

	virtual void OnAnimationActivityComplete( int activity )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnAnimationActivityComplete( activity );
		}
	}
	virtual void OnAnimationActivityInterrupted( int activity )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnAnimationActivityInterrupted( activity );
		}
	}
	virtual void OnAnimationEvent( animevent_t *event )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnAnimationEvent( event );
		}
	}

	virtual void OnIgnite( void )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnIgnite();
		}
	}
	virtual void OnInjured( const CTakeDamageInfo &info )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnInjured( info );
		}
	}
	virtual void OnKilled( const CTakeDamageInfo &info )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnKilled( info );
		}
	}
	virtual void OnOtherKilled( CBaseCombatCharacter *victim, const CTakeDamageInfo &info )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnOtherKilled( victim, info );
		}
	}

	virtual void OnSight( CBaseEntity *subject )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnSight( subject );
		}
	}
	virtual void OnLostSight( CBaseEntity *subject )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnLostSight( subject );
		}
	}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual void OnThreatChanged(CBaseEntity *subject)
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnThreatChanged( subject );
		}
	}
#endif
	
	virtual void OnSound( CBaseEntity *source, const Vector &pos, KeyValues *keys )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnSound( source, pos, keys );
		}
	}
	virtual void OnSpokeConcept( CBaseCombatCharacter *who, AIConcept_t concept, AI_Response *response )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnSpokeConcept( who, concept, response );
		}
	}
	
#if SOURCE_ENGINE == SE_TF2
	virtual void OnWeaponFired( CBaseCombatCharacter *whoFired, CBaseCombatWeapon *weapon )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnWeaponFired( whoFired, weapon );
		}
	}
#endif
	
	virtual void OnNavAreaChanged( CNavArea *newArea, CNavArea *oldArea )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnNavAreaChanged( newArea, oldArea );
		}
	}

	virtual void OnModelChanged( void )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnModelChanged();
		}
	}

	virtual void OnPickUp( CBaseEntity *item, CBaseCombatCharacter *giver )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnPickUp( item, giver );
		}
	}
	virtual void OnDrop( CBaseEntity *item )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnDrop( item );
		}
	}
	
#if SOURCE_ENGINE == SE_TF2
	virtual void OnActorEmoted( CBaseCombatCharacter *emoter, int emote )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnActorEmoted( emoter, emote );
		}
	}

	virtual void OnCommandAttack( CBaseEntity *victim );	// attack the given entity
	virtual void OnCommandApproach( const Vector &pos, float range = 0.0f );	// move to within range of the given position
	virtual void OnCommandApproach( CBaseEntity *goal );	// follow the given leader
	virtual void OnCommandRetreat( CBaseEntity *threat, float range = 0.0f );	// retreat from the threat at least range units away (0 == infinite)
	virtual void OnCommandPause( float duration = 0.0f );	// pause for the given duration (0 == forever)
	virtual void OnCommandResume( void );					// resume after a pause
	virtual void OnCommandString( const char *command );	// for debugging: respond to an arbitrary string representing a generalized command
#endif
	
	virtual void OnShoved( CBaseEntity *pusher )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnShoved( pusher );
		}
	}
	virtual void OnBlinded( CBaseEntity *blinder )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnBlinded( blinder );
		}
	}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual void OnEnteredSpit()
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnEnteredSpit();
		}
	}
	virtual void OnHitByVomitJar(CBaseEntity *victim)
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnHitByVomitJar(victim);
		}
	}

	virtual void OnCommandAttack( CBaseEntity *victim );	// attack the given entity
	virtual void OnCommandAssault()
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnCommandAssault();
		}
	}
	virtual void OnCommandApproach( const Vector &pos, float range = 0.0f );	// move to within range of the given position
	virtual void OnCommandApproach( CBaseEntity *goal );	// follow the given leader
	virtual void OnCommandRetreat( CBaseEntity *threat, float range = 0.0f );	// retreat from the threat at least range units away (0 == infinite)
	virtual void OnCommandPause( float duration = 0.0f );	// pause for the given duration (0 == forever)
	virtual void OnCommandResume( void );					// resume after a pause
	virtual void OnCommandString( const char *command );	// for debugging: respond to an arbitrary string representing a generalized command
#endif
	
#if SOURCE_ENGINE == SE_TF2
	virtual void OnTerritoryContested( int territoryID )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnTerritoryContested( territoryID );
		}
	}
	virtual void OnTerritoryCaptured( int territoryID )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnTerritoryCaptured( territoryID );
		}
	}
	virtual void OnTerritoryLost( int territoryID )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnTerritoryLost( territoryID );
		}
	}

	virtual void OnWin( void )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnWin();
		}
	}
	virtual void OnLose( void )
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			sub->OnLose();
		}
	}
#endif
};

inline void INextBotEventResponder::OnCommandAttack( CBaseEntity *victim )
{
	for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
	{
		sub->OnCommandAttack( victim );
	}	
}

inline void INextBotEventResponder::OnCommandApproach( const Vector &pos, float range )
{
	for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
	{
		sub->OnCommandApproach( pos, range );
	}	
}

inline void INextBotEventResponder::OnCommandApproach( CBaseEntity *goal )
{
	for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
	{
		sub->OnCommandApproach( goal );
	}	
}

inline void INextBotEventResponder::OnCommandRetreat( CBaseEntity *threat, float range )
{
	for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
	{
		sub->OnCommandRetreat( threat, range );
	}	
}

inline void INextBotEventResponder::OnCommandPause( float duration )
{
	for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
	{
		sub->OnCommandPause( duration );
	}	
}

inline void INextBotEventResponder::OnCommandResume( void )
{
	for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
	{
		sub->OnCommandResume();
	}	
}

inline void INextBotEventResponder::OnCommandString( const char *command )
{
	for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
	{
		sub->OnCommandString( command );
	}	
}

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
	
	float GetUpdateInterval() 
	{ 
		return m_curInterval; 
	}
	
	float m_lastUpdateTime;
	float m_curInterval;

	INextBot *m_bot;
	INextBotComponent *m_nextComponent;									// simple linked list of components in the bot
};

enum TraverseWhenType 
{ 
	IMMEDIATELY,		// the entity will not block our motion - we'll carry right through
	EVENTUALLY			// the entity will block us until we spend effort to open/destroy it
};

class ILocomotion : public INextBotComponent
{
public:
	virtual ~ILocomotion() = 0;
	
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
	virtual bool ClimbUpToLedge( const Vector &landingGoal, const Vector &landingForward, CBaseEntity *obstacle ) = 0;	// initiate a jump to an adjacent high ledge, return false if climb can't start
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
#if SOURCE_ENGINE == SE_TF2
	virtual bool IsAbleToJumpAcrossGaps( void ) = 0;		// return true if this bot can jump across gaps in its path
	virtual bool IsAbleToClimb( void ) = 0;				// return true if this bot can climb arbitrary geometry it encounters
#endif
	
	virtual const Vector &GetFeet( void ) = 0;			// return position of "feet" - the driving point where the bot contacts the ground

	virtual float GetStepHeight( void ) = 0;				// if delta Z is greater than this, we have to jump to get up
	virtual float GetMaxJumpHeight( void ) = 0;			// return maximum height of a jump
	virtual float GetDeathDropHeight( void ) = 0;			// distance at which we will die if we fall

	virtual float GetRunSpeed( void ) = 0;				// get maximum running speed
	virtual float GetWalkSpeed( void ) = 0;				// get maximum walking speed

#if SOURCE_ENGINE == SE_TF2
	virtual float GetMaxAcceleration( void ) = 0;			// return maximum acceleration of locomotor
	virtual float GetMaxDeceleration( void ) = 0;			// return maximum deceleration of locomotor
#endif
	
	virtual const Vector &GetVelocity( void ) = 0;		// return current world space velocity
	virtual float GetSpeed( void ) const = 0;					// return current world space speed (magnitude of velocity)
	virtual const Vector &GetMotionVector( void ) = 0;	// return unit vector describing our direction of motion - even if we are currently not moving

	virtual bool IsAreaTraversable( const CNavArea *baseArea ) = 0;	// return true if given area can be used for navigation

	virtual float GetTraversableSlopeLimit( void ) = 0;	// return Z component of unit normal of steepest traversable slope

	// return true if the given entity can be ignored during locomotion
	using TraverseWhenType = ::TraverseWhenType;

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
#if SOURCE_ENGINE == SE_TF2
	virtual bool ShouldCollideWith( CBaseEntity *object ) = 0;
#endif
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual const Vector &GetLastApproachPosition() const = 0;
#endif
	
	virtual void AdjustPosture( const Vector &moveGoal ) = 0;
	
#if SOURCE_ENGINE == SE_TF2
	virtual void StuckMonitor( void ) = 0;
#endif
	
	void TraceHull( const Vector& start, const Vector& end, const Vector &mins, const Vector &maxs, unsigned int fMask, ITraceFilter *pFilter, trace_t *pTrace ) const
	{
	//	VPROF_BUDGET( "ILocomotion::TraceHull", "TraceHull" );
		Ray_t ray;
		ray.Init( start, end, mins, maxs );
		enginetrace->TraceRay( ray, fMask, pFilter, pTrace );
	}
	
#if SOURCE_ENGINE == SE_TF2
	Vector m_motionVector;
	Vector m_groundMotionVector;
	float m_speed;
	float m_groundSpeed;

	// stuck monitoring
	bool m_isStuck;									// if true, we are stuck
	IntervalTimer m_stuckTimer;						// how long we've been stuck
	CountdownTimer m_stillStuckTimer;				// for resending stuck events
	Vector m_stuckPos;								// where we got stuck
	IntervalTimer m_moveRequestTimer;
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	char pad1[88];
#endif
};

enum LookAtPriorityType
{
	BORING,
	INTERESTING,				// last known enemy location, dangerous sound location
	IMPORTANT,					// a danger
	CRITICAL,					// an active threat to our safety
	MANDATORY					// nothing can interrupt this look at - two simultaneous look ats with this priority is an error
};

enum PostureType
{
	STAND,
	CROUCH,
	SIT,
	CRAWL,
	LIE
};

enum ActivityType 
{ 
	MOTION_CONTROLLED_XY	= 0x0001,	// XY position and orientation of the bot is driven by the animation.
	MOTION_CONTROLLED_Z		= 0x0002,	// Z position of the bot is driven by the animation.
	ACTIVITY_UNINTERRUPTIBLE= 0x0004,	// activity can't be changed until animation finishes
	ACTIVITY_TRANSITORY		= 0x0008,	// a short animation that takes over from the underlying animation momentarily, resuming it upon completion
	ENTINDEX_PLAYBACK_RATE	= 0x0010,	// played back at different rates based on entindex
};

enum ArousalType
{
	NEUTRAL,
	ALERT,
	INTENSE
};

class IBody : public INextBotComponent
{
public:
	IBody( INextBot *bot, bool reg ) : INextBotComponent( bot, reg ) { }
	virtual ~IBody() {}

	virtual void Reset( void )  { INextBotComponent::Reset(); }			// reset to initial state
	virtual void Update( void )  {  }										// update internal state

	/**
	 * Move the bot to a new position.
	 * If the body is not currently movable or if it
	 * is in a motion-controlled animation activity 
	 * the position will not be changed and false will be returned.
	 */
	virtual bool SetPosition( const Vector &pos );

	virtual const Vector &GetEyePosition( void );
	virtual const Vector &GetViewVector( void );

	virtual void AimHeadTowards( const Vector &lookAtPos, 
								 LookAtPriorityType priority = BORING, 
								 float duration = 0.0f,
								 INextBotReply *replyWhenAimed = NULL,
								 const char *reason = NULL );
	virtual void AimHeadTowards( CBaseEntity *subject,
								 LookAtPriorityType priority = BORING, 
								 float duration = 0.0f,
								 INextBotReply *replyWhenAimed = NULL,
								 const char *reason = NULL );

	virtual bool IsHeadAimingOnTarget( void ) const { return false; }				// return true if the bot's head has achieved its most recent lookat target
	virtual bool IsHeadSteady( void ) const { return true; }						// return true if head is not rapidly turning to look somewhere else
	virtual float GetHeadSteadyDuration( void ) const { return 0.0f; }				// return the duration that the bot's head has not been rotating
	
#if SOURCE_ENGINE == SE_TF2
	virtual float GetHeadAimSubjectLeadTime( void ) const { return 0.0f; }			// return how far into the future we should predict our moving subject's position to aim at when tracking subject look-ats
	virtual float GetHeadAimTrackingInterval( void ) const { return 0.0f; }			// return how often we should sample our target's position and velocity to update our aim tracking, to allow realistic slop in tracking
	virtual void ClearPendingAimReply( void ) {}					// clear out currently pending replyWhenAimed callback
#endif
	
	virtual float GetMaxHeadAngularVelocity( void ) const { return 1000.0f; }			// return max turn rate of head in degrees/second

	/**
	 * Begin an animation activity, return false if we cant do that right now.
	 */
	virtual bool StartActivity( Activity act, unsigned int flags = 0 ) { return false; }
	virtual int SelectAnimationSequence( Activity act ) const { return -1; }			// given an Activity, select and return a specific animation sequence within it

	virtual Activity GetActivity( void ) const { return ACT_INVALID; }							// return currently animating activity
	virtual bool IsActivity( Activity act ) const { return GetActivity() == act; }						// return true if currently animating activity matches the given one
	virtual bool HasActivityType( unsigned int flags ) const { return false; }			// return true if currently animating activity has any of the given flags

	virtual void SetDesiredPosture( PostureType posture )  {}			// request a posture change
	virtual PostureType GetDesiredPosture( void ) const { return STAND; }				// get posture body is trying to assume
	virtual bool IsDesiredPosture( PostureType posture ) const { return GetDesiredPosture() == posture; }			// return true if body is trying to assume this posture
	virtual bool IsInDesiredPosture( void ) const { return GetActualPosture() == GetDesiredPosture(); }						// return true if body's actual posture matches its desired posture

	virtual PostureType GetActualPosture( void ) const { return STAND; }					// return body's current actual posture
	virtual bool IsActualPosture( PostureType posture ) const { return GetActualPosture() == posture; }			// return true if body is actually in the given posture

	virtual bool IsPostureMobile( void ) const
	{
		switch(GetActualPosture()) {
			case LIE:
			case SIT:
			{ return false; }
		}
		return true;
	}
	virtual bool IsPostureChanging( void ) const { return false; }						// return true if body's posture is in the process of changing to new posture
	
	
	/**
	 * "Arousal" is the level of excitedness/arousal/anxiety of the body.
	 * Is changes instantaneously to avoid complex interactions with posture transitions.
	 */
	virtual void SetArousal( ArousalType arousal ) {}					// arousal level change
	virtual ArousalType GetArousal( void ) const { return NEUTRAL; }						// get arousal level
	virtual bool IsArousal( ArousalType arousal ) const { return GetArousal() == arousal; }				// return true if body is at this arousal level


	virtual float GetHullWidth( void ) { return 26.0f; }							// width of bot's collision hull in XY plane
	virtual float GetHullHeight( void )
	{
		switch( GetActualPosture() )
		{
		case LIE:
			return 16.0f;

		case SIT:
		case CROUCH:
			return GetCrouchHullHeight();

		case STAND:
		default:
			return GetStandHullHeight();
		}
	}
	virtual float GetStandHullHeight( void ) { return 68.0f; }						// height of bot's collision hull when standing
	virtual float GetCrouchHullHeight( void ) { return 32.0f; }					// height of bot's collision hull when crouched
	virtual const Vector &GetHullMins( void )
	{
		static Vector hullMins;

		hullMins.x = -GetHullWidth()/2.0f;
		hullMins.y = hullMins.x;
		hullMins.z = 0.0f;

		return hullMins;
	}
	virtual const Vector &GetHullMaxs( void )
	{
		static Vector hullMaxs;

		hullMaxs.x = GetHullWidth()/2.0f;
		hullMaxs.y = hullMaxs.x;
		hullMaxs.z = GetHullHeight();

		return hullMaxs;
	}

	virtual unsigned int GetSolidMask( void ) { return MASK_NPCSOLID; }					// return the bot's collision mask (hack until we get a general hull trace abstraction here or in the locomotion interface)
	
#if SOURCE_ENGINE == SE_TF2
	virtual unsigned int GetCollisionGroup( void ) { return COLLISION_GROUP_NPC; }
#endif

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual CBaseEntity *GetEntity();
#endif
};

class INextBotEntityFilter
{
public:
	// return true if the given entity passes this filter
	virtual bool IsAllowed( CBaseEntity *entity ) = 0;
};

#if SOURCE_ENGINE == SE_TF2
class CKnownEntity;

class IForEachKnownEntity
{
public:
	virtual bool Inspect( const CKnownEntity &known ) = 0;
};

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
	
	CHandle< CBaseEntity > m_who;
	Vector m_lastKnownPostion;
	bool m_hasLastKnownPositionBeenSeen;
	CNavArea *m_lastKnownArea;
	float m_whenLastSeen;
	float m_whenLastBecameVisible;
	float m_whenLastKnown;			// last seen or heard, confirming its existance
	float m_whenBecameKnown;
	bool m_isVisible;				// flagged by IVision update as visible or not
};
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
class RecognizeInfo;
class RecognizedActor;
#endif

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
#if SOURCE_ENGINE == SE_TF2
	virtual bool ForEachKnownEntity( IForEachKnownEntity &func ) = 0;

	virtual void CollectKnownEntities( CUtlVector< CKnownEntity > *knownVector ) = 0;	// populate given vector with all currently known entities
#endif
	
#if SOURCE_ENGINE == SE_TF2
	virtual const CKnownEntity *GetPrimaryKnownThreat( bool onlyVisibleThreats = false ) const = 0;	// return the biggest threat to ourselves that we are aware of
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual CBaseEntity *GetPrimaryRecognizedThreat() const = 0;
#endif

	virtual float GetTimeSinceVisible( int team ) const = 0;				// return time since we saw any member of the given team

#if SOURCE_ENGINE == SE_TF2
	virtual const CKnownEntity *GetClosestKnown( int team = TEAM_ANY ) const = 0;	// return the closest known entity
	
	virtual int GetKnownCount( int team, bool onlyVisible = false, float rangeLimit = -1.0f ) const = 0;
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual CBaseEntity *GetClosestRecognized( int team = TEAM_ANY ) const = 0;
	
	virtual int GetRecognizedCount( int team, float rangeLimit = -1.0f ) const = 0;
#endif

#if SOURCE_ENGINE == SE_TF2
	virtual const CKnownEntity *GetClosestKnown( const INextBotEntityFilter &filter ) const = 0;	// return the closest known entity that passes the given filter

	virtual const CKnownEntity *GetKnown( const CBaseEntity *entity ) const = 0;		// given an entity, return our known version of it (or NULL if we don't know of it)
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual CBaseEntity *GetClosestRecognized( const INextBotEntityFilter &filter ) const = 0;
#endif
	
	// Introduce a known entity into the system. Its position is assumed to be known
	// and will be updated, and it is assumed to not yet have been seen by us, allowing for learning
	// of known entities by being told about them, hearing them, etc.
#if SOURCE_ENGINE == SE_TF2
	virtual void AddKnownEntity( CBaseEntity *entity ) = 0;

	virtual void ForgetEntity( CBaseEntity *forgetMe ) = 0;			// remove the given entity from our awareness (whether we know if it or not)
	virtual void ForgetAllKnownEntities( void ) = 0;

	//-- physical vision interface follows ------------------------------------------------------

	/**
	 * Populate "potentiallyVisible" with the set of all entities we could potentially see. 
	 * Entities in this set will be tested for visibility/recognition in IVision::Update()
	 */
	virtual void CollectPotentiallyVisibleEntities( CUtlVector< CBaseEntity * > *potentiallyVisible ) = 0;
#endif
	
	virtual float GetMaxVisionRange( void ) = 0;				// return maximum distance vision can reach
	virtual float GetMinRecognizeTime( void ) = 0;			// return VISUAL reaction time

	/**
	 * IsAbleToSee() returns true if the viewer can ACTUALLY SEE the subject or position,
	 * taking into account blindness, smoke effects, invisibility, etc.
	 * If 'visibleSpot' is non-NULL, the highest priority spot on the subject that is visible is returned.
	 */ 
	virtual bool IsAbleToSee( CBaseEntity *subject, FieldOfViewCheckType checkFOV, Vector *visibleSpot = NULL ) const = 0;
	virtual bool IsAbleToSee( const Vector &pos, FieldOfViewCheckType checkFOV ) const = 0;

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual bool IsNoticed(CBaseEntity *) = 0;
#endif
	
	virtual bool IsIgnored( CBaseEntity *subject ) const = 0;		// return true to completely ignore this entity (may not be in sight when this is called)
	
#if SOURCE_ENGINE == SE_TF2
	virtual bool IsVisibleEntityNoticed( CBaseEntity *subject ) const = 0;		// return true if we 'notice' the subject, even though we have LOS to it
#endif
	
	/**
	 * Check if 'subject' is within the viewer's field of view
	 */
	virtual bool IsInFieldOfView( const Vector &pos ) const = 0;
	virtual bool IsInFieldOfView( CBaseEntity *subject ) const = 0;
	virtual float GetDefaultFieldOfView( void ) = 0;			// return default FOV in degrees
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
	
#if SOURCE_ENGINE == SE_TF2
	CountdownTimer m_scanTimer;			// for throttling update rate
	
	float m_FOV;						// current FOV in degrees
	float m_cosHalfFOV;					// the cosine of FOV/2
	
	CUtlVector< CKnownEntity > m_knownEntityVector;		// the set of enemies/friends we are aware of
	mutable CHandle< CBaseEntity > m_primaryThreat;

	float m_lastVisionUpdateTimestamp;
	IntervalTimer m_notVisibleTimer[ MAX_TEAMS ];		// for tracking interval since last saw a member of the given team
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	char pad1[308];
#endif
};

enum QueryResultType
{
	ANSWER_NO,
	ANSWER_YES,
	ANSWER_UNDEFINED
};

#if SOURCE_ENGINE == SE_TF2
class CKnownEntity;
#endif

class IContextualQuery
{
public:
	virtual ~IContextualQuery() {}

	virtual QueryResultType			ShouldPickUp( const INextBot *me, CBaseEntity *item ) const { return ANSWER_UNDEFINED; }
	virtual QueryResultType			ShouldHurry( const INextBot *me ) const { return ANSWER_UNDEFINED; }
#if SOURCE_ENGINE == SE_TF2
	virtual QueryResultType			ShouldRetreat( const INextBot *me ) const { return ANSWER_UNDEFINED; }
	virtual QueryResultType			ShouldAttack( const INextBot *me, const CKnownEntity *them ) const { return ANSWER_UNDEFINED; }
#endif
	virtual QueryResultType			IsHindrance( const INextBot *me, CBaseEntity *blocker ) const { return ANSWER_UNDEFINED; }

	virtual Vector					SelectTargetPoint( const INextBot *me, CBaseCombatCharacter *subject ) const { return vec3_origin; }

	/**
	 * Allow bot to approve of positions game movement tries to put him into.
	 * This is most useful for bots derived from CBasePlayer that go through
	 * the player movement system.
	 */
	virtual QueryResultType IsPositionAllowed( const INextBot *me, const Vector &pos ) const { return ANSWER_UNDEFINED; }

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual PathFollower * QueryCurrentPath( const INextBot * ) const { return NULL; }
#endif
	
#if SOURCE_ENGINE == SE_TF2
	virtual const CKnownEntity *	SelectMoreDangerousThreat( const INextBot *me, 
															   CBaseCombatCharacter *subject,
															   const CKnownEntity *threat1, 
															   const CKnownEntity *threat2 ) const { return NULL; }
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual CBaseCombatCharacter *	SelectMoreDangerousThreat( const INextBot *me, 
															   CBaseCombatCharacter *subject,
															   CBaseCombatCharacter *threat1, 
															   CBaseCombatCharacter *threat2 ) const { return NULL; }
#endif
};

class IIntention : public INextBotComponent, public IContextualQuery
{
public:
	IIntention( INextBot *bot, bool reg ) : INextBotComponent( bot, reg ) { }
	
	virtual QueryResultType			ShouldPickUp( const INextBot *me, CBaseEntity *item ) const
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				QueryResultType result = query->ShouldPickUp( me, item );
				if ( result != ANSWER_UNDEFINED )
				{
					return result;
				}
			}
		}	
		return ANSWER_UNDEFINED;
	}
	virtual QueryResultType			ShouldHurry( const INextBot *me ) const
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				QueryResultType result = query->ShouldHurry( me );
				if ( result != ANSWER_UNDEFINED )
				{
					return result;
				}
			}
		}	
		return ANSWER_UNDEFINED;
	}
#if SOURCE_ENGINE == SE_TF2
	virtual QueryResultType			ShouldRetreat( const INextBot *me ) const
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				QueryResultType result = query->ShouldRetreat( me );
				if ( result != ANSWER_UNDEFINED )
				{
					return result;
				}
			}
		}	
		return ANSWER_UNDEFINED;
	}
	virtual QueryResultType			ShouldAttack( const INextBot *me, const CKnownEntity *them ) const
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				QueryResultType result = query->ShouldAttack( me, them );
				if ( result != ANSWER_UNDEFINED )
				{
					return result;
				}
			}
		}	
		return ANSWER_UNDEFINED;
	}
#endif
	virtual QueryResultType			IsHindrance( const INextBot *me, CBaseEntity *blocker ) const
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				QueryResultType result = query->IsHindrance( me, blocker );
				if ( result != ANSWER_UNDEFINED )
				{
					return result;
				}
			}
		}	
		return ANSWER_UNDEFINED;
	}

	virtual Vector					SelectTargetPoint( const INextBot *me, CBaseCombatCharacter *subject ) const
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				Vector result = query->SelectTargetPoint( me, subject );
				if ( result != vec3_origin )
				{
					return result;
				}
			}
		}	

		// no answer, use a reasonable position
		Vector threatMins, threatMaxs;
		subject->CollisionProp()->WorldSpaceAABB( &threatMins, &threatMaxs );
		Vector targetPoint = subject->GetAbsOrigin();
		targetPoint.z += 0.7f * ( threatMaxs.z - threatMins.z );

		return targetPoint;
	}

	/**
	 * Allow bot to approve of positions game movement tries to put him into.
	 * This is most useful for bots derived from CBasePlayer that go through
	 * the player movement system.
	 */
	virtual QueryResultType IsPositionAllowed( const INextBot *me, const Vector &pos ) const
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				QueryResultType result = query->IsPositionAllowed( me, pos );
				if ( result != ANSWER_UNDEFINED )
				{
					return result;
				}
			}
		}	
		return ANSWER_UNDEFINED;
	}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual PathFollower * QueryCurrentPath( const INextBot *me ) const
	{
		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				PathFollower *result = query->QueryCurrentPath( me );
				if ( result )
				{
					return result;
				}
			}
		}
		
		return nullptr;
	}
#endif
	
#if SOURCE_ENGINE == SE_TF2
	virtual const CKnownEntity *	SelectMoreDangerousThreat( const INextBot *me, 
															   CBaseCombatCharacter *subject,
															   const CKnownEntity *threat1, 
															   const CKnownEntity *threat2 ) const
	{
		if ( !threat1 || threat1->IsObsolete() )
		{
			if ( threat2 && !threat2->IsObsolete() )
				return threat2;

			return NULL;
		}
		else if ( !threat2 || threat2->IsObsolete() )
		{
			return threat1;
		}

		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				const CKnownEntity *result = query->SelectMoreDangerousThreat( me, subject, threat1, threat2 );
				if ( result )
				{
					return result;
				}
			}
		}

		// no specific decision was made - return closest threat as most dangerous
		float range1 = ( subject->GetAbsOrigin() - threat1->GetLastKnownPosition() ).LengthSqr();
		float range2 = ( subject->GetAbsOrigin() - threat2->GetLastKnownPosition() ).LengthSqr();

		if ( range1 < range2 )
		{
			return threat1;
		}

		return threat2;
	}
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual CBaseCombatCharacter *	SelectMoreDangerousThreat( const INextBot *me, 
															   CBaseCombatCharacter *subject,
															   CBaseCombatCharacter *threat1, 
															   CBaseCombatCharacter *threat2 ) const
	{
		if ( !threat1 )
		{
			if ( threat2 )
				return threat2;

			return NULL;
		}
		else if ( !threat2 )
		{
			return threat1;
		}

		for ( INextBotEventResponder *sub = FirstContainedResponder(); sub; sub = NextContainedResponder( sub ) )
		{
			const IContextualQuery *query = dynamic_cast< const IContextualQuery * >( sub );
			if ( query )
			{
				// return the response of the first responder that gives a definitive answer
				CBaseCombatCharacter *result = query->SelectMoreDangerousThreat( me, subject, threat1, threat2 );
				if ( result )
				{
					return result;
				}
			}
		}

		// no specific decision was made - return closest threat as most dangerous
		float range1 = ( subject->GetAbsOrigin() - threat1->GetAbsOrigin() ).LengthSqr();
		float range2 = ( subject->GetAbsOrigin() - threat2->GetAbsOrigin() ).LengthSqr();

		if ( range1 < range2 )
		{
			return threat1;
		}

		return threat2;
	}
#endif
};

class IIntentionStub : public IIntention
{
public:
	void Update() override
	{
	}

	virtual const char *GetDebugString() const { return "IIntentionStub"; }
	
	IIntentionStub( INextBot *bot, bool reg ) : IIntention( bot, reg ) {

	}
	
	static IIntentionStub *create(INextBot *bot, bool reg, IdentityToken_t *)
	{
		return new IIntentionStub(bot, reg);
	}
};

ConVar *NextBotDebugHistory = nullptr;

enum NextBotDebugType 
{
	NEXTBOT_DEBUG_NONE = 0,
	NEXTBOT_BEHAVIOR	= 0x0001,
	NEXTBOT_LOOK_AT		= 0x0002,
	NEXTBOT_PATH		= 0x0004,
	NEXTBOT_ANIMATION	= 0x0008,
	NEXTBOT_LOCOMOTION	= 0x0010,
	NEXTBOT_VISION		= 0x0020,
	NEXTBOT_HEARING		= 0x0040,
	NEXTBOT_EVENTS		= 0x0080,
	NEXTBOT_ERRORS		= 0x0100,		// when things go wrong, like being stuck

	NEXTBOT_DEBUG_ALL	= 0xFFFF
};

class INextBot : public INextBotEventResponder
{
public:
	virtual void Reset( void ) = 0;										// (EXTEND) reset to initial state
	virtual void Update( void ) = 0;									// (EXTEND) update internal state
	virtual void Upkeep( void ) = 0;									// (EXTEND) lightweight update guaranteed to occur every server tick
	
	virtual bool IsRemovedOnReset( void ) const = 0;	// remove this bot when the NextBot manager calls Reset
	
	virtual CBaseCombatCharacter *GetEntity( void )	= 0;
	virtual class NextBotCombatCharacter *GetNextBotCombatCharacter( void ) = 0;
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual class SurvivorBot *MySurvivorBotPointer() const { return NULL; }
#endif
	
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
	
	virtual bool IsEnemy( CBaseEntity *them ) const = 0;			// return true if given entity is our enemy
	virtual bool IsFriend( CBaseEntity *them ) const = 0;			// return true if given entity is our friend
	virtual bool IsSelf( CBaseEntity *them ) const = 0;			// return true if 'them' is actually me

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual bool IsAllowedToClimb() const = 0;
#endif
	
	/**
	 * Can we climb onto this entity?
	 */	
	virtual bool IsAbleToClimbOnto( CBaseEntity *object ) const = 0;

	/**
	 * Can we break this entity?
	 */	
	virtual bool IsAbleToBreak( CBaseEntity *object ) const = 0;

	/**
	 * Sometimes we want to pass through other NextBots. OnContact() will always
	 * be invoked, but collision resolution can be skipped if this
	 * method returns false.
	 */
	virtual bool IsAbleToBlockMovementOf( INextBot *botInMotion ) const = 0;

	/**
	 * Should we ever care about noticing physical contact with this entity?
	 */
	virtual bool ShouldTouch( CBaseEntity *object ) const = 0;

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual bool ReactToSurvivorVisibility() const = 0;
	virtual bool ReactToSurvivorNoise() const = 0;
	virtual bool ReactToSurvivorContact() const = 0;
#endif
	
	/**
	 * This immobile system is used to track the global state of "am I actually moving or not".
	 * The OnStuck() event is only emitted when following a path, and paths can be recomputed, etc.
	 */
	virtual bool IsImmobile( void ) const = 0;					// return true if we haven't moved in awhile
	virtual float GetImmobileDuration( void ) const = 0;		// how long have we been immobile
	virtual void ClearImmobileStatus( void ) = 0;		
	virtual float GetImmobileSpeedThreshold( void ) const = 0;	// return units/second below which this actor is considered "immobile"

	/**
	 * Get the last PathFollower we followed. This method gives other interfaces a
	 * single accessor to the most recent Path being followed by the myriad of 
	 * different PathFollowers used in the various behaviors the bot may be doing.
	 */
#if SOURCE_ENGINE == SE_TF2
	virtual const PathFollower *GetCurrentPath( void ) const = 0;
	virtual void SetCurrentPath( const PathFollower *path ) = 0;
	virtual void NotifyPathDestruction( const PathFollower *path ) = 0;		// this PathFollower is going away, which may or may not be ours
#endif
	
	// between distance utility methods
	virtual bool IsRangeLessThan( CBaseEntity *subject, float range ) const = 0;
	virtual bool IsRangeLessThan( const Vector &pos, float range ) const = 0;
	virtual bool IsRangeGreaterThan( CBaseEntity *subject, float range ) const = 0;
	virtual bool IsRangeGreaterThan( const Vector &pos, float range ) const = 0;
	virtual float GetRangeTo( CBaseEntity *subject ) const = 0;
	virtual float GetRangeTo( const Vector &pos ) const = 0;
	virtual float GetRangeSquaredTo( CBaseEntity *subject ) const = 0;
	virtual float GetRangeSquaredTo( const Vector &pos ) const = 0;

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual float Get2DRangeTo( CBaseEntity *subject ) const = 0;
	virtual float Get2DRangeTo( const Vector &pos ) const = 0;
#endif
	
#if SOURCE_ENGINE == SE_TF2
	virtual bool IsDebugging( unsigned int type ) const = 0;		// return true if this bot is debugging any of the given types
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	bool IsDebugging( unsigned int type ) const
	{
		return call_mfunc<bool, INextBot, unsigned int>(this, INextBotIsDebuggingPtr, type);
	}
#endif

	virtual const char *GetDebugIdentifier( void ) const = 0;		// return the name of this bot for debugging purposes
	virtual bool IsDebugFilterMatch( const char *name ) const = 0;	// return true if we match the given debug symbol
	virtual void DisplayDebugText( const char *text ) const = 0;
	
	bool BeginUpdate()
	{
		return call_mfunc<bool>(this, INextBotBeginUpdatePtr);
	}
	
	void EndUpdate()
	{
		call_mfunc<void>(this, INextBotEndUpdatePtr);
	}
	
	void RegisterComponent( INextBotComponent *comp )
	{
		comp->m_nextComponent = m_componentList;
		m_componentList = comp;
	}
	
	void UnregisterComponent( INextBotComponent *comp )
	{
		if(!comp) {
			return;
		}
		
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
		bool found = false;
		
		if(whom) {
			for(INextBotComponent *cur = m_componentList, *prev = nullptr; cur != nullptr; prev = cur, cur = cur->m_nextComponent) {
				if (cur == whom) {
					with->m_nextComponent = cur->m_nextComponent;
					if(prev != nullptr) {
						prev->m_nextComponent = with;
					} else {
						m_componentList = with;
					}
					found = true;
					break;
				}
			}
		}
		
		if(!found) {
			RegisterComponent(with);
		}
	}
	
	#define MAX_NEXTBOT_DEBUG_HISTORY 100
	#define MAX_NEXTBOT_DEBUG_LINE_LENGTH 256

	struct NextBotDebugLineType
	{
		NextBotDebugType debugType;
		char data[ MAX_NEXTBOT_DEBUG_LINE_LENGTH ];
	};
	
	INextBotComponent *m_componentList;						// the first component

#if SOURCE_ENGINE == SE_TF2
	const PathFollower *m_currentPath;						// the path we most recently followed
#endif
	
	int m_id;
	bool m_bFlaggedForUpdate;
	int m_tickLastUpdate;

	unsigned int m_debugType;
	mutable int m_debugDisplayLine;

	Vector m_immobileAnchor;
	CountdownTimer m_immobileCheckTimer;
	IntervalTimer m_immobileTimer;
	
#if SOURCE_ENGINE == SE_TF2
	mutable ILocomotion *m_baseLocomotion;
	
	mutable IBody		*m_baseBody;
	
	mutable IIntention	*m_baseIntention;
	
	mutable IVision		*m_baseVision;
#else
	char pad3[sizeof(ILocomotion)];
	char pad4[sizeof(IBody)];
	char pad5[sizeof(IIntention)];
	char pad6[sizeof(IVision)];
	
	ILocomotion &baseLocomotion()
	{
		return (ILocomotion &)pad3;
	}
	
	IBody &baseBody()
	{
		return (IBody &)pad4;
	}
	
	IIntention &baseIntention()
	{
		return (IIntention &)pad5;
	}
	
	IVision &baseVision()
	{
		return (IVision &)pad6;
	}
#endif
	
	CUtlVector< NextBotDebugLineType * > m_debugHistory;
};

void IBody::AimHeadTowards( const Vector &lookAtPos, 
								LookAtPriorityType priority, 
								float duration,
								INextBotReply *replyWhenAimed,
								const char *reason )
{
	INextBot *bot = GetBot();
	
	if ( replyWhenAimed )
	{
		replyWhenAimed->OnFail( bot, INextBotReply::FAILED );
	}
	
	if(bot->IsDebugging(NEXTBOT_LOOK_AT)) {
		DevMsg("%f: IBody->AimHeadTowards [%f, %f, %f]: %s\n", gpGlobals->curtime, lookAtPos.x, lookAtPos.y, lookAtPos.z, reason);
	}
}
void IBody::AimHeadTowards( CBaseEntity *subject,
								LookAtPriorityType priority, 
								float duration,
								INextBotReply *replyWhenAimed,
								const char *reason )
{
	INextBot *bot = GetBot();
	
	if ( replyWhenAimed )
	{
		replyWhenAimed->OnFail( bot, INextBotReply::FAILED );
	}
	
	if(bot->IsDebugging(NEXTBOT_LOOK_AT)) {
		DevMsg("%f: IBody->AimHeadTowards #%i: %s\n", gpGlobals->curtime, subject->entindex(), reason);
	}
}

using NextBotDebugLineType = INextBot::NextBotDebugLineType;

ConVar *developer = nullptr;
ConVar *r_visualizetraces = nullptr;

SH_DECL_MANUALHOOK0_void(GenericDtor, 1, 0, 0);
SH_DECL_MANUALHOOK0_void(UpdateOnRemove, 0, 0, 0);

class NextBotCombatCharacter : public CBaseCombatCharacter
{
public:
	void ResetDebugHistory()
	{
		CUtlVector< NextBotDebugLineType * > &m_debugHistory = MyNextBotPointer()->m_debugHistory;
		
		for ( int i=0; i<m_debugHistory.Count(); ++i )
		{
			delete m_debugHistory[i];
		}

		m_debugHistory.RemoveAll();
	}
	
	void DebugConColorMsg( NextBotDebugType debugType, const Color &color, const char *fmt, ... )
	{
		bool isDataFormatted = false;

		va_list argptr;
		char data[ MAX_NEXTBOT_DEBUG_LINE_LENGTH ];

		if ( developer->GetBool() && IsDebugging( debugType ) )
		{
			va_start(argptr, fmt);
			Q_vsnprintf(data, sizeof( data ), fmt, argptr);
			va_end(argptr);
			isDataFormatted = true;

			ConColorMsg( color, "%s", data );
		}
		
		CUtlVector< NextBotDebugLineType * > &m_debugHistory = MyNextBotPointer()->m_debugHistory;

		if ( !NextBotDebugHistory->GetBool() )
		{
			if ( m_debugHistory.Count() )
			{
				ResetDebugHistory();
			}
			return;
		}

		// Don't bother with event data - it's spammy enough to overshadow everything else.
		if ( debugType == NEXTBOT_EVENTS )
			return;

		if ( !isDataFormatted )
		{
			va_start(argptr, fmt);
			Q_vsnprintf(data, sizeof( data ), fmt, argptr);
			va_end(argptr);
			isDataFormatted = true;
		}

		int lastLine = m_debugHistory.Count() - 1;
		if ( lastLine >= 0 )
		{
			NextBotDebugLineType *line = m_debugHistory[lastLine];
			if ( line->debugType == debugType && V_strstr( line->data, "\n" ) == NULL )
			{
				// append onto previous line
				V_strncat( line->data, data, MAX_NEXTBOT_DEBUG_LINE_LENGTH );
				return;
			}
		}

		// Prune out an old line if needed, keeping a pointer to re-use the memory
		NextBotDebugLineType *line = NULL;
		if ( m_debugHistory.Count() == MAX_NEXTBOT_DEBUG_HISTORY )
		{
			line = m_debugHistory[0];
			m_debugHistory.Remove( 0 );
		}

		// Add to debug history
		if ( !line )
		{
			line = new NextBotDebugLineType;
		}
		line->debugType = debugType;
		V_strncpy( line->data, data, MAX_NEXTBOT_DEBUG_LINE_LENGTH );
		m_debugHistory.AddToTail( line );
	}
	
	const char *GetDebugIdentifier( void ) const
	{
		return const_cast<NextBotCombatCharacter *>(this)->MyNextBotPointer()->GetDebugIdentifier();
	}
	
	bool IsDebugging( unsigned int type )
	{
		return MyNextBotPointer()->IsDebugging(type);
	}
	
	void DisplayDebugText( const char *text )
	{
		MyNextBotPointer()->DisplayDebugText(text);
	}

	static NextBotCombatCharacter *create(size_t size_modifier)
	{
		NextBotCombatCharacter *bytes = (NextBotCombatCharacter *)engine->PvAllocEntPrivateData(sizeofNextBotCombatCharacter + size_modifier);
		call_mfunc<void>(bytes, NextBotCombatCharacterCTOR);
		return bytes;
	}
};

#include <sourcehook/sh_memory.h>

#include "NextBotBehavior.h"

#ifdef __HAS_DAMAGERULES
IDamageRules *g_pDamageRules = nullptr;
#endif

SH_DECL_MANUALHOOK0(Classify, 0, 0, 0, Class_T)

#if SOURCE_ENGINE == SE_LEFT4DEAD2
SH_DECL_MANUALHOOK0(MyInfectedPointer, 0, 0, 0, CBaseEntity *)
SH_DECL_MANUALHOOK0(GetClass, 0, 0, 0, int)
SH_DECL_MANUALHOOK1(CanBeA, 0, 0, 0, bool, int)
SH_DECL_HOOK0(CBaseEntity, GetDataDescMap, SH_NOATTRIB, 0, datamap_t *);

namespace __cxxabiv1
{
	struct vtable_prefix 
	{
		// Offset to most derived object.
		ptrdiff_t whole_object;

		// Additional padding if necessary.
#ifdef _GLIBCXX_VTABLE_PADDING
		ptrdiff_t padding1;
#endif

		// Pointer to most derived type_info.
		const __class_type_info *whole_type;

		// Additional padding if necessary.
#ifdef _GLIBCXX_VTABLE_PADDING
		ptrdiff_t padding2;
#endif

		// What a class's vptr points to.
		const void *origin;
	};
}

void **infectedvtable = nullptr;

class NextBotCombatCharacterInfected;

std::unordered_map<NextBotCombatCharacterInfected *, int> nbinfectclsmap{};

class NextBotCombatCharacterInfected : public NextBotCombatCharacter
{
public:
	void HookDoBloodEffect(float, CTakeDamageInfo const&, Vector const&, CGameTrace*)
	{
		
	}
	
	bool HookIsSacrificeFor(int) const
	{
		return false;
	}
	
	bool HookTryToCull()
	{
		return false;
	}
	
	Action<NextBotCombatCharacterInfected> *HookCreateDeathAction(CTakeDamageInfo const&)
	{
		return nullptr;
	}
	
	float HookGetEyeOffsetUpdateInterval() const
	{
		return 0.0f;
	}
	
	void HookMakeLowViolence()
	{
		
	}
	
	void HookCreateComponents()
	{
		
	}
	
	void HookUpdate()
	{ MyNextBotPointer()->Update(); }
	
	void HookUpkeep()
	{ MyNextBotPointer()->Upkeep(); }
	
	bool HookReactToSurvivorVisibility()
	{ return MyNextBotPointer()->ReactToSurvivorVisibility(); }
	
	bool HookReactToSurvivorNoise()
	{ return MyNextBotPointer()->ReactToSurvivorNoise(); }
	
	bool HookReactToSurvivorContact()
	{ return MyNextBotPointer()->ReactToSurvivorContact(); }
	
	ILocomotion *HookGetLocomotionInterface()
	{ return MyNextBotPointer()->GetLocomotionInterface(); }
	
	IBody *HookGetBodyInterface()
	{ return MyNextBotPointer()->GetBodyInterface(); }
	
	IIntention *HookGetIntentionInterface()
	{ return MyNextBotPointer()->GetIntentionInterface(); }
	
	IVision *HookGetVisionInterface()
	{ return MyNextBotPointer()->GetVisionInterface(); }
	
	void HookOnIgnite()
	{ return MyNextBotPointer()->OnIgnite(); }
	
	bool HookShouldTouch(CBaseEntity *object)
	{ return MyNextBotPointer()->ShouldTouch(object); }
	
	bool HookIsAbleToClimbOnto(CBaseEntity *object)
	{ return MyNextBotPointer()->IsAbleToClimbOnto(object); }
	
	bool HookIsAbleToBreak(CBaseEntity *object)
	{ return MyNextBotPointer()->IsAbleToBreak(object); }
	
	bool HookIsAbleToBlockMovementOf(INextBot *object)
	{ return MyNextBotPointer()->IsAbleToBlockMovementOf(object); }
	
	CBaseEntity *HookMyInfectedPointer()
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, this);
	}
	
	int GetDesiredClass()
	{
		return nbinfectclsmap[this];
	}
	
	#define NBINFECT_MAGIC_NUMBER 69
	
	bool HookCanBeA(int cls)
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, cls == GetDesiredClass() || cls == NBINFECT_MAGIC_NUMBER);
	}
	
	int HookGetClass()
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, GetDesiredClass());
	}
	
	Class_T HookClassify()
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, 4);
	}
	
	void dtor()
	{
		NextBotCombatCharacterInfected *pEntity = META_IFACEPTR(NextBotCombatCharacterInfected);
		nbinfectclsmap.erase(this);
		SH_REMOVE_MANUALHOOK(GenericDtor, pEntity, SH_MEMBER(this, &NextBotCombatCharacterInfected::dtor), false);
		SH_REMOVE_MANUALHOOK(MyInfectedPointer, pEntity, SH_MEMBER(this, &NextBotCombatCharacterInfected::HookMyInfectedPointer), false);
		SH_REMOVE_MANUALHOOK(CanBeA, pEntity, SH_MEMBER(this, &NextBotCombatCharacterInfected::HookCanBeA), false);
		SH_REMOVE_MANUALHOOK(GetClass, pEntity, SH_MEMBER(this, &NextBotCombatCharacterInfected::HookGetClass), false);
		SH_REMOVE_MANUALHOOK(Classify, pEntity, SH_MEMBER(this, &NextBotCombatCharacterInfected::HookClassify), false);
		RETURN_META(MRES_IGNORED);
	}
	
	static NextBotCombatCharacter *create(size_t size_modifier, int cls)
	{
		NextBotCombatCharacterInfected *bytes = (NextBotCombatCharacterInfected *)engine->PvAllocEntPrivateData(sizeofInfected + size_modifier);
		call_mfunc<void>(bytes, NextBotCombatCharacterCTOR);
		
		static void **fakeinfectvtbl = nullptr;
		if(!fakeinfectvtbl) {
			void *infectfuncs[] =
			{
				func_to_void(&NextBotCombatCharacterInfected::HookUpdate),
				func_to_void(&NextBotCombatCharacterInfected::HookUpkeep),
				func_to_void(&NextBotCombatCharacterInfected::HookDoBloodEffect),
				func_to_void(&NextBotCombatCharacterInfected::HookReactToSurvivorVisibility),
				func_to_void(&NextBotCombatCharacterInfected::HookReactToSurvivorNoise),
				func_to_void(&NextBotCombatCharacterInfected::HookReactToSurvivorContact),
				func_to_void(&NextBotCombatCharacterInfected::HookGetLocomotionInterface),
				func_to_void(&NextBotCombatCharacterInfected::HookGetBodyInterface),
				func_to_void(&NextBotCombatCharacterInfected::HookGetIntentionInterface),
				func_to_void(&NextBotCombatCharacterInfected::HookGetVisionInterface),
				func_to_void(&NextBotCombatCharacterInfected::HookOnIgnite),
				func_to_void(&NextBotCombatCharacterInfected::HookIsAbleToClimbOnto),
				func_to_void(&NextBotCombatCharacterInfected::HookIsAbleToBreak),
				func_to_void(&NextBotCombatCharacterInfected::HookIsAbleToBlockMovementOf),
				func_to_void(&NextBotCombatCharacterInfected::HookShouldTouch),
				func_to_void(&NextBotCombatCharacterInfected::HookIsSacrificeFor),
				func_to_void(&NextBotCombatCharacterInfected::HookTryToCull),
				func_to_void(&NextBotCombatCharacterInfected::HookCreateDeathAction),
				func_to_void(&NextBotCombatCharacterInfected::HookGetEyeOffsetUpdateInterval),
				func_to_void(&NextBotCombatCharacterInfected::HookMakeLowViolence),
				func_to_void(&NextBotCombatCharacterInfected::HookCreateComponents),
			};
			int infectfuncsnum = (sizeof(infectfuncs) / 4);
			
			using vtable_prefix = __cxxabiv1::vtable_prefix;
			
			unsigned char *tablememory = (unsigned char *)calloc(1, sizeof(vtable_prefix) + (((NextBotCombatCharacterGetNextBotCombatCharacter + infectfuncsnum) + 1) * 4));
			
			void **nbcbvtable = *(void ***)bytes;
			
			vtable_prefix &prefix = *(vtable_prefix *)tablememory;
			if(infectedvtable) {
				prefix = *(vtable_prefix *)(((unsigned char *)infectedvtable) - sizeof(vtable_prefix));
			} else {
				prefix = *(vtable_prefix *)(((unsigned char *)nbcbvtable) - sizeof(vtable_prefix));
			}
			
			fakeinfectvtbl = (void **)&tablememory[sizeof(vtable_prefix)];
			
			int i = 0;
			for(; i <= NextBotCombatCharacterGetNextBotCombatCharacter; ++i) {
				fakeinfectvtbl[i] = nbcbvtable[i];
			}
			for(int j = 0; j < infectfuncsnum; ++j) {
				fakeinfectvtbl[i++] = infectfuncs[j];
			}
		}
		
		nbinfectclsmap.emplace(std::pair<NextBotCombatCharacterInfected *, int>{bytes, cls});
		
		(*(void ***)bytes) = fakeinfectvtbl;
		
		SH_ADD_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(bytes, &NextBotCombatCharacterInfected::dtor), false);
		SH_ADD_MANUALHOOK(MyInfectedPointer, bytes, SH_MEMBER(bytes, &NextBotCombatCharacterInfected::HookMyInfectedPointer), false);
		SH_ADD_MANUALHOOK(CanBeA, bytes, SH_MEMBER(bytes, &NextBotCombatCharacterInfected::HookCanBeA), false);
		SH_ADD_MANUALHOOK(GetClass, bytes, SH_MEMBER(bytes, &NextBotCombatCharacterInfected::HookGetClass), false);
		SH_ADD_MANUALHOOK(Classify, bytes, SH_MEMBER(bytes, &NextBotCombatCharacterInfected::HookClassify), false);
		
		return bytes;
	}
};
#endif

HandleType_t BehaviorEntryHandleType = 0;

using SPActor = NextBotCombatCharacter;

class SPActionResult : public ActionResult<SPActor>
{
public:
	using BaseClass = ActionResult<SPActor>;
	
	void set_reason(std::string &&reason_)
	{
		m_reason = std::move(reason_);
	}
};

class SPEventDesiredResult : public EventDesiredResult<SPActor>
{
public:
	using BaseClass = EventDesiredResult<SPActor>;
	
	
};

using SPBehavior = Behavior<SPActor>;

class SPAction;

struct SPActionEntry
{
	SPActionEntry(std::string &&name_, IdentityToken_t *id)
		: name{std::move(name_)}, pId{id}
	{
	}
	
	std::string name;
	
	IPluginFunction *onstart = nullptr;
	IPluginFunction *onupdate = nullptr;
	IPluginFunction *onsus = nullptr;
	IPluginFunction *onresume = nullptr;
	IPluginFunction *onend = nullptr;
	IPluginFunction *intialact = nullptr;
	
	IPluginFunction *leavegrnd = nullptr;
	IPluginFunction *landgrnd = nullptr;
	IPluginFunction *oncontact = nullptr;
	IPluginFunction *animcompl = nullptr;
	IPluginFunction *animinter = nullptr;
	IPluginFunction *animevent = nullptr;
	IPluginFunction *otherkilled = nullptr;
	IPluginFunction *onsight = nullptr;
	IPluginFunction *onlosesight = nullptr;
	IPluginFunction *shoved = nullptr;
	IPluginFunction *blinded = nullptr;
	IPluginFunction *terrcontest = nullptr;
	IPluginFunction *terrcap = nullptr;
	IPluginFunction *terrlost = nullptr;
	IPluginFunction *threachngd = nullptr;
	IPluginFunction *hitvom = nullptr;
	IPluginFunction *drop = nullptr;
	IPluginFunction *movesucc = nullptr;
	IPluginFunction *stuck = nullptr;
	IPluginFunction *unstuck = nullptr;
	IPluginFunction *ignite = nullptr;
	IPluginFunction *injured = nullptr;
	IPluginFunction *killed = nullptr;
	IPluginFunction *win = nullptr;
	IPluginFunction *lose = nullptr;
	IPluginFunction *enterspit = nullptr;
	IPluginFunction *mdlchnd = nullptr;
	IPluginFunction *movefail = nullptr;
	IPluginFunction *sound = nullptr;
	IPluginFunction *wepfired = nullptr;
	IPluginFunction *actemote = nullptr;
	IPluginFunction *pickup = nullptr;
	
	IdentityToken_t *pId = nullptr;
	
	bool set_function(std::string_view name, IPluginFunction *func)
	{
		if(name == "OnStart"sv) {
			onstart = func;
			return true;
		} else if(name == "Update"sv) {
			onupdate = func;
			return true;
		} else if(name == "OnSuspend"sv) {
			onsus = func;
			return true;
		} else if(name == "OnResume"sv) {
			onresume = func;
			return true;
		} else if(name == "OnEnd"sv) {
			onend = func;
			return true;
		} else if(name == "OnEnd"sv) {
			onend = func;
			return true;
		} else if(name == "InitialContainedAction"sv) {
			intialact = func;
			return true;
		} else if(name == "OnLandOnGround"sv) {
			landgrnd = func;
			return true;
		} else if(name == "OnContact"sv) {
			oncontact = func;
			return true;
		} else if(name == "OnAnimationActivityComplete"sv) {
			animcompl = func;
			return true;
		} else if(name == "OnAnimationActivityInterrupted"sv) {
			animinter = func;
			return true;
		} else if(name == "OnAnimationEvent"sv) {
			animevent = func;
			return true;
		} else if(name == "OnOtherKilled"sv) {
			otherkilled = func;
			return true;
		} else if(name == "OnSight"sv) {
			onsight = func;
			return true;
		} else if(name == "OnLostSight"sv) {
			onlosesight = func;
			return true;
		} else if(name == "OnShoved"sv) {
			shoved = func;
			return true;
		} else if(name == "OnBlinded"sv) {
			blinded = func;
			return true;
		} else if(name == "OnTerritoryContested"sv) {
			terrcontest = func;
			return true;
		} else if(name == "OnTerritoryCaptured"sv) {
			terrcap = func;
			return true;
		} else if(name == "OnTerritoryLost"sv) {
			terrlost = func;
			return true;
		} else if(name == "OnThreatChanged"sv) {
			threachngd = func;
			return true;
		} else if(name == "OnHitByVomitJar"sv) {
			hitvom = func;
			return true;
		} else if(name == "OnDrop"sv) {
			drop = func;
			return true;
		} else if(name == "OnMoveToSuccess"sv) {
			movesucc = func;
			return true;
		} else if(name == "OnStuck"sv) {
			stuck = func;
			return true;
		} else if(name == "OnUnStuck"sv) {
			unstuck = func;
			return true;
		} else if(name == "OnIgnite"sv) {
			ignite = func;
			return true;
		} else if(name == "OnInjured"sv) {
			injured = func;
			return true;
		} else if(name == "OnKilled"sv) {
			killed = func;
			return true;
		} else if(name == "OnWin"sv) {
			win = func;
			return true;
		} else if(name == "OnLose"sv) {
			lose = func;
			return true;
		} else if(name == "OnEnteredSpit"sv) {
			enterspit = func;
			return true;
		} else if(name == "OnModelChanged"sv) {
			mdlchnd = func;
			return true;
		} else if(name == "OnMoveToFailure"sv) {
			movefail = func;
			return true;
		} else if(name == "OnSound"sv) {
			sound = func;
			return true;
		} else if(name == "OnWeaponFired"sv) {
			wepfired = func;
			return true;
		} else if(name == "OnActorEmoted"sv) {
			actemote = func;
			return true;
		} else if(name == "OnPickUp"sv) {
			pickup = func;
			return true;
		}
		
		return false;
	}
	
	Handle_t hndl = BAD_HANDLE;
	IPluginContext *pContext = nullptr;
	
	std::vector<SPAction *> actions{};
	
	void on_destroyed(SPAction *action)
	{
		auto it = actions.begin();
		while(it != actions.end()) {
			if(*it == action) {
				actions.erase(it);
				break;
			}
			
			++it;
		}
	}
	
	~SPActionEntry();
	
	SPAction *create();
};

#define RESVARS_REASON_SIZE 64
#define RESVARS_SIZE (RESVARS_REASON_SIZE+2)

#include <npcevent.h>

using spvarmap_t = std::unordered_map<std::string, std::vector<cell_t>>;

class SPAction : public Action<SPActor>
{
public:
	using BaseClass = Action<SPActor>;
	
	virtual const char *GetName( void ) const { return name.c_str(); }
	
	static void initvars(cell_t *vars, SPActionResult &result)
	{
		
	}
	
	static void initvars(cell_t *vars, SPEventDesiredResult &result)
	{
		vars[1] = (cell_t)RESULT_TRY;
	}
	
	static void varstoresult(cell_t *vars, SPActionResult &result)
	{
		result.m_action = (SPAction *)vars[0];
		
		std::string reason{(char *)&vars[2]};
		result.set_reason(std::move(reason));
	}
	
	static void varstoresult(cell_t *vars, SPEventDesiredResult &result)
	{
		varstoresult(vars, (SPActionResult &)result);
		
		result.m_priority = (EventResultPriorityType)vars[1];
	}
	
	virtual ~SPAction()
	{
		if(entry) {
			entry->on_destroyed(this);
		}
	}
	
	SPActionEntry *entry = nullptr;
	spvarmap_t data{};
	std::string name{};
	
	virtual BaseClass *InitialContainedAction(SPActor *me)
	{
		if(!entry) {
			return nullptr;
		}
		
		IPluginFunction *func = entry->intialact;
		
		if(!func) {
			return nullptr;
		}
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		SPAction *act = nullptr;
		func->Execute((cell_t *)&act);
		
		return act;
	}
	
	virtual SPActionResult::BaseClass OnStart(SPActor *me, BaseClass *priorAction)
	{
		if(!entry) {
			return Done("missing entry");
		}
		
		IPluginFunction *func = entry->onstart;
		
		if(!func) {
			return Continue();
		}
		
		SPActionResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell((cell_t)priorAction);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	
	virtual SPActionResult::BaseClass Update(SPActor *me, float interval)
	{
		if(!entry) {
			return Done("missing entry");
		}
		
		IPluginFunction *func = entry->onupdate;
		
		if(!func) {
			return Continue();
		}
		
		SPActionResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushFloat(interval);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	
	virtual SPActionResult::BaseClass OnSuspend(SPActor *me, BaseClass *interruptingAction)
	{
		if(!entry) {
			return Done("missing entry");
		}
		
		IPluginFunction *func = entry->onsus;
		
		if(!func) {
			return Continue();
		}
		
		SPActionResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell((cell_t)interruptingAction);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	
	virtual SPActionResult::BaseClass OnResume(SPActor *me, BaseClass *interruptingAction)
	{
		if(!entry) {
			return Done("missing entry");
		}
		
		IPluginFunction *func = entry->onresume;
		
		if(!func) {
			return Continue();
		}
		
		SPActionResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell((cell_t)interruptingAction);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	
	virtual void OnEnd(SPActor *me, BaseClass *nextAction)
	{
		if(!entry) {
			return;
		}
		
		IPluginFunction *func = entry->onend;
		
		if(!func) {
			return;
		}
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell((cell_t)nextAction);
		func->Execute(nullptr);
	}
	
	virtual SPEventDesiredResult::BaseClass OnLeaveGround(SPActor *me, CBaseEntity *ground )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->leavegrnd;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(ground));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnLandOnGround(SPActor *me, CBaseEntity *ground )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->landgrnd;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(ground));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnContact(SPActor *me, CBaseEntity *other, CGameTrace *trace = NULL )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->oncontact;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(other));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnMoveToSuccess(SPActor *me, const Path *path )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->movesucc;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnMoveToFailure(SPActor *me, const Path *path, MoveToFailureType reason )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->movefail;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(reason);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnStuck(SPActor *me )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->stuck;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnUnStuck(SPActor *me )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->unstuck;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnPostureChanged(SPActor *me )											{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnAnimationActivityComplete(SPActor *me, int activity )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->animcompl;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(activity);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnAnimationActivityInterrupted(SPActor *me, int activity )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->animinter;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(activity);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnAnimationEvent(SPActor *me, animevent_t *event )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->animevent;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		func->PushCell(event->Event());
#elif SOURCE_ENGINE == SE_TF2
		func->PushCell(event->event);
#endif
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnIgnite(SPActor *me )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->ignite;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnInjured(SPActor *me, const CTakeDamageInfo &info )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->injured;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
#ifdef __HAS_DAMAGERULES
		if(g_pDamageRules) {
			g_pDamageRules->PushDamageInfo(func, info);
		}
#endif
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnKilled(SPActor *me, const CTakeDamageInfo &info )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->killed;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
#ifdef __HAS_DAMAGERULES
		if(g_pDamageRules) {
			g_pDamageRules->PushDamageInfo(func, info);
		}
#endif
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnOtherKilled(SPActor *me, CBaseCombatCharacter *victim, const CTakeDamageInfo &info )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->otherkilled;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(victim));
#ifdef __HAS_DAMAGERULES
		if(g_pDamageRules) {
			g_pDamageRules->PushDamageInfo(func, info);
		}
#endif
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnSight(SPActor *me, CBaseEntity *subject )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->onsight;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(subject));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnLostSight(SPActor *me, CBaseEntity *subject )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->onlosesight;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(subject));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnSound(SPActor *me, CBaseEntity *source, const Vector &pos, KeyValues *keys )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->sound;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(source));
		cell_t addr[3]{sp_ftoc(pos.x), sp_ftoc(pos.y), sp_ftoc(pos.z)};
		func->PushArray(addr, 3);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnSpokeConcept(SPActor *me, CBaseCombatCharacter *who, AIConcept_t concept, AI_Response *response )	{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnWeaponFired(SPActor *me, CBaseCombatCharacter *whoFired, CBaseCombatWeapon *weapon )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->wepfired;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(whoFired));
		func->PushCell(gamehelpers->EntityToBCompatRef((CBaseEntity *)weapon));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnNavAreaChanged(SPActor *me, CNavArea *newArea, CNavArea *oldArea )		{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnModelChanged(SPActor *me )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->mdlchnd;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnPickUp(SPActor *me, CBaseEntity *item, CBaseCombatCharacter *giver )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->pickup;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(giver));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnDrop(SPActor *me, CBaseEntity *item )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->pickup;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(item));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnActorEmoted(SPActor *me, CBaseCombatCharacter *emoter, int emote )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->actemote;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(emoter));
		func->PushCell(emote);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}

	virtual SPEventDesiredResult::BaseClass OnCommandAttack(SPActor *me, CBaseEntity *victim )						{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnCommandApproach(SPActor *me, const Vector &pos, float range )			{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnCommandApproach(SPActor *me, CBaseEntity *goal )						{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnCommandRetreat(SPActor *me, CBaseEntity *threat, float range )			{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnCommandPause(SPActor *me, float duration )								{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnCommandResume(SPActor *me )											{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
	virtual SPEventDesiredResult::BaseClass OnCommandString(SPActor *me, const char *command )						{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }

	virtual SPEventDesiredResult::BaseClass OnShoved(SPActor *me, CBaseEntity *pusher )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->shoved;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(pusher));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnBlinded(SPActor *me, CBaseEntity *blinder )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->blinded;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(blinder));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnTerritoryContested(SPActor *me, int territoryID )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->terrcontest;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(territoryID);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnTerritoryCaptured(SPActor *me, int territoryID )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->terrcap;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(territoryID);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnTerritoryLost(SPActor *me, int territoryID )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->terrlost;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(territoryID);
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnWin(SPActor *me )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->win;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnLose(SPActor *me )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->lose;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}

	virtual SPEventDesiredResult::BaseClass OnThreatChanged(SPActor *me, CBaseEntity * territoryID )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->threachngd;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(territoryID));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnEnteredSpit(SPActor *me )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->enterspit;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnHitByVomitJar(SPActor *me, CBaseEntity * territoryID )
	{
		if(!entry) {
			return TryDone(RESULT_CRITICAL, "missing entry");
		}
		
		IPluginFunction *func = entry->hitvom;
		
		if(!func) {
			return TryContinue();
		}
		
		SPEventDesiredResult result{};
		
		func->PushCell((cell_t)this);
		func->PushCell(gamehelpers->EntityToBCompatRef(me));
		func->PushCell(gamehelpers->EntityToBCompatRef(territoryID));
		cell_t resvars[RESVARS_SIZE]{0};
		initvars(resvars, result);
		func->PushArray(resvars, RESVARS_SIZE, SM_PARAM_COPYBACK);
		func->Execute((cell_t *)&result.m_type);
		
		varstoresult(resvars, result);
		
		return result;
	}
	virtual SPEventDesiredResult::BaseClass OnCommandAssault(SPActor *me )														{ return entry ? TryContinue() : TryDone(RESULT_CRITICAL, "missing entry"); }
};

SPAction *SPActionEntry::create()
{
	SPAction *action = new SPAction{};
	action->entry = this;
	action->name = name;
	actions.emplace_back(action);
	return action;
}

class IPluginNextBotComponent;

std::unordered_map<IdentityToken_t *, std::vector<IPluginNextBotComponent *>> spnbcomponents{};

class IPluginNextBotComponent
{
public:
	IPluginNextBotComponent(IdentityToken_t *id)
		: pId{id}
	{
		spnbcomponents[pId].emplace_back(this);
	}
	
	virtual ~IPluginNextBotComponent()
	{
		std::vector<IPluginNextBotComponent *> &vec = spnbcomponents[pId];
		
		auto it = vec.begin();
		while(it != vec.end()) {
			if(*it == this) {
				vec.erase(it);
				break;
			}
			
			++it;
		}
	}
	
	cell_t handle_set_function(IPluginContext *pContext, const cell_t *params)
	{
		if(pId != pContext->GetIdentity()) {
			return pContext->ThrowNativeError("this plugin doenst own this component");
		}
		
		char *name_ptr = nullptr;
		pContext->LocalToString(params[2], &name_ptr);
		std::string_view name{name_ptr};
		
		IPluginFunction *func = pContext->GetFunctionById(params[3]);
		
		if(!set_function(name, func)) {
			return pContext->ThrowNativeError("invalid name %s", name_ptr);
		}
		
		return 0;
	}
	
	virtual void plugin_unloaded()
	{
		pId = nullptr;
	}
	
	virtual bool set_function(std::string_view name, IPluginFunction *func)
	{ return false; }
	
	IdentityToken_t *pId = nullptr;
	
	spvarmap_t data{};
};

#define IS_ANY_HINDRANCE_POSSIBLE	( (CBaseEntity*)0xFFFFFFFF )

class IIntentionCustom : public IIntention, public IPluginNextBotComponent
{
public:
	virtual const char *GetDebugString() const
	{
		return m_behavior ? m_behavior->GetDebugString() : "< NULL behavior >";
	}
	
	IIntentionCustom( INextBot *bot, bool reg, IdentityToken_t *id, IPluginFunction *initact_, std::string &&name_ )
		: IIntention( bot, reg ), IPluginNextBotComponent{id}
	{
		initact = initact_;
		name = std::move(name_);
		m_behavior = new SPBehavior( initialaction(bot), name.c_str() );

		CBaseCombatCharacter *pEntity = bot->GetEntity();

		SH_ADD_MANUALHOOK(UpdateOnRemove, pEntity, SH_MEMBER(this, &IIntentionCustom::HookUpdateOnRemove), false);
		hooked_remove = true;
	}

	template <typename... Args>
	static IIntentionCustom *create(INextBot *bot, bool reg, IdentityToken_t *id, Args &&... args)
	{
		return new IIntentionCustom(bot, reg, id, std::forward<Args>(args)...);
	}
	
	void HookUpdateOnRemove()
	{
		CBaseCombatCharacter *pEntity = META_IFACEPTR(CBaseCombatCharacter);

		SH_REMOVE_MANUALHOOK(UpdateOnRemove, pEntity, SH_MEMBER(this, &IIntentionCustom::HookUpdateOnRemove), false);
		hooked_remove = false;

		delete m_behavior;
		m_behavior = nullptr;

		RETURN_META(MRES_IGNORED);
	}
	
	virtual ~IIntentionCustom()
	{
		if(hooked_remove) {
			INextBot *bot = GetBot();
			CBaseCombatCharacter *pEntity = bot->GetEntity();

			SH_REMOVE_MANUALHOOK(UpdateOnRemove, pEntity, SH_MEMBER(this, &IIntentionCustom::HookUpdateOnRemove), false);
		}

		delete m_behavior;
	}
	
	virtual void plugin_unloaded()
	{
		IPluginNextBotComponent::plugin_unloaded();
		
		initact = nullptr;
		ishinder = nullptr;
	}
	
	virtual bool set_function(std::string_view name, IPluginFunction *func) override
	{
		if(name == "InitialContainedAction"sv) {
			initact = func;
			return true;
		} else if(name == "IsHindrance"sv) {
			ishinder = func;
			return true;
		}
		
		return IPluginNextBotComponent::set_function(name, func);
	}
	
	bool IsTankImmediatelyDangerousTo(CBasePlayer *pPlayer, CBaseCombatCharacter *pBCC)
	{
		INextBot *bot = GetBot();
		
		if(bot->GetRangeTo(pPlayer) <= 100.0f) {
			return true;
		}
		
		return false;
	}
	
	SPAction *initialaction(INextBot *bot)
	{
		if(!initact) {
			return nullptr;
		}
		
		SPAction *act = nullptr;
		initact->PushCell(gamehelpers->EntityToBCompatRef(bot->GetEntity()));
		initact->Execute((cell_t *)&act);
		
		return act;
	}
	
	virtual QueryResultType IsHindrance( const INextBot *me, CBaseEntity *blocker ) const
	{
		QueryResultType result = IIntention::IsHindrance(me, blocker);
		if(result != ANSWER_UNDEFINED) {
			return result;
		}
		
		if(ishinder) {
			ishinder->PushCell((cell_t)this);
			ishinder->PushCell((cell_t)me);
			if(blocker != IS_ANY_HINDRANCE_POSSIBLE) {
				ishinder->PushCell(gamehelpers->EntityToBCompatRef(blocker));
			} else {
				ishinder->PushCell((cell_t)IS_ANY_HINDRANCE_POSSIBLE);
			}
			QueryResultType res = ANSWER_UNDEFINED;
			ishinder->Execute((cell_t *)&res);
			if(res != ANSWER_UNDEFINED) {
				return res;
			}
		}
		
		return ANSWER_UNDEFINED;
	}
	
	virtual void Reset( void )
	{
		IIntention::Reset();
		
		delete m_behavior;
		m_behavior = new SPBehavior( initialaction(GetBot()), name.c_str() );
	}
	
	virtual void Update( void )
	{
		IIntention::Update();
		
		m_behavior->Update( (SPActor *)GetBot()->GetEntity() , GetUpdateInterval() );
	}
	
	virtual INextBotEventResponder *FirstContainedResponder( void ) const  { return m_behavior; }
	virtual INextBotEventResponder *NextContainedResponder( INextBotEventResponder *current ) const { return NULL; }
	
	SPBehavior *m_behavior = nullptr;
	std::string name{};
	
	IPluginFunction *initact = nullptr;
	IPluginFunction *ishinder = nullptr;

	bool hooked_remove{false};
};

SPActionEntry::~SPActionEntry()
{
	for(SPAction *act : actions) {
		act->entry = nullptr;
	}
}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
CBaseEntity *IBody::GetEntity() { return GetBot()->GetEntity(); }
#endif

bool IBody::SetPosition( const Vector &pos )
{
	if(!IsPostureMobile() ||
		HasActivityType(MOTION_CONTROLLED_XY) ||
		HasActivityType(MOTION_CONTROLLED_Z)) {
		return false;
	}

	GetBot()->GetEntity()->SetAbsOrigin( pos );
	return true;
}

const Vector &IBody::GetEyePosition( void )
{
	static Vector eye;
	eye = GetBot()->GetEntity()->WorldSpaceCenter();
	return eye;
}
const Vector &IBody::GetViewVector( void )
{
	static Vector view;
	AngleVectors( GetBot()->GetEntity()->EyeAngles(), &view );
	return view;
}

SH_DECL_MANUALHOOK0(MyNextBotPointer, 0, 0, 0, INextBot *)
SH_DECL_MANUALHOOK4_void(PerformCustomPhysics, 0, 0, 0, Vector *, Vector *, QAngle *, QAngle *)
SH_DECL_MANUALHOOK1(IsAreaTraversable, 0, 0, 0, bool, const CNavArea *)
SH_DECL_MANUALHOOK0_void(Spawn, 0, 0, 0)

class INextBotGeneric : public INextBot
{
public:
	struct vars_t
	{
		EHANDLE m_lastAttacker{};
		bool m_didModelChange = false;
		CBaseEntity *pEntity = nullptr;
	};
	
	unsigned char *vars_ptr()
	{ return (((unsigned char *)this) + sizeof(INextBot)); }
	vars_t &getvars()
	{ return *(vars_t *)vars_ptr(); }
	
	void dtor()
	{
		SH_REMOVE_MANUALHOOK(GenericDtor, this, SH_MEMBER(this, &INextBotGeneric::dtor), false);
		
		getvars().~vars_t();
		
		RETURN_META(MRES_IGNORED);
	}
	
	CBaseCombatCharacter *HookGetEntity( void )
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, (CBaseCombatCharacter *)getvars().pEntity);
	}
	class NextBotCombatCharacter *HookGetNextBotCombatCharacter( void )
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, nullptr);
	}
	
	void HookEntityDtor()
	{
		CBaseEntity *pEntity = META_IFACEPTR(CBaseEntity);
		
		SH_REMOVE_MANUALHOOK(GenericDtor, pEntity, SH_MEMBER(this, &INextBotGeneric::HookEntityDtor), false);
		SH_REMOVE_MANUALHOOK(MyNextBotPointer, pEntity, SH_MEMBER(this, &INextBotGeneric::HookMyNextBotPointer), false);
		SH_REMOVE_MANUALHOOK(PerformCustomPhysics, pEntity, SH_MEMBER(this, &INextBotGeneric::HookPerformCustomPhysics), false);
		SH_REMOVE_MANUALHOOK(IsAreaTraversable, pEntity, SH_MEMBER(this, &INextBotGeneric::HookIsAreaTraversable), false);
		//SH_REMOVE_MANUALHOOK(Spawn, pEntity, SH_MEMBER(this, &INextBotGeneric::HookSpawn), true);
		
		delete this;
		
		RETURN_META(MRES_IGNORED);
	}
	
	INextBot *HookMyNextBotPointer()
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, this);
	}
	
	bool HookIsAreaTraversable(const CNavArea *area);
	
	void HookPerformCustomPhysics( Vector *pNewPosition, Vector *pNewVelocity, QAngle *pNewAngles, QAngle *pNewAngVelocity );
	
	void HookSpawn()
	{
		CBaseEntity *pEntity = META_IFACEPTR(CBaseEntity);
		
		Reset();
		
		pEntity->SetMoveType(MOVETYPE_CUSTOM);
		
		RETURN_META(MRES_IGNORED);
	}
	
	static INextBotGeneric *create(CBaseEntity *pEntity);
};

void INextBotGeneric::HookPerformCustomPhysics( Vector *pNewPosition, Vector *pNewVelocity, QAngle *pNewAngles, QAngle *pNewAngVelocity )
{
	ILocomotion *mover = GetLocomotionInterface();
	if ( mover )
	{
		// hack to keep ground entity from being NULL'd when Z velocity is positive
		getvars().pEntity->SetGroundEntity( mover->GetGround() );
	}
	
	RETURN_META(MRES_SUPERCEDE);
}

bool INextBotGeneric::HookIsAreaTraversable(const CNavArea *area)
{
	if ( !area )
		RETURN_META_VALUE(MRES_SUPERCEDE, false);
	ILocomotion *mover = GetLocomotionInterface();
	if ( mover && !mover->IsAreaTraversable( area ) )
		RETURN_META_VALUE(MRES_SUPERCEDE, false);
	RETURN_META_VALUE(MRES_IGNORED, false);
}

#if SOURCE_ENGINE == SE_TF2
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
	
	NextBotCombatCharacter *m_nextBot;
	
	Vector m_priorPos;										// last update's position
	Vector m_lastValidPos;									// last valid position (not interpenetrating)
	
	Vector m_acceleration;
	Vector m_velocity;
	
	float m_desiredSpeed;									// speed bot wants to be moving
	float m_actualSpeed;									// actual speed bot is moving

	float m_maxRunSpeed;

	float m_forwardLean;
	float m_sideLean;
	QAngle m_desiredLean;
	
	bool m_isJumping;										// if true, we have jumped and have not yet hit the ground
	bool m_isJumpingAcrossGap;								// if true, we have jumped across a gap and have not yet hit the ground
	EHANDLE m_ground;										// have to manage this ourselves, since MOVETYPE_CUSTOM always NULLs out GetGroundEntity()
	Vector m_groundNormal;									// surface normal of the ground we are in contact with
	bool m_isClimbingUpToLedge;									// true if we are jumping up to an adjacent ledge
	Vector m_ledgeJumpGoalPos;
	bool m_isUsingFullFeetTrace;							// true if we're in the air and tracing the lowest StepHeight in ResolveCollision

	const CNavLadder *m_ladder;								// ladder we are currently climbing/descending
	const CNavArea *m_ladderDismountGoal;					// the area we enter when finished with our ladder move
	bool m_isGoingUpLadder;									// if false, we're going down

	CountdownTimer m_inhibitObstacleAvoidanceTimer;			// when active, turn off path following feelers

	CountdownTimer m_wiggleTimer;							// for wiggling
	NavRelativeDirType m_wiggleDirection;

	mutable Vector m_eyePos;								// for use with GetEyes(), etc.

	Vector m_moveVector;									// the direction of our motion in XY plane
	float m_moveYaw;										// global yaw of movement direction

	Vector m_accumApproachVectors;							// weighted sum of Approach() calls since last update
	float m_accumApproachWeights;
	bool m_bRecomputePostureOnCollision;

	CountdownTimer m_ignorePhysicsPropTimer;				// if active, don't collide with physics props (because we got stuck in one)
	EHANDLE m_ignorePhysicsProp;							// which prop to ignore
};
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
class ZombieBotLocomotion : public ILocomotion
{
public:
	virtual const Vector &GetGroundAcceleration( void ) = 0;	// return current world space acceleration
	virtual float GetMaxYawRate( void ) = 0;				// return max rate of yaw rotation
	virtual void SetAcceleration( const Vector &accel ) = 0;	// set world space acceleration
	virtual void SetVelocity( const Vector &vel ) = 0;		// set world space velocity
	virtual const Vector &GetMoveVector( void ) = 0;
	
	NextBotCombatCharacter *m_nextBot;
	
	Vector m_priorPos;										// last update's position
	Vector m_lastValidPos;									// last valid position (not interpenetrating)
	
	Vector m_acceleration;
	Vector m_velocity;
	
	float m_desiredSpeed;									// speed bot wants to be moving
	float m_actualSpeed;									// actual speed bot is moving

	float m_maxRunSpeed;

	float m_forwardLean;
	float m_sideLean;
	QAngle m_desiredLean;
	
	bool m_isJumping;										// if true, we have jumped and have not yet hit the ground
	bool m_isJumpingAcrossGap;								// if true, we have jumped across a gap and have not yet hit the ground
	EHANDLE m_ground;										// have to manage this ourselves, since MOVETYPE_CUSTOM always NULLs out GetGroundEntity()
	Vector m_groundNormal;									// surface normal of the ground we are in contact with
	bool m_isClimbingUpToLedge;									// true if we are jumping up to an adjacent ledge
	Vector m_ledgeJumpGoalPos;
	bool m_isUsingFullFeetTrace;							// true if we're in the air and tracing the lowest StepHeight in ResolveCollision

	const CNavLadder *m_ladder;								// ladder we are currently climbing/descending
	const CNavArea *m_ladderDismountGoal;					// the area we enter when finished with our ladder move
	bool m_isGoingUpLadder;									// if false, we're going down

	CountdownTimer m_inhibitObstacleAvoidanceTimer;			// when active, turn off path following feelers

	CountdownTimer m_wiggleTimer;							// for wiggling
	NavRelativeDirType m_wiggleDirection;

	mutable Vector m_eyePos;								// for use with GetEyes(), etc.

	Vector m_moveVector;									// the direction of our motion in XY plane
	float m_moveYaw;										// global yaw of movement direction

	Vector m_accumApproachVectors;							// weighted sum of Approach() calls since last update
	float m_accumApproachWeights;
	bool m_bRecomputePostureOnCollision;

	CountdownTimer m_ignorePhysicsPropTimer;				// if active, don't collide with physics props (because we got stuck in one)
	EHANDLE m_ignorePhysicsProp;							// which prop to ignore
	
	char pad1[12];
};
#endif

bool IgnoreActorsTraceFilterFunction( IHandleEntity *pServerEntity, int contentsMask )
{
	CBaseEntity *entity = EntityFromEntityHandle( pServerEntity );
	return ( entity->MyCombatCharacterPointer() == NULL );	// includes all bots, npcs, players, and TF2 buildings
}

typedef bool (*ShouldHitFunc_t)( IHandleEntity *pHandleEntity, int contentsMask );

CTraceFilterSimple::CTraceFilterSimple( const IHandleEntity *passedict, int collisionGroup,
									   ShouldHitFunc_t pExtraShouldHitFunc )
{
	m_pPassEnt = passedict;
	m_collisionGroup = collisionGroup;
	m_pExtraShouldHitCheckFunction = pExtraShouldHitFunc;
}

CTraceFilterSkipClassname::CTraceFilterSkipClassname( const IHandleEntity *passentity, const char *pchClassname, int collisionGroup ) :
CTraceFilterSimple( passentity, collisionGroup ), m_pchClassname( pchClassname )
{
}

CTraceFilterSimpleClassnameList::CTraceFilterSimpleClassnameList( const IHandleEntity *passentity, int collisionGroup ) :
CTraceFilterSimple( passentity, collisionGroup )
{
}

bool CTraceFilterSimpleClassnameList::ShouldHitEntity( IHandleEntity *pHandleEntity, int contentsMask )
{
	CBaseEntity *pEntity = EntityFromEntityHandle( pHandleEntity );
	if ( !pEntity )
		return false;

	for ( int i = 0; i < m_PassClassnames.Count(); ++i )
	{
		if ( FClassnameIs( pEntity, m_PassClassnames[ i ] ) )
			return false;
	}

	return CTraceFilterSimple::ShouldHitEntity( pHandleEntity, contentsMask );
}

void CTraceFilterSimpleClassnameList::AddClassnameToIgnore( const char *pchClassname )
{
	m_PassClassnames.AddToTail( pchClassname );
}

bool CTraceFilterSkipClassname::ShouldHitEntity( IHandleEntity *pHandleEntity, int contentsMask )
{
	CBaseEntity *pEntity = EntityFromEntityHandle( pHandleEntity );
	if ( !pEntity || FClassnameIs( pEntity, m_pchClassname ) )
		return false;

	return CTraceFilterSimple::ShouldHitEntity( pHandleEntity, contentsMask );
}

bool CTraceFilterSimple::ShouldHitEntity( IHandleEntity *pHandleEntity, int contentsMask )
{
	return call_mfunc<bool, CTraceFilterSimple, IHandleEntity *, int>(this, CTraceFilterSimpleShouldHitEntity, pHandleEntity, contentsMask);
}

class NextBotTraceFilterIgnoreActors : public CTraceFilterSimple
{
public:
	NextBotTraceFilterIgnoreActors( const IHandleEntity *passentity, int collisionGroup )
		: CTraceFilterSimple( passentity, collisionGroup, IgnoreActorsTraceFilterFunction )
	{
	}
};

SH_DECL_HOOK0_void(ILocomotion, Reset, SH_NOATTRIB, 0);
SH_DECL_HOOK2_void(ILocomotion, Approach, SH_NOATTRIB, 0, const Vector &, float);
SH_DECL_HOOK1_void(ILocomotion, SetDesiredSpeed, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetDesiredSpeed, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetGroundNormal, SH_NOATTRIB, 0, const Vector &);
SH_DECL_HOOK0(ILocomotion, GetVelocity, SH_NOATTRIB, 0, const Vector &);
SH_DECL_HOOK1_void(ILocomotion, FaceTowards, SH_NOATTRIB, 0, const Vector &);
SH_DECL_HOOK0(ILocomotion, GetFeet, SH_NOATTRIB, 0, const Vector &);

SH_DECL_HOOK0(ILocomotion, GetMaxJumpHeight, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetStepHeight, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetDeathDropHeight, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetRunSpeed, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetWalkSpeed, SH_NOATTRIB, 0, float);
#if SOURCE_ENGINE == SE_TF2
SH_DECL_HOOK0(ILocomotion, GetMaxAcceleration, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetMaxDeceleration, SH_NOATTRIB, 0, float);
#endif
SH_DECL_HOOK0(ILocomotion, GetSpeedLimit, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(ILocomotion, GetTraversableSlopeLimit, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0_void(ILocomotion, Update, SH_NOATTRIB, 0);
SH_DECL_HOOK2_void(ILocomotion, ClimbLadder, SH_NOATTRIB, 0, const CNavLadder *, const CNavArea *);
SH_DECL_HOOK2_void(ILocomotion, DescendLadder, SH_NOATTRIB, 0, const CNavLadder *, const CNavArea *);
SH_DECL_HOOK3(ILocomotion, ClimbUpToLedge, SH_NOATTRIB, 0, bool, const Vector &, const Vector &, CBaseEntity * );
SH_DECL_HOOK1(ILocomotion, ShouldCollideWith, SH_NOATTRIB, 0, bool, CBaseEntity * );

bool g_bInCustomLocomotion = false;
bool g_bInFlyingLocomotion = false;

struct customlocomotion_base_vars_t : IPluginNextBotComponent
{
public:
	customlocomotion_base_vars_t(IdentityToken_t *id)
		: IPluginNextBotComponent{id} {}
	
	virtual ~customlocomotion_base_vars_t() {}
	
	bool update_hooked = false;

	float step = 18.0f;
	float jump = 180.0f;
	float death = 200.0f;
	float run = 150.0f;
	float walk = 75.0f;
#if SOURCE_ENGINE == SE_TF2
	float accel = 500.0f;
	float deaccel = 500.0f;
#endif
	float limit = 99999999.9f;
	float slope = 0.6f;
	spvarmap_t data{};
	
	IPluginFunction *climbladdr = nullptr;
	IPluginFunction *desceladdr = nullptr;
	IPluginFunction *climbledge = nullptr;
	IPluginFunction *collidewith = nullptr;
	
	virtual void plugin_unloaded()
	{
		IPluginNextBotComponent::plugin_unloaded();
		
		climbladdr = nullptr;
		desceladdr = nullptr;
		climbledge = nullptr;
		collidewith = nullptr;
	}
	
	void HookClimbLadder(const CNavLadder *ladder, const CNavArea *area)
	{
		if(!climbladdr) {
			RETURN_META(MRES_IGNORED);
		}
		
		ILocomotion *loc = META_IFACEPTR(ILocomotion);
		
		climbladdr->PushCell((cell_t)loc);
		climbladdr->PushCell((cell_t)ladder);
		climbladdr->PushCell((cell_t)area);
		climbladdr->Execute(nullptr);
		
		RETURN_META(MRES_SUPERCEDE);
	}
	
	void HookDescendLadder(const CNavLadder *ladder, const CNavArea *area)
	{
		if(!desceladdr) {
			RETURN_META(MRES_IGNORED);
		}
		
		ILocomotion *loc = META_IFACEPTR(ILocomotion);
		
		desceladdr->PushCell((cell_t)loc);
		desceladdr->PushCell((cell_t)ladder);
		desceladdr->PushCell((cell_t)area);
		desceladdr->Execute(nullptr);
		
		RETURN_META(MRES_SUPERCEDE);
	}
	
	bool HookClimbUpToLedge( const Vector &landingGoal, const Vector &landingForward, CBaseEntity *obstacle )
	{
		if(!climbledge) {
			RETURN_META_VALUE(MRES_IGNORED, false);
		}
		
		ILocomotion *loc = META_IFACEPTR(ILocomotion);
		
		climbledge->PushCell((cell_t)loc);
		cell_t goalarr[3] = {sp_ftoc(landingGoal.x), sp_ftoc(landingGoal.y), sp_ftoc(landingGoal.z)};
		climbledge->PushArray(goalarr, 3);
		cell_t landarr[3] = {sp_ftoc(landingForward.x), sp_ftoc(landingForward.y), sp_ftoc(landingForward.z)};
		climbledge->PushArray(landarr, 3);
		climbledge->PushCell(gamehelpers->EntityToBCompatRef(obstacle));
		cell_t res = 0;
		climbledge->Execute(&res);
		
		RETURN_META_VALUE(MRES_SUPERCEDE, res);
	}

	bool HookShouldCollideWith( CBaseEntity *object )
	{
		if(!collidewith) {
			RETURN_META_VALUE(MRES_IGNORED, true);
		}

		ILocomotion *loc = META_IFACEPTR(ILocomotion);
		
		collidewith->PushCell((cell_t)loc);
		collidewith->PushCell(gamehelpers->EntityToBCompatRef(object));
		cell_t res = 0;
		collidewith->Execute(&res);
		
		RETURN_META_VALUE(MRES_SUPERCEDE, res);
	}
	
	virtual bool set_function(std::string_view name, IPluginFunction *func) override
	{
		if(name == "ClimbLadder"sv) {
			climbladdr = func;
			return true;
		} else if(name == "DescendLadder"sv) {
			desceladdr = func;
			return true;
		} else if(name == "ClimbUpToLedge"sv) {
			climbledge = func;
			return true;
		} else if(name == "ShouldCollideWith"sv) {
			collidewith = func;
			return true;
		}
		
		return IPluginNextBotComponent::set_function(name, func);
	}

	void HookUpdate()
	{
		ILocomotion *loc = META_IFACEPTR(ILocomotion);
		g_bInCustomLocomotion = true;
		SH_CALL(loc, &ILocomotion::Update)();
		g_bInCustomLocomotion = false;
		RETURN_META(MRES_SUPERCEDE);
	}
	
	virtual void remove_hooks(ILocomotion *bytes)
	{
		SH_REMOVE_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::dtor), false);
		if(update_hooked) {
			SH_REMOVE_HOOK(ILocomotion, Update, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookUpdate), false);
		}

		SH_REMOVE_HOOK(ILocomotion, GetMaxJumpHeight, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetMaxJumpHeight), false);
		SH_REMOVE_HOOK(ILocomotion, GetStepHeight, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetStepHeight), false);
		SH_REMOVE_HOOK(ILocomotion, GetDeathDropHeight, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetDeathDropHeight), false);
		SH_REMOVE_HOOK(ILocomotion, GetRunSpeed, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetRunSpeed), false);
		SH_REMOVE_HOOK(ILocomotion, GetWalkSpeed, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetWalkSpeed), false);
#if SOURCE_ENGINE == SE_TF2
		SH_REMOVE_HOOK(ILocomotion, GetMaxAcceleration, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetMaxAcceleration), false);
		SH_REMOVE_HOOK(ILocomotion, GetMaxDeceleration, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetMaxDeceleration), false);
#endif
		SH_REMOVE_HOOK(ILocomotion, GetSpeedLimit, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetSpeedLimit), false);
		SH_REMOVE_HOOK(ILocomotion, GetTraversableSlopeLimit, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetTraversableSlopeLimit), false);
		
		SH_REMOVE_HOOK(ILocomotion, ClimbLadder, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookClimbLadder), false);
		SH_REMOVE_HOOK(ILocomotion, DescendLadder, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookDescendLadder), false);
		SH_REMOVE_HOOK(ILocomotion, ClimbUpToLedge, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookClimbUpToLedge), false);
		SH_REMOVE_HOOK(ILocomotion, ShouldCollideWith, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookShouldCollideWith), false);
	}
	
	void dtor()
	{
		ILocomotion *loc = META_IFACEPTR(ILocomotion);
		
		remove_hooks(loc);
		
		this->~customlocomotion_base_vars_t();
		
		RETURN_META(MRES_IGNORED);
	}
	
	float HookGetMaxJumpHeight()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, jump); }
	float HookGetStepHeight()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, step); }
	float HookGetDeathDropHeight()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, death); }
	float HookGetRunSpeed()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, run); }
	float HookGetWalkSpeed()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, walk); }
#if SOURCE_ENGINE == SE_TF2
	float HookGetMaxAcceleration()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, accel); }
	float HookGetMaxDeceleration()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, deaccel); }
#endif
	float HookGetSpeedLimit()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, limit); }
	float HookGetTraversableSlopeLimit()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, slope); }
	
	virtual void add_hooks(ILocomotion *bytes, bool hook_update = true)
	{
		SH_ADD_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::dtor), false);
		if(hook_update) {
			SH_ADD_HOOK(ILocomotion, Update, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookUpdate), false);
			update_hooked = true;
		}

		SH_ADD_HOOK(ILocomotion, GetMaxJumpHeight, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetMaxJumpHeight), false);
		SH_ADD_HOOK(ILocomotion, GetStepHeight, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetStepHeight), false);
		SH_ADD_HOOK(ILocomotion, GetDeathDropHeight, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetDeathDropHeight), false);
		SH_ADD_HOOK(ILocomotion, GetRunSpeed, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetRunSpeed), false);
		SH_ADD_HOOK(ILocomotion, GetWalkSpeed, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetWalkSpeed), false);
#if SOURCE_ENGINE == SE_TF2
		SH_ADD_HOOK(ILocomotion, GetMaxAcceleration, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetMaxAcceleration), false);
		SH_ADD_HOOK(ILocomotion, GetMaxDeceleration, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetMaxDeceleration), false);
#endif
		SH_ADD_HOOK(ILocomotion, GetSpeedLimit, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetSpeedLimit), false);
		SH_ADD_HOOK(ILocomotion, GetTraversableSlopeLimit, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookGetTraversableSlopeLimit), false);
		
		SH_ADD_HOOK(ILocomotion, ClimbLadder, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookClimbLadder), false);
		SH_ADD_HOOK(ILocomotion, DescendLadder, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookDescendLadder), false);
		SH_ADD_HOOK(ILocomotion, ClimbUpToLedge, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookClimbUpToLedge), false);
		SH_ADD_HOOK(ILocomotion, ShouldCollideWith, bytes, SH_MEMBER(this, &customlocomotion_base_vars_t::HookShouldCollideWith), false);
	}
};

void *NDebugOverlayLine = nullptr;
void *NDebugOverlayVertArrow = nullptr;
void *NDebugOverlayHorzArrow = nullptr;
void *NDebugOverlayTriangle = nullptr;
void *NDebugOverlayCircle = nullptr;

namespace NDebugOverlay
{
	void Line( const Vector &origin, const Vector &target, int r, int g, int b, bool noDepthTest, float flDuration )
	{
		(void_to_func<void(*)(const Vector &, const Vector &, int, int, int, bool, float)>(NDebugOverlayLine))(origin, target, r, g, b, noDepthTest, flDuration);
	}
	
	void VertArrow( const Vector &startPos, const Vector &endPos, float width, int r, int g, int b, int a, bool noDepthTest, float flDuration)
	{
		(void_to_func<void(*)(const Vector &, const Vector &, float, int, int, int, int, bool, float)>(NDebugOverlayVertArrow))(startPos, endPos, width, r, g, b, a, noDepthTest, flDuration);
	}
	
	void HorzArrow( const Vector &startPos, const Vector &endPos, float width, int r, int g, int b, int a, bool noDepthTest, float flDuration)
	{
		(void_to_func<void(*)(const Vector &, const Vector &, float, int, int, int, int, bool, float)>(NDebugOverlayHorzArrow))(startPos, endPos, width, r, g, b, a, noDepthTest, flDuration);
	}
	
	void Triangle( const Vector &p1, const Vector &p2, const Vector &p3, int r, int g, int b, int a, bool noDepthTest, float duration )
	{
		(void_to_func<void(*)(const Vector &, const Vector &, const Vector &, int, int, int, int, bool, float)>(NDebugOverlayTriangle))(p1, p2, p3, r, g, b, a, noDepthTest, duration);
	}
	
	void Circle( const Vector &position, const QAngle &angles, float radius, int r, int g, int b, int a, bool bNoDepthTest, float flDuration )
	{
		(void_to_func<void(*)(const Vector &, const QAngle &, float, int, int, int, int, bool, float)>(NDebugOverlayCircle))(position, angles, radius, r, g, b, a, bNoDepthTest, flDuration);
	}
}

void DebugDrawLine( const Vector& vecAbsStart, const Vector& vecAbsEnd, int r, int g, int b, bool test, float duration )
{
	NDebugOverlay::Line( vecAbsStart + Vector( 0,0,0.1), vecAbsEnd + Vector( 0,0,0.1), r,g,b, test, duration );
}

void CNavArea::DrawFilled( int r, int g, int b, int a, float deltaT, bool noDepthTest, float margin ) const
{
	Vector nw = GetCorner( NORTH_WEST ) + Vector( margin, margin, 0.0f );
	Vector ne = GetCorner( NORTH_EAST ) + Vector( -margin, margin, 0.0f );
	Vector sw = GetCorner( SOUTH_WEST ) + Vector( margin, -margin, 0.0f );
	Vector se = GetCorner( SOUTH_EAST ) + Vector( -margin, -margin, 0.0f );

	if ( a == 0 )
	{
		NDebugOverlay::Line( nw, ne, r, g, b, true, deltaT );
		NDebugOverlay::Line( nw, sw, r, g, b, true, deltaT );
		NDebugOverlay::Line( sw, se, r, g, b, true, deltaT );
		NDebugOverlay::Line( se, ne, r, g, b, true, deltaT );
	}
	else
	{
		NDebugOverlay::Triangle( nw, se, ne, r, g, b, a, noDepthTest, deltaT );
		NDebugOverlay::Triangle( se, nw, sw, r, g, b, a, noDepthTest, deltaT );
	}

	// backside
	NDebugOverlay::Triangle( nw, ne, se, r, g, b, a, noDepthTest, deltaT );
	NDebugOverlay::Triangle( se, sw, nw, r, g, b, a, noDepthTest, deltaT );
}

#if SOURCE_ENGINE == SE_TF2
SH_DECL_HOOK0(NextBotGroundLocomotion, GetMaxYawRate, SH_NOATTRIB, 0, float);
#define GameLocomotion NextBotGroundLocomotion
#define GameLocomotionCustom NextBotGroundLocomotionCustom
#define SH_ADD_HOOK_GAMELOCOMOTION(x2, x3, x4, x5) \
	SH_ADD_HOOK(NextBotGroundLocomotion, x2, x3, x4, x5);
#define SH_REMOVE_HOOK_GAMELOCOMOTION(x2, x3, x4, x5) \
	SH_REMOVE_HOOK(NextBotGroundLocomotion, x2, x3, x4, x5);
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
SH_DECL_HOOK0(ZombieBotLocomotion, GetMaxYawRate, SH_NOATTRIB, 0, float);
#define GameLocomotion ZombieBotLocomotion
#define GameLocomotionCustom ZombieBotLocomotionCustom
#define SH_ADD_HOOK_GAMELOCOMOTION(x2, x3, x4, x5) \
	SH_ADD_HOOK(ZombieBotLocomotion, x2, x3, x4, x5);
#define SH_REMOVE_HOOK_GAMELOCOMOTION(x2, x3, x4, x5) \
	SH_REMOVE_HOOK(ZombieBotLocomotion, x2, x3, x4, x5);
#endif

class GameLocomotionCustom;
	
struct customlocomotion_vars_t : customlocomotion_base_vars_t
{
	customlocomotion_vars_t(IdentityToken_t *id)
		: customlocomotion_base_vars_t{id} {}
	
	float yaw = 250.0f;
	
	IPluginFunction *travladdr = nullptr;
	
	float HookGetMaxYawRate()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, yaw); }
	
	virtual void plugin_unloaded()
	{
		customlocomotion_base_vars_t::plugin_unloaded();
		
		travladdr = nullptr;
	}
	
	bool TraverseLadder(GameLocomotionCustom *loc)
	{
		if(!travladdr) {
			return false;
		}
		
		travladdr->PushCell((cell_t)loc);
		cell_t res = 0;
		travladdr->Execute(&res);
		
		return res;
	}
	
	virtual bool set_function(std::string_view name, IPluginFunction *func) override
	{
		if(customlocomotion_base_vars_t::set_function(name, func)) {
			return true;
		} else if(name == "TraverseLadder"sv) {
			travladdr = func;
			return true;
		}
		
		return IPluginNextBotComponent::set_function(name, func);
	}
	
	virtual void remove_hooks(ILocomotion *loc)
	{
		customlocomotion_base_vars_t::remove_hooks(loc);
		
		GameLocomotion *gameloc = (GameLocomotion *)loc;
		
		SH_REMOVE_HOOK_GAMELOCOMOTION(GetMaxYawRate, gameloc, SH_MEMBER(this, &customlocomotion_vars_t::HookGetMaxYawRate), false);
	}
	
	virtual void add_hooks(ILocomotion *loc)
	{
		customlocomotion_base_vars_t::add_hooks(loc);
		
		GameLocomotion *gameloc = (GameLocomotion *)loc;
		
		SH_ADD_HOOK_GAMELOCOMOTION(GetMaxYawRate, gameloc, SH_MEMBER(this, &customlocomotion_vars_t::HookGetMaxYawRate), false);
	}
};

#if SOURCE_ENGINE == SE_TF2
SH_DECL_HOOK0(NextBotGroundLocomotion, GetGravity, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(NextBotGroundLocomotion, GetFrictionForward, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(NextBotGroundLocomotion, GetFrictionSideways, SH_NOATTRIB, 0, float);

class NextBotGroundLocomotionCustom : public NextBotGroundLocomotion
{
public:
	struct vars_t : customlocomotion_vars_t
	{
		vars_t(IdentityToken_t *id)
			: customlocomotion_vars_t{id} {}
		
		float gravity = 1000.0f;
		float fricforward = 0.0f;
		float fricsideway = 3.0f;
	};
	
	float HookGetGravity()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().gravity); }
	float HookGetFrictionForward()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().fricforward); }
	float HookGetFrictionSideways()
	{ RETURN_META_VALUE(MRES_SUPERCEDE, getvars().fricsideway); }
	
	unsigned char *vars_ptr()
	{ return (((unsigned char *)this) + sizeof(NextBotGroundLocomotion)); }
	vars_t &getvars()
	{ return *(vars_t *)vars_ptr(); }
	
	void DetourReset()
	{
		m_bRecomputePostureOnCollision = false;
		m_ignorePhysicsPropTimer.Invalidate();

		m_nextBot = (NextBotCombatCharacter *)( GetBot()->GetEntity() );
		
		m_desiredSpeed = 0.0f;
		m_velocity = vec3_origin;
		m_acceleration = vec3_origin;

		m_desiredLean.x = 0.0f;
		m_desiredLean.y = 0.0f;
		m_desiredLean.z = 0.0f;
		
		m_ladder = NULL;

		m_isJumping = false;
		m_isJumpingAcrossGap = false;
		m_ground = NULL;
		m_groundNormal = Vector( 0, 0, 1.0f );
		m_isClimbingUpToLedge = false;
		m_isUsingFullFeetTrace = false;

		m_moveVector = Vector( 1, 0, 0 );
		
		m_priorPos = m_bot->GetPosition();
		m_lastValidPos = m_bot->GetPosition();

		m_inhibitObstacleAvoidanceTimer.Invalidate();

		m_accumApproachVectors = vec3_origin;
		m_accumApproachWeights = 0.0f;
	}
	
	const Vector &DetourGetFeet()
	{
		return m_bot->GetPosition();
	}
	
	bool DidJustJump( void )
	{
		return IsClimbingOrJumping() && (((CBaseEntity *)m_nextBot)->GetAbsVelocity().z > 0.0f);
	}
	
	void DetourApplyAccumulatedApproach( void )
	{
		Vector rawPos = GetFeet();

		const float deltaT = GetUpdateInterval();

		if ( deltaT <= 0.0f )
			return;

		if ( m_accumApproachWeights > 0.0f )
		{
			Vector approachDelta = m_accumApproachVectors / m_accumApproachWeights;

			// limit total movement to our max speed
			float maxMove = GetRunSpeed() * deltaT;

			float desiredMove = approachDelta.NormalizeInPlace();
			if ( desiredMove > maxMove )
			{
				desiredMove = maxMove;
			}

			rawPos += desiredMove * approachDelta;

			m_accumApproachVectors = vec3_origin;
			m_accumApproachWeights = 0.0f;
		}

		// can only move in 2D - geometry moves us up and down
		Vector pos( rawPos.x, rawPos.y, GetFeet().z );
			
		if ( !GetBot()->GetBodyInterface()->IsPostureMobile() )
		{
			// body is not in a movable state right now
			return;
		}

		Vector currentPos = m_bot->GetPosition();

		// compute unit vector to goal position
		m_moveVector = pos - currentPos;
		m_moveVector.z = 0.0f;
		float change = m_moveVector.NormalizeInPlace();

		const float epsilon = 0.001f;
		if ( change < epsilon )
		{
			// no motion
			m_forwardLean = 0.0f;
			m_sideLean = 0.0f;
			return;
		}

	/*	
		// lean forward/backward based on acceleration
		float desiredLean = m_acceleration / NextBotLeanForwardAccel.GetFloat();
		QAngle lean = GetDesiredLean();
		lean.x = NextBotLeanMaxAngle.GetFloat() * clamp( desiredLean, -1.0f, 1.0f );	
		SetDesiredLean( lean );
	*/	

		Vector newPos;

		// if we just started a jump, don't snap to the ground - let us get in the air first
		if ( DidJustJump() || !IsOnGround() )
		{
			if ( false && m_isClimbingUpToLedge )	// causes bots to hang in air stuck against edges
			{
				// drive towards the approach position in XY to help reach ledge
				m_moveVector = m_ledgeJumpGoalPos - currentPos;
				m_moveVector.z = 0.0f;
				m_moveVector.NormalizeInPlace();
				
				m_acceleration += GetMaxAcceleration() * m_moveVector;
			}
		}
		else if ( IsOnGround() )
		{
			// on the ground - move towards the approach position
			m_isClimbingUpToLedge = false;
			
			// snap forward movement vector along floor
			const Vector &groundNormal = GetGroundNormal();
			
			Vector left( -m_moveVector.y, m_moveVector.x, 0.0f );
			m_moveVector = CrossProduct( left, groundNormal );
			m_moveVector.NormalizeInPlace();
			
			// limit maximum forward speed from self-acceleration
			float forwardSpeed = DotProduct( m_velocity, m_moveVector );
			
			float maxSpeed = MIN( m_desiredSpeed, GetSpeedLimit() );
			
			if ( forwardSpeed < maxSpeed )
			{
				float ratio = ( forwardSpeed <= 0.0f ) ? 0.0f : ( forwardSpeed / maxSpeed );
				float governor = 1.0f - ( ratio * ratio * ratio * ratio );
				
				// accelerate towards goal
				m_acceleration += governor * GetMaxAcceleration() * m_moveVector;
			}
		}
	}
	
	Vector ResolveCollision( const Vector &from, const Vector &to, int recursionLimit )
	{
		return call_mfunc<Vector, NextBotGroundLocomotion, const Vector &, const Vector &, int>(this, NextBotGroundLocomotionResolveCollision, from, to, recursionLimit);
	}
	
	void DetourUpdatePosition( const Vector &newPos )
	{
		if ( NextBotStop->GetBool() || (((CBaseEntity *)m_nextBot)->GetFlags() & FL_FROZEN) != 0 || newPos == m_bot->GetPosition() )
		{
			return;
		}

		// avoid very nearby Actors to simulate "mushy" collisions between actors in contact with each other
		//Vector adjustedNewPos = ResolveZombieCollisions( newPos );
		Vector adjustedNewPos = newPos;

		// check for collisions during move and resolve them	
		const int recursionLimit = 3;
		Vector safePos = ResolveCollision( m_bot->GetPosition(), adjustedNewPos, recursionLimit );

		// set the bot's position
		if ( GetBot()->GetIntentionInterface()->IsPositionAllowed( GetBot(), safePos ) != ANSWER_NO )
		{
			m_bot->SetPosition( safePos );
		}
	}
	
	void DetourUpdateGroundConstraint( void )
	{
		// if we're up on the upward arc of our jump, don't interfere by snapping to ground
		// don't do ground constraint if we're climbing a ladder
		if ( DidJustJump() || IsAscendingOrDescendingLadder() )
		{
			m_isUsingFullFeetTrace = false;
			return;
		}
			
		IBody *body = GetBot()->GetBodyInterface();
		if ( body == NULL )
		{
			return;
		}

		float halfWidth = body->GetHullWidth()/2.0f;
		
		// since we only care about ground collisions, keep hull short to avoid issues with low ceilings
		/// @TODO: We need to also check actual hull height to avoid interpenetrating the world
		float hullHeight = GetStepHeight();
		
		// always need tolerance even when jumping/falling to make sure we detect ground penetration
		// must be at least step height to avoid 'falling' down stairs
		const float stickToGroundTolerance = GetStepHeight() + 0.01f;

		trace_t ground;
		NextBotTraceFilterIgnoreActors filter( ((CBaseEntity *)m_nextBot), body->GetCollisionGroup() );

		TraceHull( m_bot->GetPosition() + Vector( 0, 0, GetStepHeight() + 0.001f ),
						m_bot->GetPosition() + Vector( 0, 0, -stickToGroundTolerance ), 
						Vector( -halfWidth, -halfWidth, 0 ), 
						Vector( halfWidth, halfWidth, hullHeight ), 
						body->GetSolidMask(), &filter, &ground );

		if ( ground.startsolid )
		{
			// we're inside the ground - bad news
			if ( GetBot()->IsDebugging( NEXTBOT_LOCOMOTION ) && !( gpGlobals->framecount % 60 ) )
			{
				DevMsg( "%3.2f: Inside ground, ( %.0f, %.0f, %.0f )\n", gpGlobals->curtime, m_bot->GetPosition().x, m_bot->GetPosition().y, m_bot->GetPosition().z );
			}
			return;
		}

		if ( ground.fraction < 1.0f )
		{
			// there is ground below us
			m_groundNormal = ground.plane.normal;

			m_isUsingFullFeetTrace = false;
			
			// zero velocity normal to the ground
			float normalVel = DotProduct( m_groundNormal, m_velocity );
			m_velocity -= normalVel * m_groundNormal;
			
			// check slope limit
			if ( ground.plane.normal.z < GetTraversableSlopeLimit() )
			{
				// too steep to stand here

				// too steep to be ground - treat it like a wall hit
				if ( ( m_velocity.x * ground.plane.normal.x + m_velocity.y * ground.plane.normal.y ) <= 0.0f )
				{
					GetBot()->OnContact( ground.m_pEnt, &ground );			
				}
				
				// we're contacting some kind of ground
				// zero accelerations normal to the ground

				float normalAccel = DotProduct( m_groundNormal, m_acceleration );
				m_acceleration -= normalAccel * m_groundNormal;

				if ( GetBot()->IsDebugging( NEXTBOT_LOCOMOTION ) )
				{
					DevMsg( "%3.2f: NextBotGroundLocomotion - Too steep to stand here\n", gpGlobals->curtime );
					NDebugOverlay::Line( GetFeet(), GetFeet() + 20.0f * ground.plane.normal, 255, 150, 0, true, 5.0f );
				}

				// clear out upward velocity so we don't walk up lightpoles
				m_velocity.z = MIN( 0, m_velocity.z );
				m_acceleration.z = MIN( 0, m_acceleration.z );

				return;
			}
			
			// inform other components of collision if we didn't land on the 'world'
			if ( ground.m_pEnt && !ground.m_pEnt->IsWorld() )
			{
				GetBot()->OnContact( ground.m_pEnt, &ground );
			}

			// snap us to the ground 
			m_bot->SetPosition( ground.endpos );

			if ( !IsOnGround() )
			{
				// just landed
				((CBaseEntity *)m_nextBot)->SetGroundEntity( ground.m_pEnt );
				m_ground = ground.m_pEnt;

				// landing stops any jump in progress
				m_isJumping = false;
				m_isJumpingAcrossGap = false;

				GetBot()->OnLandOnGround( ground.m_pEnt );
			}
		}
		else
		{
			// not on the ground
			if ( IsOnGround() )
			{
				GetBot()->OnLeaveGround( ((CBaseEntity *)m_nextBot)->GetGroundEntity() );
				if ( !IsClimbingUpToLedge() && !IsJumpingAcrossGap() )
				{
					m_isUsingFullFeetTrace = true; // We're in the air and there's space below us, so use the full trace
					m_acceleration.z -= GetGravity(); // start our gravity now
				}
			}		
		}
	}

	bool DetourClimbUpToLedge( const Vector &landingGoal, const Vector &landingForward, const CBaseEntity *obstacle )
	{
		Vector vecMyPos = GetBot()->GetPosition();
		vecMyPos.z += GetStepHeight();

		float flActualHeight = landingGoal.z - vecMyPos.z;
		float height = flActualHeight;
		if (height < 16.0)
		{
			height = 16.0;
		}

		float additionalHeight = 20.0;
		if (height < 32)
		{
			additionalHeight += 8.0;
		}

		height += additionalHeight;

		float speed = sqrt(2.0 * GetGravity() * height);
		float time = speed / GetGravity();

		time += sqrt((2.0 * additionalHeight) / GetGravity());

		Vector vecJumpVel = landingGoal - vecMyPos;
		vecJumpVel /= time;
		vecJumpVel.z = speed;

		float flJumpSpeed = vecJumpVel.Length();
		float flMaxSpeed = 650.0;
		if (flJumpSpeed > flMaxSpeed)
		{
			vecJumpVel[0] *= flMaxSpeed / flJumpSpeed;
			vecJumpVel[1] *= flMaxSpeed / flJumpSpeed;
			vecJumpVel[2] *= flMaxSpeed / flJumpSpeed;
		}

		GetBot()->SetPosition(vecMyPos);
		SetVelocity(vecJumpVel);
		return true;
	}
	
	static bool vtable_assigned;
	
	static NextBotGroundLocomotionCustom *create(INextBot *bot, bool reg, IdentityToken_t *id)
	{
		NextBotGroundLocomotionCustom *bytes = (NextBotGroundLocomotionCustom *)calloc(1, sizeof(NextBotGroundLocomotion) + sizeof(vars_t));
		call_mfunc<void, NextBotGroundLocomotion, INextBot *>(bytes, NextBotGroundLocomotionCTOR, bot);
		
		new (bytes->vars_ptr()) vars_t(id);
		
		bytes->getvars().add_hooks(bytes);
		
		SH_ADD_HOOK(NextBotGroundLocomotion, GetGravity, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetGravity), false);
		SH_ADD_HOOK(NextBotGroundLocomotion, GetFrictionForward, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetFrictionForward), false);
		SH_ADD_HOOK(NextBotGroundLocomotion, GetFrictionSideways, bytes, SH_MEMBER(bytes, &NextBotGroundLocomotionCustom::HookGetFrictionSideways), false);
		
		if(!vtable_assigned) {
			void **vtable = *(void ***)bytes;
		
			SourceHook::SetMemAccess(vtable, sizeof(void **), SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);
			
			vtable[vfunc_index(&NextBotGroundLocomotion::Reset)] = func_to_void(&NextBotGroundLocomotionCustom::DetourReset);
			vtable[vfunc_index(&NextBotGroundLocomotion::GetFeet)] = func_to_void(&NextBotGroundLocomotionCustom::DetourGetFeet);
			vtable[vfunc_index(&NextBotGroundLocomotion::ClimbUpToLedge)] = func_to_void(&NextBotGroundLocomotionCustom::DetourClimbUpToLedge);
			
			vtable_assigned = true;
		}
		
		if(!reg) {
			bot->m_componentList = bytes->m_nextComponent;
			bytes->m_nextComponent = nullptr;
		}
		
		return bytes;
	}
};

bool NextBotGroundLocomotionCustom::vtable_assigned = false;

DETOUR_DECL_MEMBER0(ApplyAccumulatedApproach, void)
{
	((NextBotGroundLocomotionCustom *)this)->DetourApplyAccumulatedApproach();
}

DETOUR_DECL_MEMBER1(UpdatePosition, void, const Vector &, pos)
{
	((NextBotGroundLocomotionCustom *)this)->DetourUpdatePosition(pos);
}

DETOUR_DECL_MEMBER0(UpdateGroundConstraint, void)
{
	((NextBotGroundLocomotionCustom *)this)->DetourUpdateGroundConstraint();
}

ConVar tf_bot_npc_minion_avoid_range( "tf_bot_npc_minion_avoid_range", "100" );
ConVar tf_bot_npc_minion_avoid_force( "tf_bot_npc_minion_avoid_force", "100" );

ConVar tf_bot_npc_minion_deflect_range( "tf_bot_npc_minion_deflect_range", "300" );
ConVar tf_bot_npc_minion_deflect_force( "tf_bot_npc_minion_deflect_force", "2000" );

class CNextBotFlyingLocomotion : public ILocomotion
{
public:
	struct vars_t : customlocomotion_base_vars_t
	{
		vars_t(IdentityToken_t *id)
			: customlocomotion_base_vars_t{id} {}
		
		float m_desiredSpeed = 0.0f;
		float m_currentSpeed = 0.0f;
		Vector m_forward = vec3_origin;
		float m_desiredAltitude = 50.0f;
		Vector m_velocity = vec3_origin;
		Vector m_acceleration = vec3_origin;

		float accel = 500.0f;
		float hdamp = 2.0f;
		float vdamp = 1.0f;

		float yaw = 250.0f;
	};

	unsigned char *vars_ptr()
	{ return (((unsigned char *)this) + sizeof(ILocomotion)); }
	vars_t &getvars()
	{ return *(vars_t *)vars_ptr(); }

	void HookApproach(const Vector &goalPos, float goalWeight)
	{
		SH_CALL(this, &ILocomotion::Approach)(goalPos, goalWeight);

		Vector flyGoal = goalPos;
		flyGoal.z += getvars().m_desiredAltitude;

		Vector toGoal = flyGoal - GetBot()->GetEntity()->GetAbsOrigin();
		// altitude is handled in Update()
		toGoal.z = 0.0f;
		toGoal.NormalizeInPlace();

		getvars().m_acceleration += getvars().accel * toGoal;

		RETURN_META(MRES_SUPERCEDE);
	}

	void HookReset()
	{
		SH_CALL(this, &ILocomotion::Reset)();

		getvars().m_velocity = vec3_origin;
		getvars().m_acceleration = vec3_origin;
		getvars().m_desiredSpeed = 0.0f;
		getvars().m_currentSpeed = 0.0f;
		getvars().m_forward = vec3_origin;
		getvars().m_desiredAltitude = 50.0f;

		RETURN_META(MRES_SUPERCEDE);
	}

	void MaintainAltitude( void )
	{
		CBaseCombatCharacter *me = GetBot()->GetEntity();

		trace_t result;
		//CTraceFilterSimple filter( me, COLLISION_GROUP_NONE );
		CTraceFilterSimpleClassnameList filter( me, COLLISION_GROUP_NONE );
		filter.AddClassnameToIgnore( "bot_npc_minion" );

		// find ceiling
		TraceHull( me->GetAbsOrigin(), me->GetAbsOrigin() + Vector( 0, 0, 1000.0f ), 
				   me->WorldAlignMins(), me->WorldAlignMaxs(), 
				   GetBot()->GetBodyInterface()->GetSolidMask(), &filter, &result );

		float ceiling = result.endpos.z - me->GetAbsOrigin().z;

		// trace wider hull to account for nearby ledges we want to float over
		TraceHull( me->GetAbsOrigin() + Vector( 0, 0, ceiling ),
				   me->GetAbsOrigin() + Vector( 0, 0, -1000.0f ), 
				   Vector( 2.0f * me->WorldAlignMins().x, 2.0f * me->WorldAlignMins().y, me->WorldAlignMins().z ), 
				   Vector( 2.0f * me->WorldAlignMaxs().x, 2.0f * me->WorldAlignMaxs().y, me->WorldAlignMaxs().z ), 
				   GetBot()->GetBodyInterface()->GetSolidMask(), &filter, &result );

		float groundZ = result.endpos.z;

		float currentAltitude = me->GetAbsOrigin().z - groundZ;
		float error = getvars().m_desiredAltitude - currentAltitude;
		float accelZ = clamp( error, -getvars().accel, getvars().accel );

		getvars().m_acceleration.z += accelZ;
	}

	void HookUpdate()
	{
		g_bInFlyingLocomotion = true;

		SH_CALL(this, &ILocomotion::Update)();

		CBaseCombatCharacter *me = GetBot()->GetEntity();
		const float deltaT = GetUpdateInterval();

		Vector pos = me->GetAbsOrigin();

		// always maintain altitude, even if not trying to move (ie: no Approach call)
		MaintainAltitude();

		getvars().m_forward = getvars().m_velocity;
		getvars().m_currentSpeed = getvars().m_forward.NormalizeInPlace();

		Vector damping( getvars().hdamp, getvars().hdamp, getvars().vdamp );
		Vector totalAccel = getvars().m_acceleration - getvars().m_velocity * damping;

		// avoid other minions
	#if 0
		CBaseEntity *minion = NULL;
		while( ( minion = gEntList.FindEntityByClassname( minion, "bot_npc_minion" ) ) != NULL )
		{
			if ( me == minion )
				continue;

			Vector toPeer = minion->GetAbsOrigin() - me->GetAbsOrigin();
			toPeer.z = 0.0f;
			float range = toPeer.NormalizeInPlace();

			if ( range < tf_bot_npc_minion_avoid_range.GetFloat() )
			{
				totalAccel += -tf_bot_npc_minion_avoid_force.GetFloat() * toPeer;
			}
		}
	#endif

		getvars().m_velocity += totalAccel * deltaT;
		me->SetAbsVelocity( getvars().m_velocity );

		pos += getvars().m_velocity * deltaT;

		// check for collisions along move	
		trace_t result;
		CTraceFilterSkipClassname filter( me, "bot_npc_minion", COLLISION_GROUP_NONE );
		Vector from = me->GetAbsOrigin();
		Vector to = pos;
		Vector desiredGoal = to;
		Vector resolvedGoal;
		int recursionLimit = 3;

		int hitCount = 0;
		Vector surfaceNormal = vec3_origin;

		bool didHitWorld = false;

		while( true )
		{
			TraceHull( from, desiredGoal, me->WorldAlignMins(), me->WorldAlignMaxs(), GetBot()->GetBodyInterface()->GetSolidMask(), &filter, &result );

			if ( !result.DidHit() )
			{
				resolvedGoal = pos;
				break;
			}

			if ( result.DidHitWorld() )
			{
				didHitWorld = true;
			}

			++hitCount;
			surfaceNormal += result.plane.normal;

			// If we hit really close to our target, then stop
			if ( !result.startsolid && desiredGoal.DistToSqr( result.endpos ) < 1.0f )
			{
				resolvedGoal = result.endpos;
				break;
			}

			if ( result.startsolid )
			{
				// stuck inside solid; don't move
				resolvedGoal = me->GetAbsOrigin();
				break;
			}

			if ( --recursionLimit <= 0 )
			{
				// reached recursion limit, no more adjusting allowed
				resolvedGoal = result.endpos;
				break;
			}

			// slide off of surface we hit
			Vector fullMove = desiredGoal - from;
			Vector leftToMove = fullMove * ( 1.0f - result.fraction );

			float blocked = DotProduct( result.plane.normal, leftToMove );

			Vector unconstrained = fullMove - blocked * result.plane.normal;

			// check for collisions along remainder of move
			// But don't bother if we're not going to deflect much
			Vector remainingMove = from + unconstrained;
			if ( remainingMove.DistToSqr( result.endpos ) < 1.0f )
			{
				resolvedGoal = result.endpos;
				break;
			}

			desiredGoal = remainingMove;
		}

		if ( hitCount > 0 )
		{
			surfaceNormal.NormalizeInPlace();

			// bounce
			getvars().m_velocity = getvars().m_velocity - 2.0f * DotProduct( getvars().m_velocity, surfaceNormal ) * surfaceNormal;

			if ( didHitWorld )
			{
				//me->EmitSound( "Minion.Bounce" );
			}
		}

		GetBot()->GetEntity()->SetAbsOrigin( result.endpos );

		getvars().m_acceleration = vec3_origin;

		g_bInFlyingLocomotion = false;
		RETURN_META(MRES_SUPERCEDE);
	}

	void HookSetDesiredSpeed(float speed)
	{
		getvars().m_desiredSpeed = speed;
		RETURN_META(MRES_SUPERCEDE);
	}

	float HookGetDesiredSpeed( void )
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, getvars().m_desiredSpeed);
	}

	void HookFaceTowards( const Vector &target )
	{
		CBaseCombatCharacter *me = GetBot()->GetEntity();

	#if 0
		Vector toTarget = target - me->WorldSpaceCenter();
		toTarget.z = 0.0f;

		QAngle angles;
		VectorAngles( toTarget, angles );

		me->SetAbsAngles( angles );
	#else
		const float deltaT = GetUpdateInterval();
	
		QAngle angles = me->GetLocalAngles();
		
		float desiredYaw = UTIL_VecToYaw( target - GetFeet() );

		float angleDiff = UTIL_AngleDiff( desiredYaw, angles.y );
		
		float deltaYaw = GetMaxYawRate() * deltaT;
		
		if (angleDiff < -deltaYaw)
		{
			angles.y -= deltaYaw;
		}
		else if (angleDiff > deltaYaw)
		{
			angles.y += deltaYaw;
		}
		else
		{
			angles.y += angleDiff;
		}
		
		me->SetLocalAngles( angles );
	#endif

		RETURN_META(MRES_SUPERCEDE);
	}

	const Vector &HookGetFeet( void )
	{
		static Vector feet;
		CBaseCombatCharacter *me = GetBot()->GetEntity();

		trace_t result;
		CTraceFilterSimpleClassnameList filter( me, COLLISION_GROUP_NONE );
		filter.AddClassnameToIgnore( "bot_npc_minion" );

		feet = me->GetAbsOrigin();

		UTIL_TraceLine( feet, feet + Vector( 0, 0, -2000.0f ), MASK_PLAYERSOLID_BRUSHONLY, &filter, &result );

		feet.z = result.endpos.z;

		RETURN_META_VALUE(MRES_SUPERCEDE, feet);
	}

	const Vector &HookGetGroundNormal( void )
	{
		static Vector up( 0, 0, 1.0f );

		RETURN_META_VALUE(MRES_SUPERCEDE, up);
	}

	const Vector &HookGetVelocity( void )
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, getvars().m_velocity);
	}

	float GetMaxYawRate( void )
	{
		return getvars().yaw;
	}

	void Deflect( CBaseEntity *deflector )
	{
		if ( deflector )
		{
			Vector fromDeflector = GetBot()->GetEntity()->WorldSpaceCenter() - deflector->EyePosition();
			float range = fromDeflector.NormalizeInPlace();

			if ( range < tf_bot_npc_minion_deflect_range.GetFloat() )
			{
				getvars().m_velocity += ( 1.0f - ( range / tf_bot_npc_minion_deflect_range.GetFloat() ) ) * tf_bot_npc_minion_deflect_force.GetFloat() * fromDeflector;
			}
		}
	}

	static CNextBotFlyingLocomotion *create(INextBot *bot, bool reg, IdentityToken_t *id)
	{
		CNextBotFlyingLocomotion *bytes = (CNextBotFlyingLocomotion *)calloc(1, sizeof(ILocomotion) + sizeof(vars_t));
		call_mfunc<void, ILocomotion, INextBot *>(bytes, ILocomotionCTOR, bot);
		
		new (bytes->vars_ptr()) vars_t(id);
		
		bytes->getvars().add_hooks(bytes, false);

		SH_ADD_HOOK(ILocomotion, Reset, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookReset), false);
		SH_ADD_HOOK(ILocomotion, Update, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookUpdate), false);
		SH_ADD_HOOK(ILocomotion, Approach, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookApproach), false);
		SH_ADD_HOOK(ILocomotion, FaceTowards, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookFaceTowards), false);
		SH_ADD_HOOK(ILocomotion, SetDesiredSpeed, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookSetDesiredSpeed), false);
		SH_ADD_HOOK(ILocomotion, GetDesiredSpeed, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookGetDesiredSpeed), false);
		SH_ADD_HOOK(ILocomotion, GetGroundNormal, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookGetGroundNormal), false);
		SH_ADD_HOOK(ILocomotion, GetVelocity, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookGetVelocity), false);
		SH_ADD_HOOK(ILocomotion, GetFeet, bytes, SH_MEMBER(bytes, &CNextBotFlyingLocomotion::HookGetFeet), false);
		
		if(!reg) {
			bot->m_componentList = bytes->m_nextComponent;
			bytes->m_nextComponent = nullptr;
		}
		
		return bytes;
	}
};
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
class ZombieBotLocomotionCustom : public ZombieBotLocomotion
{
public:
	struct vars_t : customlocomotion_vars_t
	{
		vars_t(IdentityToken_t *id)
			: customlocomotion_vars_t{id} {}
	};
	
	unsigned char *vars_ptr()
	{ return (((unsigned char *)this) + sizeof(ZombieBotLocomotion)); }
	vars_t &getvars()
	{ return *(vars_t *)vars_ptr(); }
	
	static ZombieBotLocomotionCustom *create(INextBot *bot, bool reg, IdentityToken_t *id)
	{
		ZombieBotLocomotionCustom *bytes = (ZombieBotLocomotionCustom *)calloc(1, sizeof(ZombieBotLocomotion) + sizeof(vars_t));
		call_mfunc<void, ZombieBotLocomotion, INextBot *>(bytes, ZombieBotLocomotionCTOR, bot);
		
		new (bytes->vars_ptr()) vars_t(id);
		
		bytes->getvars().add_hooks(bytes);
		
		if(!reg) {
			bot->m_componentList = bytes->m_nextComponent;
			bytes->m_nextComponent = nullptr;
		}
		
		return bytes;
	}
};
#endif

DETOUR_DECL_MEMBER0(TraverseLadder, bool)
{
	if(!g_bInCustomLocomotion && !g_bInFlyingLocomotion) {
		return DETOUR_MEMBER_CALL(TraverseLadder)();
	}

	bool val = false;

	if(g_bInCustomLocomotion) {
		GameLocomotionCustom *gameloc = (GameLocomotionCustom *)this;
		val = gameloc->getvars().TraverseLadder(gameloc);
	} else if(g_bInFlyingLocomotion) {
		
	}

	return val;
}

SH_DECL_HOOK0(INextBot, GetEntity, SH_NOATTRIB, 0, CBaseCombatCharacter *);
SH_DECL_HOOK0(INextBot, GetNextBotCombatCharacter, SH_NOATTRIB, 0, NextBotCombatCharacter *);

INextBotGeneric *INextBotGeneric::create(CBaseEntity *pEntity)
{
	INextBotGeneric *bytes = (INextBotGeneric *)calloc(1, sizeof(INextBot) + sizeof(vars_t));
	call_mfunc<void>(bytes, INextBotCTOR);
	new (bytes->vars_ptr()) vars_t();
	
	bytes->getvars().pEntity = pEntity;
	
	SH_ADD_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(bytes, &INextBotGeneric::dtor), false);
	
	SH_ADD_HOOK(INextBot, GetEntity, bytes, SH_MEMBER(bytes, &INextBotGeneric::HookGetEntity), false);
	SH_ADD_HOOK(INextBot, GetNextBotCombatCharacter, bytes, SH_MEMBER(bytes, &INextBotGeneric::HookGetNextBotCombatCharacter), false);
	
	SH_ADD_MANUALHOOK(GenericDtor, pEntity, SH_MEMBER(bytes, &INextBotGeneric::HookEntityDtor), false);
	SH_ADD_MANUALHOOK(MyNextBotPointer, pEntity, SH_MEMBER(bytes, &INextBotGeneric::HookMyNextBotPointer), false);
	SH_ADD_MANUALHOOK(PerformCustomPhysics, pEntity, SH_MEMBER(bytes, &INextBotGeneric::HookPerformCustomPhysics), false);
	//SH_ADD_MANUALHOOK(Spawn, pEntity, SH_MEMBER(bytes, &INextBotGeneric::HookSpawn), true);
	
	if(pEntity->MyCombatCharacterPointer()) {
		SH_ADD_MANUALHOOK(IsAreaTraversable, pEntity, SH_MEMBER(bytes, &INextBotGeneric::HookIsAreaTraversable), false);
	}
	
	bytes->Reset();
	pEntity->SetMoveType(MOVETYPE_CUSTOM);
	
	return bytes;
}

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

class IBodyCustom : public IBody, public IPluginNextBotComponent
{
public:
	IBodyCustom( INextBot *bot, bool reg, IdentityToken_t *id )
		: IBody( bot, reg ), IPluginNextBotComponent(id)
	{
		HullWidth = 26.0f;
		HullHeight = 68.0f;
	}

	IPluginFunction *selectseq = nullptr;
	IPluginFunction *translact = nullptr;

	Activity TranslateActivity(Activity act)
	{
		if(translact != nullptr) {
			translact->PushCell((cell_t)this);
			translact->PushCell((cell_t)act);
			translact->Execute((cell_t *)&act);
		}

		return act;
	}

	int SelectAnimationSequence( Activity act ) const override
	{
		int seq = -1;

		INextBot *bot = GetBot();
		CBaseCombatCharacter *pEntity = bot->GetEntity();

		if(selectseq != nullptr) {
			selectseq->PushCell((cell_t)this);
			selectseq->PushCell(gamehelpers->EntityToBCompatRef(pEntity));
			selectseq->PushCell((cell_t)act);
			selectseq->Execute((cell_t *)&seq);
		}

		if(seq == -1) {
			seq = pEntity->SelectWeightedSequence(act);
		}

		if(bot->IsDebugging(NEXTBOT_ANIMATION)) {
			//TODO!!!! activity and sequence name
			DevMsg("%s: IBodyCustom::SelectAnimationSequence([%i, %s]) = [%i, %s]\n", bot->GetDebugIdentifier(), act, "", seq, "");
		}

		return seq;
	}

	bool StartActivity( Activity act, unsigned int flags = 0 ) override
	{
		INextBot *bot = GetBot();

		if(bot->IsDebugging(NEXTBOT_ANIMATION)) {
			//TODO!!!! activity name
			DevMsg("%s: IBodyCustom::StartActivity([%i, %s], %i)\n", bot->GetDebugIdentifier(), act, "", flags);
		}

		if(act == ACT_INVALID) {
			return false;
		}

		act = TranslateActivity(act);

		if(act == ACT_INVALID) {
			return false;
		}

		int seq = SelectAnimationSequence(act);
		if(seq == -1) {
			return false;
		}

		if((current_flags & ACTIVITY_UNINTERRUPTIBLE) && !(flags & ACTIVITY_UNINTERRUPTIBLE)) {
			return false;
		}

		if(flags & ACTIVITY_UNINTERRUPTIBLE) {
			current_flags |= ACTIVITY_UNINTERRUPTIBLE;
			flags &= ~ACTIVITY_UNINTERRUPTIBLE;
		}

		if(flags & ACTIVITY_TRANSITORY) {
			flags &= ~ACTIVITY_TRANSITORY;
			if(current_act[1].act != act) {
				current_act[1].seq = seq;
			}
			current_act[1].act = act;
			current_act[1].flags = flags;
			current_flags |= ACTIVITY_TRANSITORY;
		} else {
			if(current_act[0].act != act) {
				current_act[0].seq = seq;
			}
			current_act[0].act = act;
			current_act[0].flags = flags;
		}

		return true;
	}

	Activity GetActivity() const override
	{
		if(current_flags & ACTIVITY_TRANSITORY) {
			return current_act[1].act;
		} else {
			return current_act[0].act;
		}
	}

	int GetSequence()
	{
		if(current_flags & ACTIVITY_TRANSITORY) {
			return current_act[1].seq;
		} else {
			return current_act[0].seq;
		}
	}

	bool HasActivityType( unsigned int flags ) const override
	{
		if(flags == ACTIVITY_TRANSITORY) {
			return !!(current_flags & ACTIVITY_TRANSITORY);
		} else if(flags == ACTIVITY_UNINTERRUPTIBLE) {
			return !!(current_flags & ACTIVITY_UNINTERRUPTIBLE);
		}

		flags &= ~(ACTIVITY_TRANSITORY|ACTIVITY_UNINTERRUPTIBLE);

		if(current_flags & ACTIVITY_TRANSITORY) {
			return !!(current_act[1].flags & flags);
		} else {
			return !!(current_act[0].flags & flags);
		}
	}

	void SetDesiredPosture( PostureType posture ) override
	{ desired_posture = posture; }
	PostureType GetDesiredPosture( void ) const override
	{ return desired_posture; }

	PostureType GetActualPosture( void ) const override
	{ return current_posture; }

	bool IsPostureChanging( void ) const override
	{
		if(!!(current_flags & ACTIVITY_TRANSITORY)) {
			return (current_act[1].act == ACT_TRANSITION);
		}

		return false;
	}

	void SetArousal( ArousalType arousal ) override
	{ current_arousal = arousal; }
	ArousalType GetArousal( void ) const override
	{ return current_arousal; }

	struct act_info_t
	{
		Activity act = ACT_INVALID;
		int seq = -1;
		unsigned int flags = 0;

		void reset()
		{
			act = ACT_INVALID;
			seq = -1;
			flags = 0;
		}
	};

	act_info_t current_act[2];
	Activity last_activity = ACT_INVALID;
	unsigned int current_flags = 0;
	ArousalType current_arousal = NEUTRAL;
	PostureType current_posture = STAND;
	PostureType desired_posture = STAND;

	void Reset() override
	{
		IBody::Reset();

		current_act[0].reset();
		current_act[1].reset();
		last_activity = ACT_INVALID;

		current_flags = 0;
		current_arousal = NEUTRAL;
		current_posture = STAND;
		desired_posture = STAND;
	}

	void Update() override
	{
		IBody::Update();

		INextBot *bot = GetBot();
		CBaseCombatCharacter *pEntity = bot->GetEntity();

		if(last_activity != ACT_INVALID && !IsActivity(last_activity)) {
			if(bot->IsDebugging(NEXTBOT_ANIMATION)) {
				//TODO!!!! activity name
				DevMsg("%s: IBodyCustom::OnAnimationActivityInterrupted([%i, %s])\n", bot->GetDebugIdentifier(), last_activity, "");
			}

			bot->OnAnimationActivityInterrupted(last_activity);
		}

		last_activity = GetActivity();

		if(!!(current_flags & ACTIVITY_TRANSITORY)) {
			if(current_act[1].seq == -1) {
				if(current_act[1].act != ACT_INVALID) {
					current_act[1].seq = SelectAnimationSequence(current_act[1].act);
				}
			}

			if(current_act[1].seq != -1) {
				pEntity->ResetSequence(current_act[1].seq);

				if(pEntity->GetSequeceFinished()) {
					current_act[1].seq = -1;

					if(current_act[1].act != ACT_INVALID) {
						if(bot->IsDebugging(NEXTBOT_ANIMATION)) {
							//TODO!!!! activity name
							DevMsg("%s: IBodyCustom::OnAnimationActivityComplete([%i, %s])\n", bot->GetDebugIdentifier(), current_act[1].act, "");
						}

						bot->OnAnimationActivityComplete(current_act[1].act);
					}

					pEntity->SetSequeceFinished(false);
				}
			}

			if(current_act[1].seq == -1) {
				Activity act = current_act[1].act;

				current_flags &= ~ACTIVITY_TRANSITORY;
				current_act[1].reset();

				if(act == ACT_TRANSITION && current_posture != desired_posture) {
					if(bot->IsDebugging(NEXTBOT_ANIMATION)) {
						DevMsg("%s: IBodyCustom::OnPostureChanged(%i, %i)\n", bot->GetDebugIdentifier(), current_posture, desired_posture);
					}

					current_posture = desired_posture;

					bot->OnPostureChanged();
				}
			}
		} else {
			if(current_posture != desired_posture) {
				if(StartActivity(ACT_TRANSITION, ACTIVITY_TRANSITORY)) {
					return;
				} else {
					if(bot->IsDebugging(NEXTBOT_ANIMATION)) {
						DevMsg("%s: IBodyCustom::OnPostureChanged(%i, %i)\n", bot->GetDebugIdentifier(), current_posture, desired_posture);
					}

					current_posture = desired_posture;

					bot->OnPostureChanged();
				}
			}

			if(current_act[0].seq == -1) {
				if(current_act[0].act != ACT_INVALID) {
					current_act[0].seq = SelectAnimationSequence(current_act[0].act);
				}
			}

			if(current_act[0].seq != -1) {
				pEntity->ResetSequence(current_act[0].seq);

				if(pEntity->GetSequeceFinished()) {
					if(current_act[0].act != ACT_INVALID) {
						current_act[0].seq = SelectAnimationSequence(current_act[0].act);

						if(bot->IsDebugging(NEXTBOT_ANIMATION)) {
							//TODO!!!! activity name
							DevMsg("%s: IBodyCustom::OnAnimationActivityComplete([%i, %s])\n", bot->GetDebugIdentifier(), current_act[0].act, "");
						}

						bot->OnAnimationActivityComplete(current_act[0].act);
					}

					pEntity->SetSequeceFinished(false);
				}
			}
		}

		ILocomotion *locomotion = bot->GetLocomotionInterface();

		float anim_speed = pEntity->GetGroundSpeed();
		if(anim_speed < 0.1f) {
			if(IsActivity(ACT_RUN) || IsActivity(ACT_WALK)) {
				if(locomotion->IsRunning()) {
					auto it{data.find("run_anim_speed"s)};
					if(it != data.cend()) {
						anim_speed = sp_ctof(it->second[0]);
					}
				} else {
					auto it{data.find("walk_anim_speed"s)};
					if(it != data.cend()) {
						anim_speed = sp_ctof(it->second[0]);
					}
				}
			}
		}

		float ground_speed = locomotion->GetGroundSpeed();

		float playback_rate = 1.0f;

		if(ground_speed > 0.1f && anim_speed > 0.1f) {
			playback_rate = (ground_speed / anim_speed);
		}

		if(playback_rate > 2.0f) {
			playback_rate = 2.0f;
		}

		if(playback_rate < -4.0f) {
			playback_rate = -4.0f;
		} else if(playback_rate > 12.0f) {
			playback_rate = 12.0f;
		}

		pEntity->SetPlaybackRate(playback_rate);

		pEntity->StudioFrameAdvance();
		pEntity->DispatchAnimEvents();
	}

	void plugin_unloaded() override
	{
		IPluginNextBotComponent::plugin_unloaded();

		selectseq = nullptr;
		translact = nullptr;
	}

	bool set_function(std::string_view name, IPluginFunction *func) override
	{
		if(name == "SelectAnimationSequence"sv) {
			selectseq = func;
			return true;
		} else if(name == "TranslateActivity"sv) {
			translact = func;
			return true;
		}

		return IPluginNextBotComponent::set_function(name, func);
	}

	virtual const char *GetDebugString() const { return "IBodyCustom"; }
	
	void SetHullWidth(float height)
	{
		HullWidth = height;
	}
	
	void SetHullHeight(float height)
	{
		HullHeight = height;
	}
	
	float HullWidth;
	float HullHeight;
	float StandHullHeight = 68.0f;
	float CrouchHullHeight = 32.0f;
	int SolidMask = MASK_NPCSOLID;
#if SOURCE_ENGINE == SE_TF2
	int CollisionGroup = COLLISION_GROUP_NPC;
#endif
	
	float GetHullWidth( void ) override { return HullWidth; }							// width of bot's collision hull in XY plane
	float GetHullHeight( void ) override { return HullHeight; }							// height of bot's current collision hull based on posture
	float GetStandHullHeight( void ) override { return StandHullHeight; }						// height of bot's collision hull when standing
	float GetCrouchHullHeight( void ) override { return CrouchHullHeight; }					// height of bot's collision hull when crouched

	unsigned int GetSolidMask( void ) override { return SolidMask; }					// return the bot's collision mask (hack until we get a general hull trace abstraction here or in the locomotion interface)
#if SOURCE_ENGINE == SE_TF2
	unsigned int GetCollisionGroup( void ) override { return CollisionGroup; }
#endif

	static IBodyCustom *create(INextBot *bot, bool reg, IdentityToken_t *id)
	{
		return new IBodyCustom(bot, reg, id);
	}
};

SH_DECL_HOOK0(IVision, GetMaxVisionRange, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(IVision, GetMinRecognizeTime, SH_NOATTRIB, 0, float);
SH_DECL_HOOK0(IVision, GetDefaultFieldOfView, SH_NOATTRIB, 0, float);

#if SOURCE_ENGINE == SE_LEFT4DEAD2
class ZombieBotVision : public IVision
{
public:
	char pad1[36];
};
#endif

#if SOURCE_ENGINE == SE_TF2
#define GameVision IVision
#define GameVisionCustom IVisionCustom
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
#define GameVision ZombieBotVision
#define GameVisionCustom ZombieBotVisionCustom
#endif

struct customvision_base_vars_t : IPluginNextBotComponent
{
public:
	customvision_base_vars_t(IdentityToken_t *id)
		: IPluginNextBotComponent{id} {}
	
	virtual ~customvision_base_vars_t() {}
	
	float maxrange = 2000.0;
	float minreco = 0.0;
	float deffov = 90.0;
	spvarmap_t data{};
	
	virtual void plugin_unloaded()
	{
		IPluginNextBotComponent::plugin_unloaded();
	}
	
	virtual bool set_function(std::string_view name, IPluginFunction *func) override
	{
		return IPluginNextBotComponent::set_function(name, func);
	}
	
	virtual void remove_hooks(IVision *bytes)
	{
		SH_REMOVE_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(this, &customvision_base_vars_t::dtor), false);
	
		SH_REMOVE_HOOK(IVision, GetMaxVisionRange, bytes, SH_MEMBER(this, &customvision_base_vars_t::HookGetMaxVisionRange), false);
		SH_REMOVE_HOOK(IVision, GetMinRecognizeTime, bytes, SH_MEMBER(this, &customvision_base_vars_t::HookGetMinRecognizeTime), false);
		SH_REMOVE_HOOK(IVision, GetDefaultFieldOfView, bytes, SH_MEMBER(this, &customvision_base_vars_t::HookGetDefaultFieldOfView), false);
	}
	
	void dtor()
	{
		IVision *loc = META_IFACEPTR(IVision);
		
		remove_hooks(loc);
		
		this->~customvision_base_vars_t();
		
		RETURN_META(MRES_IGNORED);
	}
	
	float HookGetMaxVisionRange() { RETURN_META_VALUE(MRES_SUPERCEDE, maxrange); }
	float HookGetMinRecognizeTime() { RETURN_META_VALUE(MRES_SUPERCEDE, minreco); }
	float HookGetDefaultFieldOfView() { RETURN_META_VALUE(MRES_SUPERCEDE, deffov); }
	
	virtual void add_hooks(IVision *bytes)
	{
		SH_ADD_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(this, &customvision_base_vars_t::dtor), false);
	
		SH_ADD_HOOK(IVision, GetMaxVisionRange, bytes, SH_MEMBER(this, &customvision_base_vars_t::HookGetMaxVisionRange), false);
		SH_ADD_HOOK(IVision, GetMinRecognizeTime, bytes, SH_MEMBER(this, &customvision_base_vars_t::HookGetMinRecognizeTime), false);
		SH_ADD_HOOK(IVision, GetDefaultFieldOfView, bytes, SH_MEMBER(this, &customvision_base_vars_t::HookGetDefaultFieldOfView), false);
	}
};

class GameVisionCustom : public GameVision
{
public:
#if SOURCE_ENGINE == SE_TF2
	static void *getctorptr() { return IVisionCTOR; }
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	static void *getctorptr() { return ZombieBotVisionCTOR; }
#endif

	struct vars_t : customvision_base_vars_t
	{
		vars_t(IdentityToken_t *id)
			: customvision_base_vars_t{id} {}
	};
	
	unsigned char *vars_ptr()
	{ return (((unsigned char *)this) + sizeof(GameVision)); }
	vars_t &getvars()
	{ return *(vars_t *)vars_ptr(); }
	
	static GameVisionCustom *create(INextBot *bot, bool reg, IdentityToken_t *id)
	{
		GameVisionCustom *bytes = (GameVisionCustom *)calloc(1, sizeof(GameVision) + sizeof(vars_t));
		call_mfunc<void, GameVision, INextBot *>(bytes, getctorptr(), bot);
		
		new (bytes->vars_ptr()) vars_t(id);
		
		bytes->getvars().add_hooks(bytes);

		if(!reg) {
			bot->m_componentList = bytes->m_nextComponent;
			bytes->m_nextComponent = nullptr;
		}
		
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

ConVar path_compute_debug("path_compute_debug", "0");
#if SOURCE_ENGINE == SE_LEFT4DEAD2
ConVar path_compute_manual("path_compute_manual", "0");
#endif

class Path
{
public:
	virtual ~Path() = 0;
	
	static Path *create()
	{
		Path *bytes = (Path *)calloc(1, sizeof(Path));
		call_mfunc<void>(bytes, PathCTOR);
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

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	virtual void SetSubject( CBaseEntity * ) = 0;
#endif
	
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
	
	virtual void ComputeAreaCrossing( INextBot *bot, const CNavArea *from, const Vector &fromPos, const CNavArea *to, NavDirType dir, Vector *crossPos ) = 0;
	
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
	bool Compute(INextBot *bot, CBaseCombatCharacter *subject, CostFunctor &costFunc, float maxPathLength, bool includeGoalIfPathFails = true)
	{
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		if(path_compute_manual.GetBool() || path_compute_debug.GetBool()) {
#endif
		Invalidate();
		
		m_subject = subject;
		
		const Vector &start = bot->GetPosition();
		
		CNavArea *startArea = bot->GetEntity()->GetLastKnownArea();
		if ( !startArea )
		{
			OnPathChanged( bot, NO_PATH );
			if(path_compute_debug.GetBool()) {
				DevMsg("%f: Path->Compute #%i failed: bot has no last known area\n", gpGlobals->curtime, subject->entindex());
			}
			return false;
		}

		CNavArea *subjectArea = subject->GetLastKnownArea();
		if ( !subjectArea )
		{
			OnPathChanged( bot, NO_PATH );
			if(path_compute_debug.GetBool()) {
				DevMsg("%f: Path->Compute #%i failed: subject has no last known area\n", gpGlobals->curtime, subject->entindex());
			}
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
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		bool pathResult = NavAreaBuildPath_real( startArea, subjectArea, &start, &subjectPos, costFunc, &closestArea, maxPathLength, bot->GetEntity()->GetTeamNumber(), false );
#elif SOURCE_ENGINE == SE_TF2
		bool pathResult = NavAreaBuildPath_real( startArea, subjectArea, &subjectPos, costFunc, &closestArea, maxPathLength, bot->GetEntity()->GetTeamNumber(), false );
#endif
		if(path_compute_debug.GetBool()) {
			DevMsg("%f: Path->Compute #%i built: %i\n", gpGlobals->curtime, subject->entindex(), pathResult);
		}
		
		// Failed?
		if ( closestArea == NULL ) {
			if(path_compute_debug.GetBool()) {
				DevMsg("%f: Path->Compute #%i failed: no closest area\n", gpGlobals->curtime, subject->entindex());
			}
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
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		} else {
			return call_mfunc<bool, Path, INextBot *, CBaseCombatCharacter *, CostFunctor &, float>(this, PathComputeEntity, bot, subject, costFunc, maxPathLength);
		}
#endif
	}
	
	template <typename CostFunctor>
	bool Compute(INextBot *bot, const Vector &goal, CostFunctor &costFunc, float maxPathLength, bool includeGoalIfPathFails = true)
	{
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		if(path_compute_manual.GetBool() || path_compute_debug.GetBool()) {
#endif
		Invalidate();
		
		const Vector &start = bot->GetPosition();
		
		CNavArea *startArea = bot->GetEntity()->GetLastKnownArea();
		if ( !startArea )
		{
			OnPathChanged( bot, NO_PATH );
			if(path_compute_debug.GetBool()) {
				DevMsg("%f: Path->Compute [%f, %f, %f] failed: bot has no last known area\n", gpGlobals->curtime, goal.x, goal.y, goal.z);
			}
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
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		bool pathResult = NavAreaBuildPath_real( startArea, goalArea, &start, &goal, costFunc, &closestArea, maxPathLength, bot->GetEntity()->GetTeamNumber(), false );
#elif SOURCE_ENGINE == SE_TF2
		bool pathResult = NavAreaBuildPath_real( startArea, goalArea, &goal, costFunc, &closestArea, maxPathLength, bot->GetEntity()->GetTeamNumber(), false );
#endif
		if(path_compute_debug.GetBool()) {
			DevMsg("%f: Path->Compute [%f, %f, %f] built: %i\n", gpGlobals->curtime, goal.x, goal.y, goal.z, pathResult);
		}
		
		// Failed?
		if ( closestArea == NULL ) {
			if(path_compute_debug.GetBool()) {
				DevMsg("%f: Path->Compute [%f, %f, %f] failed: no closest area\n", gpGlobals->curtime, goal.x, goal.y, goal.z);
			}
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
			if(path_compute_debug.GetBool()) {
				DevMsg("%f: Path->Compute [%f, %f, %f] failed: compute details failed\n", gpGlobals->curtime, goal.x, goal.y, goal.z);
			}
			return false;
		}

		// remove redundant nodes and clean up path
		Optimize( bot );
		
		PostProcess();

		OnPathChanged( bot, pathResult ? COMPLETE_PATH : PARTIAL_PATH );

		return pathResult;
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		} else {
			return call_mfunc<bool, Path, INextBot *, const Vector &, CostFunctor &, float>(this, PathComputeVector, bot, goal, costFunc, maxPathLength);
		}
#endif
	}
	
	bool ComputePathDetails( INextBot *bot, const Vector &start )
	{
		return call_mfunc<bool, Path, INextBot *, const Vector &>(this, PathComputePathDetails, bot, start);
	}
	
	bool BuildTrivialPath( INextBot *bot, const Vector &goal )
	{
		return call_mfunc<bool, Path, INextBot *, const Vector &>(this, PathBuildTrivialPath, bot, goal);
	}
	
	int FindNextOccludedNode( INextBot *bot, int anchorIndex )
	{
		return call_mfunc<int, Path, INextBot *, int>(this, PathFindNextOccludedNode, bot, anchorIndex);
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
		call_mfunc<void>(this, PathPostProcess);
	}
	
	void AssemblePrecomputedPath( INextBot *bot, const Vector &goal, CNavArea *endArea )
	{
		const Vector &start = bot->GetPosition();

		// get count
		int count = 0;
		CNavArea *area;
		for( area = endArea; area; area = area->GetParent() )
		{
			++count;
		}

		// save room for endpoint
		if ( count > MAX_PATH_SEGMENTS-1 )
		{
			count = MAX_PATH_SEGMENTS-1;
		}
		else if ( count == 0 )
		{
			return;
		}

		if ( count == 1 )
		{
			BuildTrivialPath( bot, goal );
			return;
		}

		// assemble path
		m_segmentCount = count;
		for( area = endArea; count && area; area = area->GetParent() )
		{
			--count;
			m_path[ count ].area = area;
			m_path[ count ].how = area->GetParentHow();
			m_path[ count ].type = ON_GROUND;
		}

		// append actual goal position
		m_path[ m_segmentCount ].area = endArea;
		m_path[ m_segmentCount ].pos = goal;
		m_path[ m_segmentCount ].ladder = NULL;
		m_path[ m_segmentCount ].how = NUM_TRAVERSE_TYPES;
		m_path[ m_segmentCount ].type = ON_GROUND;
		++m_segmentCount;

		// compute path positions
		if ( ComputePathDetails( bot, start ) == false )
		{
			Invalidate();
			OnPathChanged( bot, NO_PATH );
			return;
		}

		// remove redundant nodes and clean up path
		Optimize( bot );

		PostProcess();

		OnPathChanged( bot, COMPLETE_PATH );
	}
};

DETOUR_DECL_MEMBER1(PathOptimize, void, INextBot *, bot)
{
	((Path *)this)->Optimize(bot);
}

class NextBotTraceFilterOnlyActors : public CTraceFilterSimple
{
public:
	NextBotTraceFilterOnlyActors( const IHandleEntity *passentity, int collisionGroup )
		: CTraceFilterSimple( passentity, collisionGroup )
	{
	}

	virtual TraceType_t	GetTraceType() const
	{
		return TRACE_ENTITIES_ONLY;
	}

	virtual bool ShouldHitEntity( IHandleEntity *pServerEntity, int contentsMask )
	{
		if ( CTraceFilterSimple::ShouldHitEntity( pServerEntity, contentsMask ) )
		{
			CBaseEntity *entity = EntityFromEntityHandle( pServerEntity );

#if SOURCE_ENGINE == SE_LEFT4DEAD2
			CBasePlayer *player = entity->IsPlayer();
			if ( player && player->IsGhost() )
				return false;
#endif

			return ( entity->MyNextBotPointer() || entity->IsPlayer() );
		}
		return false;
	}
};

bool CGameTrace::DidHitWorld() const
{
	return m_pEnt && gamehelpers->EntityToBCompatRef(m_pEnt) == 0;
}

bool CGameTrace::DidHitNonWorldEntity() const
{
	return m_pEnt != NULL && !DidHitWorld();
}

class PathFollower : public Path
{
public:
	static PathFollower *create()
	{
		PathFollower *bytes = (PathFollower *)calloc(1, sizeof(PathFollower));
		call_mfunc<void>(bytes, PathFollowerCTOR);
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

#if SOURCE_ENGINE == SE_TF2
	float m_goalTolerance;
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	char pad1[16];
#endif
	
#if SOURCE_ENGINE == SE_TF2
	void SetGoalTolerance(float val)
	{
		m_goalTolerance = val;
	}
	
	float GetGoalTolerance()
	{
		return m_goalTolerance;
	}
#endif
	
	float GetMinLookAheadDistance()
	{
		return m_minLookAheadRange;
	}
	
	bool IsAtGoal( INextBot *bot ) const
	{
		ILocomotion *mover = bot->GetLocomotionInterface();
		IBody *body = bot->GetBodyInterface();

		//
		// m_goal is the node we are moving toward along the path
		// current is the node just behind us
		//
		const Segment *current = PriorSegment( m_goal );
		Vector toGoal = m_goal->pos - mover->GetFeet();

	// 	if ( m_goal->type == JUMP_OVER_GAP && !mover->IsOnGround() )
	// 	{
	// 		// jumping over a gap, don't skip ahead until we land
	// 		return false;
	// 	}

		if ( current == NULL )
		{
			// passed goal
			return true;
		}
		else if ( m_goal->type == DROP_DOWN )
		{
			// m_goal is the top of the drop-down, and the following segment is the landing point
			const Segment *landing = NextSegment( m_goal );

			if ( landing == NULL )
			{
				// passed goal or corrupt path
				return true;
			}		
			else
			{
				// did we reach the ground
				if ( mover->GetFeet().z - landing->pos.z < mover->GetStepHeight() )
				{
					// reached goal
					return true;
				}
			}
					
			/// @todo: it is possible to fall into a bad place and get stuck - should move back onto the path
			
		}
		else if ( m_goal->type == CLIMB_UP )
		{
			// once jump is started, assume it is successful, since
			// nav mesh may be substantially off from actual ground height at landing
			const Segment *landing = NextSegment( m_goal );

			if ( landing == NULL )
			{
				// passed goal or corrupt path
				return true;
			}		
			else if ( /*!mover->IsOnGround() && */ mover->GetFeet().z > m_goal->pos.z + mover->GetStepHeight() )
			{
				// we're off the ground, presumably climbing - assume we reached the goal
				return true;
			}
			/* This breaks infected climbing up holes in the ceiling - they can get within 2D range of m_goal before finding a ledge to climb up to
			else if ( mover->IsOnGround() )
			{
				// proximity check
				// Z delta can be anything, since we may be climbing over a tall fence, a physics prop, etc.
				const float rangeTolerance = 10.0f;
				if ( toGoal.AsVector2D().IsLengthLessThan( rangeTolerance ) )
				{
					// reached goal
					return true;
				}
			}
			*/
		}
		else
		{
			const Segment *next = NextSegment( m_goal );

			if ( next )
			{
				// because mover may be off the path, check if it crossed the plane of the goal
				// check against average of current and next forward vectors
				Vector2D dividingPlane;

				if ( current->ladder )
				{
					dividingPlane = m_goal->forward.AsVector2D();
				}
				else
				{
					dividingPlane = current->forward.AsVector2D() + m_goal->forward.AsVector2D();
				}

				if ( DotProduct2D( toGoal.AsVector2D(), dividingPlane ) < 0.0001f &&
					abs( toGoal.z ) < body->GetStandHullHeight() )
				{	
					// only skip higher Z goal if next goal is directly reachable
					// can't use this for positions below us because we need to be able
					// to climb over random objects along our path that we can't actually
					// move *through*
					if ( toGoal.z < mover->GetStepHeight() && ( mover->IsPotentiallyTraversable( mover->GetFeet(), next->pos ) && !mover->HasPotentialGap( mover->GetFeet(), next->pos ) ) )
					{
						// passed goal
						return true;
					}
				}
			}

#if SOURCE_ENGINE == SE_TF2
			// proximity check
			// Z delta can be anything, since we may be climbing over a tall fence, a physics prop, etc.
			if ( toGoal.AsVector2D().IsLengthLessThan( m_goalTolerance ) )
			{
				// reached goal
				return true;
			}
#endif
		}

		return false;	
	}
};

void PathFollower::Update( INextBot *bot )
{
	static int vtableindex = -1;
	if(vtableindex == -1) {
		vtableindex = vfunc_index(&PathFollower::Update);
	}
	
	call_vfunc<void, PathFollower, INextBot *>(this, vtableindex, bot);
}

SH_DECL_HOOK0_void(Path, Invalidate, SH_NOATTRIB, 0);

class DirectChasePath;

class ChasePath : public PathFollower
{
public:
	enum SubjectChaseType
	{
		LEAD_SUBJECT,
		DONT_LEAD_SUBJECT
	};
	
	CountdownTimer m_failTimer;							// throttle re-pathing if last path attempt failed
	CountdownTimer m_throttleTimer;						// require a minimum time between re-paths
	CountdownTimer m_lifetimeTimer;
	EHANDLE m_lastPathSubject;							// the subject used to compute the current/last path
	SubjectChaseType m_chaseHow;
	
	using IsRepathNeeded_t = bool (ChasePath::*)( INextBot *, CBaseEntity * );
	using Update_t = void (ChasePath::*)( INextBot *, CBaseEntity *, const IPathCost &, Vector * );
	using ComputeAreaCrossing_t = void (DirectChasePath::*)( INextBot *bot, const CNavArea *, const Vector &, const CNavArea *, NavDirType, Vector * );
	using RemoveHooks_t = void (ChasePath::*)();
	
	struct vars_t
	{
		float radius = 500.0f;
		float maxlen = 0.0f;
		float life = 0.0f;
		float tolerancerate = 0.33f;
		float mintolerance = 0.0f;
		float repathtime = 0.5f;
		
		IsRepathNeeded_t pIsRepathNeeded = nullptr;
		Update_t pUpdate = nullptr;
		ComputeAreaCrossing_t pComputeAreaCrossing = nullptr;
		RemoveHooks_t pRemoveHooks = nullptr;
	};
	
	unsigned char *vars_ptr()
	{ return (((unsigned char *)this) + sizeof(ChasePath)); }
	vars_t &getvars()
	{ return *(vars_t *)vars_ptr(); }
	
	void HookInvalidate()
	{
		m_throttleTimer.Invalidate();
		m_lifetimeTimer.Invalidate();
		
		RETURN_META(MRES_IGNORED);
	}
	
	void ctor(SubjectChaseType chaseHow)
	{
		new (&m_failTimer) CountdownTimer();
		new (&m_throttleTimer) CountdownTimer();
		new (&m_lifetimeTimer) CountdownTimer();
		new (&m_lastPathSubject) EHANDLE();
		
		m_failTimer.Invalidate();
		m_throttleTimer.Invalidate();
		m_lifetimeTimer.Invalidate();
		m_lastPathSubject = NULL;
		m_chaseHow = chaseHow;
	}
	
	void RemoveHooks()
	{
		SH_REMOVE_HOOK(Path, Invalidate, this, SH_MEMBER(this, &ChasePath::HookInvalidate), false);
	}
	
	void dtor()
	{
		(this->*getvars().pRemoveHooks)();
		
		m_failTimer.~CountdownTimer();
		m_throttleTimer.~CountdownTimer();
		m_lifetimeTimer.~CountdownTimer();
		m_lastPathSubject.~EHANDLE();
		
		getvars().~vars_t();
		
		SH_REMOVE_MANUALHOOK(GenericDtor, this, SH_MEMBER(this, &ChasePath::dtor), false);
		
		RETURN_META(MRES_IGNORED);
	}
	
	Vector PredictSubjectPosition( INextBot *bot, CBaseEntity *subject )
	{
		ILocomotion *mover = bot->GetLocomotionInterface();

		const Vector &subjectPos = subject->GetAbsOrigin();

		Vector to = subjectPos - bot->GetPosition();
		to.z = 0.0f;
		float flRangeSq = to.LengthSqr();

		// don't lead if subject is very far away
		float flLeadRadiusSq = GetLeadRadius();
		flLeadRadiusSq *= flLeadRadiusSq;
		if ( flRangeSq > flLeadRadiusSq )
			return subjectPos;

		// Normalize in place
		float range = sqrt( flRangeSq );
		to /= ( range + 0.0001f );	// avoid divide by zero

		// estimate time to reach subject, assuming maximum speed
		float leadTime = 0.5f + ( range / ( mover->GetRunSpeed() + 0.0001f ) );
		
		// estimate amount to lead the subject	
		Vector lead = leadTime * subject->GetAbsVelocity();
		lead.z = 0.0f;

		if ( DotProduct( to, lead ) < 0.0f )
		{
			// the subject is moving towards us - only pay attention 
			// to his perpendicular velocity for leading
			Vector2D to2D = to.AsVector2D();
			to2D.NormalizeInPlace();

			Vector2D perp( -to2D.y, to2D.x );

			float enemyGroundSpeed = lead.x * perp.x + lead.y * perp.y;

			lead.x = enemyGroundSpeed * perp.x;
			lead.y = enemyGroundSpeed * perp.y;
		}

		// compute our desired destination
		Vector pathTarget = subjectPos + lead;

		// validate this destination

		// don't lead through walls
		if ( lead.LengthSqr() > 36.0f )
		{
			float fraction;
			if ( !mover->IsPotentiallyTraversable( subjectPos, pathTarget, ILocomotion::TraverseWhenType::IMMEDIATELY, &fraction ) )
			{
				// tried to lead through an unwalkable area - clip to walkable space
				pathTarget = subjectPos + fraction * ( pathTarget - subjectPos );
			}
		}

		// don't lead over cliffs
		CNavArea *leadArea = NULL;

		CBaseCombatCharacter *pBCC = subject->MyCombatCharacterPointer();
		if ( pBCC && CloseEnough( pathTarget, subjectPos, 3.0 ) )
		{
			pathTarget = subjectPos;
			leadArea = pBCC->GetLastKnownArea(); // can return null?
		}
		else
		{
			struct CacheEntry_t
			{
				CacheEntry_t() : pArea(NULL) {}
				Vector target;
				CNavArea *pArea;
			};

			static int iServer;
			static CacheEntry_t cache[4];
			static int iNext;
			int i;

			bool bFound = false;
			if ( iServer != gpGlobals->serverCount )
			{
				for ( i = 0; i < ARRAYSIZE(cache); i++ )
				{
					cache[i].pArea = NULL;
				}
				iServer = gpGlobals->serverCount;
			}
			else
			{
				for ( i = 0; i < ARRAYSIZE(cache); i++ )
				{
					if ( cache[i].pArea && CloseEnough( cache[i].target, pathTarget, 2.0 ) )
					{
						pathTarget = cache[i].target;
						leadArea = cache[i].pArea;
						bFound = true;
						break;
					}
				}
			}

			if ( !bFound )
			{
				leadArea = TheNavMesh->GetNearestNavArea( pathTarget );
				if ( leadArea )
				{
					cache[iNext].target = pathTarget;
					cache[iNext].pArea = leadArea;
					iNext = ( iNext + 1 ) % ARRAYSIZE( cache );
				}
			}
		}


		if ( !leadArea || leadArea->GetZ( pathTarget.x, pathTarget.y ) < pathTarget.z - mover->GetMaxJumpHeight() )
		{
			// would fall off a cliff
			return subjectPos;		
		}
		
		/** This needs more thought - it is preventing bots from using dropdowns
		if ( mover->HasPotentialGap( subjectPos, pathTarget, &fraction ) )
		{
			// tried to lead over a cliff - clip to safe region
			pathTarget = subjectPos + fraction * ( pathTarget - subjectPos );
		}
		*/
		
		return pathTarget;
	}
	
	bool IsRepathNeeded( INextBot *bot, CBaseEntity *subject )
	{
		// the closer we get, the more accurate our path needs to be
		Vector to = subject->GetAbsOrigin() - bot->GetPosition();

		float tolerance = getvars().mintolerance + getvars().tolerancerate * to.Length();

		return ( subject->GetAbsOrigin() - GetEndPosition() ).IsLengthGreaterThan( tolerance );
	}
	
	void RefreshPath( INextBot *bot, CBaseEntity *subject, const IPathCost &cost, Vector *pPredictedSubjectPos )
	{
		ILocomotion *mover = bot->GetLocomotionInterface();

		// don't change our path if we're on a ladder
		if ( IsValid() && mover->IsUsingLadder() )
		{
			if ( bot->IsDebugging( NEXTBOT_PATH ) )
			{
				static float lastprint = 0.0f;
				if(lastprint <= gpGlobals->curtime) {
					DevMsg( "%3.2f: bot(#%d) ChasePath::RefreshPath failed. Bot is on a ladder.\n", gpGlobals->curtime, bot->GetEntity()->entindex() );
					lastprint = gpGlobals->curtime + 2.0f;
				}
			}

			// don't allow repath until a moment AFTER we have left the ladder
			m_throttleTimer.Start( 1.0f );

			return;
		}

		if ( subject == NULL )
		{
			if ( bot->IsDebugging( NEXTBOT_PATH ) )
			{
				static float lastprint = 0.0f;
				if(lastprint <= gpGlobals->curtime) {
					DevMsg( "%3.2f: bot(#%d) CasePath::RefreshPath failed. No subject.\n", gpGlobals->curtime, bot->GetEntity()->entindex() );
					lastprint = gpGlobals->curtime + 2.0f;
				}
			}
			return;
		}

		if ( !m_failTimer.IsElapsed() )
		{
	 		if ( bot->IsDebugging( NEXTBOT_PATH ) )
	 		{
				static float lastprint = 0.0f;
				if(lastprint <= gpGlobals->curtime) {
					DevMsg( "%3.2f: bot(#%d) ChasePath::RefreshPath failed. Fail timer not elapsed.\n", gpGlobals->curtime, bot->GetEntity()->entindex() );
					lastprint = gpGlobals->curtime + 2.0f;
				}
	 		}
			return;
		}

		// if our path subject changed, repath immediately
		if ( subject != m_lastPathSubject )
		{
			if ( bot->IsDebugging( NEXTBOT_PATH ) )
			{
				static float lastprint = 0.0f;
				if(lastprint <= gpGlobals->curtime) {
					DevMsg( "%3.2f: bot(#%d) Chase path subject changed (from %p (#%d) to %p (#%d)).\n", gpGlobals->curtime, bot->GetEntity()->entindex(), m_lastPathSubject.Get(), m_lastPathSubject.Get() ? m_lastPathSubject.Get()->entindex() : -1, subject, subject ? subject->entindex() : -1 );
					lastprint = gpGlobals->curtime + 2.0f;
				}
			}

			Invalidate();

			// new subject, fresh attempt
			m_failTimer.Invalidate();
		}

		if ( IsValid() && !m_throttleTimer.IsElapsed() )
		{
			// require a minimum time between repaths, as long as we have a path to follow
	 		if ( bot->IsDebugging( NEXTBOT_PATH ) )
	 		{
				static float lastprint = 0.0f;
				if(lastprint <= gpGlobals->curtime) {
					DevMsg( "%3.2f: bot(#%d) ChasePath::RefreshPath failed. Rate throttled.\n", gpGlobals->curtime, bot->GetEntity()->entindex() );
					lastprint = gpGlobals->curtime + 2.0f;
				}
	 		}
			return;
		}

		if ( IsValid() && m_lifetimeTimer.HasStarted() && m_lifetimeTimer.IsElapsed() )
		{
			// this path's lifetime has elapsed
			Invalidate();
		}
		
		if ( !IsValid() || (this->*getvars().pIsRepathNeeded)( bot, subject ) )
		{
			// the situation has changed - try a new path
			bool isPath;
			Vector pathTarget = subject->GetAbsOrigin();

			if ( m_chaseHow == LEAD_SUBJECT )
			{
				pathTarget = pPredictedSubjectPos ? *pPredictedSubjectPos : PredictSubjectPosition( bot, subject );
				isPath = Compute( bot, pathTarget, cost, GetMaxPathLength() );
			}
			else if ( subject->MyCombatCharacterPointer() && subject->MyCombatCharacterPointer()->GetLastKnownArea() )
			{
				isPath = Compute( bot, subject->MyCombatCharacterPointer(), cost, GetMaxPathLength() );
			}
			else
			{
				isPath = Compute( bot, pathTarget, cost, GetMaxPathLength() );
			}

			if ( isPath )
			{
				if ( bot->IsDebugging( NEXTBOT_PATH ) )
				{
					const float size = 20.0f;			
					NDebugOverlay::VertArrow( bot->GetPosition() + Vector( 0, 0, size ), bot->GetPosition(), size, 255, RandomInt( 0, 200 ), 255, 255, true, 30.0f );

					static float lastprint = 0.0f;
					if(lastprint <= gpGlobals->curtime) {
						DevMsg( "%3.2f: bot(#%d) REPATH\n", gpGlobals->curtime, bot->GetEntity()->entindex() );
						lastprint = gpGlobals->curtime + 2.0f;
					}
				}

				m_lastPathSubject = subject;

				m_throttleTimer.Start( getvars().repathtime );

				// track the lifetime of this new path
				float lifetime = GetLifetime();
				if ( lifetime > 0.0f )
				{
					m_lifetimeTimer.Start( lifetime );
				}
				else
				{
					m_lifetimeTimer.Invalidate();
				}
			}
			else
			{
				// can't reach subject - throttle retry based on range to subject
				float failtime = 0.005f * ( bot->GetRangeTo( subject ) );
				
				m_failTimer.Start( failtime );
				
				// allow bot to react to path failure
				bot->OnMoveToFailure( this, FAIL_NO_PATH_EXISTS );

				if ( bot->IsDebugging( NEXTBOT_PATH ) )
				{
					const float size = 20.0f;	
					const float dT = 90.0f;		
					int c = RandomInt( 0, 100 );
					NDebugOverlay::VertArrow( bot->GetPosition() + Vector( 0, 0, size ), bot->GetPosition(), size, 255, c, c, 255, true, dT );
					NDebugOverlay::HorzArrow( bot->GetPosition(), pathTarget, 5.0f, 255, c, c, 255, true, dT );

					static float lastprint = 0.0f;
					if(lastprint <= gpGlobals->curtime) {
						DevMsg( "%3.2f: bot(#%d) REPATH FAILED %f\n", gpGlobals->curtime, bot->GetEntity()->entindex(), failtime );
						lastprint = gpGlobals->curtime + 2.0f;
					}
				}

				Invalidate();
			}
		}
	}
	
	static ChasePath *create(SubjectChaseType chaseHow)
	{
		ChasePath *bytes = (ChasePath *)calloc(1, sizeof(ChasePath) + sizeof(vars_t));
		call_mfunc<void>(bytes, PathFollowerCTOR);
		new (bytes->vars_ptr()) vars_t();
		bytes->ctor(chaseHow);
		SH_ADD_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(bytes, &ChasePath::dtor), false);
		SH_ADD_HOOK(Path, Invalidate, bytes, SH_MEMBER(bytes, &ChasePath::HookInvalidate), false);
		bytes->getvars().pIsRepathNeeded = &ChasePath::IsRepathNeeded;
		bytes->getvars().pUpdate = &ChasePath::Update;
		bytes->getvars().pRemoveHooks = &ChasePath::RemoveHooks;
		return bytes;
	}
	
	float GetLeadRadius( void ) 
	{ 
		return getvars().radius; // 1000.0f; 
	}

	float GetMaxPathLength( void )
	{
		// no limit
		return getvars().maxlen;
	}

	float GetLifetime( void )
	{
		// infinite duration
		return getvars().life;
	}
	
	void Update( INextBot *bot, CBaseEntity *subject, const IPathCost &cost, Vector *pPredictedSubjectPos )
	{
		// maintain the path to the subject
		RefreshPath( bot, subject, cost, pPredictedSubjectPos );

		// move along the path towards the subject
		PathFollower::Update( bot );
	}
};

SH_DECL_HOOK6_void(Path, ComputeAreaCrossing, SH_NOATTRIB, 0, INextBot *, const CNavArea *, const Vector &, const CNavArea *, NavDirType, Vector *);

class DirectChasePath : public ChasePath
{
public:
	void RemoveHooks()
	{
		ChasePath::RemoveHooks();
		
		SH_REMOVE_HOOK(Path, ComputeAreaCrossing, this, SH_MEMBER(this, &DirectChasePath::HookComputeAreaCrossing), false);
	}
	
	static DirectChasePath *create(SubjectChaseType chaseHow)
	{
		DirectChasePath *bytes = (DirectChasePath *)ChasePath::create(chaseHow);
		SH_ADD_HOOK(Path, ComputeAreaCrossing, bytes, SH_MEMBER(bytes, &DirectChasePath::HookComputeAreaCrossing), false);
		bytes->getvars().pIsRepathNeeded = func_to_func<IsRepathNeeded_t>(&DirectChasePath::IsRepathNeeded);
		bytes->getvars().pUpdate = func_to_func<Update_t>(&DirectChasePath::Update);
		bytes->getvars().pComputeAreaCrossing = func_to_func<ComputeAreaCrossing_t>(&DirectChasePath::DoComputeAreaCrossing);
		bytes->getvars().pRemoveHooks = func_to_func<RemoveHooks_t>(&DirectChasePath::RemoveHooks);
		return bytes;
	}
	
	void Update( INextBot *me, CBaseEntity *victim, const IPathCost &pathCost, Vector *pPredictedSubjectPos = NULL )	// update path to chase target and move bot along path
	{
		Assert( !pPredictedSubjectPos );
		bool bComputedPredictedPosition;
		Vector vecPredictedPosition;
		if ( !DirectChase( &bComputedPredictedPosition, &vecPredictedPosition, me, victim ) )
		{
			// path around obstacles to reach our victim
			ChasePath::Update( me, victim, pathCost, bComputedPredictedPosition ? &vecPredictedPosition : NULL );
		}
		NotifyVictim( me, victim );
	}
	
	bool IsRepathNeeded( INextBot *bot, CBaseEntity *subject )			// return true if situation has changed enough to warrant recomputing the current path
	{
		if ( ChasePath::IsRepathNeeded( bot, subject ) )
		{
			return true;
		}

		return bot->GetLocomotionInterface()->IsStuck() && bot->GetLocomotionInterface()->GetStuckDuration() > 2.0f;
	}
	
	void NotifyVictim( INextBot *me, CBaseEntity *victim )
	{
		CBaseCombatCharacter *pBCCVictim = victim->MyCombatCharacterPointer();
		if ( !pBCCVictim )
			return;
		
		//pBCCVictim->OnPursuedBy( me );
	}
	
	bool DirectChase( bool *pPredictedPositionComputed, Vector *pPredictedPos, INextBot *me, CBaseEntity *victim )		// if there is nothing between us and our victim, run directly at them
	{
		*pPredictedPositionComputed = false;

		ILocomotion *mover = me->GetLocomotionInterface();

		if ( me->IsImmobile() || mover->IsScrambling() )
		{
			return false;
		}

		if ( IsDiscontinuityAhead( me, CLIMB_UP ) )
		{
			return false;
		}

		if ( IsDiscontinuityAhead( me, JUMP_OVER_GAP ) )
		{
			return false;
		}

		Vector leadVictimPos = PredictSubjectPosition( me, victim );

		// Don't want to have to compute the predicted position twice.
		*pPredictedPositionComputed = true;
		*pPredictedPos = leadVictimPos;

		if ( !mover->IsPotentiallyTraversable( mover->GetFeet(), leadVictimPos  ) )
		{
			return false;
		}

		// the way is clear - move directly towards our victim
		mover->FaceTowards( leadVictimPos );
		mover->Approach( leadVictimPos );

		me->GetBodyInterface()->AimHeadTowards( victim );

		// old path is no longer useful since we've moved off of it
		Invalidate();

		return true;
	}
	
	void DoComputeAreaCrossing( INextBot *bot, const CNavArea *from, const Vector &fromPos, const CNavArea *to, NavDirType dir, Vector *crossPos )
	{
		Vector center;
		float halfWidth;
		from->ComputePortal( to, dir, &center, &halfWidth );

		*crossPos = center;
	}
	
	void HookComputeAreaCrossing( INextBot *bot, const CNavArea *from, const Vector &fromPos, const CNavArea *to, NavDirType dir, Vector *crossPos )
	{
		(this->*getvars().pComputeAreaCrossing)(bot, from, fromPos, to, dir, crossPos);
		RETURN_META(MRES_SUPERCEDE);
	}
};

#if SOURCE_ENGINE == SE_LEFT4DEAD2
class InfectedChasePath : public DirectChasePath
{
public:
	static InfectedChasePath *create(SubjectChaseType chaseHow)
	{
		InfectedChasePath *bytes = (InfectedChasePath *)DirectChasePath::create(chaseHow);
		bytes->getvars().pComputeAreaCrossing = func_to_func<ComputeAreaCrossing_t>(&InfectedChasePath::DoComputeAreaCrossing);
		return bytes;
	}
	
	void DoComputeAreaCrossing( INextBot *bot, const CNavArea *from, const Vector &fromPos, const CNavArea *to, NavDirType dir, Vector *crossPos )
	{
		call_mfunc<void, InfectedChasePath, INextBot *, const CNavArea *, const Vector &, const CNavArea *, NavDirType, Vector *>(this, InfectedChasePathComputeAreaCrossing, bot, from, fromPos, to, dir, crossPos);
	}
};
#endif

class RetreatPathBuilder
{
public:
	RetreatPathBuilder( INextBot *me, CBaseEntity *threat, float retreatRange = 500.0f )
	{
		m_me = me;
		m_mover = me->GetLocomotionInterface();
		
		m_threat = threat;
		m_retreatRange = retreatRange;
	}

	CNavArea *ComputePath( void )
	{
		if ( m_mover == NULL )
			return NULL;
		
		CNavArea *startArea = m_me->GetEntity()->GetLastKnownArea();

		if ( startArea == NULL )
			return NULL;

		CNavArea *retreatFromArea = TheNavMesh->GetNearestNavArea( m_threat->GetAbsOrigin() );
		if ( retreatFromArea == NULL )
			return NULL;

		startArea->SetParent( NULL );

		// start search
#if 1
		CNavArea::ClearSearchLists();
#endif

		float initCost = Cost( startArea, NULL, NULL );
		if ( initCost < 0.0f )
			return NULL;

		int teamID = m_me->GetEntity()->GetTeamNumber();

		startArea->SetTotalCost( initCost );

#if 1
		startArea->AddToOpenList();
#endif
		
		// keep track of the area farthest away from the threat
		CNavArea *farthestArea = NULL;
		float farthestRange = 0.0f;

		//
		// Dijkstra's algorithm (since we don't know our goal).
		// Build a path as far away from the retreat area as possible.
		// Minimize total path length and danger.
		// Maximize distance to threat of end of path.
		//
#if 1
		while( !CNavArea::IsOpenListEmpty() )
		{
			// get next area to check
			CNavArea *area = CNavArea::PopOpenList();

			area->AddToClosedList();

			// don't consider blocked areas
			if ( area->IsBlocked( teamID ) )
				continue;

			// build adjacent area array
			CollectAdjacentAreas( area );
			
			// search adjacent areas
			for( int i=0; i<m_adjAreaIndex; ++i )
			{
				CNavArea *newArea = m_adjAreaVector[ i ].area;
				
				// only visit each area once
				if ( newArea->IsClosed() )
					continue;
				
				// don't consider blocked areas
				if ( newArea->IsBlocked( teamID ) )
					continue;

				// don't use this area if it is out of range
				if ( ( newArea->GetCenter() - m_me->GetEntity()->GetAbsOrigin() ).IsLengthGreaterThan( m_retreatRange ) )
					continue;
				
				// determine cost of traversing this area
				float newCost = Cost( newArea, area, m_adjAreaVector[ i ].ladder );
				
				// don't use adjacent area if cost functor says it is a dead-end
				if ( newCost < 0.0f )
					continue;
					
				if ( newArea->IsOpen() && newArea->GetTotalCost() <= newCost )
				{
					// we have already visited this area, and it has a better path
					continue;
				}
				else
				{
					// whether this area has been visited or not, we now have a better path
					newArea->SetParent( area, m_adjAreaVector[ i ].how );
					newArea->SetTotalCost( newCost );

					// use 'cost so far' to hold cumulative cost
					newArea->SetCostSoFar( newCost );

					// tricky bit here - relying on OpenList being sorted by cost
					if ( newArea->IsOpen() )
					{
						// area already on open list, update the list order to keep costs sorted
						newArea->UpdateOnOpenList();
					}
					else
					{
						newArea->AddToOpenList();
					}

					// keep track of area farthest from threat
					float threatRange = ( newArea->GetCenter() - m_threat->GetAbsOrigin() ).Length();
					if ( threatRange > farthestRange )
					{
						farthestArea = newArea;
						farthestRange = threatRange;
					}
				}
			}
		}
#endif

		return farthestArea;
	}


	/**
	 * Build a vector of adjacent areas reachable from the given area
	 */
	void CollectAdjacentAreas( CNavArea *area )
	{
		m_adjAreaIndex = 0;			

		const NavConnectVector &adjNorth = *area->GetAdjacentAreas( NORTH );		
		FOR_EACH_VEC( adjNorth, it )
		{
			if ( m_adjAreaIndex >= MAX_ADJ_AREAS )
				break;

			m_adjAreaVector[ m_adjAreaIndex ].area = adjNorth[ it ].area;
			m_adjAreaVector[ m_adjAreaIndex ].how = GO_NORTH;
			m_adjAreaVector[ m_adjAreaIndex ].ladder = NULL;
			++m_adjAreaIndex;
		}

		const NavConnectVector &adjSouth = *area->GetAdjacentAreas( SOUTH );		
		FOR_EACH_VEC( adjSouth, it )
		{
			if ( m_adjAreaIndex >= MAX_ADJ_AREAS )
				break;

			m_adjAreaVector[ m_adjAreaIndex ].area = adjSouth[ it ].area;
			m_adjAreaVector[ m_adjAreaIndex ].how = GO_SOUTH;
			m_adjAreaVector[ m_adjAreaIndex ].ladder = NULL;
			++m_adjAreaIndex;
		}

		const NavConnectVector &adjWest = *area->GetAdjacentAreas( WEST );		
		FOR_EACH_VEC( adjWest, it )
		{
			if ( m_adjAreaIndex >= MAX_ADJ_AREAS )
				break;

			m_adjAreaVector[ m_adjAreaIndex ].area = adjWest[ it ].area;
			m_adjAreaVector[ m_adjAreaIndex ].how = GO_WEST;
			m_adjAreaVector[ m_adjAreaIndex ].ladder = NULL;
			++m_adjAreaIndex;
		}

		const NavConnectVector &adjEast = *area->GetAdjacentAreas( EAST );	
		FOR_EACH_VEC( adjEast, it )
		{
			if ( m_adjAreaIndex >= MAX_ADJ_AREAS )
				break;

			m_adjAreaVector[ m_adjAreaIndex ].area = adjEast[ it ].area;
			m_adjAreaVector[ m_adjAreaIndex ].how = GO_EAST;
			m_adjAreaVector[ m_adjAreaIndex ].ladder = NULL;
			++m_adjAreaIndex;
		}

		const NavLadderConnectVector &adjUpLadder = *area->GetLadders( CNavLadder::LADDER_UP );
		FOR_EACH_VEC( adjUpLadder, it )
		{
			CNavLadder *ladder = adjUpLadder[ it ].ladder;

			if ( ladder->m_topForwardArea && m_adjAreaIndex < MAX_ADJ_AREAS )
			{
				m_adjAreaVector[ m_adjAreaIndex ].area = ladder->m_topForwardArea;
				m_adjAreaVector[ m_adjAreaIndex ].how = GO_LADDER_UP;
				m_adjAreaVector[ m_adjAreaIndex ].ladder = ladder;
				++m_adjAreaIndex;
			}

			if ( ladder->m_topLeftArea && m_adjAreaIndex < MAX_ADJ_AREAS )
			{
				m_adjAreaVector[ m_adjAreaIndex ].area = ladder->m_topLeftArea;
				m_adjAreaVector[ m_adjAreaIndex ].how = GO_LADDER_UP;
				m_adjAreaVector[ m_adjAreaIndex ].ladder = ladder;
				++m_adjAreaIndex;
			}

			if ( ladder->m_topRightArea && m_adjAreaIndex < MAX_ADJ_AREAS )
			{
				m_adjAreaVector[ m_adjAreaIndex ].area = ladder->m_topRightArea;
				m_adjAreaVector[ m_adjAreaIndex ].how = GO_LADDER_UP;
				m_adjAreaVector[ m_adjAreaIndex ].ladder = ladder;
				++m_adjAreaIndex;
			}
		}

		const NavLadderConnectVector &adjDownLadder = *area->GetLadders( CNavLadder::LADDER_DOWN );
		FOR_EACH_VEC( adjDownLadder, it )
		{
			CNavLadder *ladder = adjDownLadder[ it ].ladder;

			if ( m_adjAreaIndex >= MAX_ADJ_AREAS )
				break;

			if ( ladder->m_bottomArea )
			{
				m_adjAreaVector[ m_adjAreaIndex ].area = ladder->m_bottomArea;
				m_adjAreaVector[ m_adjAreaIndex ].how = GO_LADDER_DOWN;
				m_adjAreaVector[ m_adjAreaIndex ].ladder = ladder;
				++m_adjAreaIndex;
			}
		}
	}
	
	/**
	 * Cost minimizes path length traveled thus far and "danger" (proximity to threat(s))
	 */
	float Cost( CNavArea *area, CNavArea *fromArea, const CNavLadder *ladder )
	{
		// check if we can use this area
		if ( !m_mover->IsAreaTraversable( area ) )
		{
			return -1.0f;
		}

		int teamID = m_me->GetEntity()->GetTeamNumber();
		if ( area->IsBlocked( teamID ) )
		{
			return -1.0f;
		}
		
		const float debugDeltaT = 3.0f;

		float cost;

		const float maxThreatRange = 500.0f;
		const float dangerDensity = 1000.0f;

		if ( fromArea == NULL )
		{
			cost = 0.0f;
		
			/*if ( area->Contains( m_threat->GetAbsOrigin() ) )
			{
				// maximum danger - threat is in the area with us
				cost += 10.0f * dangerDensity;
				
				if ( m_me->IsDebugging( NEXTBOT_PATH ) )
				{
					area->DrawFilled( 255, 0, 0, 128 );
				}
			}
			else*/
			{
				// danger proportional to range to us
				float rangeToThreat = ( m_threat->GetAbsOrigin() - m_me->GetEntity()->GetAbsOrigin() ).Length();

				if ( rangeToThreat < maxThreatRange )
				{
					cost += dangerDensity * ( 1.0f - ( rangeToThreat / maxThreatRange ) );

					if ( m_me->IsDebugging( NEXTBOT_PATH ) )
					{
						NDebugOverlay::Line( m_me->GetEntity()->GetAbsOrigin(), m_threat->GetAbsOrigin(), 255, 0, 0, true, debugDeltaT );
					}
				}				
			}
		}
		else
		{
			// compute distance traveled along path so far
			float dist;

			if ( ladder )
			{
				const float ladderCostFactor = 100.0f;
				dist = ladderCostFactor * ladder->m_length;
			}
			else
			{
				Vector to = area->GetCenter() - fromArea->GetCenter();

				dist = to.Length();

				// check for vertical discontinuities
				Vector closeFrom, closeTo;
				area->GetClosestPointOnArea( fromArea->GetCenter(), &closeTo );
				fromArea->GetClosestPointOnArea( area->GetCenter(), &closeFrom );
				
				float deltaZ = closeTo.z - closeFrom.z;

				if ( deltaZ > m_mover->GetMaxJumpHeight() )
				{
					// too high to jump
					return -1.0f;
				}
				else if ( -deltaZ > m_mover->GetDeathDropHeight() )
				{
					// too far down to drop
					return -1.0f;
				}

				// prefer to maintain our level
				const float climbCost = 10.0f;
				dist += climbCost * fabs( deltaZ );
			}

			cost = dist + fromArea->GetTotalCost();

			
			// Add in danger cost due to threat
			// Assume straight line between areas and find closest point
			// to the threat along that line segment. The distance between
			// the threat and closest point on the line is the danger cost.		
			
			// path danger is CUMULATIVE
			float dangerCost = fromArea->GetCostSoFar();
			
			Vector close;
			float t;
			CalcClosestPointOnLineSegment( m_threat->GetAbsOrigin(), area->GetCenter(), fromArea->GetCenter(), close, &t );
			if ( t < 0.0f )
			{
				close = area->GetCenter();
			}
			else if ( t > 1.0f )
			{
				close = fromArea->GetCenter();
			}

			float rangeToThreat = ( m_threat->GetAbsOrigin() - close ).Length();

			if ( rangeToThreat < maxThreatRange )
			{
				float dangerFactor = 1.0f - ( rangeToThreat / maxThreatRange );
				dangerCost = dangerDensity * dangerFactor;

				if ( m_me->IsDebugging( NEXTBOT_PATH ) )
				{
					NDebugOverlay::HorzArrow( fromArea->GetCenter(), area->GetCenter(), 5, 255 * dangerFactor, 0, 0, 255, true, debugDeltaT );

					Vector to = close - m_threat->GetAbsOrigin();
					to.NormalizeInPlace();

					NDebugOverlay::Line( close, close - 50.0f * to, 255, 0, 0, true, debugDeltaT );
				}
			}
			
			cost += dangerCost;
		}

		return cost;
	}
	
private:	
	INextBot *m_me;
	ILocomotion *m_mover;
	
	CBaseEntity *m_threat;
	float m_retreatRange;

	enum { MAX_ADJ_AREAS = 64 };
	
	struct AdjInfo
	{
		CNavArea *area;
		CNavLadder *ladder;
		NavTraverseType how;		
	};
	
	AdjInfo m_adjAreaVector[ MAX_ADJ_AREAS ];
	int m_adjAreaIndex;
	
};

class RetreatPath : public PathFollower
{
public:
	CountdownTimer m_throttleTimer;						// require a minimum time between re-paths
	EHANDLE m_pathThreat;								// the threat of our existing path
	Vector m_pathThreatPos;								// where the threat was when the path was built
	
	struct vars_t
	{
		float maxlen = 1000.0f;
	};
	
	unsigned char *vars_ptr()
	{ return (((unsigned char *)this) + sizeof(RetreatPath)); }
	vars_t &getvars()
	{ return *(vars_t *)vars_ptr(); }
	
	void HookInvalidate()
	{
		m_throttleTimer.Invalidate();
		m_pathThreat = NULL;
		
		RETURN_META(MRES_IGNORED);
	}
	
	void dtor()
	{
		getvars().~vars_t();
		
		SH_REMOVE_MANUALHOOK(GenericDtor, this, SH_MEMBER(this, &RetreatPath::dtor), false);
		SH_REMOVE_HOOK(Path, Invalidate, this, SH_MEMBER(this, &RetreatPath::HookInvalidate), false);
		
		RETURN_META(MRES_IGNORED);
	}
	
	void Update( INextBot *bot, CBaseEntity *threat )
	{
		if ( threat == NULL )
		{
			return;
		}

		// if our path threat changed, repath immediately
		if ( threat != m_pathThreat )
		{
			if ( bot->IsDebugging( NEXTBOT_PATH ) )
			{
				DevMsg( "%3.2f: bot(#%d) Chase path threat changed (from %X to %X).\n", gpGlobals->curtime, bot->GetEntity()->entindex(), m_pathThreat.Get(), threat );
			}

			Invalidate();
		}

		// maintain the path away from the threat
		RefreshPath( bot, threat );

		// move along the path towards the threat
		PathFollower::Update( bot );
	}
	
	void RefreshPath( INextBot *bot, CBaseEntity *threat )
	{
		if ( threat == NULL )
		{
			if ( bot->IsDebugging( NEXTBOT_PATH ) )
			{
				DevMsg( "%3.2f: bot(#%d) CasePath::RefreshPath failed. No threat.\n", gpGlobals->curtime, bot->GetEntity()->entindex() );
			}
			return;
		}

		// don't change our path if we're on a ladder
		ILocomotion *mover = bot->GetLocomotionInterface();
		if ( IsValid() && mover && mover->IsUsingLadder() )
		{
			if ( bot->IsDebugging( NEXTBOT_PATH ) )
			{
				DevMsg( "%3.2f: bot(#%d) RetreatPath::RefreshPath failed. Bot is on a ladder.\n", gpGlobals->curtime, bot->GetEntity()->entindex() );
			}
			return;
		}

		// the closer we get, the more accurate our path needs to be
		Vector to = threat->GetAbsOrigin() - bot->GetPosition();
		
		const float minTolerance = 0.0f;
		const float toleranceRate = 0.33f;
		
		float tolerance = minTolerance + toleranceRate * to.Length();

		if ( !IsValid() || ( threat->GetAbsOrigin() - m_pathThreatPos ).IsLengthGreaterThan( tolerance ) )
		{
			if ( !m_throttleTimer.IsElapsed() )
			{
				// require a minimum time between repaths, as long as we have a path to follow
				if ( bot->IsDebugging( NEXTBOT_PATH ) )
				{
					DevMsg( "%3.2f: bot(#%d) RetreatPath::RefreshPath failed. Rate throttled.\n", gpGlobals->curtime, bot->GetEntity()->entindex() );
				}
				return;
			}

			// remember our path threat
			m_pathThreat = threat;
			m_pathThreatPos = threat->GetAbsOrigin();

			RetreatPathBuilder retreat( bot, threat, GetMaxPathLength() );

			CNavArea *goalArea = retreat.ComputePath();

			if ( goalArea )
			{
				AssemblePrecomputedPath( bot, goalArea->GetCenter(), goalArea );
			}	
			else
			{
				// all adjacent areas are too far away - just move directly away from threat
				Vector to = threat->GetAbsOrigin() - bot->GetPosition();

				BuildTrivialPath( bot, bot->GetPosition() - to );
			}
				
			const float minRepathInterval = 0.5f;
			m_throttleTimer.Start( minRepathInterval );
		}
	}
	
	float GetMaxPathLength( void )
	{
		return getvars().maxlen;
	}
	
	static RetreatPath *create()
	{
		RetreatPath *bytes = (RetreatPath *)calloc(1, sizeof(RetreatPath) + sizeof(vars_t));
		call_mfunc<void>(bytes, PathFollowerCTOR);
		new (bytes->vars_ptr()) vars_t();
		SH_ADD_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(bytes, &RetreatPath::dtor), false);
		SH_ADD_HOOK(Path, Invalidate, bytes, SH_MEMBER(bytes, &RetreatPath::HookInvalidate), false);
		return bytes;
	}
};

#if SOURCE_ENGINE == SE_TF2
class CTFPathFollower : public PathFollower
{
public:
	static CTFPathFollower *create()
	{
		CTFPathFollower *bytes = (CTFPathFollower *)calloc(1, sizeof(CTFPathFollower));
		call_mfunc<void>(bytes, CTFPathFollowerCTOR);
		return bytes;
	}
	
	const Path::Segment *m_goal;					// our current goal along the path
	float m_minLookAheadRange;
	
	float GetMinLookAheadDistance()
	{
		return m_minLookAheadRange;
	}
};
#endif

HandleType_t PathHandleType = 0;
HandleType_t PathFollowerHandleType = 0;
#if SOURCE_ENGINE == SE_TF2
HandleType_t CTFPathFollowerHandleType = 0;
#endif
HandleType_t ChasePathHandleType = 0;
HandleType_t DirectChasePathHandleType = 0;
HandleType_t RetreatPathHandleType = 0;
#if SOURCE_ENGINE == SE_LEFT4DEAD2
HandleType_t InfectedChasePathHandleType = 0;
#endif

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
	
	INextBot *bot = (INextBot *)params[2];
	
	cell_t *value = nullptr;
	pContext->LocalToPhysAddr(params[3], &value);
	Vector goal = Vector(sp_ctof(value[0]), sp_ctof(value[1]), sp_ctof(value[2]));
	
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

cell_t ChasePathUpdateNative(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	ChasePath *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], ChasePathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[3]);
	
	INextBot *bot = (INextBot *)params[2];
	
	IPluginFunction *callback = pContext->GetFunctionById(params[4]);
	SPPathCost cost(bot, callback, params[5]);
	
	Vector PredictedSubjectPos;
	Vector *pPredictedSubjectPos = nullptr;
	
	cell_t *pNullVec = pContext->GetNullRef(SP_NULL_VECTOR);
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[6], &addr);
	
	if(addr != pNullVec) {
		PredictedSubjectPos.x = sp_ctof(addr[0]);
		PredictedSubjectPos.y = sp_ctof(addr[1]);
		PredictedSubjectPos.z = sp_ctof(addr[2]);
		pPredictedSubjectPos = &PredictedSubjectPos;
	}
	
	(obj->*obj->getvars().pUpdate)(bot, pSubject, cost, pPredictedSubjectPos);
	
	return 0;
}

cell_t ChasePathPredictSubjectPosition(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	ChasePath *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], ChasePathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[3]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[3]);
	}
	
	INextBot *bot = (INextBot *)params[2];
	
	Vector PredictedSubjectPos = obj->PredictSubjectPosition(bot, pSubject);

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	addr[0] = sp_ftoc(PredictedSubjectPos.x);
	addr[1] = sp_ftoc(PredictedSubjectPos.y);
	addr[2] = sp_ftoc(PredictedSubjectPos.z);
	
	return 0;
}

cell_t RetreatPathUpdateNative(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	RetreatPath *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], RetreatPathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[3]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[3]);
	}
	
	INextBot *bot = (INextBot *)params[2];
	
	obj->Update(bot, pSubject);
	
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

cell_t CNavAreaGetRandomPoint(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	Vector vec = area->GetRandomPoint();
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

#if SOURCE_ENGINE == SE_TF2
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
#endif

cell_t CNavAreaGetPlayerCount(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->GetPlayerCount(params[2]);
}

#if SOURCE_ENGINE == SE_TF2
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
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
cell_t TerrorNavAreaGetSpawnAttributes(IPluginContext *pContext, const cell_t *params)
{
	TerrorNavArea *area = (TerrorNavArea *)params[1];
	return area->GetSpawnAttributes();
}
#endif

cell_t CNavLadderLengthget(IPluginContext *pContext, const cell_t *params)
{
	CNavLadder *area = (CNavLadder *)params[1];
	return sp_ftoc(area->m_length);
}

cell_t CNavMeshGetNearestNavAreaVector(IPluginContext *pContext, const cell_t *params)
{
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[1], &addr);
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	return (cell_t)TheNavMesh->GetNearestNavArea(pos, params[2], sp_ctof(params[3]), params[4], params[5], params[6]);
}

cell_t CNavMeshGetGroundHeightNative(IPluginContext *pContext, const cell_t *params)
{
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[1], &addr);
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	cell_t *heightaddr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	
	Vector normal{};
	
	cell_t *pNullVec = pContext->GetNullRef(SP_NULL_VECTOR);
	
	float height = 0.0f;
	bool ret = TheNavMesh->GetGroundHeight(pos, &height, &normal);
	
	addr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	
	if(addr != pNullVec) {
		addr[0] = sp_ftoc(normal.x);
		addr[1] = sp_ftoc(normal.y);
		addr[2] = sp_ftoc(normal.z);
	}
	
	*heightaddr = sp_ftoc(height);
	
	return ret;
}

cell_t CNavMeshGetSimpleGroundHeightNative(IPluginContext *pContext, const cell_t *params)
{
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[1], &addr);
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	cell_t *heightaddr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	
	Vector normal{};
	
	cell_t *pNullVec = pContext->GetNullRef(SP_NULL_VECTOR);
	
	float height = 0.0f;
	bool ret = TheNavMesh->GetSimpleGroundHeight(pos, &height, &normal);
	
	addr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	
	if(addr != pNullVec) {
		addr[0] = sp_ftoc(normal.x);
		addr[1] = sp_ftoc(normal.y);
		addr[2] = sp_ftoc(normal.z);
	}
	
	*heightaddr = sp_ftoc(height);
	
	return ret;
}

cell_t CNavMeshNavAreaCountget(IPluginContext *pContext, const cell_t *params)
{
	return TheNavMesh->GetNavAreaCount();
}

cell_t CNavMeshGetPlace(IPluginContext *pContext, const cell_t *params)
{
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[1], &addr);
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	return TheNavMesh->GetPlace(pos);
}

cell_t CNavMeshPlaceToName(IPluginContext *pContext, const cell_t *params)
{
	const char *name = TheNavMesh->PlaceToName(params[1]);
	if(!name) {
		name = "";
	}

	size_t written = 0;
	pContext->StringToLocalUTF8(params[2], params[3], name, &written);

	return written;
}

cell_t CNavMeshNameToPlace(IPluginContext *pContext, const cell_t *params)
{
	char *name = nullptr;
	pContext->LocalToString(params[1], &name);
	
	return TheNavMesh->NameToPlace(name);
}

cell_t CNavMeshGetNavAreaByID(IPluginContext *pContext, const cell_t *params)
{
	return (cell_t)TheNavMesh->GetNavAreaByID(params[1]);
}

cell_t CNavMeshGetNavAreaEntity(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[1]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	return (cell_t)TheNavMesh->GetNavArea(pEntity, params[2], sp_ctof(params[3]));
}

cell_t CNavMeshGetNavAreaVector(IPluginContext *pContext, const cell_t *params)
{
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[1], &addr);
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	return (cell_t)TheNavMesh->GetNavArea(pos, sp_ctof(params[2]));
}

cell_t CNavMeshGetNearestNavAreaEntity(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[1]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	return (cell_t)TheNavMesh->GetNearestNavArea(pEntity, params[2], sp_ctof(params[3]));
}

cell_t CNavMeshGetMemory(IPluginContext *pContext, const cell_t *params)
{
	return (cell_t)TheNavMesh;
}

cell_t CNavAreaGetDanger(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return sp_ftoc(area->GetDanger(params[2]));
}

cell_t CNavAreaUnderwaterget(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->IsUnderwater();
}

cell_t CNavAreaAvoidanceObstacleHeightget(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return sp_ftoc(area->GetAvoidanceObstacleHeight());
}

cell_t CNavAreaSizeXget(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return sp_ftoc(area->GetSizeX());
}

cell_t CNavAreaSizeYget(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return sp_ftoc(area->GetSizeY());
}

cell_t CNavAreaDamagingget(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->IsDamaging();
}

cell_t CNavAreaPlaceget(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->GetPlace();
}

cell_t CNavAreaHasAvoidanceObstacle(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->HasAvoidanceObstacle(sp_ctof(params[2]));
}

cell_t CNavAreaIsPotentiallyVisibleToTeam(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return area->IsPotentiallyVisibleToTeam(params[2]);
}

cell_t CNavAreaGetZ(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];
	return sp_ftoc(area->GetZ(sp_ctof(params[2]), sp_ctof(params[3])));
}

cell_t CNavAreaGetClosestPointOnArea(IPluginContext *pContext, const cell_t *params)
{
	CNavArea *area = (CNavArea *)params[1];

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector pos(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));

	Vector close;
	area->GetClosestPointOnArea(pos, &close);

	pContext->LocalToPhysAddr(params[3], &addr);

	addr[0] = sp_ftoc(close.x);
	addr[1] = sp_ftoc(close.y);
	addr[2] = sp_ftoc(close.z);

	return 0;
}

cell_t ILocomotionIsAreaTraversable(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	CNavArea *other = (CNavArea *)params[2];
	return area->IsAreaTraversable(other);
}

cell_t ILocomotionIsPotentiallyTraversable(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector from(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	pContext->LocalToPhysAddr(params[3], &addr);
	Vector to(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));

	pContext->LocalToPhysAddr(params[5], &addr);

	float fraction = 0.0f;
	bool ret = area->IsPotentiallyTraversable(from, to, (TraverseWhenType)params[4], &fraction);

	*addr = sp_ftoc(fraction);

	return ret;
}

cell_t ILocomotionHasPotentialGap(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector from(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	pContext->LocalToPhysAddr(params[3], &addr);
	Vector to(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));

	pContext->LocalToPhysAddr(params[4], &addr);

	float fraction = 0.0f;
	bool ret = area->HasPotentialGap(from, to, &fraction);

	*addr = sp_ftoc(fraction);

	return ret;
}

cell_t ILocomotionIsGap(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector from(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	pContext->LocalToPhysAddr(params[3], &addr);
	Vector to(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));

	bool ret = area->IsGap(from, to);

	return ret;
}

cell_t ILocomotionIsEntityTraversable(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	bool ret = area->IsEntityTraversable(pEntity, (TraverseWhenType)params[3]);

	return ret;
}

cell_t ILocomotionClearStuckStatus(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	
	char *reason = nullptr;
	pContext->LocalToString(params[2], &reason);

	area->ClearStuckStatus(reason);

	return 0;
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

#if SOURCE_ENGINE == SE_TF2
cell_t CTFPathFollowerCTORNative(IPluginContext *pContext, const cell_t *params)
{
	CTFPathFollower *obj = CTFPathFollower::create();
	return handlesys->CreateHandle(CTFPathFollowerHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
}
#endif

cell_t ChasePathCTORNative(IPluginContext *pContext, const cell_t *params)
{
	ChasePath *obj = ChasePath::create((ChasePath::SubjectChaseType)params[1]);
	return handlesys->CreateHandle(ChasePathHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
}

cell_t DirectChasePathCTORNative(IPluginContext *pContext, const cell_t *params)
{
	DirectChasePath *obj = DirectChasePath::create((ChasePath::SubjectChaseType)params[1]);
	return handlesys->CreateHandle(DirectChasePathHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
cell_t InfectedChasePathCTORNative(IPluginContext *pContext, const cell_t *params)
{
	InfectedChasePath *obj = InfectedChasePath::create((ChasePath::SubjectChaseType)params[1]);
	return handlesys->CreateHandle(InfectedChasePathHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
}
#endif

cell_t RetreatPathCTORNative(IPluginContext *pContext, const cell_t *params)
{
	RetreatPath *obj = RetreatPath::create();
	return handlesys->CreateHandle(RetreatPathHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
}

cell_t INextBotEntityget(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = bot->GetEntity();
	if(!pEntity) {
		return -1;
	}
	
	return gamehelpers->EntityToBCompatRef(pEntity);
}

cell_t INextBotBeginUpdate(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	return bot->BeginUpdate();
}

cell_t INextBotUpdate(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	bot->Update();
	return 0;
}

cell_t INextBotReset(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	bot->Reset();
	return 0;
}

cell_t INextBotIsRangeLessThanEntity(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return sp_ftoc(bot->IsRangeLessThan(pEntity, sp_ctof(params[3])));
}

cell_t INextBotIsRangeLessThanVector(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec( sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]) );
	
	return sp_ftoc(bot->IsRangeLessThan(vec, sp_ctof(params[3])));
}

cell_t INextBotIsRangeGreaterThanEntity(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return sp_ftoc(bot->IsRangeGreaterThan(pEntity, sp_ctof(params[3])));
}

cell_t INextBotIsRangeGreaterThanVector(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec( sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]) );
	
	return sp_ftoc(bot->IsRangeGreaterThan(vec, sp_ctof(params[3])));
}

cell_t INextBotGetRangeToEntity(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return sp_ftoc(bot->GetRangeTo(pEntity));
}

cell_t INextBotGetRangeToVector(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec( sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]) );
	
	return sp_ftoc(bot->GetRangeTo(vec));
}

cell_t INextBotGetRangeSquaredToEntity(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return sp_ftoc(bot->GetRangeSquaredTo(pEntity));
}

cell_t INextBotGetRangeSquaredToVector(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec( sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]) );
	
	return sp_ftoc(bot->GetRangeSquaredTo(vec));
}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
cell_t INextBotGet2DRangeToEntity(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return sp_ftoc(bot->Get2DRangeTo(pEntity));
}

cell_t INextBotGet2DRangeToVector(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec( sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]) );
	
	return sp_ftoc(bot->Get2DRangeTo(vec));
}
#endif

cell_t INextBotIsFriend(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return bot->IsFriend(pEntity);
}

cell_t INextBotIsEnemy(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return bot->IsEnemy(pEntity);
}

cell_t INextBotIsSelf(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	
	CBaseEntity *pEntity = gamehelpers->ReferenceToEntity(params[2]);
	if(!pEntity)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return bot->IsSelf(pEntity);
}

cell_t INextBotIsDebuggingNative(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	return bot->IsDebugging(params[2]);
}

cell_t INextBotEndUpdate(IPluginContext *pContext, const cell_t *params)
{
	INextBot *bot = (INextBot *)params[1];
	bot->EndUpdate();
	return 0;
}

cell_t INextBotComponentBotget(IPluginContext *pContext, const cell_t *params)
{
	INextBotComponent *bot = (INextBotComponent *)params[1];
	
	return (cell_t)bot->GetBot();
}

cell_t INextBotComponentBotReset(IPluginContext *pContext, const cell_t *params)
{
	INextBotComponent *bot = (INextBotComponent *)params[1];
	
	bot->Reset();
	
	return 0;
}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
SH_DECL_HOOK0(INextBot, GetLocomotionInterface, const, 0, ILocomotion *)
SH_DECL_HOOK0(INextBot, GetBodyInterface, const, 0, IBody *)
SH_DECL_HOOK0(INextBot, GetVisionInterface, const, 0, IVision *)
SH_DECL_HOOK0(INextBot, GetIntentionInterface, const, 0, IIntention *)

struct pointer_holder_t
{
	ILocomotion *locomotion = nullptr;
	IBody *body = nullptr;
	IIntention *inte = nullptr;
	IVision *vis = nullptr;
	
	bool hooked_loc = false;
	bool hooked_bod = false;
	bool hooked_inte = false;
	bool hooked_vis = false;
	
	INextBot *m_bot = nullptr;
	
	pointer_holder_t(INextBot *bot);
	~pointer_holder_t();
	
	void dtor(INextBot *bot);

	void HookBotDtor()
	{
		INextBot *bot = META_IFACEPTR(INextBot);
		dtor(bot);
		RETURN_META(MRES_IGNORED);
	}
	
	ILocomotion *HookGetLocomotionInterface()
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, locomotion);
	}
	
	IBody *HookGetBodyInterface()
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, body);
	}
	
	IIntention *HookGetIntentionInterface()
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, inte);
	}
	
	IVision *HookGetVisionInterface()
	{
		RETURN_META_VALUE(MRES_SUPERCEDE, vis);
	}
	
	void set_locomotion(ILocomotion *loc)
	{
		if(!hooked_loc) {
			SH_ADD_HOOK(INextBot, GetLocomotionInterface, m_bot, SH_MEMBER(this, &pointer_holder_t::HookGetLocomotionInterface), false);
			hooked_loc = true;
		}
		
		locomotion = loc;
	}
	
	void set_body(IBody *loc)
	{
		if(!hooked_bod) {
			SH_ADD_HOOK(INextBot, GetBodyInterface, m_bot, SH_MEMBER(this, &pointer_holder_t::HookGetBodyInterface), false);
			hooked_bod = true;
		}
		
		body = loc;
	}
	
	void set_vision(IVision *loc)
	{
		if(!hooked_vis) {
			SH_ADD_HOOK(INextBot, GetVisionInterface, m_bot, SH_MEMBER(this, &pointer_holder_t::HookGetVisionInterface), false);
			hooked_vis = true;
		}
		
		vis = loc;
	}
	
	void set_intention(IIntention *loc)
	{
		if(!hooked_inte) {
			SH_ADD_HOOK(INextBot, GetIntentionInterface, m_bot, SH_MEMBER(this, &pointer_holder_t::HookGetIntentionInterface), false);
			hooked_inte = true;
		}
		
		inte = loc;
	}
};

void pointer_holder_t::dtor(INextBot *bot)
{
	if(hooked_loc) {
		SH_REMOVE_HOOK(INextBot, GetLocomotionInterface, m_bot, SH_MEMBER(this, &pointer_holder_t::HookGetLocomotionInterface), false);
	}
	
	if(hooked_bod) {
		SH_REMOVE_HOOK(INextBot, GetBodyInterface, m_bot, SH_MEMBER(this, &pointer_holder_t::HookGetBodyInterface), false);
	}
	
	if(hooked_vis) {
		SH_REMOVE_HOOK(INextBot, GetVisionInterface, m_bot, SH_MEMBER(this, &pointer_holder_t::HookGetVisionInterface), false);
	}
	
	if(hooked_inte) {
		SH_REMOVE_HOOK(INextBot, GetIntentionInterface, m_bot, SH_MEMBER(this, &pointer_holder_t::HookGetIntentionInterface), false);
	}
	
	delete this;
}

using pointer_holder_map_t = std::unordered_map<INextBot *, pointer_holder_t *>;
pointer_holder_map_t pointermap{};

pointer_holder_t::pointer_holder_t(INextBot *bot)
	: m_bot{bot}
{
	SH_ADD_MANUALHOOK(GenericDtor, m_bot, SH_MEMBER(this, &pointer_holder_t::HookBotDtor), false);
	
	pointermap[m_bot] = this;
}

pointer_holder_t::~pointer_holder_t()
{
	if(locomotion) {
		delete locomotion;
	}
	
	if(body) {
		delete body;
	}
	
	if(vis) {
		delete vis;
	}
	
	if(inte) {
		delete inte;
	}
	
	pointermap.erase(m_bot);
}
#endif

#if SOURCE_ENGINE == SE_TF2
template <typename T, typename GF, typename V, typename ...Args>
cell_t INextBotAllocateCustomComponent(IPluginContext *pContext, const cell_t *params, GF getfunc, V var, Args &&... args)
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
template <typename T, typename GF, typename BF, typename V, typename SF, typename ...Args>
cell_t INextBotAllocateCustomComponent(IPluginContext *pContext, const cell_t *params, GF getfunc, BF basefunc, V var, SF setfunc, Args &&... args)
#endif
{
	INextBot *bot = (INextBot *)params[1];
	
	T *locomotion = T::create(bot, false, pContext->GetIdentity(), std::forward<Args>(args)...);
	
	auto old = (bot->*getfunc)();
	
	bot->ReplaceComponent(old, locomotion);
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	pointer_holder_t *holder = nullptr;
#endif
	
	bool needs_holder = false;
	
	if(old) {
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		if(old == &(bot->*basefunc)()) {
			needs_holder = true;
		} else {
			pointer_holder_map_t::iterator it{pointermap.find(bot)};
			if(it != pointermap.end()) {
				holder = it->second;
				needs_holder = true;
			}
			
			delete old;
		}
#elif SOURCE_ENGINE == SE_TF2
		if(old == (bot->*var)) {
			needs_holder = true;
		}

		delete old;
#endif
	}
	
	if(!needs_holder) {
		CBaseEntity *pEntity = bot->GetEntity();
		
		int index = vfunc_index(getfunc);
		
		void **vtable = *(void ***)bot;
		unsigned char *func = (unsigned char *)vtable[index];
		
		unsigned char offset1 = func[25];
		unsigned char offset2 = func[26];
		int finaloffset = offset1 | offset2 << 8;
		
		*(void **)((unsigned char *)pEntity + finaloffset) = locomotion;
	} else {
#if SOURCE_ENGINE == SE_TF2
		(bot->*var) = locomotion;
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
		if(!holder) {
			pointer_holder_map_t::iterator it{pointermap.find(bot)};
			if(it != pointermap.end()) {
				holder = it->second;
			} else {
				holder = new pointer_holder_t{bot};
			}
		}
		
		(holder->*setfunc)(locomotion);
#endif
	}
	
	locomotion->Reset();
	
	return (cell_t)locomotion;
}

cell_t INextBotAllocateCustomLocomotion(IPluginContext *pContext, const cell_t *params)
{
#if SOURCE_ENGINE == SE_TF2
	return INextBotAllocateCustomComponent<NextBotGroundLocomotionCustom>(pContext, params, &INextBot::GetLocomotionInterface, &INextBot::m_baseLocomotion);
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	return INextBotAllocateCustomComponent<ZombieBotLocomotionCustom>(pContext, params, &INextBot::GetLocomotionInterface, &INextBot::baseLocomotion, &pointer_holder_t::locomotion, &pointer_holder_t::set_locomotion);
#endif
}

cell_t INextBotAllocateFlyingLocomotion(IPluginContext *pContext, const cell_t *params)
{
	return INextBotAllocateCustomComponent<CNextBotFlyingLocomotion>(pContext, params, &INextBot::GetLocomotionInterface, &INextBot::m_baseLocomotion);
}

cell_t INextBotAllocateCustomBody(IPluginContext *pContext, const cell_t *params)
{
#if SOURCE_ENGINE == SE_TF2
	return INextBotAllocateCustomComponent<IBodyCustom>(pContext, params, &INextBot::GetBodyInterface, &INextBot::m_baseBody);
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	return INextBotAllocateCustomComponent<IBodyCustom>(pContext, params, &INextBot::GetBodyInterface, &INextBot::baseBody, &pointer_holder_t::body, &pointer_holder_t::set_body);
#endif
}

cell_t INextBotAllocateCustomVision(IPluginContext *pContext, const cell_t *params)
{
#if SOURCE_ENGINE == SE_TF2
	return INextBotAllocateCustomComponent<IVisionCustom>(pContext, params, &INextBot::GetVisionInterface, &INextBot::m_baseVision);
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	return INextBotAllocateCustomComponent<ZombieBotVisionCustom>(pContext, params, &INextBot::GetVisionInterface, &INextBot::baseVision, &pointer_holder_t::vis, &pointer_holder_t::set_vision);
#endif
}

template <typename T, typename ...Args>
cell_t INextBotAllocateIntention(IPluginContext *pContext, const cell_t *params, Args &&... args)
{
#if SOURCE_ENGINE == SE_TF2
	return INextBotAllocateCustomComponent<T>(pContext, params, &INextBot::GetIntentionInterface, &INextBot::m_baseIntention, std::forward<Args>(args)...);
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	return INextBotAllocateCustomComponent<T>(pContext, params, &INextBot::GetIntentionInterface, &INextBot::baseIntention, &pointer_holder_t::inte, &pointer_holder_t::set_intention, std::forward<Args>(args)...);
#endif
}

cell_t INextBotStubIntention(IPluginContext *pContext, const cell_t *params)
{
	return INextBotAllocateIntention<IIntentionStub>(pContext, params);
}

cell_t INextBotAllocateCustomIntention(IPluginContext *pContext, const cell_t *params)
{
	IPluginFunction *initact = pContext->GetFunctionById(params[2]);
	
	char *nameptr = nullptr;
	pContext->LocalToString(params[3], &nameptr);
	
	std::string name{nameptr};
	
	return INextBotAllocateIntention<IIntentionCustom>(pContext, params, initact, std::move(name));
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

#if SOURCE_ENGINE == SE_LEFT4DEAD2
cell_t PathSubjectset(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	Path *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	CBaseEntity *subject = gamehelpers->ReferenceToEntity(params[2]);
	
	obj->SetSubject(subject);
	
	return 0;
}
#endif

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

#if SOURCE_ENGINE == SE_TF2
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
#endif

cell_t PathFollowerHindranceget(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	PathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	CBaseEntity *pEntity = obj->GetHindrance();
	if(!pEntity) {
		return -1;
	}
	
	return gamehelpers->EntityToBCompatRef(pEntity);
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

cell_t PathFollowerIsAtGoal(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	PathFollower *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], PathFollowerHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	INextBot *bot = (INextBot *)params[2];
	
	return obj->IsAtGoal(bot);
}

#if SOURCE_ENGINE == SE_TF2
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
#endif

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

#if SOURCE_ENGINE == SE_TF2
cell_t IBodyCollisionGroupget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return sp_ftoc(area->GetCollisionGroup());
}
#endif

#if SOURCE_ENGINE == SE_TF2
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
#endif

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

cell_t ILocomotionSpeedget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetSpeed());
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

cell_t ILocomotionOnGroundget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsOnGround();
}

cell_t ILocomotionAttemptingToMoveget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsAttemptingToMove();
}

cell_t ILocomotionUsingLadderget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsUsingLadder();
}

cell_t ILocomotionAscendingOrDescendingLadderget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return area->IsAscendingOrDescendingLadder();
}

cell_t ILocomotionStuckDurationget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];
	return sp_ftoc(area->GetStuckDuration());
}

cell_t ILocomotionGroundget(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	CBaseEntity *pEntity = area->GetGround();

	return pEntity ? gamehelpers->EntityToBCompatRef(pEntity) : -1;
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

cell_t ILocomotionGetMotionVector(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	const Vector &ang = area->GetMotionVector();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(ang.x);
	addr[1] = sp_ftoc(ang.y);
	addr[2] = sp_ftoc(ang.z);
	
	return 0;
}

cell_t ILocomotionGetFeet(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	const Vector &ang = area->GetFeet();
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	addr[0] = sp_ftoc(ang.x);
	addr[1] = sp_ftoc(ang.y);
	addr[2] = sp_ftoc(ang.z);
	
	return 0;
}

cell_t ILocomotionGetGroundNormal(IPluginContext *pContext, const cell_t *params)
{
	ILocomotion *area = (ILocomotion *)params[1];

	const Vector &ang = area->GetGroundNormal();
	
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

cell_t IBodyIsPostureMobile(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->IsPostureMobile();
}

cell_t IBodyIsPostureChanging(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->IsPostureChanging();
}

cell_t IBodyStartActivity(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->StartActivity((Activity)params[2], (unsigned int)params[3]);
}

cell_t IBodySelectAnimationSequence(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->SelectAnimationSequence((Activity)params[2]);
}

cell_t IBodyGetActivity(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->GetActivity();
}

cell_t IBodyInDesiredPosture(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->IsInDesiredPosture();
}

cell_t IBodyIsActivity(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->IsActivity((Activity)params[2]);
}

cell_t IBodyIsDesiredPosture(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->IsDesiredPosture((PostureType)params[2]);
}

cell_t IBodyIsArousal(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->IsArousal((ArousalType)params[2]);
}

cell_t IBodyIsActualPosture(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->IsActualPosture((PostureType)params[2]);
}

cell_t IBodyHasActivityType(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->HasActivityType((unsigned int)params[2]);
}

cell_t IBodyDesiredPostureget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->GetDesiredPosture();
}

cell_t IBodyActualPostureget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->GetActualPosture();
}

cell_t IBodyDesiredPostureset(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	area->SetDesiredPosture((PostureType)params[2]);
	return 0;
}

cell_t IBodyArousalget(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	return area->GetArousal();
}

cell_t IBodyArousalset(IPluginContext *pContext, const cell_t *params)
{
	IBody *area = (IBody *)params[1];
	area->SetArousal((ArousalType)params[2]);
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

cell_t handle_set_data(IPluginContext *pContext, const cell_t *params, spvarmap_t &data)
{
	char *name = nullptr;
	pContext->LocalToString(params[2], &name);
	
	auto it{data.find(name)};
	if(it == data.end()) {
		it = data.emplace(spvarmap_t::value_type{name, {}}).first;
	}
	
	std::vector<cell_t> &vec = it->second;
	if(vec.size() == 0) {
		vec.resize(1);
	}
	
	vec[0] = params[3];
	
	return 0;
}

cell_t handle_get_data(IPluginContext *pContext, const cell_t *params, spvarmap_t &data)
{
	char *name = nullptr;
	pContext->LocalToString(params[2], &name);
	
	auto it{data.find(name)};
	if(it == data.end() || it->second.size() == 0) {
		return pContext->ThrowNativeError("theres no data with the name %s", name);
	}
	
	return it->second[0];
}

cell_t handle_has_data(IPluginContext *pContext, const cell_t *params, spvarmap_t &data)
{
	char *name = nullptr;
	pContext->LocalToString(params[2], &name);
	
	auto it{data.find(name)};
	return (it != data.end() && it->second.size() > 0);
}

cell_t handle_set_data_array(IPluginContext *pContext, const cell_t *params, spvarmap_t &data)
{
	char *name = nullptr;
	pContext->LocalToString(params[2], &name);
	
	auto it{data.find(name)};
	if(it == data.end()) {
		it = data.emplace(spvarmap_t::value_type{name, {}}).first;
	}
	
	std::vector<cell_t> &vec = it->second;
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	
	size_t len = params[4];
	vec.resize(len);
	for(int i = 0; i < len; ++i) {
		vec[i] = addr[i];
	}
	
	return 0;
}

cell_t handle_get_data_array(IPluginContext *pContext, const cell_t *params, spvarmap_t &data)
{
	char *name = nullptr;
	pContext->LocalToString(params[2], &name);
	
	auto it{data.find(name)};
	if(it == data.end()) {
		it = data.emplace(spvarmap_t::value_type{name, {}}).first;
	}
	
	std::vector<cell_t> &vec = it->second;
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[3], &addr);
	
	size_t len = params[4];
	vec.resize(len);
	for(int i = 0; i < len; ++i) {
		addr[i] = vec[i];
	}
	
	return 0;
}

template <typename T>
cell_t CustomComponentset_function(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	auto &vars = locomotion->getvars();
	return vars.handle_set_function(pContext, params);
}

template <typename T>
cell_t CustomComponentset_data(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	auto &vars = locomotion->getvars();
	return handle_set_data(pContext, params, vars.data);
}

template <typename T>
cell_t CustomComponentget_data(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	auto &vars = locomotion->getvars();
	return handle_get_data(pContext, params, vars.data);
}

template <typename T>
cell_t CustomComponenthas_data(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	auto &vars = locomotion->getvars();
	return handle_has_data(pContext, params, vars.data);
}

template <typename T>
cell_t CustomComponentset_data_array(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	auto &vars = locomotion->getvars();
	return handle_set_data_array(pContext, params, vars.data);
}

template <typename T>
cell_t CustomComponentget_data_array(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	auto &vars = locomotion->getvars();
	return handle_get_data_array(pContext, params, vars.data);
}

cell_t NextBotFlyingLocomotionset_function(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_function<CNextBotFlyingLocomotion>(pContext, params); }
cell_t NextBotFlyingLocomotionset_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_data<CNextBotFlyingLocomotion>(pContext, params); }
cell_t NextBotFlyingLocomotionget_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentget_data<CNextBotFlyingLocomotion>(pContext, params); }
cell_t NextBotFlyingLocomotionhas_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponenthas_data<CNextBotFlyingLocomotion>(pContext, params); }
cell_t NextBotFlyingLocomotionset_data_array(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_data_array<CNextBotFlyingLocomotion>(pContext, params); }
cell_t NextBotFlyingLocomotionget_data_array(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentget_data_array<CNextBotFlyingLocomotion>(pContext, params); }

cell_t GameLocomotionCustomset_function(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_function<GameLocomotionCustom>(pContext, params); }
cell_t GameLocomotionCustomset_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_data<GameLocomotionCustom>(pContext, params); }
cell_t GameLocomotionCustomget_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentget_data<GameLocomotionCustom>(pContext, params); }
cell_t GameLocomotionCustomhas_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponenthas_data<GameLocomotionCustom>(pContext, params); }
cell_t GameLocomotionCustomset_data_array(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_data_array<GameLocomotionCustom>(pContext, params); }
cell_t GameLocomotionCustomget_data_array(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentget_data_array<GameLocomotionCustom>(pContext, params); }

cell_t GameVisionCustomset_function(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_function<GameVisionCustom>(pContext, params); }
cell_t GameVisionCustomset_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_data<GameVisionCustom>(pContext, params); }
cell_t GameVisionCustomget_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentget_data<GameVisionCustom>(pContext, params); }
cell_t GameVisionCustomhas_data(IPluginContext *pContext, const cell_t *params)
{ return CustomComponenthas_data<GameVisionCustom>(pContext, params); }
cell_t GameVisionCustomset_data_array(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentset_data_array<GameVisionCustom>(pContext, params); }
cell_t GameVisionCustomget_data_array(IPluginContext *pContext, const cell_t *params)
{ return CustomComponentget_data_array<GameVisionCustom>(pContext, params); }

#if SOURCE_ENGINE == SE_TF2
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
#endif

cell_t NextBotFlyingLocomotionDesiredAltitudeset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().m_desiredAltitude = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionDesiredAltitudeget(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	return sp_ftoc(locomotion->getvars().m_desiredAltitude);
}

cell_t NextBotFlyingLocomotionAccelerationset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().accel = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionAccelerationget(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	return sp_ftoc(locomotion->getvars().accel);
}

cell_t NextBotFlyingLocomotionHorizontalDampset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().hdamp = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionHorizontalDampget(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	return sp_ftoc(locomotion->getvars().hdamp);
}

cell_t NextBotFlyingLocomotionVerticalDampset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().vdamp = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionVerticalDampget(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	return sp_ftoc(locomotion->getvars().vdamp);
}

cell_t GameLocomotionMaxYawRateget(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotion *area = (GameLocomotion *)params[1];
	return sp_ftoc(area->GetMaxYawRate());
}

cell_t NextBotFlyingLocomotionMaxYawRateget(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *area = (CNextBotFlyingLocomotion *)params[1];
	return sp_ftoc(area->GetMaxYawRate());
}

cell_t GameLocomotionLadderget(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotion *area = (GameLocomotion *)params[1];
	return (cell_t)area->m_ladder;
}

cell_t GameLocomotionLadderDismountGoalget(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotion *area = (GameLocomotion *)params[1];
	return (cell_t)area->m_ladderDismountGoal;
}

cell_t GameLocomotionGoingUpLadderget(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotion *area = (GameLocomotion *)params[1];
	return area->m_isGoingUpLadder;
}

cell_t GameLocomotionLadderset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotion *area = (GameLocomotion *)params[1];
	area->m_ladder = (CNavLadder *)params[2];
	return 0;
}

cell_t GameLocomotionLadderDismountGoalset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotion *area = (GameLocomotion *)params[1];
	area->m_ladderDismountGoal = (CNavArea *)params[2];
	return 0;
}

cell_t GameLocomotionGoingUpLadderset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotion *area = (GameLocomotion *)params[1];
	area->m_isGoingUpLadder = params[2];
	return 0;
}

cell_t GameLocomotionClimbingUpToLedgeset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotion *area = (GameLocomotion *)params[1];
	area->m_isClimbingUpToLedge = params[2];
	return 0;
}

cell_t GameLocomotionCustomMaxYawRateset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotionCustom *locomotion = (GameLocomotionCustom *)params[1];
	locomotion->getvars().yaw = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionMaxYawRateset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().yaw = sp_ctof(params[2]);
	return 0;
}

#if SOURCE_ENGINE == SE_TF2
cell_t NextBotFlyingLocomotionMaxAccelerationset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().accel = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionMaxDecelerationset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().deaccel = sp_ctof(params[2]);
	return 0;
}
#endif

cell_t NextBotFlyingLocomotionSpeedLimitset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().limit = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionTraversableSlopeLimitset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().slope = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionRunSpeedset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().run = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionWalkSpeedset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().walk = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionMaxJumpHeightset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().jump = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionDeathDropHeightset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().death = sp_ctof(params[2]);
	return 0;
}

cell_t NextBotFlyingLocomotionStepHeightset(IPluginContext *pContext, const cell_t *params)
{
	CNextBotFlyingLocomotion *locomotion = (CNextBotFlyingLocomotion *)params[1];
	locomotion->getvars().step = sp_ctof(params[2]);
	return 0;
}

#if SOURCE_ENGINE == SE_TF2
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
#endif

cell_t GameLocomotionCustomSpeedLimitset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotionCustom *locomotion = (GameLocomotionCustom *)params[1];
	locomotion->getvars().limit = sp_ctof(params[2]);
	return 0;
}

cell_t GameLocomotionCustomTraversableSlopeLimitset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotionCustom *locomotion = (GameLocomotionCustom *)params[1];
	locomotion->getvars().slope = sp_ctof(params[2]);
	return 0;
}

cell_t GameLocomotionCustomRunSpeedset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotionCustom *locomotion = (GameLocomotionCustom *)params[1];
	locomotion->getvars().run = sp_ctof(params[2]);
	return 0;
}

cell_t GameLocomotionCustomWalkSpeedset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotionCustom *locomotion = (GameLocomotionCustom *)params[1];
	locomotion->getvars().walk = sp_ctof(params[2]);
	return 0;
}

cell_t GameLocomotionCustomMaxJumpHeightset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotionCustom *locomotion = (GameLocomotionCustom *)params[1];
	locomotion->getvars().jump = sp_ctof(params[2]);
	return 0;
}

cell_t GameLocomotionCustomDeathDropHeightset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotionCustom *locomotion = (GameLocomotionCustom *)params[1];
	locomotion->getvars().death = sp_ctof(params[2]);
	return 0;
}

cell_t GameLocomotionCustomStepHeightset(IPluginContext *pContext, const cell_t *params)
{
	GameLocomotionCustom *locomotion = (GameLocomotionCustom *)params[1];
	locomotion->getvars().step = sp_ctof(params[2]);
	return 0;
}

cell_t GameVisionCustomMaxVisionRangeset(IPluginContext *pContext, const cell_t *params)
{
	GameVisionCustom *locomotion = (GameVisionCustom *)params[1];
	locomotion->getvars().maxrange = sp_ctof(params[2]);
	return 0;
}

cell_t GameVisionCustomMinRecognizeTimeset(IPluginContext *pContext, const cell_t *params)
{
	GameVisionCustom *locomotion = (GameVisionCustom *)params[1];
	locomotion->getvars().minreco = sp_ctof(params[2]);
	return 0;
}

cell_t GameVisionCustomDefaultFieldOfViewset(IPluginContext *pContext, const cell_t *params)
{
	GameVisionCustom *locomotion = (GameVisionCustom *)params[1];
	locomotion->getvars().deffov = sp_ctof(params[2]);
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

#if SOURCE_ENGINE == SE_TF2
cell_t IBodyCustomCollisionGroupset(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *locomotion = (IBodyCustom *)params[1];
	locomotion->CollisionGroup = params[2];
	return 0;
}
#endif

cell_t IBodyCustomSequenceget(IPluginContext *pContext, const cell_t *params)
{
	IBodyCustom *locomotion = (IBodyCustom *)params[1];
	return locomotion->GetSequence();
}

#if SOURCE_ENGINE == SE_TF2
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
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
cell_t IVisionGetClosestRecognizedTeam(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	
	CBaseEntity *pEntity = locomotion->GetClosestRecognized(params[2]);
	if(!pEntity) {
		return -1;
	}
	
	return gamehelpers->EntityToBCompatRef(pEntity);
}

cell_t IVisionGetRecognizedCount(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return locomotion->GetRecognizedCount(params[2], sp_ctof(params[3]));
}

cell_t IVisionGetPrimaryRecognizedThreat(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	
	CBaseEntity *pEntity = locomotion->GetPrimaryRecognizedThreat();
	if(!pEntity) {
		return -1;
	}
	
	return gamehelpers->EntityToBCompatRef(pEntity);
}

class SPGetClosestRecognized : public INextBotEntityFilter
{
public:
	SPGetClosestRecognized(IPluginFunction *callbacl_, cell_t data_)
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

cell_t IVisionGetClosestRecognizedFilter(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	IPluginFunction *callback = pContext->GetFunctionById(params[2]);
	SPGetClosestRecognized each(callback, params[3]);
	
	CBaseEntity *pEntity = locomotion->GetClosestRecognized(each);
	if(!pEntity) {
		return -1;
	}
	
	return gamehelpers->EntityToBCompatRef(pEntity);
}
#endif

cell_t IVisionGetTimeSinceVisible(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];
	return sp_ftoc(locomotion->GetTimeSinceVisible(params[2]));
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

#if SOURCE_ENGINE == SE_TF2
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
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
cell_t IVisionIsNoticed(IPluginContext *pContext, const cell_t *params)
{
	IVision *locomotion = (IVision *)params[1];

	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[2]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}
	
	return locomotion->IsNoticed(pSubject);
}
#endif

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

#if SOURCE_ENGINE == SE_TF2
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
#endif

cell_t AllocateNextBotCombatCharacter(IPluginContext *pContext, const cell_t *params)
{
	return (cell_t)NextBotCombatCharacter::create(params[1]);
}

cell_t GetNextBotCombatCharacterSize(IPluginContext *pContext, const cell_t *params)
{
	return sizeofNextBotCombatCharacter;
}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
cell_t AllocateInfectedNextBotCombatCharacter(IPluginContext *pContext, const cell_t *params)
{
	return (cell_t)NextBotCombatCharacterInfected::create(params[1], params[2]);
}

cell_t GetInfectedSize(IPluginContext *pContext, const cell_t *params)
{
	return sizeofInfected;
}
#endif

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

cell_t MakeEntityNextBot(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	INextBot *pNextBot = pSubject->MyNextBotPointer();
	if(pNextBot)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	return (cell_t)INextBotGeneric::create(pSubject);
}

class INextBotCustom;

std::unordered_map<int, INextBotCustom *> nbcustomap{};

#if SOURCE_ENGINE == SE_LEFT4DEAD2
SH_DECL_HOOK0(INextBot, IsAllowedToClimb, const, 0, bool);
SH_DECL_HOOK0(INextBot, ReactToSurvivorVisibility, const, 0, bool);
SH_DECL_HOOK0(INextBot, ReactToSurvivorNoise, const, 0, bool);
SH_DECL_HOOK0(INextBot, ReactToSurvivorContact, const, 0, bool);
#endif
SH_DECL_HOOK1(INextBot, IsAbleToClimbOnto, const, 0, bool, CBaseEntity *);
SH_DECL_HOOK1(INextBot, IsAbleToBreak, const, 0, bool, CBaseEntity *);
SH_DECL_HOOK1(INextBot, IsAbleToBlockMovementOf, const, 0, bool, INextBot *);
SH_DECL_HOOK1(INextBot, ShouldTouch, const, 0, bool, CBaseEntity *);

class INextBotCustom : public IPluginNextBotComponent
{
public:
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	bool allowedclimb = true;
	bool reactvis = true;
	bool reactnoise = true;
	bool reactcontact = true;
#endif
	
	IPluginFunction *climbunto = nullptr;
	IPluginFunction *ablebreak = nullptr;
	IPluginFunction *ableblock = nullptr;
	IPluginFunction *shouldtouch = nullptr;
	IPluginFunction *enemy = nullptr;
	IPluginFunction *isfren = nullptr;

	int ref = -1;
	
	INextBotCustom(INextBot *pNextBot, IdentityToken_t *id)
		: IPluginNextBotComponent(id), ref{gamehelpers->EntityToBCompatRef(pNextBot->GetEntity())}
	{
		bot = pNextBot;

		nbcustomap[ref] = this;
		
		SH_ADD_MANUALHOOK(GenericDtor, bot, SH_MEMBER(this, &INextBotCustom::dtor), false);
		
		SH_ADD_HOOK(INextBot, IsAbleToBlockMovementOf, bot, SH_MEMBER(this, &INextBotCustom::HookIsAbleToBlockMovementOf), false);
		SH_ADD_HOOK(INextBot, ShouldTouch, bot, SH_MEMBER(this, &INextBotCustom::HookShouldTouch), false);
		
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		SH_ADD_HOOK(INextBot, IsAllowedToClimb, bot, SH_MEMBER(this, &INextBotCustom::HookIsAllowedToClimb), false);
		SH_ADD_HOOK(INextBot, ReactToSurvivorVisibility, bot, SH_MEMBER(this, &INextBotCustom::HookReactToSurvivorVisibility), false);
		SH_ADD_HOOK(INextBot, ReactToSurvivorNoise, bot, SH_MEMBER(this, &INextBotCustom::HookReactToSurvivorNoise), false);
		SH_ADD_HOOK(INextBot, ReactToSurvivorContact, bot, SH_MEMBER(this, &INextBotCustom::HookReactToSurvivorContact), false);
#endif
	}
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	bool HookIsAllowedToClimb() { RETURN_META_VALUE(MRES_SUPERCEDE, allowedclimb); }
	bool HookReactToSurvivorVisibility() { RETURN_META_VALUE(MRES_SUPERCEDE, reactvis); }
	bool HookReactToSurvivorNoise() { RETURN_META_VALUE(MRES_SUPERCEDE, reactnoise); }
	bool HookReactToSurvivorContact() { RETURN_META_VALUE(MRES_SUPERCEDE, reactcontact); }
#endif
	
	bool HookIsAbleToBlockMovementOf(INextBot *bot)
	{
		if(!ableblock) {
			RETURN_META_VALUE(MRES_IGNORED, true);
		}
		
		INextBot *pThis = META_IFACEPTR(INextBot);
		
		ableblock->PushCell((cell_t)this);
		ableblock->PushCell((cell_t)pThis);
		ableblock->PushCell((cell_t)bot);
		cell_t res = 0;
		ableblock->Execute(&res);
		
		RETURN_META_VALUE(MRES_SUPERCEDE, res);
	}
	
	bool HookShouldTouch(CBaseEntity *bot)
	{
		if(!shouldtouch) {
			RETURN_META_VALUE(MRES_IGNORED, true);
		}
		
		INextBot *pThis = META_IFACEPTR(INextBot);
		
		shouldtouch->PushCell((cell_t)this);
		shouldtouch->PushCell((cell_t)pThis);
		shouldtouch->PushCell(gamehelpers->EntityToBCompatRef(bot));
		cell_t res = 0;
		shouldtouch->Execute(&res);
		
		RETURN_META_VALUE(MRES_SUPERCEDE, res);
	}
	
	void dtor()
	{
		INextBot *bytes = META_IFACEPTR(INextBot);
		
		SH_REMOVE_MANUALHOOK(GenericDtor, bytes, SH_MEMBER(this, &INextBotCustom::dtor), false);
		
		SH_REMOVE_HOOK(INextBot, IsAbleToBlockMovementOf, bytes, SH_MEMBER(this, &INextBotCustom::HookIsAbleToBlockMovementOf), false);
		SH_REMOVE_HOOK(INextBot, ShouldTouch, bytes, SH_MEMBER(this, &INextBotCustom::HookShouldTouch), false);
		
#if SOURCE_ENGINE == SE_LEFT4DEAD2
		SH_REMOVE_HOOK(INextBot, IsAllowedToClimb, bytes, SH_MEMBER(this, &INextBotCustom::HookIsAllowedToClimb), false);
		SH_REMOVE_HOOK(INextBot, ReactToSurvivorVisibility, bytes, SH_MEMBER(this, &INextBotCustom::HookReactToSurvivorVisibility), false);
		SH_REMOVE_HOOK(INextBot, ReactToSurvivorNoise, bytes, SH_MEMBER(this, &INextBotCustom::HookReactToSurvivorNoise), false);
		SH_REMOVE_HOOK(INextBot, ReactToSurvivorContact, bytes, SH_MEMBER(this, &INextBotCustom::HookReactToSurvivorContact), false);
#endif
		
		delete this;
		
		RETURN_META(MRES_IGNORED);
	}
	
	~INextBotCustom()
	{
		nbcustomap.erase(ref);
	}
	
	virtual void plugin_unloaded()
	{
		IPluginNextBotComponent::plugin_unloaded();
		
		climbunto = nullptr;
		ablebreak = nullptr;
		ableblock = nullptr;
		shouldtouch = nullptr;
		enemy = nullptr;
		isfren = nullptr;
	}
	
	virtual bool set_function(std::string_view name, IPluginFunction *func) override
	{
		if(name == "IsAbleToBlockMovementOf"sv) {
			ableblock = func;
			return true;
		} else if(name == "ShouldTouch"sv) {
			shouldtouch = func;
			return true;
		}
		
		return IPluginNextBotComponent::set_function(name, func);
	}
	
	INextBot *bot = nullptr;
};

cell_t INextBotMakeCustom(IPluginContext *pContext, const cell_t *params)
{
	INextBot *pNextBot = (INextBot *)params[1];

	int ref{gamehelpers->EntityToBCompatRef(pNextBot->GetEntity())};

	INextBotCustom *pcustom = nullptr;
	
	auto it = nbcustomap.find(ref);
	if(it != nbcustomap.end()) {
		pcustom = it->second;
	} else {
		pcustom = new INextBotCustom(pNextBot, pContext->GetIdentity());
	}
	
	return (cell_t)pcustom;
}

cell_t INextBotCustomBotget(IPluginContext *pContext, const cell_t *params)
{
	INextBotCustom *pNextBot = (INextBotCustom *)params[1];
	return (cell_t)pNextBot->bot;
}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
cell_t INextBotCustomAllowedToClimbset(IPluginContext *pContext, const cell_t *params)
{
	INextBotCustom *pNextBot = (INextBotCustom *)params[1];
	pNextBot->allowedclimb = params[2];
	return 0;
}

cell_t INextBotCustomReactToSurvivorVisibilityset(IPluginContext *pContext, const cell_t *params)
{
	INextBotCustom *pNextBot = (INextBotCustom *)params[1];
	pNextBot->reactvis = params[2];
	return 0;
}

cell_t INextBotCustomReactToSurvivorNoiseset(IPluginContext *pContext, const cell_t *params)
{
	INextBotCustom *pNextBot = (INextBotCustom *)params[1];
	pNextBot->reactnoise = params[2];
	return 0;
}

cell_t INextBotCustomReactToSurvivorContactset(IPluginContext *pContext, const cell_t *params)
{
	INextBotCustom *pNextBot = (INextBotCustom *)params[1];
	pNextBot->reactcontact = params[2];
	return 0;
}
#endif

template <typename T>
cell_t Componentset_function(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	return locomotion->handle_set_function(pContext, params);
}

template <typename T>
cell_t Componentset_data(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	return handle_set_data(pContext, params, locomotion->data);
}

template <typename T>
cell_t Componentget_data(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	return handle_get_data(pContext, params, locomotion->data);
}

template <typename T>
cell_t Componenthas_data(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	return handle_has_data(pContext, params, locomotion->data);
}

template <typename T>
cell_t Componentset_data_array(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	return handle_set_data_array(pContext, params, locomotion->data);
}

template <typename T>
cell_t Componentget_data_array(IPluginContext *pContext, const cell_t *params)
{
	T *locomotion = (T *)params[1];
	return handle_get_data_array(pContext, params, locomotion->data);
}

cell_t INextBotCustomset_function(IPluginContext *pContext, const cell_t *params)
{ return Componentset_function<INextBotCustom>(pContext, params); }
cell_t INextBotCustomset_data(IPluginContext *pContext, const cell_t *params)
{ return Componentset_data<INextBotCustom>(pContext, params); }
cell_t INextBotCustomget_data(IPluginContext *pContext, const cell_t *params)
{ return Componentget_data<INextBotCustom>(pContext, params); }
cell_t INextBotCustomhas_data(IPluginContext *pContext, const cell_t *params)
{ return Componenthas_data<INextBotCustom>(pContext, params); }
cell_t INextBotCustomset_data_array(IPluginContext *pContext, const cell_t *params)
{ return Componentset_data_array<INextBotCustom>(pContext, params); }
cell_t INextBotCustomget_data_array(IPluginContext *pContext, const cell_t *params)
{ return Componentget_data_array<INextBotCustom>(pContext, params); }

cell_t BehaviorActionEntryCTOR(IPluginContext *pContext, const cell_t *params)
{
	char *name = nullptr;
	pContext->LocalToString(params[1], &name);
	
	std::string str{name};
	
	SPActionEntry *obj = new SPActionEntry{std::move(name), pContext->GetIdentity()};
	Handle_t hndl = handlesys->CreateHandle(BehaviorEntryHandleType, obj, pContext->GetIdentity(), myself->GetIdentity(), nullptr);
	obj->hndl = hndl;
	obj->pContext = pContext;
	
	return hndl;
}

cell_t BehaviorActionEntryset_function(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	SPActionEntry *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], BehaviorEntryHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	if(obj->pId != pContext->GetIdentity()) {
		return pContext->ThrowNativeError("this plugin doenst own this entry");
	}
	
	char *name_ptr = nullptr;
	pContext->LocalToString(params[2], &name_ptr);
	std::string_view name{name_ptr};
	
	IPluginFunction *func = pContext->GetFunctionById(params[3]);
	
	if(!obj->set_function(name, func)) {
		return pContext->ThrowNativeError("invalid name %s", name_ptr);
	}
	
	return 0;
}

cell_t BehaviorActionEntrycreate(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	SPActionEntry *obj = nullptr;
	HandleError err = handlesys->ReadHandle(params[1], BehaviorEntryHandleType, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}
	
	return (cell_t)obj->create();
}

cell_t BehaviorActionEntryget(IPluginContext *pContext, const cell_t *params)
{
	SPAction *obj = (SPAction *)params[1];
	
	if(!obj->entry) {
		return 0;
	}
	
	return obj->entry->hndl;
}

cell_t IIntentionCustomResetBehavior(IPluginContext *pContext, const cell_t *params)
{
	IIntentionCustom *obj = (IIntentionCustom *)params[1];
	SPAction *act = (SPAction *)params[2];
	
	obj->m_behavior->Reset(act);
	
	return 0;
}

cell_t IIntentionCustomset_name(IPluginContext *pContext, const cell_t *params)
{
	IIntentionCustom *obj = (IIntentionCustom *)params[1];
	
	char *nameptr = nullptr;
	pContext->LocalToString(params[2], &nameptr);
	
	obj->name = nameptr;
	
	return 0;
}

cell_t BehaviorActionset_data(IPluginContext *pContext, const cell_t *params)
{ return Componentset_data<SPAction>(pContext, params); }
cell_t BehaviorActionget_data(IPluginContext *pContext, const cell_t *params)
{ return Componentget_data<SPAction>(pContext, params); }
cell_t BehaviorActionhas_data(IPluginContext *pContext, const cell_t *params)
{ return Componenthas_data<SPAction>(pContext, params); }
cell_t BehaviorActionset_data_array(IPluginContext *pContext, const cell_t *params)
{ return Componentset_data_array<SPAction>(pContext, params); }
cell_t BehaviorActionget_data_array(IPluginContext *pContext, const cell_t *params)
{ return Componentget_data_array<SPAction>(pContext, params); }

cell_t IIntentionCustomset_function(IPluginContext *pContext, const cell_t *params)
{ return Componentset_function<IIntentionCustom>(pContext, params); }
cell_t IIntentionCustomset_data(IPluginContext *pContext, const cell_t *params)
{ return Componentset_data<IIntentionCustom>(pContext, params); }
cell_t IIntentionCustomget_data(IPluginContext *pContext, const cell_t *params)
{ return Componentget_data<IIntentionCustom>(pContext, params); }
cell_t IIntentionCustomhas_data(IPluginContext *pContext, const cell_t *params)
{ return Componenthas_data<IIntentionCustom>(pContext, params); }
cell_t IIntentionCustomset_data_array(IPluginContext *pContext, const cell_t *params)
{ return Componentset_data_array<IIntentionCustom>(pContext, params); }
cell_t IIntentionCustomget_data_array(IPluginContext *pContext, const cell_t *params)
{ return Componentget_data_array<IIntentionCustom>(pContext, params); }

cell_t IBodyCustomset_function(IPluginContext *pContext, const cell_t *params)
{ return Componentset_function<IBodyCustom>(pContext, params); }
cell_t IBodyCustomset_data(IPluginContext *pContext, const cell_t *params)
{ return Componentset_data<IBodyCustom>(pContext, params); }
cell_t IBodyCustomget_data(IPluginContext *pContext, const cell_t *params)
{ return Componentget_data<IBodyCustom>(pContext, params); }
cell_t IBodyCustomhas_data(IPluginContext *pContext, const cell_t *params)
{ return Componenthas_data<IBodyCustom>(pContext, params); }
cell_t IBodyCustomset_data_array(IPluginContext *pContext, const cell_t *params)
{ return Componentset_data_array<IBodyCustom>(pContext, params); }
cell_t IBodyCustomget_data_array(IPluginContext *pContext, const cell_t *params)
{ return Componentget_data_array<IBodyCustom>(pContext, params); }

cell_t GetNavAreaVectorCount(IPluginContext *pContext, const cell_t *params)
{
	return TheNavAreas->Count();
}

cell_t GetNavAreaFromVector(IPluginContext *pContext, const cell_t *params)
{
	int idx = params[1];
	
	if(idx < 0 || idx >= TheNavAreas->Count()) {
		return pContext->ThrowNativeError("invalid index %i", idx);
	}
	
	return (cell_t)(*TheNavAreas)[idx];
}

cell_t CollectSurroundingAreasNative(IPluginContext *pContext, const cell_t *params);

cell_t CombatCharacterEventKilled(IPluginContext *pContext, const cell_t *params)
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
	
#ifdef __HAS_DAMAGERULES
	CTakeDamageInfo info{};
	if(g_pDamageRules) {
		g_pDamageRules->ParamToDamageInfo(pContext, params[2], info);
	}
	pCombat->Event_Killed(info);
#else
	return pContext->ThrowNativeError("Ext was compiled without damagerules");
#endif

	return 0;
}

cell_t DirectionBetweenEntityVector(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}
	
	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);
	Vector vec(sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]));
	
	Vector diff = (pSubject->GetAbsOrigin() - vec);

	QAngle angles;
	VectorAngles(diff, angles);

	angles = TransformAnglesToLocalSpace(angles, pSubject->EntityToWorldTransform());

	NavDirType dir = AngleToDirection(angles.y);

	switch(dir) {
		case NORTH: return LEFT;
		case EAST: return BACKWARD;
		case SOUTH: return RIGHT;
		case WEST: return FORWARD;
	}
	
	return LEFT;
}

cell_t EntityVisibleEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	CBaseEntity *pBlocker = nullptr;
	bool vis =  pSubject->FVisible(pTarget, params[3], &pBlocker);

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[4], &addr);

	*addr = gamehelpers->EntityToBCompatRef(pBlocker);

	return vis;
}

cell_t EntityVisibleVec(IPluginContext *pContext, const cell_t *params)
{
	CBaseEntity *pSubject = gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);

	Vector target{sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	CBaseEntity *pBlocker = nullptr;
	bool vis =  pSubject->FVisible(target, params[3], &pBlocker);

	addr = nullptr;
	pContext->LocalToPhysAddr(params[4], &addr);

	*addr = gamehelpers->EntityToBCompatRef(pBlocker);

	return vis;
}

cell_t CombatCharacterIsHiddenByFogVec(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);

	Vector target{sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	return pSubject->IsHiddenByFog(target);
}

cell_t CombatCharacterIsHiddenByFogEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	return pSubject->IsHiddenByFog(pTarget);
}

cell_t CombatCharacterIsHiddenByFogR(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	return pSubject->IsHiddenByFog(sp_ctof(params[2]));
}

cell_t CombatCharacterGetFogObscuredRatioVec(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);

	Vector target{sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	return sp_ftoc(pSubject->GetFogObscuredRatio(target));
}

cell_t CombatCharacterGetFogObscuredRatioEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	return sp_ctof(pSubject->GetFogObscuredRatio(pTarget));
}

cell_t CombatCharacterGetFogObscuredRatioR(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	return sp_ctof(pSubject->GetFogObscuredRatio(sp_ctof(params[2])));
}

cell_t CombatCharacterIsLookingTowardsVec(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);

	Vector target{sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	return pSubject->IsLookingTowards(target, sp_ctof(params[3]));
}

cell_t CombatCharacterIsLookingTowardsEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	return pSubject->IsLookingTowards(pTarget, sp_ctof(params[3]));
}

cell_t CombatCharacterIsInFieldOfViewVec(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);

	Vector target{sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	return pSubject->IsInFieldOfView(target);
}

cell_t CombatCharacterIsInFieldOfViewEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	return pSubject->IsInFieldOfView(pTarget);
}

cell_t CombatCharacterIsLineOfSightClearVec(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);

	Vector target{sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[4]);

	return pSubject->IsLineOfSightClear(target, (LineOfSightCheckType)params[3], pTarget);
}

cell_t CombatCharacterIsLineOfSightClearEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	return pSubject->IsLineOfSightClear(pTarget, (LineOfSightCheckType)params[3]);
}

cell_t CombatCharacterInAimConeVec(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);

	Vector target{sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	return pSubject->FInAimCone(target);
}

cell_t CombatCharacterInAimConeEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	return pSubject->FInAimCone(pTarget);
}

cell_t CombatCharacterInViewConeVec(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	cell_t *addr = nullptr;
	pContext->LocalToPhysAddr(params[2], &addr);

	Vector target{sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2])};

	return pSubject->FInViewCone(target);
}

cell_t CombatCharacterInViewConeEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	return pSubject->FInViewCone(pTarget);
}

cell_t CombatCharacterIsAbleToSeeEnt(IPluginContext *pContext, const cell_t *params)
{
	CBaseCombatCharacter *pSubject = (CBaseCombatCharacter *)gamehelpers->ReferenceToEntity(params[1]);
	if(!pSubject)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[1]);
	}

	CBaseEntity *pTarget = gamehelpers->ReferenceToEntity(params[2]);
	if(!pTarget)
	{
		return pContext->ThrowNativeError("Invalid Entity Reference/Index %i", params[2]);
	}

	return pSubject->IsAbleToSee(pTarget, (FieldOfViewCheckType)params[3]);
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
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	{"Path.Subject.set", PathSubjectset},
#endif
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
	{"ChasePath.Update", ChasePathUpdateNative},
	{"ChasePath.PredictSubjectPosition", ChasePathPredictSubjectPosition},
	{"RetreatPath.Update", RetreatPathUpdateNative},
	{"PathFollower.MinLookAheadDistance.get", PathFollowerMinLookAheadDistanceget},
	{"PathFollower.MinLookAheadDistance.set", PathFollowerMinLookAheadDistanceset},
#if SOURCE_ENGINE == SE_TF2
	{"PathFollower.GoalTolerance.get", PathFollowerGoalToleranceget},
	{"PathFollower.GoalTolerance.set", PathFollowerGoalToleranceset},
#endif
	{"PathFollower.IsDiscontinuityAhead", PathFollowerIsDiscontinuityAhead},
	{"PathFollower.IsAtGoal", PathFollowerIsAtGoal},
	{"PathFollower.Hindrance.get", PathFollowerHindranceget},
#if SOURCE_ENGINE == SE_TF2
	{"CTFPathFollower.MinLookAheadDistance.get", CTFPathFollowerMinLookAheadDistanceget},
#endif
	{"CNavArea.CostSoFar.get", CNavAreaCostSoFarget},
	{"CNavArea.ID.get", CNavAreaIDget},
	{"CNavArea.GetCenter", CNavAreaGetCenter},
	{"CNavArea.GetRandomPoint", CNavAreaGetRandomPoint},
	{"CNavArea.ComputeAdjacentConnectionHeightChange", CNavAreaComputeAdjacentConnectionHeightChange},
	{"CNavArea.HasAttributes", CNavAreaHasAttributes},
#if SOURCE_ENGINE == SE_TF2
	{"CNavArea.ComputeFuncNavCost", CNavAreaComputeFuncNavCost},
#endif
	{"CNavArea.GetPlayerCount", CNavAreaGetPlayerCount},
#if SOURCE_ENGINE == SE_TF2
	{"CTFNavArea.InCombat.get", CTFNavAreaInCombatget},
	{"CTFNavArea.CombatIntensity.get", CTFNavAreaCombatIntensityget},
	{"CTFNavArea.HasAttributeTF", CTFNavAreaHasAttributeTF},
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	{"TerrorNavArea.GetSpawnAttributes", TerrorNavAreaGetSpawnAttributes},
#endif
	{"CNavMesh.GetNearestNavAreaVector", CNavMeshGetNearestNavAreaVector},
	{"CNavMesh.GetGroundHeight", CNavMeshGetGroundHeightNative},
	{"CNavMesh.GetSimpleGroundHeight", CNavMeshGetSimpleGroundHeightNative},
	{"CNavMesh.GetNavAreaCount", CNavMeshNavAreaCountget},
	{"CNavMesh.GetPlace", CNavMeshGetPlace},
	{"CNavMesh.PlaceToName", CNavMeshPlaceToName},
	{"CNavMesh.NameToPlace", CNavMeshNameToPlace},
	{"CNavMesh.GetNavAreaByID", CNavMeshGetNavAreaByID},
	{"CNavMesh.GetNavAreaEntity", CNavMeshGetNavAreaEntity},
	{"CNavMesh.GetNavAreaVector", CNavMeshGetNavAreaVector},
	{"CNavMesh.GetNearestNavAreaEntity", CNavMeshGetNearestNavAreaEntity},
	{"CNavMesh.GetMemory", CNavMeshGetMemory},
	{"CNavArea.GetDanger", CNavAreaGetDanger},
	{"CNavArea.Underwater.get", CNavAreaUnderwaterget},
	{"CNavArea.AvoidanceObstacleHeight.get", CNavAreaAvoidanceObstacleHeightget},
	{"CNavArea.SizeX.get", CNavAreaSizeXget},
	{"CNavArea.SizeY.get", CNavAreaSizeYget},
	{"CNavArea.Damaging.get", CNavAreaDamagingget},
	{"CNavArea.Place.get", CNavAreaPlaceget},
	{"CNavArea.HasAvoidanceObstacle", CNavAreaHasAvoidanceObstacle},
	{"CNavArea.IsPotentiallyVisibleToTeam", CNavAreaIsPotentiallyVisibleToTeam},
	{"CNavArea.GetZ", CNavAreaGetZ},
	{"CNavArea.GetClosestPointOnArea", CNavAreaGetClosestPointOnArea},
	{"CNavLadder.Length.get", CNavLadderLengthget},
	{"ILocomotion.StepHeight.get", ILocomotionStepHeightget},
	{"ILocomotion.MaxJumpHeight.get", ILocomotionMaxJumpHeightget},
	{"ILocomotion.DeathDropHeight.get", ILocomotionDeathDropHeightget},
	{"ILocomotion.RunSpeed.get", ILocomotionRunSpeedget},
	{"ILocomotion.WalkSpeed.get", ILocomotionWalkSpeedget},
#if SOURCE_ENGINE == SE_TF2
	{"ILocomotion.MaxAcceleration.get", ILocomotionMaxAccelerationget},
	{"ILocomotion.MaxDeceleration.get", ILocomotionMaxDecelerationget},
#endif
	{"ILocomotion.SpeedLimit.get", ILocomotionSpeedLimitget},
	{"ILocomotion.TraversableSlopeLimit.get", ILocomotionTraversableSlopeLimitget},
	{"ILocomotion.DesiredSpeed.get", ILocomotionDesiredSpeedget},
	{"ILocomotion.DesiredSpeed.set", ILocomotionDesiredSpeedset},
	{"ILocomotion.IsAreaTraversable", ILocomotionIsAreaTraversable},
	{"ILocomotion.IsPotentiallyTraversable", ILocomotionIsPotentiallyTraversable},
	{"ILocomotion.HasPotentialGap", ILocomotionHasPotentialGap},
	{"ILocomotion.IsGap", ILocomotionIsGap},
	{"ILocomotion.IsEntityTraversable", ILocomotionIsEntityTraversable},
	{"ILocomotion.ClimbingOrJumping.get", ILocomotionIsClimbingOrJumping},
	{"ILocomotion.ClimbingUpToLedge.get", ILocomotionIsClimbingUpToLedge},
	{"ILocomotion.JumpingAcrossGap.get", ILocomotionIsJumpingAcrossGap},
	{"ILocomotion.Scrambling.get", ILocomotionIsScrambling},
	{"ILocomotion.Running.get", ILocomotionIsRunning},
	{"ILocomotion.Stuck.get", ILocomotionIsStuck},
	{"ILocomotion.OnGround.get", ILocomotionOnGroundget},
	{"ILocomotion.AttemptingToMove.get", ILocomotionAttemptingToMoveget},
	{"ILocomotion.UsingLadder.get", ILocomotionUsingLadderget},
	{"ILocomotion.AscendingOrDescendingLadder.get", ILocomotionAscendingOrDescendingLadderget},
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
	{"ILocomotion.GetMotionVector", ILocomotionGetMotionVector},
	{"ILocomotion.GetFeet", ILocomotionGetFeet},
	{"ILocomotion.GetGroundNormal", ILocomotionGetGroundNormal},
	{"ILocomotion.GetVelocity", ILocomotionGetVelocity},
	{"ILocomotion.StuckDuration.get", ILocomotionStuckDurationget},
	{"ILocomotion.ClearStuckStatus", ILocomotionClearStuckStatus},
	{"ILocomotion.Ground.get", ILocomotionGroundget},
	{"ILocomotion.Speed.get", ILocomotionSpeedget},
	{"INextBot.INextBot", INextBotget},
	{"INextBot.LocomotionInterface.get", INextBotLocomotionInterfaceget},
	{"INextBot.VisionInterface.get", INextBotVisionInterfaceget},
	{"INextBot.BodyInterface.get", INextBotBodyInterfaceget},
	{"INextBot.AllocateCustomLocomotion", INextBotAllocateCustomLocomotion},
	{"INextBot.AllocateFlyingLocomotion", INextBotAllocateFlyingLocomotion},
	{"INextBot.AllocateCustomBody", INextBotAllocateCustomBody},
	{"INextBot.AllocateCustomVision", INextBotAllocateCustomVision},
	{"INextBot.AllocateCustomIntention", INextBotAllocateCustomIntention},
	{"INextBot.StubIntention", INextBotStubIntention},
	{"INextBot.Entity.get", INextBotEntityget},
	{"INextBot.BeginUpdate", INextBotBeginUpdate},
	{"INextBot.Update", INextBotUpdate},
	{"INextBot.EndUpdate", INextBotEndUpdate},
	{"INextBot.Reset", INextBotReset},
	{"INextBot.IsRangeLessThanEntity", INextBotIsRangeLessThanEntity},
	{"INextBot.IsRangeLessThanVector", INextBotIsRangeLessThanVector},
	{"INextBot.IsRangeGreaterThanEntity", INextBotIsRangeGreaterThanEntity},
	{"INextBot.IsRangeGreaterThanVector", INextBotIsRangeGreaterThanVector},
	{"INextBot.GetRangeToEntity", INextBotGetRangeToEntity},
	{"INextBot.GetRangeToVector", INextBotGetRangeToVector},
	{"INextBot.GetRangeSquaredToEntity", INextBotGetRangeSquaredToEntity},
	{"INextBot.GetRangeSquaredToVector", INextBotGetRangeSquaredToVector},
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	{"INextBot.Get2DRangeToEntity", INextBotGet2DRangeToEntity},
	{"INextBot.Get2DRangeToVector", INextBotGet2DRangeToVector},
#endif
	{"INextBot.IsFriend", INextBotIsFriend},
	{"INextBot.IsEnemy", INextBotIsEnemy},
	{"INextBot.IsSelf", INextBotIsSelf},
	{"INextBot.IsDebugging", INextBotIsDebuggingNative},
	{"INextBot.MakeCustom", INextBotMakeCustom},
	{"INextBotComponent.Bot.get", INextBotComponentBotget},
	{"INextBotComponent.Reset", INextBotComponentBotReset},
#if SOURCE_ENGINE == SE_TF2
	{"CTFPathFollower.CTFPathFollower", CTFPathFollowerCTORNative},
#endif
	{"ChasePath.ChasePath", ChasePathCTORNative},
	{"DirectChasePath.DirectChasePath", DirectChasePathCTORNative},
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	{"InfectedChasePath.InfectedChasePath", InfectedChasePathCTORNative},
#endif
	{"RetreatPath.RetreatPath", RetreatPathCTORNative},
#if SOURCE_ENGINE == SE_TF2
	{"NextBotGroundLocomotionCustom.MaxAcceleration.set", NextBotGroundLocomotionCustomMaxAccelerationset},
	{"NextBotGroundLocomotionCustom.MaxDeceleration.set", NextBotGroundLocomotionCustomMaxDecelerationset},
	{"NextBotGroundLocomotion.Gravity.get", NextBotGroundLocomotionGravityget},
	{"NextBotGroundLocomotion.FrictionForward.get", NextBotGroundLocomotionFrictionForwardget},
	{"NextBotGroundLocomotion.FrictionSideways.get", NextBotGroundLocomotionFrictionSidewaysget},
	{"NextBotGroundLocomotionCustom.Gravity.set", NextBotGroundLocomotionCustomGravityset},
	{"NextBotGroundLocomotionCustom.FrictionForward.set", NextBotGroundLocomotionCustomFrictionForwardset},
	{"NextBotGroundLocomotionCustom.FrictionSideways.set", NextBotGroundLocomotionCustomFrictionSidewaysset},
#endif

	{"NextBotFlyingLocomotion.DesiredAltitude.get", NextBotFlyingLocomotionDesiredAltitudeget},
	{"NextBotFlyingLocomotion.DesiredAltitude.set", NextBotFlyingLocomotionDesiredAltitudeset},
	{"NextBotFlyingLocomotion.Acceleration.get", NextBotFlyingLocomotionAccelerationget},
	{"NextBotFlyingLocomotion.Acceleration.set", NextBotFlyingLocomotionAccelerationset},
	{"NextBotFlyingLocomotion.HorizontalDamp.get", NextBotFlyingLocomotionHorizontalDampget},
	{"NextBotFlyingLocomotion.HorizontalDamp.set", NextBotFlyingLocomotionHorizontalDampset},
	{"NextBotFlyingLocomotion.VerticalDamp.get", NextBotFlyingLocomotionVerticalDampget},
	{"NextBotFlyingLocomotion.VerticalDamp.set", NextBotFlyingLocomotionVerticalDampset},
	{"NextBotFlyingLocomotion.MaxYawRate.get", NextBotFlyingLocomotionMaxYawRateget},
	{"NextBotFlyingLocomotion.MaxYawRate.set", NextBotFlyingLocomotionMaxYawRateset},

#if SOURCE_ENGINE == SE_TF2
	{"NextBotFlyingLocomotion.MaxAcceleration.set", NextBotFlyingLocomotionMaxAccelerationset},
	{"NextBotFlyingLocomotion.MaxDeceleration.set", NextBotFlyingLocomotionMaxDecelerationset},
#endif
	{"NextBotFlyingLocomotion.StepHeight.set", NextBotFlyingLocomotionStepHeightset},
	{"NextBotFlyingLocomotion.MaxJumpHeight.set", NextBotFlyingLocomotionMaxJumpHeightset},
	{"NextBotFlyingLocomotion.DeathDropHeight.set", NextBotFlyingLocomotionDeathDropHeightset},
	{"NextBotFlyingLocomotion.RunSpeed.set", NextBotFlyingLocomotionRunSpeedset},
	{"NextBotFlyingLocomotion.WalkSpeed.set", NextBotFlyingLocomotionWalkSpeedset},
	{"NextBotFlyingLocomotion.SpeedLimit.set", NextBotFlyingLocomotionSpeedLimitset},
	{"NextBotFlyingLocomotion.TraversableSlopeLimit.set", NextBotFlyingLocomotionTraversableSlopeLimitset},

	{"NextBotFlyingLocomotion.set_function", NextBotFlyingLocomotionset_function},
	{"NextBotFlyingLocomotion.set_data", NextBotFlyingLocomotionset_data},
	{"NextBotFlyingLocomotion.get_data", NextBotFlyingLocomotionget_data},
	{"NextBotFlyingLocomotion.has_data", NextBotFlyingLocomotionhas_data},
	{"NextBotFlyingLocomotion.set_data_array", NextBotFlyingLocomotionset_data_array},
	{"NextBotFlyingLocomotion.get_data_array", NextBotFlyingLocomotionget_data_array},

#define MACRO_STRINGIFY(x) #x
#define MACRO_FUNC(x, y, f) \
	{MACRO_STRINGIFY(x) y, x##f},

	MACRO_FUNC(GameLocomotionCustom, ".StepHeight.set", StepHeightset)
	MACRO_FUNC(GameLocomotionCustom, ".MaxJumpHeight.set", MaxJumpHeightset)
	MACRO_FUNC(GameLocomotionCustom, ".DeathDropHeight.set", DeathDropHeightset)
	MACRO_FUNC(GameLocomotionCustom, ".RunSpeed.set", RunSpeedset)
	MACRO_FUNC(GameLocomotionCustom, ".WalkSpeed.set", WalkSpeedset)
	MACRO_FUNC(GameLocomotionCustom, ".SpeedLimit.set", SpeedLimitset)
	MACRO_FUNC(GameLocomotionCustom, ".TraversableSlopeLimit.set", TraversableSlopeLimitset)
	
	MACRO_FUNC(GameLocomotion, ".MaxYawRate.get", MaxYawRateget)
	MACRO_FUNC(GameLocomotion, ".Ladder.get", Ladderget)
	MACRO_FUNC(GameLocomotion, ".LadderDismountGoal.get", LadderDismountGoalget)
	MACRO_FUNC(GameLocomotion, ".GoingUpLadder.get", GoingUpLadderget)
	
	MACRO_FUNC(GameLocomotion, ".Ladder.set", Ladderset)
	MACRO_FUNC(GameLocomotion, ".LadderDismountGoal.set", LadderDismountGoalset)
	MACRO_FUNC(GameLocomotion, ".GoingUpLadder.set", GoingUpLadderset)
	MACRO_FUNC(GameLocomotion, ".ClimbingUpToLedge.set", ClimbingUpToLedgeset)
	
	MACRO_FUNC(GameLocomotionCustom, ".MaxYawRate.set", MaxYawRateset)
	
	MACRO_FUNC(GameVisionCustom, ".MaxVisionRange.set", MaxVisionRangeset)
	MACRO_FUNC(GameVisionCustom, ".MinRecognizeTime.set", MinRecognizeTimeset)
	MACRO_FUNC(GameVisionCustom, ".DefaultFieldOfView.set", DefaultFieldOfViewset)
	
	MACRO_FUNC(GameLocomotionCustom, ".set_function", set_function)
	MACRO_FUNC(GameLocomotionCustom, ".set_data", set_data)
	MACRO_FUNC(GameLocomotionCustom, ".get_data", get_data)
	MACRO_FUNC(GameLocomotionCustom, ".has_data", has_data)
	MACRO_FUNC(GameLocomotionCustom, ".set_data_array", set_data_array)
	MACRO_FUNC(GameLocomotionCustom, ".get_data_array", get_data_array)

	MACRO_FUNC(GameVisionCustom, ".set_function", set_function)
	MACRO_FUNC(GameVisionCustom, ".set_data", set_data)
	MACRO_FUNC(GameVisionCustom, ".get_data", get_data)
	MACRO_FUNC(GameVisionCustom, ".has_data", has_data)
	MACRO_FUNC(GameVisionCustom, ".set_data_array", set_data_array)
	MACRO_FUNC(GameVisionCustom, ".get_data_array", get_data_array)
	
	{"IIntentionCustom.set_function", IIntentionCustomset_function},
	{"IIntentionCustom.set_data", IIntentionCustomset_data},
	{"IIntentionCustom.get_data", IIntentionCustomget_data},
	{"IIntentionCustom.has_data", IIntentionCustomhas_data},
	{"IIntentionCustom.set_data_array", IIntentionCustomset_data_array},
	{"IIntentionCustom.get_data_array", IIntentionCustomget_data_array},
	
	{"IBodyCustom.set_function", IBodyCustomset_function},
	{"IBodyCustom.set_data", IBodyCustomset_data},
	{"IBodyCustom.get_data", IBodyCustomget_data},
	{"IBodyCustom.has_data", IBodyCustomhas_data},
	{"IBodyCustom.set_data_array", IBodyCustomset_data_array},
	{"IBodyCustom.get_data_array", IBodyCustomget_data_array},
	
	{"INextBotCustom.set_function", INextBotCustomset_function},
	{"INextBotCustom.set_function", INextBotCustomset_function},
	{"INextBotCustom.set_data", INextBotCustomset_data},
	{"INextBotCustom.get_data", INextBotCustomget_data},
	{"INextBotCustom.has_data", INextBotCustomhas_data},
	{"INextBotCustom.set_data_array", INextBotCustomset_data_array},
	{"INextBotCustom.get_data_array", INextBotCustomget_data_array},
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	{"INextBotCustom.AllowedToClimb.set", INextBotCustomAllowedToClimbset},
	{"INextBotCustom.ReactToSurvivorVisibility.set", INextBotCustomReactToSurvivorVisibilityset},
	{"INextBotCustom.ReactToSurvivorNoise.set", INextBotCustomReactToSurvivorNoiseset},
	{"INextBotCustom.ReactToSurvivorContact.set", INextBotCustomReactToSurvivorContactset},
#endif
#if SOURCE_ENGINE == SE_TF2
	{"IVision.ForEachKnownEntity", IVisionForEachKnownEntity},
	//{"IVision.CollectKnownEntities", IVisionCollectKnownEntities},
	{"IVision.GetPrimaryKnownThreat", IVisionGetPrimaryKnownThreat},
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	{"IVision.GetPrimaryRecognizedThreat", IVisionGetPrimaryRecognizedThreat},
#endif
	{"IVision.GetTimeSinceVisible", IVisionGetTimeSinceVisible},
#if SOURCE_ENGINE == SE_TF2
	{"IVision.GetClosestKnownTeam", IVisionGetClosestKnownTeam},
	{"IVision.GetClosestKnownFilter", IVisionGetClosestKnownFilter},
	{"IVision.GetKnownCount", IVisionGetKnownCount},
	{"IVision.GetKnown", IVisionGetKnown},
	{"IVision.AddKnownEntity", IVisionAddKnownEntity},
	{"IVision.ForgetEntity", IVisionForgetEntity},
	{"IVision.ForgetAllKnownEntities", IVisionForgetAllKnownEntities},
	//{"IVision.CollectPotentiallyVisibleEntities", IVisionCollectPotentiallyVisibleEntities},
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	{"IVision.GetClosestRecognizedTeam", IVisionGetClosestRecognizedTeam},
	{"IVision.GetRecognizedCount", IVisionGetRecognizedCount},
	{"IVision.GetClosestRecognizedFilter", IVisionGetClosestRecognizedFilter},
#endif
	{"IVision.IsAbleToSeeEntity", IVisionIsAbleToSeeEntity},
	{"IVision.IsAbleToSeeVector", IVisionIsAbleToSeeVector},
	{"IVision.IsIgnored", IVisionIsIgnored},
#if SOURCE_ENGINE == SE_TF2
	{"IVision.IsVisibleEntityNoticed", IVisionIsVisibleEntityNoticed},
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	{"IVision.IsNoticed", IVisionIsNoticed},
#endif
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
#if SOURCE_ENGINE == SE_TF2
	{"IBody.CollisionGroup.get", IBodyCollisionGroupget},
#endif
	{"IBody.GetHullMins", IBodyGetHullMins},
	{"IBody.GetHullMaxs", IBodyGetHullMaxs},
	{"IBody.PostureMobile.get", IBodyIsPostureMobile},
	{"IBody.PostureChanging.get", IBodyIsPostureChanging},
	{"IBody.StartActivity", IBodyStartActivity},
	{"IBody.SelectAnimationSequence", IBodySelectAnimationSequence},
	{"IBody.Activity.get", IBodyGetActivity},
	{"IBody.IsActivity", IBodyIsActivity},
	{"IBody.HasActivityType", IBodyHasActivityType},
	{"IBody.DesiredPosture.get", IBodyDesiredPostureget},
	{"IBody.DesiredPosture.set", IBodyDesiredPostureset},
	{"IBody.InDesiredPosture.get", IBodyInDesiredPosture},
	{"IBody.IsDesiredPosture", IBodyIsDesiredPosture},
	{"IBody.ActualPosture.get", IBodyActualPostureget},
	{"IBody.IsActualPosture", IBodyIsActualPosture},
	{"IBody.Arousal.set", IBodyArousalset},
	{"IBody.Arousal.get", IBodyArousalget},
	{"IBody.IsArousal", IBodyIsArousal},
	{"IBodyCustom.HullWidth.set", IBodyCustomHullWidthset},
	{"IBodyCustom.HullHeight.set", IBodyCustomHullHeightset},
	{"IBodyCustom.StandHullHeight.set", IBodyCustomStandHullHeightset},
	{"IBodyCustom.CrouchHullHeight.set", IBodyCustomCrouchHullHeightset},
	{"IBodyCustom.SolidMask.set", IBodyCustomSolidMaskset},
#if SOURCE_ENGINE == SE_TF2
	{"IBodyCustom.CollisionGroup.set", IBodyCustomCollisionGroupset},
#endif
	{"IBodyCustom.Sequence.get", IBodyCustomSequenceget},
#if SOURCE_ENGINE == SE_TF2
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
#endif
	{"AllocateNextBotCombatCharacter", AllocateNextBotCombatCharacter},
	{"GetNextBotCombatCharacterSize", GetNextBotCombatCharacterSize},
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	{"AllocateInfectedNextBotCombatCharacter", AllocateInfectedNextBotCombatCharacter},
	{"GetInfectedSize", GetInfectedSize},
#endif
	{"EntityIsCombatCharacter", EntityIsCombatCharacter},
	{"GetEntityLastKnownArea", GetEntityLastKnownArea},
	{"UpdateEntityLastKnownArea", UpdateEntityLastKnownArea},
	{"MakeEntityNextBot", MakeEntityNextBot},
	{"BehaviorActionEntry.BehaviorActionEntry", BehaviorActionEntryCTOR},
	{"BehaviorActionEntry.set_function", BehaviorActionEntryset_function},
	{"BehaviorActionEntry.create", BehaviorActionEntrycreate},
	{"BehaviorAction.Entry.get", BehaviorActionEntryget},
	{"BehaviorAction.set_data", BehaviorActionset_data},
	{"BehaviorAction.get_data", BehaviorActionget_data},
	{"BehaviorAction.has_data", BehaviorActionhas_data},
	{"BehaviorAction.set_data_array", BehaviorActionset_data_array},
	{"BehaviorAction.get_data_array", BehaviorActionget_data_array},
	{"IIntentionCustom.ResetBehavior", IIntentionCustomResetBehavior},
	{"IIntentionCustom.set_name", IIntentionCustomset_name},
	{"GetNavAreaVectorCount", GetNavAreaVectorCount},
	{"GetNavAreaFromVector", GetNavAreaFromVector},
	{"CombatCharacterEventKilled", CombatCharacterEventKilled},
	{"DirectionBetweenEntityVector", DirectionBetweenEntityVector},
	{"CollectSurroundingAreas", CollectSurroundingAreasNative},
	{"EntityVisibleEnt", EntityVisibleEnt},
	{"EntityVisibleVec", EntityVisibleVec},
	{"CombatCharacterIsHiddenByFogVec", CombatCharacterIsHiddenByFogVec},
	{"CombatCharacterIsHiddenByFogEnt", CombatCharacterIsHiddenByFogEnt},
	{"CombatCharacterIsHiddenByFogR", CombatCharacterIsHiddenByFogR},
	{"CombatCharacterGetFogObscuredRatioVec", CombatCharacterGetFogObscuredRatioVec},
	{"CombatCharacterGetFogObscuredRatioEnt", CombatCharacterGetFogObscuredRatioEnt},
	{"CombatCharacterGetFogObscuredRatioR", CombatCharacterGetFogObscuredRatioR},
	{"CombatCharacterIsLookingTowardsVec", CombatCharacterIsLookingTowardsVec},
	{"CombatCharacterIsLookingTowardsEnt", CombatCharacterIsLookingTowardsEnt},
	{"CombatCharacterIsInFieldOfViewVec", CombatCharacterIsInFieldOfViewVec},
	{"CombatCharacterIsInFieldOfViewEnt", CombatCharacterIsInFieldOfViewEnt},
	{"CombatCharacterIsLineOfSightClearVec", CombatCharacterIsLineOfSightClearVec},
	{"CombatCharacterIsLineOfSightClearEnt", CombatCharacterIsLineOfSightClearEnt},
	{"CombatCharacterInViewConeVec", CombatCharacterInViewConeVec},
	{"CombatCharacterInViewConeEnt", CombatCharacterInViewConeEnt},
	{"CombatCharacterInAimConeVec", CombatCharacterInAimConeVec},
	{"CombatCharacterInAimConeEnt", CombatCharacterInAimConeEnt},
	{"CombatCharacterIsAbleToSeeEnt", CombatCharacterIsAbleToSeeEnt},
	{NULL, NULL}
};

bool Sample::SDK_OnMetamodLoad(ISmmAPI *ismm, char *error, size_t maxlen, bool late)
{
	gpGlobals = ismm->GetCGlobals();
	GET_V_IFACE_ANY(GetEngineFactory, engine, IVEngineServer, INTERFACEVERSION_VENGINESERVER)
	GET_V_IFACE_ANY(GetEngineFactory, enginetrace, IEngineTrace, INTERFACEVERSION_ENGINETRACE_SERVER)
	GET_V_IFACE_ANY(GetEngineFactory, staticpropmgr, IStaticPropMgrServer, INTERFACEVERSION_STATICPROPMGR_SERVER)
	GET_V_IFACE_CURRENT(GetEngineFactory, icvar, ICvar, CVAR_INTERFACE_VERSION);
	GET_V_IFACE_ANY(GetServerFactory, servertools, IServerTools, VSERVERTOOLS_INTERFACE_VERSION)
	g_pCVar = icvar;
	ConVar_Register(0, this);
#if SOURCE_ENGINE == SE_TF2
	tf_nav_combat_decay_rate = g_pCVar->FindVar("tf_nav_combat_decay_rate");
#endif
	NextBotStop = g_pCVar->FindVar("nb_stop");
	NextBotDebugHistory = g_pCVar->FindVar("nb_debug_history");
	developer = g_pCVar->FindVar("developer");
	r_visualizetraces = g_pCVar->FindVar("r_visualizetraces");
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
	}
#if SOURCE_ENGINE == SE_TF2
	else if(type == CTFPathFollowerHandleType) {
		CTFPathFollower *obj = (CTFPathFollower *)object;
		delete obj;
	}
#endif
	else if(type == ChasePathHandleType) {
		ChasePath *obj = (ChasePath *)object;
		delete obj;
	} else if(type == DirectChasePathHandleType) {
		DirectChasePath *obj = (DirectChasePath *)object;
		delete obj;
	}
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	else if(type == InfectedChasePathHandleType) {
		InfectedChasePath *obj = (InfectedChasePath *)object;
		delete obj;
	}
#endif
	else if(type == RetreatPathHandleType) {
		RetreatPath *obj = (RetreatPath *)object;
		delete obj;
	} else if(type == BehaviorEntryHandleType) {
		SPActionEntry *obj = (SPActionEntry *)object;
		delete obj;
	}
}

bool Sample::RegisterConCommandBase(ConCommandBase *pCommand)
{
	META_REGCVAR(pCommand);
	return true;
}

#if SOURCE_ENGINE == SE_TF2
SH_DECL_HOOK0(CNavMesh, IsAuthoritative, const, 0, bool);

bool HookIsAuthoritative()
{
	RETURN_META_VALUE(MRES_SUPERCEDE, nav_authorative.GetBool());
}
#endif

IGameConfig *g_pGameConf = nullptr;

CDetour *pPathOptimize = nullptr;

CDetour *pTraverseLadder = nullptr;

CDetour *pMyNPCPointer = nullptr;
#if SOURCE_ENGINE == SE_TF2
CDetour *pApplyAccumulatedApproach = nullptr;
CDetour *pUpdatePosition = nullptr;
CDetour *pUpdateGroundConstraint = nullptr;
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
CDetour *pResolveZombieCollisions = nullptr;
CDetour *pVocalize = nullptr;
CDetour *pIsTankImmediatelyDangerousTo = nullptr;

DETOUR_DECL_MEMBER1(ResolveZombieCollisions, Vector, const Vector &, pos)
{
	ZombieBotLocomotion *loc = (ZombieBotLocomotion *)this;
	
	CBaseEntity *pEntity = loc->GetBot()->GetEntity();
	if(!pEntity->MyInfectedPointer()) {
		return pos;
	}
	
	return DETOUR_MEMBER_CALL(ResolveZombieCollisions)(pos);
}

DETOUR_DECL_MEMBER2(Vocalize, void, const char *, str, bool, unk)
{
	CBaseCombatCharacter *loc = (CBaseCombatCharacter *)this;
	
	if(loc->CanBeA(NBINFECT_MAGIC_NUMBER)) {
		return;
	}
	
	DETOUR_MEMBER_CALL(Vocalize)(str, unk);
}

DETOUR_DECL_MEMBER2(IsTankImmediatelyDangerousTo, bool, CBasePlayer *, pPlayer, CBaseCombatCharacter *, pBCC)
{
	if(pBCC->CanBeA(NBINFECT_MAGIC_NUMBER)) {
		INextBot *bot = pBCC->MyNextBotPointer();
		if(!bot) {
			return false;
		}
		
		IIntention *inte = bot->GetIntentionInterface();
		if(!inte) {
			return false;
		}
		
		IIntentionCustom *custinte = dynamic_cast<IIntentionCustom *>(inte);
		if(!custinte) {
			return false;
		}
		
		return custinte->IsTankImmediatelyDangerousTo(pPlayer, pBCC);
	}
	
	return DETOUR_MEMBER_CALL(IsTankImmediatelyDangerousTo)(pPlayer, pBCC);
}
#endif

enum npc_type : unsigned char
{
	npc_none =   0,
	npc_any =    (1 << 0),
	npc_undead = (1 << 1),
#if SOURCE_ENGINE == SE_TF2
	npc_zombie = (npc_any|npc_undead|(1 << 2)),
	npc_hhh =    (npc_any|npc_undead|(1 << 3)),
	npc_eye =    (npc_any|npc_undead|(1 << 4)),
	npc_wizard = (npc_any|npc_undead|(1 << 5)),
	npc_tank =   (npc_any|(1 << 6)),
#else
	#error
#endif
	npc_custom = (npc_any|(1 << 7))
};

static npc_type entity_to_npc_type(CBaseEntity *pEntity, std::string_view classname)
{
#if SOURCE_ENGINE == SE_TF2
	if(classname == "tf_zombie"sv) {
		return npc_zombie;
	} else if(classname == "headless_hatman"sv) {
		return npc_hhh;
	} else if(classname == "eyeball_boss"sv) {
		return npc_zombie;
	} else if(classname == "merasmus"sv) {
		return npc_wizard;
	} else if(classname == "tank_boss"sv) {
		return npc_tank;
#else
	#error
#endif
	} else if(!pEntity->IsPlayer() && pEntity->MyNextBotPointer()) {
		return npc_custom;
	}
	return npc_none;
}

#if SOURCE_ENGINE == SE_TF2
enum obj_type : unsigned char
{
	obj_none =       0,
	obj_any =        (1 << 0),
	obj_sentry =     (obj_any|(1 << 1)),
	obj_dispenser =  (obj_any|(1 << 2)),
	obj_teleporter = (obj_any|(1 << 3)),
	obj_sapper =     (obj_any|(1 << 4)),
	obj_custom =     (obj_any|(1 << 5))
};

static obj_type entity_to_obj_type(CBaseEntity *pEntity, std::string_view classname)
{
	if(classname == "obj_sentrygun"sv) {
		return obj_sentry;
	} else if(classname == "obj_dispenser"sv) {
		return obj_dispenser;
	} else if(classname == "obj_teleporter"sv) {
		return obj_teleporter;
	} else if(classname == "obj_sapper"sv) {
		return obj_sapper;
	} else if(pEntity->IsBaseObject()) {
		return obj_custom;
	}
	return obj_none;
}
#endif

int CBaseEntityBloodColor = -1;
int CBaseEntityPhysicsSolidMaskForEntity = -1;
int CBaseEntityShouldCollide = -1;
int CBaseCombatCharacterHasHumanGibs = -1;
int CBaseCombatCharacterHasAlienGibs = -1;
#if SOURCE_ENGINE == SE_TF2
int CBaseCombatCharacterGetBossType = -1;
#endif

#define CONTENTS_REDTEAM CONTENTS_TEAM1
#define CONTENTS_BLUETEAM CONTENTS_TEAM2

class EntityVTableHack
{
public:
	bool IsNPC()
	{
		CBaseEntity *pThis = (CBaseEntity *)this;

		if(!pThis->IsPlayer() && pThis->MyNextBotPointer()) {
			return true;
		}

		return false;
	}

	int BloodColor()
	{
		CBaseEntity *pThis = (CBaseEntity *)this;

		CBaseCombatCharacter *pCC = pThis->MyCombatCharacterPointer();
		if(pCC) {
			return pCC->GetBloodColor();
		}

		return DONT_BLEED;
	}

	bool ShouldCollide( int collisionGroup, int contentsMask )
	{
		CBaseEntity *pThis = (CBaseEntity *)this;

		if ( collisionGroup == COLLISION_GROUP_PLAYER_MOVEMENT )
		{
			switch( pThis->GetTeamNumber() )
			{
			case 2:
				if ( !( contentsMask & CONTENTS_REDTEAM ) )
					return false;
				break;

			case 3:
				if ( !( contentsMask & CONTENTS_BLUETEAM ) )
					return false;
				break;
			}
		}

		if ( pThis->GetCollisionGroup() == COLLISION_GROUP_DEBRIS )
		{
			if ( ! (contentsMask & CONTENTS_DEBRIS) )
				return false;
		}

		return true;
	}
};

enum HalloweenBossType
{
	HALLOWEEN_BOSS_INVALID = 0,
	HALLOWEEN_BOSS_HHH = 1,
	HALLOWEEN_BOSS_MONOCULUS = 2,
	HALLOWEEN_BOSS_MERASMUS = 3,

	//CUSTOM ADDED
	HALLOWEEN_BOSS_ZOMBIE,
	HALLOWEEN_BOSS_TANK
};

class CombatCharacterVTableHack : public EntityVTableHack
{
public:
	bool HasHumanGibs()
	{
		CBaseCombatCharacter *pThis = (CBaseCombatCharacter *)this;

		Class_T myClass = pThis->Classify();
		if(myClass == CLASS_PLAYER ||
			myClass == CLASS_PLAYER_ALLY ||
			myClass == CLASS_ZOMBIE ||
			myClass == CLASS_HHH ||
			myClass == CLASS_WIZARD) {
			return true;
		}

		return false;
	}

	bool HasAlienGibs()
	{
		CBaseCombatCharacter *pThis = (CBaseCombatCharacter *)this;

		Class_T myClass = pThis->Classify();
		if(myClass == CLASS_EYEBALL) {
			return true;
		}

		return false;
	}

	HalloweenBossType GetBossType()
	{
		CBaseCombatCharacter *pThis = (CBaseCombatCharacter *)this;

		Class_T myClass = pThis->Classify();
		switch(myClass) {
			case CLASS_ZOMBIE: return HALLOWEEN_BOSS_ZOMBIE;
			case CLASS_HHH: return HALLOWEEN_BOSS_HHH;
			case CLASS_WIZARD: return HALLOWEEN_BOSS_MERASMUS;
			case CLASS_EYEBALL: return HALLOWEEN_BOSS_MONOCULUS;
			case CLASS_TANK: return HALLOWEEN_BOSS_TANK;
		}

		return HALLOWEEN_BOSS_INVALID;
	}

	unsigned int PhysicsSolidMaskForEntity()
	{
		CBaseCombatCharacter *pThis = (CBaseCombatCharacter *)this;

		int teamContents = 0;

		int team = pThis->GetTeamNumber();
		switch(team) {
			case 2: teamContents |= CONTENTS_BLUETEAM; break;
			case 3: teamContents |= CONTENTS_REDTEAM; break;
		}

		return MASK_NPCSOLID | teamContents;
	}
};

class ZombieVTableHack : public CombatCharacterVTableHack
{
public:
	Class_T Classify()
	{
		return CLASS_ZOMBIE;
	}
};

class MerasmusVTableHack : public CombatCharacterVTableHack
{
public:
	Class_T Classify()
	{
		return CLASS_WIZARD;
	}
};

class TankVTableHack : public CombatCharacterVTableHack
{
public:
	Class_T Classify()
	{
		return CLASS_TANK;
	}
};

class HHHVTableHack : public CombatCharacterVTableHack
{
public:
	Class_T Classify()
	{
		return CLASS_HHH;
	}
};

class EyeballVTableHack : public CombatCharacterVTableHack
{
public:
	Class_T Classify()
	{
		return CLASS_EYEBALL;
	}
};

class BodyVTableHack
{
public:
	int GetSolidMask()
	{
		IBody *pThis = (IBody *)this;

		CBaseCombatCharacter *pEntity = pThis->GetBot()->GetEntity();

		int teamContents = 0;

		int team = pEntity->GetTeamNumber();
		switch(team) {
			case 2: teamContents |= CONTENTS_BLUETEAM; break;
			case 3: teamContents |= CONTENTS_REDTEAM; break;
		}

		return MASK_NPCSOLID | teamContents;
	}

	int GetCollisionGroup()
	{
		return COLLISION_GROUP_NPC;
	}
};

class MerasmusBodyVTableHack : public BodyVTableHack
{
public:
	int GetSolidMask()
	{
		IBody *pThis = (IBody *)this;

		CBaseCombatCharacter *pEntity = pThis->GetBot()->GetEntity();

		int teamContents = 0;

		int team = pEntity->GetTeamNumber();
		switch(team) {
			case 2: teamContents |= CONTENTS_BLUETEAM; break;
			case 3: teamContents |= CONTENTS_REDTEAM; break;
		}

		return MASK_NPCSOLID | CONTENTS_PLAYERCLIP | teamContents;
	}
};

class HatmanBodyVTableHack : public BodyVTableHack
{
public:
	int GetSolidMask()
	{
		IBody *pThis = (IBody *)this;

		CBaseCombatCharacter *pEntity = pThis->GetBot()->GetEntity();

		int teamContents = 0;

		int team = pEntity->GetTeamNumber();
		switch(team) {
			case 2: teamContents |= CONTENTS_BLUETEAM; break;
			case 3: teamContents |= CONTENTS_REDTEAM; break;
		}

		return MASK_NPCSOLID | CONTENTS_PLAYERCLIP | teamContents;
	}
};

class TankBodyVTableHack : public BodyVTableHack
{
public:
	int GetSolidMask()
	{
		IBody *pThis = (IBody *)this;

		CBaseCombatCharacter *pEntity = pThis->GetBot()->GetEntity();

		int teamContents = 0;

		int team = pEntity->GetTeamNumber();
		switch(team) {
			case 2: teamContents |= CONTENTS_BLUETEAM; break;
			case 3: teamContents |= CONTENTS_REDTEAM; break;
		}

		return MASK_NPCSOLID | teamContents;
	}
};

static std::vector<std::string> vtables_already_set{};

void Sample::OnEntityCreated(CBaseEntity *pEntity, const char *classname_ptr)
{
	std::string classname{classname_ptr};

	CBaseCombatCharacter *pCC = pEntity->MyCombatCharacterPointer();
	npc_type ntype{entity_to_npc_type(pEntity, classname)};

	if(pEntity->IsBaseObject()) {
		pCC->SetBloodColor(BLOOD_COLOR_MECH);
	}

	if(ntype & npc_any) {
		if(!pEntity->IsPlayer()) {
			pEntity->AddFlag(FL_NPC);
		}
		pEntity->AddIEFlags(EFL_DONTWALKON);
	}

	if(std::find(vtables_already_set.cbegin(), vtables_already_set.cend(), classname) != vtables_already_set.cend()) {
		return;
	}

	int patch_size = pCC ? CBaseCombatCharacterGetBossType : CBaseEntityPhysicsSolidMaskForEntity;
	patch_size *= sizeof(void *);
	patch_size += 4;

	void **entity_vtabl = *(void ***)pEntity;

	SourceHook::SetMemAccess(entity_vtabl, patch_size, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

	if(pEntity->IsBaseObject()) {
		//TODO!!!!!! move this to clsobj_hack
		entity_vtabl[CBaseEntityBloodColor] = func_to_void(&EntityVTableHack::BloodColor);
	}
	entity_vtabl[CBaseEntityIsNPC] = func_to_void(&EntityVTableHack::IsNPC);

	if(pCC) {
		switch(ntype) {
			case npc_zombie: {
				entity_vtabl[CBaseEntityClassify] = func_to_void(&ZombieVTableHack::Classify);

				IBody *body = pEntity->MyNextBotPointer()->GetBodyInterface();

				void **body_vtabl = *(void ***)body;

				int GetSolidMaskOffset = vfunc_index(&IBody::GetSolidMask);

				SourceHook::SetMemAccess(body_vtabl, (GetSolidMaskOffset * sizeof(void *)) + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

				body_vtabl[GetSolidMaskOffset] = func_to_void(&HatmanBodyVTableHack::GetSolidMask);
			} break;
			case npc_hhh: {
				entity_vtabl[CBaseEntityClassify] = func_to_void(&HHHVTableHack::Classify);

				IBody *body = pEntity->MyNextBotPointer()->GetBodyInterface();

				void **body_vtabl = *(void ***)body;

				int GetSolidMaskOffset = vfunc_index(&IBody::GetSolidMask);

				SourceHook::SetMemAccess(body_vtabl, (GetSolidMaskOffset * sizeof(void *)) + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

				body_vtabl[GetSolidMaskOffset] = func_to_void(&HatmanBodyVTableHack::GetSolidMask);
			} break;
			case npc_eye: {
				entity_vtabl[CBaseEntityClassify] = func_to_void(&EyeballVTableHack::Classify);
			} break;
			case npc_wizard: {
				entity_vtabl[CBaseEntityClassify] = func_to_void(&MerasmusVTableHack::Classify);

				IBody *body = pEntity->MyNextBotPointer()->GetBodyInterface();

				void **body_vtabl = *(void ***)body;

				int GetSolidMaskOffset = vfunc_index(&IBody::GetSolidMask);

				SourceHook::SetMemAccess(body_vtabl, (GetSolidMaskOffset * sizeof(void *)) + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

				body_vtabl[GetSolidMaskOffset] = func_to_void(&MerasmusBodyVTableHack::GetSolidMask);
			} break;
			case npc_tank: {
				entity_vtabl[CBaseEntityClassify] = func_to_void(&TankVTableHack::Classify);

				IBody *body = pEntity->MyNextBotPointer()->GetBodyInterface();

				void **body_vtabl = *(void ***)body;

				int GetSolidMaskOffset = vfunc_index(&IBody::GetSolidMask);

				SourceHook::SetMemAccess(body_vtabl, (GetSolidMaskOffset * sizeof(void *)) + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

				body_vtabl[GetSolidMaskOffset] = func_to_void(&TankBodyVTableHack::GetSolidMask);
			} break;
		}

		if(!pEntity->IsPlayer()) {
			if(!pEntity->IsBaseObject()) {
				entity_vtabl[CBaseEntityShouldCollide] = func_to_void(&EntityVTableHack::ShouldCollide);
			}
			if(ntype & npc_any) {
				entity_vtabl[CBaseEntityPhysicsSolidMaskForEntity] = func_to_void(&CombatCharacterVTableHack::PhysicsSolidMaskForEntity);
			}
		}
		entity_vtabl[CBaseCombatCharacterHasHumanGibs] = func_to_void(&CombatCharacterVTableHack::HasHumanGibs);
		entity_vtabl[CBaseCombatCharacterHasAlienGibs] = func_to_void(&CombatCharacterVTableHack::HasAlienGibs);
		entity_vtabl[CBaseCombatCharacterGetBossType] = func_to_void(&CombatCharacterVTableHack::GetBossType);
	}

	vtables_already_set.emplace_back(std::move(classname));
}

class CAI_BaseNPC;

DETOUR_DECL_MEMBER0(MyNPCPointer, CAI_BaseNPC *)
{
	CBaseEntity *pEntity = (CBaseEntity *)this;

	if(pEntity->MyNextBotPointer() ||
		pEntity->IsBaseObject()) {
		return nullptr;
	}

	return DETOUR_MEMBER_CALL(MyNPCPointer)();
}

IEntityFactoryDictionary *dictionary = nullptr;

static void RemoveEntity(CBaseEntity *pEntity)
{
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	static int m_iEFlagsOffset = -1;
	if(m_iEFlagsOffset == -1) {
		datamap_t *pMap = gamehelpers->GetDataMap(pEntity);
		sm_datatable_info_t info{};
		gamehelpers->FindDataMapInfo(pMap, "m_iEFlags", &info);
		m_iEFlagsOffset = info.actual_offset;
	}
	
	*(int *)((unsigned char *)pEntity + m_iEFlagsOffset) |= EFL_KILLME;
#elif SOURCE_ENGINE == SE_TF2
	servertools->RemoveEntity(pEntity);
#endif
}

int CGameRulesInitDefaultAIRelationships = -1;
int CGameRulesAIClassText = -1;
int CGameRulesShouldAutoAim = -1;
int CGameRulesGetAutoAimScale = -1;
int CGameRulesShouldCollideOffset = -1;
void *CGameRulesShouldCollidePtr = nullptr;

ConVar	sk_allow_autoaim( "sk_allow_autoaim", "1", FCVAR_REPLICATED | FCVAR_ARCHIVE_XBOX );

// Autoaim scale
ConVar	sk_autoaim_scale1( "sk_autoaim_scale1", "1.0", FCVAR_REPLICATED );
ConVar	sk_autoaim_scale2( "sk_autoaim_scale2", "1.0", FCVAR_REPLICATED );

enum
{
	GR_NOTTEAMMATE = 0,
	GR_TEAMMATE,
	GR_ENEMY,
	GR_ALLY,
	GR_NEUTRAL,
};

int CGameRulesGetSkillLevel = -1;
int CGameRulesPlayerRelationship = -1;
int CGameRulesRefreshSkillDataOffset = -1;
void *CGameRulesRefreshSkillDataPtr = nullptr;

class CGameRules
{
public:
	int GetSkillLevel()
	{
		return call_vfunc<int>(this, CGameRulesGetSkillLevel);
	}

	int PlayerRelationship( CBaseEntity *pPlayer, CBaseEntity *pTarget )
	{
		return call_vfunc<int, CGameRules, CBaseEntity *, CBaseEntity *>(this, CGameRulesPlayerRelationship, pPlayer, pTarget);
	}

	void RefreshSkillData(bool forceUpdate)
	{
		call_mfunc<bool, CGameRules, bool>(this, CGameRulesRefreshSkillDataPtr, forceUpdate);
	}

	bool ShouldCollide( int collisionGroup0, int collisionGroup1 )
	{
		return call_mfunc<bool, CGameRules, int, int>(this, CGameRulesShouldCollidePtr, collisionGroup0, collisionGroup1);
	}
};

class GameRulesVTableHack
{
public:
	const char *AIClassText(int classType)
	{
		switch (classType)
		{
			case CLASS_NONE:			return "CLASS_NONE";
			case CLASS_PLAYER:			return "CLASS_PLAYER";
			case CLASS_PLAYER_ALLY:		return "CLASS_PLAYER_ALLY";

			case CLASS_TFGOAL:			return "CLASS_TFGOAL";
			case CLASS_TFGOAL_TIMER:	return "CLASS_TFGOAL_TIMER";
			case CLASS_TFGOAL_ITEM:		return "CLASS_TFGOAL_ITEM";
			case CLASS_TFSPAWN:			return "CLASS_TFSPAWN";
			case CLASS_MACHINE:			return "CLASS_MACHINE";

			case CLASS_ZOMBIE:			return "CLASS_ZOMBIE";
			case CLASS_HHH:				return "CLASS_HHH";
			case CLASS_WIZARD:			return "CLASS_WIZARD";
			case CLASS_EYEBALL:			return "CLASS_EYEBALL";
			case CLASS_TANK:			return "CLASS_TANK";

			default:					return "MISSING CLASS in ClassifyText()";
		}
	}

	void InitDefaultAIRelationships()
	{
		CBaseCombatCharacter::AllocateDefaultRelationships();

		int i, j;
		for (i=0;i<NUM_AI_CLASSES;i++)
		{
			for (j=0;j<NUM_AI_CLASSES;j++)
			{
				CBaseCombatCharacter::SetDefaultRelationship((Class_T)i, (Class_T)j, D_NU, 0);
			}
		}

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_PLAYER, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_PLAYER_ALLY, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_TFGOAL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_TFGOAL_TIMER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_TFGOAL_ITEM, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_MACHINE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_ZOMBIE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_HHH, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_WIZARD, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_EYEBALL, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER, CLASS_TANK, D_HT, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_PLAYER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_PLAYER_ALLY, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_TFGOAL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_TFGOAL_TIMER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_TFGOAL_ITEM, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_MACHINE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_ZOMBIE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_HHH, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_WIZARD, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_EYEBALL, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_PLAYER_ALLY, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_PLAYER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_PLAYER_ALLY, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_TFGOAL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_TFGOAL_TIMER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_TFGOAL_ITEM, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_MACHINE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_ZOMBIE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_HHH, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_WIZARD, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_EYEBALL, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_PLAYER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_PLAYER_ALLY, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_TFGOAL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_TFGOAL_TIMER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_TFGOAL_ITEM, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_MACHINE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_ZOMBIE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_HHH, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_WIZARD, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_EYEBALL, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_TIMER, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_PLAYER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_PLAYER_ALLY, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_TFGOAL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_TFGOAL_TIMER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_TFGOAL_ITEM, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_MACHINE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_ZOMBIE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_HHH, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_WIZARD, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_EYEBALL, D_FR, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFGOAL_ITEM, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_PLAYER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_PLAYER_ALLY, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_TFGOAL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_TFGOAL_TIMER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_TFGOAL_ITEM, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_MACHINE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_ZOMBIE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_HHH, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_WIZARD, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_EYEBALL, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TFSPAWN, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_PLAYER, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_PLAYER_ALLY, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_TFGOAL, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_TFGOAL_TIMER, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_TFGOAL_ITEM, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_TFSPAWN, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_MACHINE, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_ZOMBIE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_HHH, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_WIZARD, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_EYEBALL, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_MACHINE, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_PLAYER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_PLAYER_ALLY, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_TFGOAL, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_TFGOAL_TIMER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_TFGOAL_ITEM, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_MACHINE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_ZOMBIE, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_HHH, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_WIZARD, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_EYEBALL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_ZOMBIE, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_PLAYER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_PLAYER_ALLY, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_TFGOAL, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_TFGOAL_TIMER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_TFGOAL_ITEM, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_MACHINE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_ZOMBIE, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_HHH, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_WIZARD, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_EYEBALL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_HHH, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_PLAYER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_PLAYER_ALLY, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_TFGOAL, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_TFGOAL_TIMER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_TFGOAL_ITEM, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_MACHINE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_ZOMBIE, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_HHH, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_WIZARD, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_EYEBALL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_WIZARD, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_PLAYER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_PLAYER_ALLY, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_TFGOAL, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_TFGOAL_TIMER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_TFGOAL_ITEM, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_MACHINE, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_ZOMBIE, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_HHH, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_WIZARD, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_EYEBALL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_EYEBALL, CLASS_TANK, D_LI, 0);

		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_NONE, D_NU, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_PLAYER, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_PLAYER_ALLY, D_HT, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_TFGOAL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_TFGOAL_TIMER, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_TFGOAL_ITEM, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_TFSPAWN, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_MACHINE, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_ZOMBIE, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_HHH, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_WIZARD, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_EYEBALL, D_LI, 0);
		CBaseCombatCharacter::SetDefaultRelationship(CLASS_TANK, CLASS_TANK, D_LI, 0);
	}

	bool ShouldAutoAim( CBasePlayer *pPlayer, edict_t *target )
	{
		CGameRules *pThis = (CGameRules *)this;

		// always autoaim, unless target is a teammate
		CBaseEntity *pTgt = target->GetIServerEntity()->GetBaseEntity();
		if ( pTgt && pTgt->IsPlayer() )
		{
			if ( pThis->PlayerRelationship( pPlayer, pTgt ) == GR_TEAMMATE )
				return false; // don't autoaim at teammates
		}

		return sk_allow_autoaim.GetBool() != 0;
	}

	float GetAutoAimScale( CBasePlayer *pPlayer )
	{
		CGameRules *pThis = (CGameRules *)this;

		switch( pThis->GetSkillLevel() )
		{
		case SKILL_EASY:
			return sk_autoaim_scale1.GetFloat();

		case SKILL_MEDIUM:
			return sk_autoaim_scale2.GetFloat();

		default:
			return 0.0f;
		}
	}

	void RefreshSkillData(bool forceUpdate)
	{
		CGameRules *pThis = (CGameRules *)this;

		pThis->RefreshSkillData(forceUpdate);

		char	szExec[256];

		Q_snprintf( szExec,sizeof(szExec), "exec skill_manifest.cfg\n" );

		engine->ServerCommand( szExec );
		engine->ServerExecute();

		Q_snprintf( szExec,sizeof(szExec), "exec skill%d.cfg\n", pThis->GetSkillLevel() );

		engine->ServerCommand( szExec );
		engine->ServerExecute();
	}

	bool ShouldCollide( int collisionGroup0, int collisionGroup1 )
	{
		CGameRules *pThis = (CGameRules *)this;

		if ( collisionGroup0 > collisionGroup1 )
		{
			// swap so that lowest is always first
			::V_swap( collisionGroup0, collisionGroup1 );
		}

		if ( collisionGroup1 == COLLISION_GROUP_NPC_ACTOR && collisionGroup0 != COLLISION_GROUP_PLAYER )
		{
			collisionGroup1 = COLLISION_GROUP_NPC;
		}

		if ( collisionGroup1 == COLLISION_GROUP_NPC_ACTOR && collisionGroup0 == COLLISION_GROUP_PLAYER )
			return false;
			
		if ( collisionGroup0 == COLLISION_GROUP_NPC_SCRIPTED && collisionGroup1 == COLLISION_GROUP_NPC_SCRIPTED )
			return false;

		return pThis->ShouldCollide( collisionGroup0, collisionGroup1 );
	}
};

static bool gamerules_vtable_assigned{false};

static bool nextbot_funcs_patched{false};
static int NextBotCombatCharacterSpawnCOLLISION_GROUP_PLAYER = -1;

void Sample::OnCoreMapStart(edict_t *pEdictList, int edictCount, int clientMax)
{
	if(!gamerules_vtable_assigned) {
		CGameRules *gamerules{(CGameRules *)g_pSDKTools->GetGameRules()};
		if(gamerules) {
			void **vtabl = *(void ***)gamerules;

			SourceHook::SetMemAccess(vtabl, (CGameRulesPlayerRelationship * sizeof(void *)) + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

			vtabl[CGameRulesInitDefaultAIRelationships] = func_to_void(&GameRulesVTableHack::InitDefaultAIRelationships);
			vtabl[CGameRulesAIClassText] = func_to_void(&GameRulesVTableHack::AIClassText);
			vtabl[CGameRulesShouldAutoAim] = func_to_void(&GameRulesVTableHack::ShouldAutoAim);
			vtabl[CGameRulesGetAutoAimScale] = func_to_void(&GameRulesVTableHack::GetAutoAimScale);
			CGameRulesRefreshSkillDataPtr = vtabl[CGameRulesRefreshSkillDataOffset];
			vtabl[CGameRulesRefreshSkillDataOffset] = func_to_void(&GameRulesVTableHack::RefreshSkillData);
			CGameRulesShouldCollidePtr = vtabl[CGameRulesShouldCollideOffset];
			vtabl[CGameRulesShouldCollideOffset] = func_to_void(&GameRulesVTableHack::ShouldCollide);

			call_vfunc<void, CGameRules>(gamerules, CGameRulesInitDefaultAIRelationships);
			call_vfunc<void, CGameRules, bool>(gamerules, CGameRulesRefreshSkillDataOffset, true);

			gamerules_vtable_assigned = true;
		}
	}

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	if(!infectedvtable) {
		IServerNetworkable *network = dictionary->Create("infected");
		CBaseEntity *pEntity = network->GetBaseEntity();
		
		infectedvtable = *(void ***)pEntity;
		
		RemoveEntity(pEntity);
	}
#endif

	if(!nextbot_funcs_patched) {
		NextBotCombatCharacter *pEntity = NextBotCombatCharacter::create(0);

		INextBot *bot = pEntity->MyNextBotPointer();
		IBody *body = bot->GetBodyInterface();

		void **entity_vtabl = *(void ***)pEntity;
		void **body_vtabl = *(void ***)body;

		RemoveEntity(pEntity);

		void *spawn = entity_vtabl[CBaseEntitySpawn];

		SourceHook::SetMemAccess(spawn, NextBotCombatCharacterSpawnCOLLISION_GROUP_PLAYER + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

		*(unsigned char *)((unsigned char *)spawn + NextBotCombatCharacterSpawnCOLLISION_GROUP_PLAYER) = COLLISION_GROUP_NPC;

		int GetSolidMaskOffset = vfunc_index(&IBody::GetSolidMask);
		int GetCollisionGroupOffset = vfunc_index(&IBody::GetCollisionGroup);

		SourceHook::SetMemAccess(body_vtabl, (GetCollisionGroupOffset * sizeof(void *)) + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

		body_vtabl[GetSolidMaskOffset] = func_to_void(&BodyVTableHack::GetSolidMask);
		body_vtabl[GetCollisionGroupOffset] = func_to_void(&BodyVTableHack::GetCollisionGroup);

		nextbot_funcs_patched = true;
	}
}

#include "funnyfile.h"

cell_t CollectSurroundingAreasNative(IPluginContext *pContext, const cell_t *params)
{
	HandleSecurity security(pContext->GetIdentity(), myself->GetIdentity());
	
	ICellArray *obj = nullptr;
	HandleError err = ((HandleSystemHack *)handlesys)->ReadCoreHandle(params[1], arraylist_handle, &security, (void **)&obj);
	if(err != HandleError_None)
	{
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);
	}

	CUtlVector<CNavArea *> nearbyAreaVector{};
	CollectSurroundingAreas(&nearbyAreaVector, (CNavArea *)params[2], sp_ftoc(params[3]), sp_ftoc(params[4]), sp_ftoc(params[5]));

	size_t len = nearbyAreaVector.Count();
	obj->resize(len);

	for(size_t i{0}; i < len; ++i) {
		*obj->at(i) = (cell_t)nearbyAreaVector[i];
	}

	return 0;
}

bool Sample::SDK_OnLoad(char *error, size_t maxlen, bool late)
{
	gameconfs->LoadGameConfigFile("nextbot", &g_pGameConf, error, maxlen);

	g_pGameConf->GetMemSig("CBaseCombatCharacter::AllocateDefaultRelationships", &CBaseCombatCharacterAllocateDefaultRelationships);

	int CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES1 = -1;
	g_pGameConf->GetOffset("CBaseCombatCharacter::AllocateDefaultRelationships::NUM_AI_CLASSES::1", &CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES1);
	int CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES2 = -1;
	g_pGameConf->GetOffset("CBaseCombatCharacter::AllocateDefaultRelationships::NUM_AI_CLASSES::2", &CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES2);
	int CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES3 = -1;
	g_pGameConf->GetOffset("CBaseCombatCharacter::AllocateDefaultRelationships::NUM_AI_CLASSES::3", &CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES3);

	g_pGameConf->GetOffset("NextBotCombatCharacter::Spawn::COLLISION_GROUP_PLAYER", &NextBotCombatCharacterSpawnCOLLISION_GROUP_PLAYER);

	SourceHook::SetMemAccess(CBaseCombatCharacterAllocateDefaultRelationships, CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES1 + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);
	SourceHook::SetMemAccess(CBaseCombatCharacterAllocateDefaultRelationships, CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES2 + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);
	SourceHook::SetMemAccess(CBaseCombatCharacterAllocateDefaultRelationships, CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES3 + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

	*(unsigned char *)((unsigned char *)CBaseCombatCharacterAllocateDefaultRelationships + CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES1) = (NUM_AI_CLASSES * sizeof(Relationship_t *));
	*(unsigned char *)((unsigned char *)CBaseCombatCharacterAllocateDefaultRelationships + CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES2) = (NUM_AI_CLASSES * sizeof(Relationship_t));
	*(unsigned char *)((unsigned char *)CBaseCombatCharacterAllocateDefaultRelationships + CBaseCombatCharacterAllocateDefaultRelationshipsNUM_AI_CLASSES3) = (NUM_AI_CLASSES * sizeof(Relationship_t *));

	void *CCleanupDefaultRelationShipsShutdown = nullptr;
	g_pGameConf->GetMemSig("CCleanupDefaultRelationShips::Shutdown", &CCleanupDefaultRelationShipsShutdown);

	int CCleanupDefaultRelationShipsShutdownNUM_AI_CLASSES = -1;
	g_pGameConf->GetOffset("CCleanupDefaultRelationShips::Shutdown::NUM_AI_CLASSES", &CCleanupDefaultRelationShipsShutdownNUM_AI_CLASSES);

	SourceHook::SetMemAccess(CCleanupDefaultRelationShipsShutdown, CCleanupDefaultRelationShipsShutdownNUM_AI_CLASSES + 4, SH_MEM_READ|SH_MEM_WRITE|SH_MEM_EXEC);

	*(unsigned char *)((unsigned char *)CCleanupDefaultRelationShipsShutdown + CCleanupDefaultRelationShipsShutdownNUM_AI_CLASSES) = (NUM_AI_CLASSES * sizeof(Relationship_t *));

	CDetourManager::Init(g_pSM->GetScriptingEngine(), g_pGameConf);

	g_pGameConf->GetOffset("sizeof(NextBotCombatCharacter)", &sizeofNextBotCombatCharacter);
	
	g_pGameConf->GetMemSig("Path::Path", &PathCTOR);
	g_pGameConf->GetMemSig("PathFollower::PathFollower", &PathFollowerCTOR);
	g_pGameConf->GetMemSig("NextBotCombatCharacter::NextBotCombatCharacter", &NextBotCombatCharacterCTOR);
#if SOURCE_ENGINE == SE_TF2
	g_pGameConf->GetMemSig("CTFPathFollower::CTFPathFollower", &CTFPathFollowerCTOR);
#endif
	g_pGameConf->GetMemSig("INextBot::INextBot", &INextBotCTOR);
	
	g_pGameConf->GetMemSig("Path::ComputePathDetails", &PathComputePathDetails);
	g_pGameConf->GetMemSig("Path::BuildTrivialPath", &PathBuildTrivialPath);
	g_pGameConf->GetMemSig("Path::FindNextOccludedNode", &PathFindNextOccludedNode);
	g_pGameConf->GetMemSig("Path::PostProcess", &PathPostProcess);
	g_pGameConf->GetMemSig("CNavMesh::GetGroundHeight", &CNavMeshGetGroundHeight);
	g_pGameConf->GetMemSig("CNavMesh::GetNearestNavArea", &CNavMeshGetNearestNavArea);
	g_pGameConf->GetMemSig("CBaseEntity::SetAbsOrigin", &CBaseEntitySetAbsOrigin);
	g_pGameConf->GetMemSig("INextBot::BeginUpdate", &INextBotBeginUpdatePtr);
	g_pGameConf->GetMemSig("INextBot::EndUpdate", &INextBotEndUpdatePtr);
	g_pGameConf->GetMemSig("CBaseEntity::CalcAbsoluteVelocity", &CBaseEntityCalcAbsoluteVelocity);
#if SOURCE_ENGINE == SE_TF2
	g_pGameConf->GetMemSig("NextBotGroundLocomotion::NextBotGroundLocomotion", &NextBotGroundLocomotionCTOR);
	g_pGameConf->GetMemSig("ILocomotion::ILocomotion", &ILocomotionCTOR);
	g_pGameConf->GetMemSig("NextBotGroundLocomotion::ResolveCollision", &NextBotGroundLocomotionResolveCollision);
	g_pGameConf->GetMemSig("IVision::IVision", &IVisionCTOR);
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	g_pGameConf->GetMemSig("ZombieBotLocomotion::ZombieBotLocomotion", &ZombieBotLocomotionCTOR);
	g_pGameConf->GetMemSig("ZombieBotVision::ZombieBotVision", &ZombieBotVisionCTOR);
#endif
	g_pGameConf->GetMemSig("CBaseEntity::SetGroundEntity", &CBaseEntitySetGroundEntity);
	g_pGameConf->GetMemSig("CTraceFilterSimple::ShouldHitEntity", &CTraceFilterSimpleShouldHitEntity);
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	g_pGameConf->GetMemSig("Path::Compute(CBaseCombatCharacter)", &PathComputeEntity);
	g_pGameConf->GetMemSig("Path::Compute(Vector)", &PathComputeVector);
	g_pGameConf->GetMemSig("INextBot::IsDebugging", &INextBotIsDebuggingPtr);
	g_pGameConf->GetMemSig("InfectedChasePath::ComputeAreaCrossing", &InfectedChasePathComputeAreaCrossing);
	g_pGameConf->GetMemSig("EntityFactoryDictionary", &EntityFactoryDictionaryPtr);

	dictionary = (void_to_func<IEntityFactoryDictionary *(*)()>(EntityFactoryDictionaryPtr))();
	
	sizeofInfected = dictionary->FindFactory("infected")->GetEntitySize();
#else
	dictionary = servertools->GetEntityFactoryDictionary();
#endif
	g_pGameConf->GetMemSig("CBaseCombatCharacter::Event_Killed", &CBaseCombatCharacterEvent_Killed);
	
	g_pGameConf->GetMemSig("NDebugOverlay::Line", &NDebugOverlayLine);
	g_pGameConf->GetMemSig("NDebugOverlay::VertArrow", &NDebugOverlayVertArrow);
	g_pGameConf->GetMemSig("NDebugOverlay::HorzArrow", &NDebugOverlayHorzArrow);
	g_pGameConf->GetMemSig("NDebugOverlay::Triangle", &NDebugOverlayTriangle);
	g_pGameConf->GetMemSig("NDebugOverlay::Circle", &NDebugOverlayCircle);
	g_pGameConf->GetMemSig("NavAreaBuildPath", &NavAreaBuildPathPtr);
	g_pGameConf->GetMemSig("CBaseEntity::CalcAbsolutePosition", &CBaseEntityCalcAbsolutePosition);
	
	g_pGameConf->GetOffset("CBaseEntity::MyNextBotPointer", &CBaseEntityMyNextBotPointer);
	SH_MANUALHOOK_RECONFIGURE(MyNextBotPointer, CBaseEntityMyNextBotPointer, 0, 0);
	
	int offset = -1;
	g_pGameConf->GetOffset("CBaseEntity::PerformCustomPhysics", &offset);
	SH_MANUALHOOK_RECONFIGURE(PerformCustomPhysics, offset, 0, 0);
	
	g_pGameConf->GetOffset("CBaseCombatCharacter::IsAreaTraversable", &offset);
	SH_MANUALHOOK_RECONFIGURE(IsAreaTraversable, offset, 0, 0);
	
	g_pGameConf->GetOffset("CBaseEntity::Spawn", &CBaseEntitySpawn);
	SH_MANUALHOOK_RECONFIGURE(Spawn, CBaseEntitySpawn, 0, 0);
	
	g_pGameConf->GetOffset("CBaseEntity::EyePosition", &CBaseEntityEyePosition);

	g_pGameConf->GetOffset("CBaseEntity::UpdateOnRemove", &offset);
	SH_MANUALHOOK_RECONFIGURE(UpdateOnRemove, offset, 0, 0);

	g_pGameConf->GetOffset("CBaseEntity::MyCombatCharacterPointer", &CBaseEntityMyCombatCharacterPointer);

#if SOURCE_ENGINE == SE_TF2
	g_pGameConf->GetOffset("CBaseEntity::IsBaseObject", &CBaseEntityIsBaseObject);
#endif

	g_pGameConf->GetOffset("CGameRules::InitDefaultAIRelationships", &CGameRulesInitDefaultAIRelationships);
	g_pGameConf->GetOffset("CGameRules::AIClassText", &CGameRulesAIClassText);
	g_pGameConf->GetOffset("CGameRules::GetSkillLevel", &CGameRulesGetSkillLevel);
	g_pGameConf->GetOffset("CGameRules::PlayerRelationship", &CGameRulesPlayerRelationship);
	g_pGameConf->GetOffset("CGameRules::RefreshSkillData", &CGameRulesRefreshSkillDataOffset);
	g_pGameConf->GetOffset("CGameRules::ShouldAutoAim", &CGameRulesShouldAutoAim);
	g_pGameConf->GetOffset("CGameRules::GetAutoAimScale", &CGameRulesGetAutoAimScale);
	g_pGameConf->GetOffset("CGameRules::ShouldCollide", &CGameRulesShouldCollideOffset);

	g_pGameConf->GetMemSig("CBaseCombatCharacter::SetDefaultRelationship", &CBaseCombatCharacterSetDefaultRelationship);

	g_pGameConf->GetOffset("CBaseEntity::Classify", &CBaseEntityClassify);
	g_pGameConf->GetOffset("CBaseEntity::IsNPC", &CBaseEntityIsNPC);
	g_pGameConf->GetOffset("CBaseCombatCharacter::HasHumanGibs", &CBaseCombatCharacterHasHumanGibs);
	g_pGameConf->GetOffset("CBaseCombatCharacter::HasAlienGibs", &CBaseCombatCharacterHasAlienGibs);
	g_pGameConf->GetOffset("CBaseEntity::BloodColor", &CBaseEntityBloodColor);
	g_pGameConf->GetOffset("CBaseEntity::PhysicsSolidMaskForEntity", &CBaseEntityPhysicsSolidMaskForEntity);
	g_pGameConf->GetOffset("CBaseEntity::ShouldCollide", &CBaseEntityShouldCollide);
#if SOURCE_ENGINE == SE_TF2
	g_pGameConf->GetOffset("CBaseCombatCharacter::GetBossType", &CBaseCombatCharacterGetBossType);
#endif
	pMyNPCPointer = DETOUR_CREATE_MEMBER(MyNPCPointer, "CBaseEntity::MyNPCPointer")
	pMyNPCPointer->EnableDetour();

#if SOURCE_ENGINE == SE_LEFT4DEAD2
	g_pGameConf->GetOffset("CBaseEntity::MyInfectedPointer", &CBaseEntityMyInfectedPointer);
	SH_MANUALHOOK_RECONFIGURE(MyInfectedPointer, CBaseEntityMyInfectedPointer, 0, 0);
	
	g_pGameConf->GetOffset("NextBotCombatCharacter::GetNextBotCombatCharacter", &NextBotCombatCharacterGetNextBotCombatCharacter);
	
	g_pGameConf->GetOffset("CBaseCombatCharacter::GetClass", &offset);
	SH_MANUALHOOK_RECONFIGURE(GetClass, offset, 0, 0);
	
	g_pGameConf->GetOffset("CBaseCombatCharacter::CanBeA", &CBaseCombatCharacterCanBeA);
	SH_MANUALHOOK_RECONFIGURE(CanBeA, CBaseCombatCharacterCanBeA, 0, 0);
#endif

	g_pGameConf->GetOffset("CBaseEntity::WorldSpaceCenter", &CBaseEntityWorldSpaceCenter);
	g_pGameConf->GetOffset("CBaseEntity::EyeAngles", &CBaseEntityEyeAngles);
	g_pGameConf->GetOffset("CBaseCombatCharacter::GetLastKnownArea", &CBaseCombatCharacterGetLastKnownArea);
	g_pGameConf->GetOffset("CBaseCombatCharacter::UpdateLastKnownArea", &CBaseCombatCharacterUpdateLastKnownArea);

	g_pGameConf->GetOffset("CBaseCombatCharacter::IsHiddenByFog(Vector)", &CBaseCombatCharacterIsHiddenByFogVec);
	g_pGameConf->GetOffset("CBaseCombatCharacter::IsHiddenByFog(CBaseEntity)", &CBaseCombatCharacterIsHiddenByFogEnt);
	g_pGameConf->GetOffset("CBaseCombatCharacter::IsHiddenByFog(float)", &CBaseCombatCharacterIsHiddenByFogR);

	g_pGameConf->GetOffset("CBaseCombatCharacter::GetFogObscuredRatio(Vector)", &CBaseCombatCharacterGetFogObscuredRatioVec);
	g_pGameConf->GetOffset("CBaseCombatCharacter::GetFogObscuredRatio(CBaseEntity)", &CBaseCombatCharacterGetFogObscuredRatioEnt);
	g_pGameConf->GetOffset("CBaseCombatCharacter::GetFogObscuredRatio(float)", &CBaseCombatCharacterGetFogObscuredRatioR);

	g_pGameConf->GetOffset("CBaseCombatCharacter::IsLookingTowards(CBaseEntity)", &CBaseCombatCharacterIsLookingTowardsEnt);
	g_pGameConf->GetOffset("CBaseCombatCharacter::IsLookingTowards(Vector)", &CBaseCombatCharacterIsLookingTowardsVec);

	g_pGameConf->GetOffset("CBaseCombatCharacter::IsInFieldOfView(CBaseEntity)", &CBaseCombatCharacterIsInFieldOfViewEnt);
	g_pGameConf->GetOffset("CBaseCombatCharacter::IsInFieldOfView(Vector)", &CBaseCombatCharacterIsInFieldOfViewVec);

	g_pGameConf->GetOffset("CBaseCombatCharacter::IsLineOfSightClear(CBaseEntity)", &CBaseCombatCharacterIsLineOfSightClearEnt);
	g_pGameConf->GetOffset("CBaseCombatCharacter::IsLineOfSightClear(Vector)", &CBaseCombatCharacterIsLineOfSightClearVec);

	g_pGameConf->GetOffset("CBaseEntity::FVisible(Vector)", &CBaseEntityFVisibleVec);
	g_pGameConf->GetOffset("CBaseEntity::FVisible(CBaseEntity)", &CBaseEntityFVisibleEnt);

	g_pGameConf->GetOffset("CBaseCombatCharacter::FInViewCone(Vector)", &CBaseCombatCharacterFInViewConeVec);
	g_pGameConf->GetOffset("CBaseCombatCharacter::FInViewCone(CBaseEntity)", &CBaseCombatCharacterFInViewConeEnt);

	g_pGameConf->GetOffset("CBaseCombatCharacter::FInAimCone(Vector)", &CBaseCombatCharacterFInAimConeVec);
	g_pGameConf->GetOffset("CBaseCombatCharacter::FInAimCone(CBaseEntity)", &CBaseCombatCharacterFInAimConeEnt);

	g_pGameConf->GetMemSig("CBaseCombatCharacter::IsAbleToSee(CBaseEntity)", &CBaseCombatCharacterIsAbleToSeeEnt);
	g_pGameConf->GetMemSig("CBaseCombatCharacter::IsAbleToSee(CBaseCombatCharacter)", &CBaseCombatCharacterIsAbleToSeeCC);

#if SOURCE_ENGINE == SE_TF2
	g_pGameConf->GetOffset("CFuncNavCost::GetCostMultiplier", &CFuncNavCostGetCostMultiplier);
#endif
	
	g_pGameConf->GetMemSig("TheNavMesh", (void **)&TheNavMesh);
	g_pGameConf->GetMemSig("TheNavAreas", (void **)&TheNavAreas);

	g_pGameConf->GetMemSig("CNavArea::m_masterMarker", (void **)&CNavArea::m_masterMarker);
	g_pGameConf->GetMemSig("CNavArea::m_openList", (void **)&CNavArea::m_openList);
	g_pGameConf->GetMemSig("CNavArea::m_openListTail", (void **)&CNavArea::m_openListTail);
	
#if SOURCE_ENGINE == SE_TF2
	SH_ADD_HOOK(CNavMesh, IsAuthoritative, TheNavMesh, SH_STATIC(HookIsAuthoritative), false);
#endif
	
	pPathOptimize = DETOUR_CREATE_MEMBER(PathOptimize, "Path::Optimize")
	pPathOptimize->EnableDetour();
	
#if SOURCE_ENGINE == SE_TF2
	pApplyAccumulatedApproach = DETOUR_CREATE_MEMBER(ApplyAccumulatedApproach, "NextBotGroundLocomotion::ApplyAccumulatedApproach")
	pApplyAccumulatedApproach->EnableDetour();
	
	pUpdatePosition = DETOUR_CREATE_MEMBER(UpdatePosition, "NextBotGroundLocomotion::UpdatePosition")
	pUpdatePosition->EnableDetour();
	
	pUpdateGroundConstraint = DETOUR_CREATE_MEMBER(UpdateGroundConstraint, "NextBotGroundLocomotion::UpdateGroundConstraint")
	pUpdateGroundConstraint->EnableDetour();
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	pResolveZombieCollisions = DETOUR_CREATE_MEMBER(ResolveZombieCollisions, "ZombieBotLocomotion::ResolveZombieCollisions")
	pResolveZombieCollisions->EnableDetour();
	
	pVocalize = DETOUR_CREATE_MEMBER(Vocalize, "Infected::Vocalize")
	pVocalize->EnableDetour();
	
	pIsTankImmediatelyDangerousTo = DETOUR_CREATE_MEMBER(IsTankImmediatelyDangerousTo, "SurvivorIntention::IsTankImmediatelyDangerousTo")
	pIsTankImmediatelyDangerousTo->EnableDetour();
#endif
	
#if SOURCE_ENGINE == SE_TF2
	pTraverseLadder = DETOUR_CREATE_MEMBER(TraverseLadder, "NextBotGroundLocomotion::TraverseLadder")
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	pTraverseLadder = DETOUR_CREATE_MEMBER(TraverseLadder, "ZombieBotLocomotion::TraverseLadder")
#endif
	pTraverseLadder->EnableDetour();
	
	sm_sendprop_info_t info{};
	gamehelpers->FindSendPropInfo("CBaseEntity", "m_iTeamNum", &info);
	m_iTeamNumOffset = info.actual_offset;
	
	gamehelpers->FindSendPropInfo("CBaseEntity", "m_hGroundEntity", &info);
	m_hGroundEntityOffset = info.actual_offset;
	
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	gamehelpers->FindSendPropInfo("CTerrorPlayer", "m_isGhost", &info);
	m_isGhostOffset = info.actual_offset;
#endif
	
	g_pEntityList = reinterpret_cast<CBaseEntityList *>(gamehelpers->GetGlobalEntityList());

	HandleSystemHack::init();

	PathHandleType = handlesys->CreateType("Path", this, 0, nullptr, nullptr, myself->GetIdentity(), nullptr);
	PathFollowerHandleType = handlesys->CreateType("PathFollower", this, PathHandleType, nullptr, nullptr, myself->GetIdentity(), nullptr);
	
#if SOURCE_ENGINE == SE_TF2
	CTFPathFollowerHandleType = ((HandleSystemHack *)handlesys)->CreateTypeAllowChild("CTFPathFollower", this, PathFollowerHandleType, nullptr, nullptr, myself->GetIdentity(), nullptr);
#endif
	ChasePathHandleType = ((HandleSystemHack *)handlesys)->CreateTypeAllowChild("ChasePath", this, PathFollowerHandleType, nullptr, nullptr, myself->GetIdentity(), nullptr);
	RetreatPathHandleType = ((HandleSystemHack *)handlesys)->CreateTypeAllowChild("RetreatPath", this, PathFollowerHandleType, nullptr, nullptr, myself->GetIdentity(), nullptr);
	DirectChasePathHandleType = ((HandleSystemHack *)handlesys)->CreateTypeAllowChild("DirectChasePath", this, ChasePathHandleType, nullptr, nullptr, myself->GetIdentity(), nullptr);
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	InfectedChasePathHandleType = ((HandleSystemHack *)handlesys)->CreateTypeAllowChild("InfectedChasePath", this, DirectChasePathHandleType, nullptr, nullptr, myself->GetIdentity(), nullptr);
#endif
	
	BehaviorEntryHandleType = handlesys->CreateType("BehaviorActionEntry", this, 0, nullptr, nullptr, myself->GetIdentity(), nullptr);
	
	plsys->AddPluginsListener(this);
	
#ifdef __HAS_DAMAGERULES
	sharesys->AddDependency(myself, "damagerules.ext", false, true);
#endif
#ifdef __HAS_ANIMHELPERS
	sharesys->AddDependency(myself, "animhelpers.ext", false, true);
#endif

	sharesys->RegisterLibrary(myself, "nextbot");
	sharesys->AddNatives(myself, natives);

	return true;
}

void Sample::OnPluginLoaded(IPlugin *plugin)
{
	
}

void Sample::OnPluginUnloaded(IPlugin *plugin)
{
	auto it = spnbcomponents.find(plugin->GetIdentity());
	if(it != spnbcomponents.end()) {
		std::vector<IPluginNextBotComponent *> &vec = it->second;
		
		for(IPluginNextBotComponent *inte : vec) {
			if(!inte) {
				continue;
			}

			inte->plugin_unloaded();
		}

		spnbcomponents.erase(it);
	}
}

ISDKHooks *g_pSDKHooks = nullptr;

void Sample::SDK_OnAllLoaded()
{
#ifdef __HAS_DAMAGERULES
	SM_GET_LATE_IFACE(DAMAGERULES, g_pDamageRules);
#endif
#ifdef __HAS_ANIMHELPERS
	SM_GET_LATE_IFACE(ANIMHELPERS, g_pAnimHelpers);
#endif
	SM_GET_LATE_IFACE(SDKHOOKS, g_pSDKHooks);
	SM_GET_LATE_IFACE(SDKTOOLS, g_pSDKTools);
	g_pSDKHooks->AddEntityListener(this);
}

bool Sample::QueryRunning(char *error, size_t maxlength)
{
	SM_CHECK_IFACE(SDKHOOKS, g_pSDKHooks);
	SM_CHECK_IFACE(SDKTOOLS, g_pSDKTools);
	return true;
}

bool Sample::QueryInterfaceDrop(SMInterface *pInterface)
{
#ifdef __HAS_DAMAGERULES
	if(pInterface == g_pDamageRules)
		return false;
	else
#endif
#ifdef __HAS_ANIMHELPERS
	if(pInterface == g_pAnimHelpers)
		return false;
	else
#endif
	if(pInterface == g_pSDKHooks)
		return false;
	else if(pInterface == g_pSDKTools)
		return false;
	return IExtensionInterface::QueryInterfaceDrop(pInterface);
}

void Sample::NotifyInterfaceDrop(SMInterface *pInterface)
{
#ifdef __HAS_DAMAGERULES
	if(strcmp(pInterface->GetInterfaceName(), SMINTERFACE_DAMAGERULES_NAME) == 0)
	{
		g_pDamageRules = NULL;
	} else 
#endif
#ifdef __HAS_ANIMHELPERS
	if(strcmp(pInterface->GetInterfaceName(), SMINTERFACE_ANIMHELPERS_NAME) == 0)
	{
		g_pAnimHelpers = NULL;
	} else 
#endif
	if(strcmp(pInterface->GetInterfaceName(), SMINTERFACE_SDKHOOKS_NAME) == 0)
	{
		g_pSDKHooks = NULL;
	}
	else if(strcmp(pInterface->GetInterfaceName(), SMINTERFACE_SDKTOOLS_NAME) == 0)
	{
		g_pSDKTools = NULL;
	}
}

void Sample::SDK_OnUnload()
{
	pPathOptimize->Destroy();
	pMyNPCPointer->Destroy();
#if SOURCE_ENGINE == SE_TF2
	pApplyAccumulatedApproach->Destroy();
	pUpdatePosition->Destroy();
	pUpdateGroundConstraint->Destroy();
#elif SOURCE_ENGINE == SE_LEFT4DEAD2
	pResolveZombieCollisions->Destroy();
	pVocalize->Destroy();
	pIsTankImmediatelyDangerousTo->Destroy();
#endif
	pTraverseLadder->Destroy();
	plsys->RemovePluginsListener(this);
	g_pSDKHooks->RemoveEntityListener(this);
	handlesys->RemoveType(PathHandleType, myself->GetIdentity());
	handlesys->RemoveType(PathFollowerHandleType, myself->GetIdentity());
#if SOURCE_ENGINE == SE_TF2
	handlesys->RemoveType(CTFPathFollowerHandleType, myself->GetIdentity());
#endif
	handlesys->RemoveType(ChasePathHandleType, myself->GetIdentity());
	handlesys->RemoveType(DirectChasePathHandleType, myself->GetIdentity());
#if SOURCE_ENGINE == SE_LEFT4DEAD2
	handlesys->RemoveType(InfectedChasePathHandleType, myself->GetIdentity());
#endif
	handlesys->RemoveType(RetreatPathHandleType, myself->GetIdentity());
	handlesys->RemoveType(BehaviorEntryHandleType, myself->GetIdentity());
	gameconfs->CloseGameConfigFile(g_pGameConf);
}
