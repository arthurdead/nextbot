#if defined nextbot_included
	#endinput
#endif
#define nextbot_included

#if !defined GAME_TF2 && !defined GAME_L4D2
	#error please define either GAME_TF2 or GAME_L4D2
#endif

#include <sdktools>
#include <dhooks>

#if defined REQUIRE_EXTENSIONS
	#define __NB_EXT_WAS_REQUIRED
#endif

#if defined REQUIRE_PLUGIN
	#define __NB_PL_WAS_REQUIRED
#endif

#undef REQUIRE_EXTENSIONS
#undef REQUIRE_PLUGIN
#tryinclude <animhelpers>

#if defined __NB_PL_WAS_REQUIRED
	#define REQUIRE_PLUGIN
#endif

#tryinclude <damagerules>
#tryinclude <datamaps>

#if defined __NB_EXT_WAS_REQUIRED
	#define REQUIRE_EXTENSIONS
#endif

#if !defined animhelpers_included
enum Activity
{
	ACT_INVALID = -1
};
#endif

#define	CONTENTS_BLOCKLOS 0x40
#define CONTENTS_REDTEAM CONTENTS_TEAM1
#define CONTENTS_BLUETEAM CONTENTS_TEAM2
#define MASK_BLOCKLOS (CONTENTS_SOLID|CONTENTS_MOVEABLE|CONTENTS_BLOCKLOS)
#define MASK_BLOCKLOS_AND_NPCS (MASK_BLOCKLOS|CONTENTS_MONSTER)

#if defined GAME_TF2
	#define TEAM_ANY -2
#elseif defined GAME_L4D2
	#define TEAM_ANY -1
#endif

#define STEP_HEIGHT 18.0

#define BEHAVIOR_REASON_MAX 64

enum EventResultPriorityType
{
	RESULT_NONE,		// no result
	RESULT_TRY,			// use this result, or toss it out, either is ok
	RESULT_IMPORTANT,	// try extra-hard to use this result
	RESULT_CRITICAL		// this result must be used - emit an error if it can't be
};

methodmap CustomBehaviorActionEntry < Handle
{
	public native CustomBehaviorActionEntry(const char[] name);
	
	public native void set_function(const char[] name, Function func);

	public native CustomBehaviorAction create();
};

methodmap BehaviorAction
{
	/*public native int GetName(char[] name, int len);
	public native bool IsNamed(const char[] name);
	public native int GetFullName(char[] name, int len);
	public native int GetDebugString(char[] name, int len);*/

	property int Actor
	{
		public native get();
	}
};

#define BehaviorAction_Null (view_as<BehaviorAction>(Address_Null))

methodmap CustomBehaviorAction < BehaviorAction
{
	property CustomBehaviorActionEntry Entry
	{
		public native get();
	}
	
	public native void set_data(const char[] name, any data);
	public native any get_data(const char[] name);
	public native bool has_data(const char[] name);
	
	public native void set_data_array(const char[] name, const any[] data, int len);
	public native void get_data_array(const char[] name, any[] data, int len);

	public native void set_function(const char[] name, Function func);
	public native Function get_function(const char[] name, Handle &pl = null);
	public native bool has_function(const char[] name);
};

#define CustomBehaviorAction_Null (view_as<CustomBehaviorAction>(Address_Null))

enum struct BehaviorResult
{
	BehaviorAction action;
	char reason[BEHAVIOR_REASON_MAX];
	EventResultPriorityType priority;

	void set_reason(const char[] str)
	{
		strcopy(this.reason, BEHAVIOR_REASON_MAX, str);
	}

	BehaviorResultType Continue()
	{
		return BEHAVIOR_CONTINUE;
	}

	BehaviorResultType ChangeTo(BehaviorAction action, const char[] reason = NULL_STRING)
	{
		this.action = action;
		if(!IsNullString(reason)) {
			this.set_reason(reason);
		}
		return BEHAVIOR_CHANGE_TO;
	}

	BehaviorResultType SuspendFor(BehaviorAction action, const char[] reason = NULL_STRING)
	{
		this.action = action;
		if(!IsNullString(reason)) {
			this.set_reason(reason);
		}
		return BEHAVIOR_SUSPEND_FOR;
	}

	BehaviorResultType Done(const char[] reason = NULL_STRING)
	{
		if(!IsNullString(reason)) {
			this.set_reason(reason);
		}
		return BEHAVIOR_DONE;
	}

	BehaviorResultType TryContinue(EventResultPriorityType priority = RESULT_TRY)
	{
		this.priority = priority;
		return BEHAVIOR_CONTINUE;
	}

	BehaviorResultType TryChangeTo(BehaviorAction action, EventResultPriorityType priority = RESULT_TRY, const char[] reason = NULL_STRING)
	{
		this.action = action;
		if(!IsNullString(reason)) {
			this.set_reason(reason);
		}
		this.priority = priority;
		return BEHAVIOR_CHANGE_TO;
	}

	BehaviorResultType TrySuspendFor(BehaviorAction action, EventResultPriorityType priority = RESULT_TRY, const char[] reason = NULL_STRING)
	{
		this.action = action;
		if(!IsNullString(reason)) {
			this.set_reason(reason);
		}
		this.priority = priority;
		return BEHAVIOR_SUSPEND_FOR;
	}

	BehaviorResultType TryDone(EventResultPriorityType priority = RESULT_TRY, const char[] reason = NULL_STRING)
	{
		if(!IsNullString(reason)) {
			this.set_reason(reason);
		}
		this.priority = priority;
		return BEHAVIOR_DONE;
	}

	BehaviorResultType TryToSustain(EventResultPriorityType priority = RESULT_TRY, const char[] reason = NULL_STRING)
	{
		if(!IsNullString(reason)) {
			this.set_reason(reason);
		}
		this.priority = priority;
		return BEHAVIOR_SUSTAIN;
	}
}

#if defined GAME_TF2
#define GameLocomotionCustom NextBotGroundLocomotionCustom
#define GameVisionCustom IVisionCustom
#elseif defined GAME_L4D2
#define GameLocomotionCustom ZombieBotLocomotionCustom
#define GameVisionCustom ZombieBotVisionCustom
#endif

typeset nb_funcs_t
{
	//INextBotCustom.IsAbleToBlockMovementOf
	function MRESReturn (INextBotCustom custom, INextBot bot, INextBot other, bool &result);

	//INextBotCustom.ShouldTouch
	function MRESReturn (INextBotCustom custom, INextBot bot, int other, bool &result);

	//IIntentionCustom.InitialContainedAction
	function BehaviorAction (int entity);

	//GameLocomotionCustom.TraverseLadder
	function MRESReturn (GameLocomotionCustom locomotion, bool &result);

	//ILocomotion.ClimbLadder
	//ILocomotion.DescendLadder
	function MRESReturn (ILocomotion locomotion, CNavLadder ladder, CNavArea goal);

	//ILocomotion.ClimbUpToLedge
	function MRESReturn (ILocomotion locomotion, const float goal[3], const float fwd[3], int obstacle, bool &result);

	//ILocomotion.ShouldCollideWith
	function MRESReturn (ILocomotion locomotion, int other, bool &result);

	//ILocomotion.IsEntityTraversable
	function MRESReturn (ILocomotion locomotion, int obstacle, TraverseWhenType when, bool &result);

	//IBodyCustom.SelectAnimationSequence
	function int (IBodyCustom body, int entity, Activity act);

	//IBodyCustom.TranslateActivity
	function Activity (IBodyCustom body, Activity act);

	//BehaviorAction.OnStart
	//BehaviorAction.OnSuspend
	//BehaviorAction.OnResume
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, BehaviorAction prior, BehaviorResult result);

	//BehaviorAction.Update
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, float interval, BehaviorResult result);

	//BehaviorAction.OnEnd
	function void (CustomBehaviorAction action, INextBot bot, int entity, BehaviorAction next);

	//BehaviorAction.InitialContainedAction
	function BehaviorAction (CustomBehaviorAction action, INextBot bot, int entity);

	//BehaviorAction.OnLeaveGround
	//BehaviorAction.OnLandOnGround
	//BehaviorAction.OnContact
	//BehaviorAction.OnSight
	//BehaviorAction.OnLostSight
	//BehaviorAction.OnShoved
	//BehaviorAction.OnBlinded
	//BehaviorAction.OnTerritoryContested
	//BehaviorAction.OnTerritoryCaptured
	//BehaviorAction.OnTerritoryLost
	//BehaviorAction.OnThreatChanged
	//BehaviorAction.OnHitByVomitJar
	//BehaviorAction.OnDrop
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, int ground, BehaviorResult result);

	//BehaviorAction.OnAnimationEvent
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, int event, BehaviorResult result);

	//BehaviorAction.OnAnimationActivityComplete
	//BehaviorAction.OnAnimationActivityInterrupted
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, Activity act, BehaviorResult result);

	//BehaviorAction.OnMoveToSuccess
	//BehaviorAction.OnStuck
	//BehaviorAction.OnUnStuck
	//BehaviorAction.OnIgnite
	//BehaviorAction.OnWin
	//BehaviorAction.OnLose
	//BehaviorAction.OnEnteredSpit
	//BehaviorAction.OnModelChanged
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, BehaviorResult result);

	//BehaviorAction.OnMoveToFailure
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, MoveToFailureType type, BehaviorResult result);

	//BehaviorAction.OnSound
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, int source, const float pos[3], BehaviorResult result);

	//BehaviorAction.OnWeaponFired
	//BehaviorAction.OnActorEmoted
	//BehaviorAction.OnPickUp
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, int who, int weapon, BehaviorResult result);

#if defined damagerules_included
	//BehaviorAction.OnInjured
	//BehaviorAction.OnKilled
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, const CTakeDamageInfo info, BehaviorResult result);

	//BehaviorAction.OnOtherKilled
	function BehaviorResultType (CustomBehaviorAction action, INextBot bot, int entity, int victim, const CTakeDamageInfo info, BehaviorResult result);
#endif

	//NextBotFlyingLocomotion.LimitPitch
	function void (NextBotFlyingLocomotion locomotion, float &pitch);

	//BehaviorAction.ShouldHurry
	//BehaviorAction.ShouldRetreat
	function QueryResultType (CustomBehaviorAction action, INextBot bot, int entity);

	//BehaviorAction.ShouldAttack
	function QueryResultType (CustomBehaviorAction action, INextBot bot, int entity, CKnownEntity them);

	//BehaviorAction.IsHindrance
	function QueryResultType (CustomBehaviorAction action, INextBot bot, int entity, int blocker);

	//BehaviorAction.SelectTargetPoint
	function void (CustomBehaviorAction action, INextBot bot, int entity, int subject, float pos[3]);

	//BehaviorAction.IsPositionAllowed
	function QueryResultType (CustomBehaviorAction action, INextBot bot, int entity, const float pos[3]);

	//BehaviorAction.SelectMoreDangerousThreat
	function CKnownEntity (CustomBehaviorAction action, INextBot bot, int entity, int subject, CKnownEntity threat1, CKnownEntity threat2);
};

enum Disposition_t 
{
	D_ER,		// Undefined - error
	D_HT,		// Hate
	D_FR,		// Fear
	D_LI,		// Like
	D_NU		// Neutral
};

native Disposition_t CombatCharacterDisposition(int entity, int target);

enum NavAttributeType
{
	NAV_MESH_INVALID		= 0,
	NAV_MESH_CROUCH			= 0x00000001,				// must crouch to use this node/area
	NAV_MESH_JUMP			= 0x00000002,				// must jump to traverse this area (only used during generation)
	NAV_MESH_PRECISE		= 0x00000004,				// do not adjust for obstacles, just move along area
	NAV_MESH_NO_JUMP		= 0x00000008,				// inhibit discontinuity jumping
	NAV_MESH_STOP			= 0x00000010,				// must stop when entering this area
	NAV_MESH_RUN			= 0x00000020,				// must run to traverse this area
	NAV_MESH_WALK			= 0x00000040,				// must walk to traverse this area
	NAV_MESH_AVOID			= 0x00000080,				// avoid this area unless alternatives are too dangerous
	NAV_MESH_TRANSIENT		= 0x00000100,				// area may become blocked, and should be periodically checked
	NAV_MESH_DONT_HIDE		= 0x00000200,				// area should not be considered for hiding spot generation
	NAV_MESH_STAND			= 0x00000400,				// bots hiding in this area should stand
	NAV_MESH_NO_HOSTAGES	= 0x00000800,				// hostages shouldn't use this area
	NAV_MESH_STAIRS			= 0x00001000,				// this area represents stairs, do not attempt to climb or jump them - just walk up
	NAV_MESH_NO_MERGE		= 0x00002000,				// don't merge this area with adjacent areas
	NAV_MESH_OBSTACLE_TOP	= 0x00004000,				// this nav area is the climb point on the tip of an obstacle
	NAV_MESH_CLIFF			= 0x00008000,				// this nav area is adjacent to a drop of at least CliffHeight
	NAV_MESH_FIRST_CUSTOM	= 0x00010000,				// apps may define custom app-specific bits starting with this value
	NAV_MESH_LAST_CUSTOM	= 0x04000000,				// apps must not define custom app-specific bits higher than with this value
#if defined GAME_TF2
	NAV_MESH_FUNC_COST		= 0x20000000,				// area has designer specified cost controlled by func_nav_cost entities
#endif
	NAV_MESH_HAS_ELEVATOR	= 0x40000000,				// area is in an elevator's path
	NAV_MESH_NAV_BLOCKER	= 0x80000000				// area is blocked by nav blocker ( Alas, needed to hijack a bit in the attributes to get within a cache line [7/24/2008 tom])
};

methodmap CNavArea
{
	property float CostSoFar
	{
		public native get();
	}

	property int ID
	{
		public native get();
	}

	property bool Underwater
	{
		public native get();
	}
	
	property int Place
	{
		public native get();
	}
	
	property float AvoidanceObstacleHeight
	{
		public native get();
	}
	
	property float SizeX
	{
		public native get();
	}
	
	property float SizeY
	{
		public native get();
	}

	property bool Damaging
	{
		public native get();
	}

	public native bool HasAttributes(NavAttributeType bits);
#if defined GAME_TF2
	public native float ComputeFuncNavCost(int entity);
#endif
	public native void GetCenter(float pos[3]);
	public native void GetRandomPoint(float pos[3]);
	public native float ComputeAdjacentConnectionHeightChange(CNavArea area);
	public native int GetPlayerCount(int team = 0);
	public native float GetDanger(int team);
	public native bool HasAvoidanceObstacle(float maxObstructionHeight = STEP_HEIGHT);

	public native float GetZ(float x, float y);

	public native void GetClosestPointOnArea(const float pos[3], float close[3]);

	public native bool IsBlocked(int teamID, bool ignoreNavBlockers = false);

	public native bool IsEntirelyVisible(const float eye[3], int ignore = -1);
	public native bool IsPartiallyVisible(const float eye[3], int ignore = -1);

	public native bool IsPotentiallyVisible(CNavArea area);
	public native bool IsPotentiallyVisibleToTeam(int team);

	public native bool IsCompletelyVisible(CNavArea area);
	public native bool IsCompletelyVisibleToTeam(int team);
};

#define CNavArea_Null view_as<CNavArea>(Address_Null)

#if defined GAME_TF2
enum TFNavAttributeType
{
	TF_NAV_INVALID						= 0x00000000,
	TF_NAV_BLOCKED						= 0x00000001,			// blocked for some TF-specific reason
	TF_NAV_SPAWN_ROOM_RED				= 0x00000002,
	TF_NAV_SPAWN_ROOM_BLUE				= 0x00000004,
	TF_NAV_SPAWN_ROOM_EXIT				= 0x00000008,
	TF_NAV_HAS_AMMO						= 0x00000010,
	TF_NAV_HAS_HEALTH					= 0x00000020,
	TF_NAV_CONTROL_POINT				= 0x00000040,
	TF_NAV_BLUE_SENTRY_DANGER			= 0x00000080,			// sentry can potentially fire upon enemies in this area
	TF_NAV_RED_SENTRY_DANGER			= 0x00000100,
	TF_NAV_BLUE_SETUP_GATE				= 0x00000800,			// this area is blocked until the setup period is over
	TF_NAV_RED_SETUP_GATE				= 0x00001000,			// this area is blocked until the setup period is over
	TF_NAV_BLOCKED_AFTER_POINT_CAPTURE	= 0x00002000,			// this area becomes blocked after the first point is capped
	TF_NAV_BLOCKED_UNTIL_POINT_CAPTURE  = 0x00004000,			// this area is blocked until the first point is capped, then is unblocked
	TF_NAV_BLUE_ONE_WAY_DOOR			= 0x00008000,
	TF_NAV_RED_ONE_WAY_DOOR				= 0x00010000,
 	TF_NAV_WITH_SECOND_POINT			= 0x00020000,			// modifier for BLOCKED_*_POINT_CAPTURE
 	TF_NAV_WITH_THIRD_POINT				= 0x00040000,			// modifier for BLOCKED_*_POINT_CAPTURE
  	TF_NAV_WITH_FOURTH_POINT			= 0x00080000,			// modifier for BLOCKED_*_POINT_CAPTURE
 	TF_NAV_WITH_FIFTH_POINT				= 0x00100000,			// modifier for BLOCKED_*_POINT_CAPTURE
	TF_NAV_SNIPER_SPOT					= 0x00200000,			// this is a good place for a sniper to lurk
	TF_NAV_SENTRY_SPOT					= 0x00400000,			// this is a good place to build a sentry
	TF_NAV_ESCAPE_ROUTE					= 0x00800000,			// for Raid mode
	TF_NAV_ESCAPE_ROUTE_VISIBLE			= 0x01000000,			// all areas that have visibility to the escape route
	TF_NAV_NO_SPAWNING					= 0x02000000,			// don't spawn bots in this area
 	TF_NAV_RESCUE_CLOSET				= 0x04000000,			// for respawning friends in Raid mode
 	TF_NAV_BOMB_CAN_DROP_HERE			= 0x08000000,			// the bomb can be dropped here and reached by the invaders in MvM
	TF_NAV_DOOR_NEVER_BLOCKS			= 0x10000000,
	TF_NAV_DOOR_ALWAYS_BLOCKS			= 0x20000000,
	TF_NAV_UNBLOCKABLE					= 0x40000000,			// this area cannot be blocked
};

#define TF_NAV_PERSISTENT_ATTRIBUTES (TF_NAV_SNIPER_SPOT|TF_NAV_SENTRY_SPOT|TF_NAV_NO_SPAWNING| \
											TF_NAV_BLUE_SETUP_GATE|TF_NAV_RED_SETUP_GATE|TF_NAV_BLOCKED_AFTER_POINT_CAPTURE| \
											TF_NAV_BLOCKED_UNTIL_POINT_CAPTURE|TF_NAV_BLUE_ONE_WAY_DOOR \
											|TF_NAV_RED_ONE_WAY_DOOR|TF_NAV_DOOR_NEVER_BLOCKS|TF_NAV_DOOR_ALWAYS_BLOCKS \
											|TF_NAV_UNBLOCKABLE|TF_NAV_WITH_SECOND_POINT|TF_NAV_WITH_THIRD_POINT \
											|TF_NAV_WITH_FOURTH_POINT|TF_NAV_WITH_FIFTH_POINT|TF_NAV_RESCUE_CLOSET)

methodmap CTFNavArea < CNavArea
{
	property bool InCombat
	{
		public native get();
	}

	property float CombatIntensity
	{
		public native get();
	}

	property bool ValidForWanderingPopulation
	{
		public native get();
	}

	property float TravelDistanceToBombTarget
	{
		public native get();
	}

	property int WanderCount
	{
		public native get();
		public native set(int count);
	}

	public native void AddToWanderCount(int count);

	public native void OnCombat();

	public native void GetEnemyInvasionAreaVector(int myTeam, ArrayList priorVector);
	public native float GetIncursionDistance(int team);
	public native CTFNavArea GetNextIncursionArea(int team);
	public native bool IsAwayFromInvasionAreas(int myTeam, float safetyRange = 1000.0);

	public native bool IsReachableByTeam(int team);

	public native bool IsPotentiallyVisibleToActor(int who);

	public native void SetAttributeTF(TFNavAttributeType flags);
	public native void ClearAttributeTF(TFNavAttributeType flags);
	public native bool HasAttributeTF(TFNavAttributeType flags);
};
#elseif defined GAME_L4D2
enum
{
	NAV_PLAYERCLIP = NAV_MESH_FIRST_CUSTOM,
	NAV_BREAKABLEWALL,
};

enum
{
	EMPTY = (1 << 1),
	STOP = (1 << 2),
	FINALE = (1 << 6),
	PLAYER_START = (1 << 7),
	BATTLEFIELD = (1 << 8),
	IGNORE_VISIBILITY = (1 << 9),
	NOT_CLEARABLE = (1 << 10),
	CHECKPOINT = (1 << 11),
	OBSCURED = (1 << 12),
	NO_MOBS = (1 << 13),
	THREAT = (1 << 14),
	RESCUE_VEHICLE = (1 << 15),
	RESCUE_CLOSET = (1 << 16),
};

methodmap TerrorNavArea < CNavArea
{
	public native int GetSpawnAttributes();
};
#endif

#define GETNAVAREA_CHECK_LOS 0x1
#define GETNAVAREA_ALLOW_BLOCKED_AREAS 0x2
#define GETNAVAREA_CHECK_GROUND 0x4

#define UNDEFINED_PLACE 0
#define ANY_PLACE 0xFFFF

methodmap CNavMesh
{
#if defined GAME_TF2
	public static native CNavArea GetNearestNavAreaVector(const float pos[3], bool anyZ = false, float maxDist = 10000.0, bool checkLOS = false, bool checkGround = true, int team = TEAM_ANY);
#elseif defined GAME_L4D2
	public static native CNavArea GetNearestNavAreaVector(const float pos[3], bool anyZ = false, float maxDist = 10000.0, bool checkLOS = false, bool checkGround = true, bool unknown = false);
#endif
	public static native CNavArea GetNearestNavAreaEntity(int entity, int nGetNavAreaFlags = GETNAVAREA_CHECK_GROUND, float maxDist = 10000.0);

	public static native CNavArea GetNavAreaEntity(int entity, int nGetNavAreaFlags, float flBeneathLimit = 120.0);
	public static native CNavArea GetNavAreaVector(const float vec[3], float beneathLimt = 120.0);
	
	public static native CNavArea GetNavAreaByID(int id);
	
	public static native int GetNavAreaCount();
	
	public static native bool GetGroundHeight(const float pos[3], float &height, float normal[3] = NULL_VECTOR);
	public static native bool GetSimpleGroundHeight(const float pos[3], float &height, float normal[3] = NULL_VECTOR);
	
	public static native int GetPlace(float vec[3]);
	public static native int PlaceToName(int id, char[] name, int len);
	public static native int NameToPlace(const char[] name);

	public static native Address GetMemory();

	public static native bool IsLoaded();
};

#if defined GAME_TF2
methodmap CTFNavMesh < CNavMesh
{
	public static native void CollectAmbushAreas(ArrayList ambushVector, CTFNavArea startArea, int teamToAmbush, float searchRadius, float incursionTolerance = 300.0);

	public static native void CollectSpawnRoomThresholdAreas(ArrayList spawnExitAreaVector, int team);

	public static native void CollectAreaWithinBombTravelRange(ArrayList spawnExitAreaVector, float minTravel, float maxTravel);

	public static native void GetSetupGateDefenseAreas(ArrayList spawnExitAreaVector);
	public static native void GetControlPointAreas(int pointIndex, ArrayList spawnExitAreaVector);
	public static native CTFNavArea GetControlPointCenterArea(int pointIndex);
	public static native void GetSpawnRoomAreas(int team, ArrayList spawnExitAreaVector);
	public static native void GetSpawnRoomExitAreas(int team, ArrayList spawnExitAreaVector);

	//public static native bool IsSentryGunHere(CTFNavArea area);
};
#endif

native int GetNavAreaVectorCount();
native CNavArea GetNavAreaFromVector(int idx);

native void CollectSurroundingAreas(ArrayList nearbyAreaVector, CNavArea startArea, float travelDistanceLimit = 1500.0, float maxStepUpLimit = STEP_HEIGHT, float maxDropDownLimit = 100.0);

typedef searchareasexecute_func_t = function bool (CNavArea area, CNavArea priorArea, float travelDistanceSoFar, any data);
typedef searchareasshould_func_t = function bool (CNavArea area, CNavArea priorArea, float travelDistanceSoFar, any data);
typedef searchareasiter_func_t = function void (CNavArea area, CNavArea priorArea, float travelDistanceSoFar, any data);
typedef searchareaspost_func_t = function void (any data);

enum struct ISearchSurroundingAreasFunctor
{
	searchareasexecute_func_t Execute;
	searchareasshould_func_t ShouldSearch;
	searchareasiter_func_t IterateAdjacentAreas;
	searchareaspost_func_t PostSearch;

	void Init()
	{
		this.Execute = INVALID_FUNCTION;
		this.ShouldSearch = INVALID_FUNCTION;
		this.IterateAdjacentAreas = INVALID_FUNCTION;
		this.PostSearch = INVALID_FUNCTION;
	}
}

#define ISearchSurroundingAreasFunctorVar any[]

native void SearchSurroundingAreas(CNavArea startArea, ISearchSurroundingAreasFunctorVar func, float travelDistanceLimit = -1.0, any data = 0);

methodmap CNavLadder
{
	property float Length
	{
		public native get();
	}
};

#define CNavLadder_Null view_as<CNavLadder>(Address_Null)

methodmap CFuncElevator
{

};

methodmap INextBotEventResponder
{

};

methodmap INextBotComponent < INextBotEventResponder
{
	property INextBot Bot
	{
		public native get();
	}

	property float UpdateInterval
	{
		public native get();
	}

	property float LastUpdateTime
	{
		public native get();
	}
	
	public native void Reset();
};

native void CollectAllBots(ArrayList botVector);

forward void OnNextbotSpawned(INextBot bot, int entity);

enum TraverseWhenType
{
	IMMEDIATELY,
	EVENTUALLY
};

enum LocomotionType
{
	Locomotion_None =    0,
	Locomotion_Any =     (1 << 0),
	Locomotion_Ground =  (view_as<int>(Locomotion_Any)|(1 << 1)),
	Locomotion_Flying =  (view_as<int>(Locomotion_Any)|(1 << 2)),
	Locomotion_Custom =  (1 << 3),
	Locomotion_AnyCustom = (Locomotion_Any|Locomotion_Custom),
	Locomotion_FlyingCustom = (Locomotion_Flying|Locomotion_Custom),
	Locomotion_GroundCustom = (Locomotion_Ground|Locomotion_Custom),
};

methodmap ILocomotion < INextBotComponent
{
	property LocomotionType Type
	{
		public native get();
	}

	property float StepHeight
	{
		public native get();
	}

	property float MaxJumpHeight
	{
		public native get();
	}

	property float DeathDropHeight
	{
		public native get();
	}

	property float RunSpeed
	{
		public native get();
	}

	property float WalkSpeed
	{
		public native get();
	}

#if defined GAME_TF2
	property float MaxAcceleration
	{
		public native get();
	}

	property float MaxDeceleration
	{
		public native get();
	}
#endif

	property float SpeedLimit
	{
		public native get();
	}

	property float TraversableSlopeLimit
	{
		public native get();
	}

	property float DesiredSpeed
	{
		public native get();
		public native set(float height);
	}

	property float GroundSpeed
	{
		public native get();
	}

	property bool ClimbingOrJumping
	{
		public native get();
	}

	property bool ClimbingUpToLedge
	{
		public native get();
	}

	property bool JumpingAcrossGap
	{
		public native get();
	}

	property bool Scrambling
	{
		public native get();
	}

	property bool Running
	{
		public native get();
	}

	property bool Stuck
	{
		public native get();
	}

	property bool OnGround
	{
		public native get();
	}
	
	property bool AttemptingToMove
	{
		public native get();
	}
	
	property bool UsingLadder
	{
		public native get();
	}
	
	property bool AscendingOrDescendingLadder
	{
		public native get();
	}

	property float StuckDuration
	{
		public native get();
	}

	property float Speed
	{
		public native get();
	}

	property int Ground
	{
		public native get();
	}

	public native void GetGroundMotionVector(float ang[3]);
	public native void GetMotionVector(float ang[3]);
	public native void GetVelocity(float ang[3]);
	public native void GetFeet(float ang[3]);
	public native void GetGroundNormal(float ang[3]);

	public native bool IsAreaTraversable(CNavArea area);
	public native bool IsPotentiallyTraversable(const float from[3], const float to[3], TraverseWhenType when = EVENTUALLY, float &fraction = 0.0);
	public native bool HasPotentialGap(const float from[3], const float to[3], float &fraction = 0.0);
	public native bool IsGap(const float pos[3], const float fwd[3]);
	public native bool IsEntityTraversable(int entity, TraverseWhenType when = EVENTUALLY);

	public native void ClearStuckStatus(const char[] reason = "");

	public native void SetDesiredLean(const float ang[3]);
	public native void GetDesiredLean(float ang[3]);

	public native void Run();
	public native void Walk();
	public native void Stop();
	public native void Jump();
	public native void JumpAcrossGap(const float goal[3], const float forwrd[3]);
	public native bool ClimbUpToLedge(const float goal[3], const float forwrd[3], int obstacle);

	public native void FaceTowards(const float pos[3]);
	public native void Approach(const float pos[3], float weight = 1.0);
	public native void DriveTo(const float pos[3]);
};

#define ILocomotion_Null (view_as<ILocomotion>(Address_Null))

methodmap NextBotFlyingLocomotion < ILocomotion
{
	property float StepHeight
	{
		public native set(float height);
	}

	property float MaxJumpHeight
	{
		public native set(float height);
	}

	property float DeathDropHeight
	{
		public native set(float height);
	}

	property float RunSpeed
	{
		public native set(float height);
	}

	property float WalkSpeed
	{
		public native set(float height);
	}

#if defined GAME_TF2
	property float MaxAcceleration
	{
		public native set(float height);
	}

	property float MaxDeceleration
	{
		public native set(float height);
	}
#endif

	property float SpeedLimit
	{
		public native set(float height);
	}

	property float TraversableSlopeLimit
	{
		public native set(float height);
	}

	property bool AllowFacing
	{
		public native set(bool value);
	}

	public native void set_function(const char[] name, Function func);

	public native void set_data(const char[] name, any data);
	public native any get_data(const char[] name);
	public native bool has_data(const char[] name);
	
	public native void set_data_array(const char[] name, const any[] data, int len);
	public native void get_data_array(const char[] name, any[] data, int len);

	property float DesiredAltitude
	{
		public native get();
		public native set(float altitude);
	}

	property float Acceleration
	{
		public native get();
		public native set(float accel);
	}

	property float HorizontalDamp
	{
		public native get();
		public native set(float damp);
	}

	property float VerticalDamp
	{
		public native get();
		public native set(float damp);
	}

	//public native void Deflect(int deflector);

	public native void SetVelocity(const float accel[3]);

	property float MaxYawRate
	{
		public native get();
		public native set(float height);
	}

	property float MaxPitchRate
	{
		public native get();
		public native set(float height);
	}
};

#if defined GAME_TF2
#define GameLocomotion NextBotGroundLocomotion
#elseif defined GAME_L4D2
#define GameLocomotion ZombieBotLocomotion
#endif

methodmap GameLocomotion < ILocomotion
{
#if defined GAME_TF2
	property float Gravity
	{
		public native get();
	}

	property float FrictionForward
	{
		public native get();
	}

	property float FrictionSideways
	{
		public native get();
	}
#endif

	property float MaxYawRate
	{
		public native get();
	}
	
	property CNavLadder Ladder
	{
		public native get();
		public native set(CNavLadder lad);
	}
	
	property CNavArea LadderDismountGoal
	{
		public native get();
		public native set(CNavArea lad);
	}
	
	property bool GoingUpLadder
	{
		public native get();
		public native set(bool lad);
	}
	
	property bool ClimbingUpToLedge
	{
		public native set(bool lad);
	}

#if defined GAME_L4D2
	/*
	public native void GetMoveVector(float vec[3]);
	public native void GetGroundAcceleration(float vec[3]);
	*/
#endif

#if defined GAME_TF2
	public native void GetAcceleration(float ang[3]);
#endif
	public native void SetAcceleration(const float ang[3]);
	public native void SetVelocity(const float ang[3]);
};

methodmap GameLocomotionCustom < GameLocomotion
{
	property float StepHeight
	{
		public native set(float height);
	}

	property float MaxJumpHeight
	{
		public native set(float height);
	}

	property float DeathDropHeight
	{
		public native set(float height);
	}

	property float RunSpeed
	{
		public native set(float height);
	}

	property float WalkSpeed
	{
		public native set(float height);
	}

#if defined GAME_TF2
	property float MaxAcceleration
	{
		public native set(float height);
	}

	property float MaxDeceleration
	{
		public native set(float height);
	}
#endif

	property float SpeedLimit
	{
		public native set(float height);
	}

	property float TraversableSlopeLimit
	{
		public native set(float height);
	}

	public native void set_function(const char[] name, Function func);

	public native void set_data(const char[] name, any data);
	public native any get_data(const char[] name);
	public native bool has_data(const char[] name);
	
	public native void set_data_array(const char[] name, const any[] data, int len);
	public native void get_data_array(const char[] name, any[] data, int len);

#if defined GAME_TF2
	property float Gravity
	{
		public native set(float height);
	}

	property float FrictionForward
	{
		public native set(float height);
	}

	property float FrictionSideways
	{
		public native set(float height);
	}
#endif

	property float MaxYawRate
	{
		public native set(float height);
	}
};

#if defined GAME_TF2
#define NextBotGroundLocomotionCustom_Null (view<NextBotGroundLocomotionCustom>(Address_Null))
#define GameLocomotionCustom_Null NextBotGroundLocomotionCustom_Null
#elseif defined GAME_L4D2
#define ZombieBotLocomotionCustom_Null (view<ZombieBotLocomotionCustom>(Address_Null))
#define GameLocomotionCustom_Null ZombieBotLocomotionCustom_Null
#endif

methodmap AnyLocomotion < ILocomotion
{
#if defined GAME_TF2
	property float Gravity
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type & Locomotion_Ground) {
				return view_as<GameLocomotion>(this).Gravity;
			}

			return 0.0;
		}
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).Gravity = height;
			}
		}
	}

	property float FrictionForward
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type & Locomotion_Ground) {
				return view_as<GameLocomotion>(this).FrictionForward;
			}

			return 0.0;
		}
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).FrictionForward = height;
			}
		}
	}

	property float FrictionSideways
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type & Locomotion_Ground) {
				return view_as<GameLocomotion>(this).FrictionSideways;
			}

			return 0.0;
		}
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).FrictionSideways = height;
			}
		}
	}
#endif

	property float MaxYawRate
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				return view_as<NextBotFlyingLocomotion>(this).MaxYawRate;
			} else if(type & Locomotion_Ground) {
				return view_as<GameLocomotion>(this).MaxYawRate;
			}

			return 0.0;
		}
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).MaxYawRate = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).MaxYawRate = height;
			}
		}
	}

	property float MaxPitchRate
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				return view_as<NextBotFlyingLocomotion>(this).MaxPitchRate;
			}

			return 0.0;
		}
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).MaxPitchRate = height;
			}
		}
	}

	property float StepHeight
	{
		public get()
		{
			return view_as<ILocomotion>(this).StepHeight;
		}
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).StepHeight = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).StepHeight = height;
			}
		}
	}

	property float MaxJumpHeight
	{
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).MaxJumpHeight = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).MaxJumpHeight = height;
			}
		}
	}

	property float DeathDropHeight
	{
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).DeathDropHeight = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).DeathDropHeight = height;
			}
		}
	}

	property float RunSpeed
	{
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).RunSpeed = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).RunSpeed = height;
			}
		}
	}

	property float WalkSpeed
	{
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).WalkSpeed = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).WalkSpeed = height;
			}
		}
	}

#if defined GAME_TF2
	property float MaxAcceleration
	{
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).MaxAcceleration = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).MaxAcceleration = height;
			}
		}
	}

	property float MaxDeceleration
	{
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).MaxDeceleration = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).MaxDeceleration = height;
			}
		}
	}
#endif

	property float SpeedLimit
	{
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).SpeedLimit = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).SpeedLimit = height;
			}
		}
	}

	property float TraversableSlopeLimit
	{
		public set(float height)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_GroundCustom) {
				view_as<GameLocomotionCustom>(this).TraversableSlopeLimit = height;
			} else if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).TraversableSlopeLimit = height;
			}
		}
	}

	public void set_function(const char[] name, Function func)
	{
		LocomotionType type = this.Type;

		if(type == Locomotion_GroundCustom) {
			view_as<GameLocomotionCustom>(this).set_function(name, func);
		} else if(type == Locomotion_FlyingCustom) {
			view_as<NextBotFlyingLocomotion>(this).set_function(name, func);
		}
	}

	public void set_data(const char[] name, any data)
	{
		LocomotionType type = this.Type;

		if(type == Locomotion_GroundCustom) {
			view_as<GameLocomotionCustom>(this).set_data(name, data);
		} else if(type == Locomotion_FlyingCustom) {
			view_as<NextBotFlyingLocomotion>(this).set_data(name, data);
		}
	}
	public any get_data(const char[] name)
	{
		LocomotionType type = this.Type;

		if(type == Locomotion_GroundCustom) {
			return view_as<GameLocomotionCustom>(this).get_data(name);
		} else if(type == Locomotion_FlyingCustom) {
			return view_as<NextBotFlyingLocomotion>(this).get_data(name);
		}

		return 0.0;
	}
	public bool has_data(const char[] name)
	{
		LocomotionType type = this.Type;

		if(type == Locomotion_GroundCustom) {
			return view_as<GameLocomotionCustom>(this).has_data(name);
		} else if(type == Locomotion_FlyingCustom) {
			return view_as<NextBotFlyingLocomotion>(this).has_data(name);
		}

		return false;
	}
	
	public void set_data_array(const char[] name, const any[] data, int len)
	{
		LocomotionType type = this.Type;

		if(type == Locomotion_GroundCustom) {
			view_as<GameLocomotionCustom>(this).set_data_array(name, data, len);
		} else if(type == Locomotion_FlyingCustom) {
			view_as<NextBotFlyingLocomotion>(this).set_data_array(name, data, len);
		}
	}
	public void get_data_array(const char[] name, any[] data, int len)
	{
		LocomotionType type = this.Type;

		if(type == Locomotion_GroundCustom) {
			view_as<GameLocomotionCustom>(this).get_data_array(name, data, len);
		} else if(type == Locomotion_FlyingCustom) {
			view_as<NextBotFlyingLocomotion>(this).get_data_array(name, data, len);
		}
	}

	property float DesiredAltitude
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				return view_as<NextBotFlyingLocomotion>(this).DesiredAltitude;
			}

			return 0.0;
		}
		public set(float altitude)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).DesiredAltitude = altitude;
			}
		}
	}

	property bool AllowFacing
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				return view_as<NextBotFlyingLocomotion>(this).AllowFacing;
			}

			return false;
		}
		public set(bool allow)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).AllowFacing = allow;
			}
		}
	}

	property float Acceleration
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				return view_as<NextBotFlyingLocomotion>(this).Acceleration;
			}

			return 0.0;
		}
		public set(float accel)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).Acceleration = accel;
			}
		}
	}

	property float HorizontalDamp
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				return view_as<NextBotFlyingLocomotion>(this).HorizontalDamp;
			}

			return 0.0;
		}
		public set(float damp)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).HorizontalDamp = damp;
			}
		}
	}

	property float VerticalDamp
	{
		public get()
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				return view_as<NextBotFlyingLocomotion>(this).VerticalDamp;
			}

			return 0.0;
		}
		public set(float damp)
		{
			LocomotionType type = this.Type;

			if(type == Locomotion_FlyingCustom) {
				view_as<NextBotFlyingLocomotion>(this).VerticalDamp = damp;
			}
		}
	}

	public void SetVelocity(const float vel[3])
	{
		LocomotionType type = this.Type;

		if(type == Locomotion_FlyingCustom) {
			view_as<NextBotFlyingLocomotion>(this).SetVelocity(vel);
		} else if(type & Locomotion_Ground) {
			view_as<GameLocomotion>(this).SetVelocity(vel);
		}
	}
};

#define AnyLocomotion_Null (view_as<AnyLocomotion>(Address_Null))

#if defined GAME_TF2
methodmap CKnownEntity
{
	property int Entity
	{
		public native get();
	}

	property bool LastKnownPositionBeenSeen
	{
		public native get();
	}

	property CNavArea LastKnownArea
	{
		public native get();
	}

	property float TimeSinceLastKnown
	{
		public native get();
	}

	property float TimeSinceBecameKnown
	{
		public native get();
	}

	property float TimeSinceBecameVisible
	{
		public native get();
	}

	property float TimeWhenBecameVisible
	{
		public native get();
	}

	property float TimeSinceLastSeen
	{
		public native get();
	}

	property bool WasEverVisible
	{
		public native get();
	}

	property bool VisibilityStatus
	{
		public native set(bool height);
	}

	property bool VisibleInFOVNow
	{
		public native get();
	}

	property bool VisibleRecently
	{
		public native get();
	}

	property bool Obsolete
	{
		public native get();
	}

	public native void MarkLastKnownPositionAsSeen();
	public native bool Is(int entity);
	public native bool IsEqual(CKnownEntity entity);
	public native void Destroy();
	public native void UpdatePosition();
	public native void GetLastKnownPosition(float pos[3]);
};

#define CKnownEntity_Null view_as<CKnownEntity>(Address_Null)
#endif

enum FieldOfViewCheckType
{
	USE_FOV,
	DISREGARD_FOV
};

native bool EntityVisibleEnt(int entity, int target, int mask = MASK_BLOCKLOS, int &blocker = -1);
native bool EntityVisibleVec(int entity, const float pos[3], int mask = MASK_BLOCKLOS, int &blocker = -1);

native bool CombatCharacterInViewConeVec(int entity, const float pos[3]);
native bool CombatCharacterInViewConeEnt(int entity, int target);

native bool CombatCharacterInAimConeVec(int entity, const float pos[3]);
native bool CombatCharacterInAimConeEnt(int entity, int target);

native bool CombatCharacterIsHiddenByFogVec(int entity, const float pos[3]);
native bool CombatCharacterIsHiddenByFogEnt(int entity, int target);
native bool CombatCharacterIsHiddenByFogR(int entity, float range);

native float CombatCharacterGetFogObscuredRatioVec(int entity, const float pos[3]);
native float CombatCharacterGetFogObscuredRatioEnt(int entity, int target);
native float CombatCharacterGetFogObscuredRatioR(int entity, float range);

#define BCC_DEFAULT_LOOK_TOWARDS_TOLERANCE 0.9

native bool CombatCharacterIsLookingTowardsVec(int entity, const float pos[3], float cosTolerance = BCC_DEFAULT_LOOK_TOWARDS_TOLERANCE);
native bool CombatCharacterIsLookingTowardsEnt(int entity, int target, float cosTolerance = BCC_DEFAULT_LOOK_TOWARDS_TOLERANCE);

native bool CombatCharacterIsInFieldOfViewVec(int entity, const float pos[3]);
native bool CombatCharacterIsInFieldOfViewEnt(int entity, int target);

native bool CombatCharacterIsAbleToSeeEnt(int entity, int target, FieldOfViewCheckType checkFOV);

enum LineOfSightCheckType
{
	IGNORE_NOTHING,
	IGNORE_ACTORS
};

native bool CombatCharacterIsLineOfSightClearVec(int entity, const float pos[3], LineOfSightCheckType checkType = IGNORE_NOTHING, int entityToIgnore = -1);
native bool CombatCharacterIsLineOfSightClearEnt(int entity, int target, LineOfSightCheckType checkType = IGNORE_NOTHING);

typedef foreachknow_func_t = function bool (CKnownEntity known, any data);
typedef closestknowfilter_func_t = function bool (int entity, any data);

methodmap IVision < INextBotComponent
{
#if defined GAME_TF2
	public native bool ForEachKnownEntity(foreachknow_func_t func, any data = 0);
	//public native void CollectKnownEntities(ArrayList list);
	public native CKnownEntity GetPrimaryKnownThreat(bool onlyVisibleThreats = false);
	public native CKnownEntity GetClosestKnownTeam(int team);
	public native CKnownEntity GetClosestKnownFilter(closestknowfilter_func_t func, any data = 0);
	public native int GetKnownCount(int team, bool onlyVisible = false, float rangeLimit = -1.0);
	public native CKnownEntity GetKnown(int entity);
	public native void AddKnownEntity(int entity);
	public native void ForgetEntity(int entity);
	public native void ForgetAllKnownEntities();
	//public native void CollectPotentiallyVisibleEntities(ArrayList list);
#elseif defined GAME_L4D2
	public native int GetPrimaryRecognizedThreat();
	public native int GetClosestRecognizedTeam(int team);
	public native int GetClosestRecognizedFilter(closestknowfilter_func_t func, any data = 0);
#endif
	public native float GetTimeSinceVisible(int team);
	public native bool IsAbleToSeeEntity(int entity, FieldOfViewCheckType checkFOV, float visibleSpot[3] = NULL_VECTOR);
	public native bool IsAbleToSeeVector(const float pos[3], FieldOfViewCheckType checkFOV);
	public native bool IsIgnored(int entity);
#if defined GAME_L4D2
	public native bool IsNoticed(int entity);
#elseif defined GAME_TF2
	public native bool IsVisibleEntityNoticed(int entity);
#endif
	public native bool IsInFieldOfViewVector(const float pos[3]);
	public native bool IsInFieldOfViewEntity(int entity);
	public native bool IsLineOfSightClear(const float pos[3]);
	public native bool IsLineOfSightClearToEntity(int entity, float visibleSpot[3] = NULL_VECTOR);
	public native bool IsLookingAtVector(const float pos[3], float cosTolerance = 0.95);
	public native bool IsLookingAtEntity(int entity, float cosTolerance = 0.95);

	property float DefaultFieldOfView
	{
		public native get();
	}

	property float FieldOfView
	{
		public native get();
		public native set(float height);
	}

	property float MaxVisionRange
	{
		public native get();
	}

	property float MinRecognizeTime
	{
		public native get();
	}
};

#if defined GAME_TF2
#define GameVision IVision
#elseif defined GAME_L4D2
#define GameVision ZombieBotVision
#endif

#if defined GAME_L4D2
methodmap ZombieBotVision < IVision
{
	
};
#endif

methodmap GameVisionCustom < GameVision
{
	property float MaxVisionRange
	{
		public native set(float height);
	}

	property float MinRecognizeTime
	{
		public native set(float height);
	}

	property float DefaultFieldOfView
	{
		public native set(float height);
	}
	
	public native void set_function(const char[] name, Function func);

	public native void set_data(const char[] name, any data);
	public native any get_data(const char[] name);
	public native bool has_data(const char[] name);
	
	public native void set_data_array(const char[] name, const any[] data, int len);
	public native void get_data_array(const char[] name, any[] data, int len);
};

enum ActivityType 
{
	NO_ACTIVITY_FLAGS = 0,
	MOTION_CONTROLLED_XY	= 0x0001,	// XY position and orientation of the bot is driven by the animation.
	MOTION_CONTROLLED_Z		= 0x0002,	// Z position of the bot is driven by the animation.
	ACTIVITY_UNINTERRUPTIBLE= 0x0004,	// activity can't be changed until animation finishes
	ACTIVITY_TRANSITORY		= 0x0008,	// a short animation that takes over from the underlying animation momentarily, resuming it upon completion
	ENTINDEX_PLAYBACK_RATE	= 0x0010,	// played back at different rates based on entindex
};

enum PostureType
{
	STAND,
	CROUCH,
	SIT,
	CRAWL,
	LIE
};

enum ArousalType
{
	NEUTRAL,
	ALERT,
	INTENSE
};

methodmap IBody < INextBotComponent
{
	property float HullWidth
	{
		public native get();
	}

	property float HullHeight
	{
		public native get();
	}

	property float StandHullHeight
	{
		public native get();
	}

	property float CrouchHullHeight
	{
		public native get();
	}

	property int SolidMask
	{
		public native get();
	}

#if defined GAME_TF2
	property float CollisionGroup
	{
		public native get();
	}
#endif

	public native void GetEyePosition(float vec[3]);
	public native void GetViewVector(float vec[3]);

	public native void GetHullMins(float vec[3]);
	public native void GetHullMaxs(float vec[3]);

	public native void SetPosition(const float pos[3]);

	public native bool StartActivity(Activity act, ActivityType flags = NO_ACTIVITY_FLAGS);
	public native int SelectAnimationSequence(Activity act);

	property Activity Activity
	{
		public native get();
	}

	public native bool IsActivity(Activity act);
	public native bool HasActivityType(ActivityType flags);

	property PostureType DesiredPosture
	{
		public native get();
		public native set(PostureType posture);
	}

	property bool InDesiredPosture
	{
		public native get();
	}

	public native bool IsDesiredPosture(PostureType posture);

	property PostureType ActualPosture
	{
		public native get();
	}

	public native bool IsActualPosture(PostureType posture);

	property bool PostureMobile
	{
		public native get();
	}

	property bool PostureChanging
	{
		public native get();
	}

	property ArousalType Arousal
	{
		public native get();
		public native set(ArousalType posture);
	}

	public native bool IsArousal(ArousalType arousal);
};

methodmap IBodyCustom < IBody
{
	property float HullWidth
	{
		public native set(float height);
	}

	property float LieHullHeight
	{
		public native set(float height);
	}

	property float StandHullHeight
	{
		public native set(float height);
	}

	property float CrouchHullHeight
	{
		public native set(float height);
	}

	property int SolidMask
	{
		public native set(int height);
	}

#if defined GAME_TF2
	property int CollisionGroup
	{
		public native set(int height);
	}
#endif

	property int Sequence
	{
		public native get();
	}

	public native void set_function(const char[] name, Function func);

	public native void set_data(const char[] name, any data);
	public native any get_data(const char[] name);
	public native bool has_data(const char[] name);
	
	public native void set_data_array(const char[] name, const any[] data, int len);
	public native void get_data_array(const char[] name, any[] data, int len);
};

enum BehaviorResultType
{ 
	BEHAVIOR_CONTINUE,			// continue executing this action next frame - nothing has changed
	BEHAVIOR_CHANGE_TO,			// change actions next frame
	BEHAVIOR_SUSPEND_FOR,		// put the current action on hold for the new action
	BEHAVIOR_DONE,				// this action has finished, resume suspended action
	BEHAVIOR_SUSTAIN,			// for use with event handlers - a way to say "It's important to keep doing what I'm doing"
};

#define BehaviorAction_Null (view_as<BehaviorAction>(Address_Null))

#define IS_ANY_HINDRANCE_POSSIBLE 0xFFFFFFFF

enum QueryResultType
{
	ANSWER_NO,
	ANSWER_YES,
	ANSWER_UNDEFINED
};

methodmap IIntention < INextBotComponent
{
	//public native QueryResultType ShouldPickUp(INextBot me, int item);
	public native QueryResultType ShouldHurry(INextBot me);
#if defined GAME_TF2
	public native QueryResultType ShouldRetreat(INextBot me);
	public native QueryResultType ShouldAttack(INextBot me, CKnownEntity them);
#endif
	public native QueryResultType IsHindrance(INextBot me, int blocker);
	public native void SelectTargetPoint(INextBot me, int subject, float pos[3]);
	public native QueryResultType IsPositionAllowed(INextBot me, const float pos[3]);
#if defined GAME_L4D2
	//public native PathFollower QueryCurrentPath(INextBot me);
#endif
#if defined GAME_TF2
	public native CKnownEntity SelectMoreDangerousThreat(INextBot me, int subject, CKnownEntity threat1, CKnownEntity threat2);
#endif
#if defined GAME_L4D2
	//public native int SelectMoreDangerousThreat(INextBot me, int subject, int threat1, int threat2);
#endif
};

#define IIntention_Null (view_as<IIntention>(Address_Null))

methodmap IIntentionCustom < IIntention
{
	public native void ResetBehavior(BehaviorAction action);
	
	public native void set_name(const char[] name);
	
	public native void set_function(const char[] name, Function func);
	
	public native void set_data(const char[] name, any data);
	public native any get_data(const char[] name);
	public native bool has_data(const char[] name);
	
	public native void set_data_array(const char[] name, const any[] data, int len);
	public native void get_data_array(const char[] name, any[] data, int len);
};

#define IIntentionCustom_Null (view_as<IIntentionCustom>(IIntention_Null))

native void UpdateEntityLastKnownArea(int entity);

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

typedef allocintent_func_t = function BehaviorAction (int entity);

methodmap INextBot < INextBotEventResponder
{
	public native INextBot(int entity);

	property int Entity
	{
		public native get();
	}

	property ILocomotion LocomotionInterface
	{
		public native get();
	}

	property IVision VisionInterface
	{
		public native get();
	}

	property IBody BodyInterface
	{
		public native get();
	}

	property IIntention IntentionInterface
	{
		public native get();
	}

	property PathFollower CurrentPath
	{
		public native get();
	}

	public native bool BeginUpdate();
	public native void Update();
	public native void EndUpdate();

	public native void Reset();

	public native bool DoThink(int entity);

	public native void GetPosition(float pos[3]);
	public native void SetPosition(const float pos[3]);

	public native bool IsRangeLessThanEntity(int entity, float range);
	public native bool IsRangeLessThanVector(const float vec[3], float range);

	public native bool IsRangeGreaterThanEntity(int entity, float range);
	public native bool IsRangeGreaterThanVector(const float vec[3], float range);

	public native float GetRangeToEntity(int entity);
	public native float GetRangeToVector(const float vec[3]);

	public native float GetRangeSquaredToEntity(int entity);
	public native float GetRangeSquaredToVector(const float vec[3]);

	public native float GetDistanceBetweenEntity(int other);
	public native float GetDistanceBetweenVector(const float other_pos[3]);

	public native bool IsDistanceBetweenLessThanEntity(int other, float range);
	public native bool IsDistanceBetweenLessThanVector(const float other_pos[3], float range);

	public native bool IsDistanceBetweenGreaterThanEntity(int other, float range);
	public native bool IsDistanceBetweenGreaterThanVector(const float other_pos[3], float range);

	public native bool IsLineOfFireClearVec(const float where[3]);
	public native bool IsLineOfFireClearEnt(int who);
	public native bool IsLineOfFireClearVecEx(const float from[3], const float to[3]);
	public native bool IsLineOfFireClearEntEx(const float from[3], int who);

	public native bool IsEntityBetweenTargetAndSelf(int other, int target);

	//public native bool IsThreatAimingTowardMe(int threat, float cosTolerance = 0.8);
	//public native bool IsThreatFiringAtMe(int threat);

#if defined GAME_L4D2
	public native float Get2DRangeToEntity(int entity);
	public native float Get2DRangeToVector(float vec[3]);
#endif

	public native bool IsFriend(int entity);
	public native bool IsEnemy(int entity);
	public native bool IsSelf(int entity);

	public native GameLocomotionCustom AllocateCustomLocomotion();
	public native NextBotFlyingLocomotion AllocateFlyingLocomotion();
	public AnyLocomotion AllocateLocomotion(bool flying)
	{
		if(flying) {
			return view_as<AnyLocomotion>(this.AllocateFlyingLocomotion());
		} else {
			return view_as<AnyLocomotion>(this.AllocateCustomLocomotion());
		}
	}
	public native GameVisionCustom AllocateCustomVision();
	public native IBodyCustom AllocateCustomBody();
	public native IIntentionCustom AllocateCustomIntention(allocintent_func_t func, const char[] name = "");
	public native void StubIntention();
	
	public native bool IsDebugging(NextBotDebugType type);
	
	public native INextBotCustom MakeCustom();
};

#define INextBot_Null (view_as<INextBot>(Address_Null))

methodmap INextBotCustom
{
#if defined GAME_L4D2
	property bool AllowedToClimb
	{
		public native set(bool val);
	}
	
	property bool ReactToSurvivorVisibility
	{
		public native set(bool val);
	}
	
	property bool ReactToSurvivorNoise
	{
		public native set(bool val);
	}
	
	property bool ReactToSurvivorContact
	{
		public native set(bool val);
	}
#endif

	property INextBot Bot
	{
		public native get();
	}

	public native void set_function(const char[] name, Function func);
	
	public native void set_data(const char[] name, any data);
	public native any get_data(const char[] name);
	public native bool has_data(const char[] name);
	
	public native void set_data_array(const char[] name, const any[] data, int len);
	public native void get_data_array(const char[] name, any[] data, int len);
};

enum SegmentType
{
	ON_GROUND,
	DROP_DOWN,
	CLIMB_UP,
	JUMP_OVER_GAP,
	LADDER_UP,
	LADDER_DOWN,
	NUM_SEGMENT_TYPES
};

enum NavTraverseType
{
	GO_NORTH = 0,
	GO_EAST,
	GO_SOUTH,
	GO_WEST,
	GO_LADDER_UP,
	GO_LADDER_DOWN,
	GO_JUMP,
	GO_ELEVATOR_UP,
	GO_ELEVATOR_DOWN,
	NUM_TRAVERSE_TYPES
};

enum NavDirType
{
	NORTH = 0,
	EAST = 1,
	SOUTH = 2,
	WEST = 3,
	NUM_DIRECTIONS
};

stock NavDirType AngleToDirection(float angle)
{
	while(angle < 0.0) {
		angle += 360.0;
	}

	while(angle > 360.0) {
		angle -= 360.0;
	}

	if(angle < 45.0 || angle > 315.0) {
		return EAST;
	}

	if(angle >= 45.0 && angle < 135.0) {
		return SOUTH;
	}

	if(angle >= 135.0 && angle < 225.0) {
		return WEST;
	}

	return NORTH;
}

enum NavRelativeDirType
{
	FORWARD = 0,
	RIGHT,
	BACKWARD,
	LEFT,
	UP,
	DOWN,
	NUM_RELATIVE_DIRECTIONS
};

native NavRelativeDirType DirectionBetweenEntityVector(int ent1, const float vec[3]);

stock NavRelativeDirType DirectionBetweenEntities(int ent1, int ent2)
{
	float vec[3];
	GetEntPropVector(ent2, Prop_Data, "m_vecAbsOrigin", vec);

	return DirectionBetweenEntityVector(ent1, vec);
}

methodmap Segment
{
	property CNavArea Area
	{
		public native get();
	}

	property CNavLadder Ladder
	{
		public native get();
	}

	property SegmentType Type
	{
		public native get();
	}

	property float Length
	{
		public native get();
	}

	property float DistanceFromStart
	{
		public native get();
	}

	property float Curvature
	{
		public native get();
	}

	property float PortalHalfWidth
	{
		public native get();
	}

	property NavTraverseType How
	{
		public native get();
	}

	public native void GetPosition(float pos[3]);
	public native void GetPortalCenter(float pos[3]);
	public native void GetForward(float pos[3]);
};

#define Segment_Null view_as<Segment>(Address_Null)

enum MoveToFailureType
{
	FAIL_NO_PATH_EXISTS,
	FAIL_STUCK,
	FAIL_FELL_OFF,
};

typedef pathcompute_func_t = function float (INextBot bot, CNavArea area, CNavArea fromArea, CNavLadder ladder, Address elevator, float length, any data);

methodmap Path < Handle
{
	public native Path();

	property Address Memory
	{
		public native get();
	}

	property float Length
	{
		public native get();
	}

	property float Age
	{
		public native get();
	}

	property bool Valid
	{
		public native get();
	}

	property Segment FirstSegment
	{
		public native get();
	}

	property Segment LastSegment
	{
		public native get();
	}

	public native Segment NextSegment(Segment seg);
	public native Segment PriorSegment(Segment seg);

	public native void GetPosition(float ret[3], float distanceFromStart, Segment start = Segment_Null);
	public native void GetClosestPosition(float ret[3], const float pos[3], Segment start = Segment_Null, float alongLimit = 0.0);

	public native void GetStartPosition(float pos[3]);
	public native void GetEndPosition(float pos[3]);

	property int Subject
	{
		public native get();
#if defined GAME_L4D2
		public native set(int entity);
#endif
	}

	property Segment CurrentGoal
	{
		public native get();
	}

	public native void Invalidate();

	public native bool ComputeVector(INextBot bot, const float goal[3], pathcompute_func_t functor, any data = 0, float maxPathLength = 0.0, bool includeGoalIfPathFails = true);
	public native bool ComputeEntity(INextBot bot, int subject, pathcompute_func_t functor, any data = 0, float maxPathLength = 0.0, bool includeGoalIfPathFails = true);
};

methodmap PathFollower < Path
{
	public native PathFollower();

	property float MinLookAheadDistance
	{
		public native get();
		public native set(float height);
	}

#if defined GAME_TF2
	property float GoalTolerance
	{
		public native get();
		public native set(float height);
	}
#endif

	property int Hindrance
	{
		public native get();
	}

	property bool AllowFacing
	{
		public native set(bool value);
	}

	public native bool IsDiscontinuityAhead(INextBot bot, SegmentType type, float range = -1.0);

	public native bool IsAtGoal(INextBot bot);

	public native void Update(INextBot bot);
};

#if defined GAME_TF2
methodmap CTFPathFollower < PathFollower
{
	public native CTFPathFollower();

	property float MinLookAheadDistance
	{
		public native get();
	}
};
#endif

methodmap RetreatPath < PathFollower
{
	public native RetreatPath();

	public native void Update(INextBot bot, int entity);
};

enum SubjectChaseType
{
	LEAD_SUBJECT,
	DONT_LEAD_SUBJECT
};

methodmap ChasePath < PathFollower
{
	public native ChasePath(SubjectChaseType chaseHow);

	property float LeadRadius
	{
		public native get();
		public native set(float height);
	}

	public native bool IsRepathNeeded(INextBot bot, int entity);

	public native void PredictSubjectPosition(INextBot bot, int entity, float predictedpos[3]);

	public native void Update(INextBot bot, int entity, pathcompute_func_t functor, any data = 0, const float predictedpos[3] = NULL_VECTOR);
};

methodmap DirectChasePath < ChasePath
{
	public native DirectChasePath(SubjectChaseType chaseHow);
};

#if defined GAME_L4D2
methodmap InfectedChasePath < DirectChasePath
{
	public native InfectedChasePath(SubjectChaseType chaseHow);
};
#endif

native bool EntityIsCombatCharacter(int entity);
native CNavArea GetEntityLastKnownArea(int entity);
native INextBot MakeEntityNextBot(int entity);

native Address AllocateNextBotCombatCharacter(int size_modifier);
native int GetNextBotCombatCharacterSize();

#if defined GAME_L4D2
native Address AllocateInfectedNextBotCombatCharacter(int size_modifier, int cls);
native int GetInfectedSize();
#endif

#if defined datamaps_included
static Address datamaps_allocatenextbot(int size_modifier, any data)
{ return AllocateNextBotCombatCharacter(size_modifier); }

#if defined GAME_L4D2
static Address datamaps_allocateinfectednextbot(int size_modifier, int data)
{ return AllocateInfectedNextBotCombatCharacter(size_modifier, data); }
#endif

stock CustomDatamap __register_nb_factory_based(const char[] classname, const char[] name, const char[] clientnet, const char[] based, CustomSendtable &table = null, CustomEntityFactory &factory = null)
{
	char netname[64];
	netname[0] = 'C';
	strcopy(netname[1], sizeof(netname)-1, name);

	factory = EntityFactoryDictionary.register_based_name(classname, based);
	table = CustomSendtable.from_factory(factory);
	if(!IsNullString(clientnet) && clientnet[0] != '\0') {
		table.override_with(clientnet);
	}
	table.set_network_name(netname);
	CustomDatamap datamap = CustomDatamap.from_factory(factory);
	datamap.set_name(netname);

	return datamap;
}

stock CustomDatamap __register_nb_factory_func(const char[] classname, const char[] name, const char[] clientnet, entityalloc_func_t alloc, int size, any data, CustomSendtable &table = null, CustomEntityFactory &factory = null)
{
	char netname[64];
	netname[0] = 'C';
	strcopy(netname[1], sizeof(netname)-1, name);

	factory = EntityFactoryDictionary.register_function(classname, alloc, size, data);
	table = CustomSendtable.from_factory(factory);
	if(!IsNullString(clientnet) && clientnet[0] != '\0') {
		table.override_with(clientnet);
	}
	CustomDatamap datamap = CustomDatamap.from_factory(factory);
	datamap.set_name(netname);

	return datamap;
}

stock CustomDatamap register_nextbot_factory(const char[] classname, const char[] name, const char[] clientnet = NULL_STRING, CustomSendtable &table = null, CustomEntityFactory &factory = null)
{ return __register_nb_factory_func(classname, name, clientnet, datamaps_allocatenextbot, GetNextBotCombatCharacterSize(), 0, table, factory); }

#if defined GAME_TF2
stock CustomDatamap register_tankboss_nextbot_factory(const char[] classname, const char[] name, CustomEntityFactory &factory = null)
{
	CustomSendtable table;
	CustomDatamap datamap = register_nextbot_factory(classname, name, "CTFTankBoss", table, factory);
	table.set_name("DT_TFBaseBoss");
	table.set_network_name("CTFBaseBoss");
	table.add_prop_float("m_lastHealthPercentage", 0.0, 1.0, 11, SPROP_NOSCALE);
	return datamap;
}

stock CustomDatamap register_robot_nextbot_factory(const char[] classname, const char[] name, CustomEntityFactory &factory = null)
{
	int m_iHealth_offset = FindSendPropInfo("CTFRobotDestruction_Robot", "m_iHealth");
	int m_iMaxHealth_offset = FindSendPropInfo("CTFRobotDestruction_Robot", "m_iMaxHealth");
	CustomSendtable table;
	CustomDatamap datamap = register_nextbot_factory(classname, name, "CTFRobotDestruction_Robot", table, factory);
	table.set_name("DT_TFRobotDestruction_Robot");
	table.set_network_name("CTFRobotDestruction_Robot");
	table.add_prop_int("m_iHealth", 4, -1, SPROP_VARINT, m_iHealth_offset);
	table.add_prop_int("m_iMaxHealth", 4, -1, SPROP_VARINT, m_iMaxHealth_offset);
	table.add_prop_int("m_eType", 4, -1, SPROP_VARINT, -1);
	return datamap;
}
#endif

#if defined GAME_L4D2
#if defined _moreinfected_inc_
stock CustomDatamap register_infected_nextbot_factory(const char[] classname, cosnt char[] name, ZombieClassType cls = ZombieClass_Common)
#else
stock CustomDatamap register_infected_nextbot_factory(const char[] classname, cosnt char[] name, int cls = 0)
#endif
{ return __register_nb_factory_func(classname, name, NULL_STRING, datamaps_allocateinfectednextbot, GetInfectedSize(), cls); }

stock CustomDatamap register_infected_factory(const char[] classname, cosnt char[] name)
{ return __register_nb_factory_based(classname, name, NULL_STRING, "infected"); }
#endif
#endif

enum baseline_cost_flags
{
	cost_flags_none =      0,
	cost_flags_mod_heavy =    (1 << 0),
	cost_flags_safest =       (1 << 1),
	cost_flags_discrete =     (1 << 2),
	cost_flags_nojumping =    (1 << 3),
	cost_flags_noladders =    (1 << 4),
	cost_flags_nocrouch =     (1 << 5),
	cost_flags_noenemyspawn = (1 << 6),
	cost_flags_nowater =      (1 << 7),
	cost_flags_fastest =      (1 << 8),
	cost_flags_mod_small =    (1 << 9),
};

#define cost_flags_onlywalk (cost_flags_nojumping| \
							cost_flags_noladders| \
							cost_flags_nocrouch)

#define cost_flags_nostance (cost_flags_nojumping| \
							cost_flags_nocrouch)

#define NB_PATHCOST_MOD_PERIOD 10.0

stock float TransientlyConsistentRandomValue(int entity, float period = 10.0, int seedValue = 0)
{
	CNavArea area = GetEntityLastKnownArea(entity);
	if(area == CNavArea_Null) {
		return 0.0;
	}

	int timeMod = (RoundToFloor(GetGameTime() / period) + 1);
	return FloatAbs(Cosine(float(seedValue + (entity * area.ID * timeMod))));
}

native float baseline_path_cost_impl(INextBot bot, CNavArea area, CNavArea fromArea, CNavLadder ladder, Address elevator, float length, baseline_cost_flags flags);

stock float baseline_path_cost(INextBot bot, CNavArea area, CNavArea fromArea, CNavLadder ladder, Address elevator, float length, baseline_cost_flags flags)
{ return baseline_path_cost_impl(bot, area, fromArea, ladder, elevator, length, flags); }

#if !defined REQUIRE_EXTENSIONS
public void __ext_nextbot_SetNTVOptional()
{
	
}
#endif

public Extension __ext_nextbot = 
{
	name = "nextbot", 
	file = "nextbot.ext", 
	
#if defined AUTOLOAD_EXTENSIONS
	autoload = 1,
#else
	autoload = 0,
#endif

#if defined REQUIRE_EXTENSIONS
	required = 1,
#else
	required = 0,
#endif
};

public SharedPlugin __pl_nextbot =
{
	name = "nextbot",
	file = "nextbot.smx",
#if defined REQUIRE_PLUGIN
	required = 1,
#else
	required = 0,
#endif
};
