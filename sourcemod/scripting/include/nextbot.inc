#if defined nextbot_included
	#endinput
#endif
#define nextbot_included

#if defined GAME_TF2
	#define TEAM_ANY -2
#elseif defined GAME_L4D2
	#define TEAM_ANY -1
#endif

#define BEHAVIOR_REASON_MAX 64

enum struct BehaviorResult
{
	BehaviorAction action;
	EventResultPriorityType priority;
	char reason[BEHAVIOR_REASON_MAX];
	
	void set_reason(const char[] str)
	{
		strcopy(this.reason, BEHAVIOR_REASON_MAX, str);
	}
}

typeset nextbot_functions
{
#if defined GAME_TF2
	//IVision.ForEachKnownEntity
	function bool (CKnownEntity known, any data);
#endif

	//IVision.GetClosestKnownFilter
	//IVision.GetClosestRecognizedFilter
	function bool (int entity, any data);

	//Path.ComputeVector
	//Path.ComputeEntity
	function float (INextBot bot, CNavArea area, CNavArea fromArea, CNavLadder ladder, Address elevator, float length, any data);
	
	//BehaviorActionEntry.OnStart
	//BehaviorActionEntry.OnSuspend
	//BehaviorActionEntry.OnResume
	function BehaviorResultType (BehaviorAction action, int entity, BehaviorAction prior, BehaviorResult result);
	//BehaviorActionEntry.Update
	function BehaviorResultType (BehaviorAction action, int entity, float interval, BehaviorResult result);
	//BehaviorActionEntry.OnEnd
	function void (BehaviorAction action, int entity, BehaviorAction next);
	//BehaviorActionEntry.InitialContainedAction
	function BehaviorAction (BehaviorAction action, int entity);
	
	//INextBot.AllocateCustomIntention
	function BehaviorAction (int entity);
	
	//BehaviorActionEntry.OnLeaveGround
	//BehaviorActionEntry.OnLandOnGround
	//BehaviorActionEntry.OnContact
	//BehaviorActionEntry.OnAnimationActivityComplete
	//BehaviorActionEntry.OnAnimationActivityInterrupted
	//BehaviorActionEntry.OnAnimationEvent
	//BehaviorActionEntry.OnSight
	//BehaviorActionEntry.OnLostSight
	//BehaviorActionEntry.OnShoved
	//BehaviorActionEntry.OnBlinded
	//BehaviorActionEntry.OnTerritoryContested
	//BehaviorActionEntry.OnTerritoryCaptured
	//BehaviorActionEntry.OnTerritoryLost
	//BehaviorActionEntry.OnThreatChanged
	//BehaviorActionEntry.OnHitByVomitJar
	//BehaviorActionEntry.OnDrop
	function BehaviorResultType (BehaviorAction action, int entity, int ground, BehaviorResult result);
	
	//BehaviorActionEntry.OnMoveToSuccess
	//BehaviorActionEntry.OnStuck
	//BehaviorActionEntry.OnUnStuck
	//BehaviorActionEntry.OnIgnite
	//BehaviorActionEntry.OnWin
	//BehaviorActionEntry.OnLose
	//BehaviorActionEntry.OnEnteredSpit
	//BehaviorActionEntry.OnModelChanged
	function BehaviorResultType (BehaviorAction action, int entity, BehaviorResult result);
	
	//BehaviorActionEntry.OnMoveToFailure
	function BehaviorResultType (BehaviorAction action, int entity, MoveToFailureType type, BehaviorResult result);
	
	//BehaviorActionEntry.OnSound
	function BehaviorResultType (BehaviorAction action, int entity, int source, float pos[3], BehaviorResult result);
	
	//BehaviorActionEntry.OnWeaponFired
	//BehaviorActionEntry.OnActorEmoted
	//BehaviorActionEntry.OnPickUp
	function BehaviorResultType (BehaviorAction action, int entity, int who, int weapon, BehaviorResult result);
	
	//BehaviorActionEntry.OnInjured
	//BehaviorActionEntry.OnKilled
	function BehaviorResultType (BehaviorAction action, int entity, BehaviorResult result);
	
	//BehaviorActionEntry.OnOtherKilled
	function BehaviorResultType (BehaviorAction action, int entity, int victim, BehaviorResult result);
};

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

	public native bool HasAttributes(NavAttributeType bits);
#if defined GAME_TF2
	public native float ComputeFuncNavCost(int entity);
#endif
	public native void GetCenter(float pos[3]);
	public native void GetRandomPoint(float pos[3]);
	public native float ComputeAdjacentConnectionHeightChange(CNavArea area);
	public native int GetPlayerCount(int team = 0);
	public native float GetDanger(int team);
	public native bool HasAvoidanceObstacle(float maxObstructionHeight = 18.0)
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

#define TF_NAV_PERSISTENT_ATTRIBUTES TF_NAV_SNIPER_SPOT|TF_NAV_SENTRY_SPOT|TF_NAV_NO_SPAWNING| \
											TF_NAV_BLUE_SETUP_GATE|TF_NAV_RED_SETUP_GATE|TF_NAV_BLOCKED_AFTER_POINT_CAPTURE| \
											TF_NAV_BLOCKED_UNTIL_POINT_CAPTURE|TF_NAV_BLUE_ONE_WAY_DOOR \
											|TF_NAV_RED_ONE_WAY_DOOR|TF_NAV_DOOR_NEVER_BLOCKS|TF_NAV_DOOR_ALWAYS_BLOCKS \
											|TF_NAV_UNBLOCKABLE|TF_NAV_WITH_SECOND_POINT|TF_NAV_WITH_THIRD_POINT \
											|TF_NAV_WITH_FOURTH_POINT|TF_NAV_WITH_FIFTH_POINT|TF_NAV_RESCUE_CLOSET

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

	public native bool HasAttributeTF(TFNavAttributeType bits);
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
	public native CNavArea GetNearestNavAreaVector(float pos[3], bool anyZ = false, float maxDist = 10000.0, bool checkLOS = false, bool checkGround = true, int team = TEAM_ANY);
#elseif defined GAME_L4D2
	public native CNavArea GetNearestNavAreaVector(float pos[3], bool anyZ = false, float maxDist = 10000.0, bool checkLOS = false, bool checkGround = true, bool unknown = false);
#endif
	public native CNavArea GetNearestNavAreaEntity(int entity, int nGetNavAreaFlags = GETNAVAREA_CHECK_GROUND, float maxDist = 10000.0);

	public native CNavArea GetNavAreaEntity(int entity, int nGetNavAreaFlags, float flBeneathLimit = 120.0);
	public native CNavArea GetNavAreaVector(float vec[3], float beneathLimt = 120.0);
	
	public native CNavArea GetNavAreaByID(int id);
	
	property int NavAreaCount
	{
		public native get();
	}
	
	public native bool GetGroundHeight(float pos[3], float &height, float normal[3] = NULL_VECTOR);
	public native bool GetSimpleGroundHeight(float pos[3], float &height, float normal[3] = NULL_VECTOR);
	
	public native int GetPlace(float vec[3]);
	public native void PlaceToName(int id, char[] name, int len);
	public native int NameToPlace(const char[] name);
};

public const CNavMesh TheNavMesh;

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
	
	public native void Reset();
};

methodmap ILocomotion < INextBotComponent
{
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

	public native void GetGroundMotionVector(float ang[3]);

	public native void GetVelocity(float ang[3]);

	public native bool IsAreaTraversable(CNavArea area);

	public native void SetDesiredLean(float ang[3]);
	public native void GetDesiredLean(float ang[3]);

	public native void Run();
	public native void Walk();
	public native void Stop();
	public native void Jump();
	public native void JumpAcrossGap(float goal[3], float forwrd[3]);
	public native bool ClimbUpToLedge(float goal[3], float forwrd[3], int obstacle);

	public native void FaceTowards(float pos[3]);
	public native void Approach(float pos[3], float weight = 1.0);
	public native void DriveTo(float pos[3]);
};

#if defined GAME_TF2
methodmap NextBotGoundLocomotion < ILocomotion
#elseif defined GAME_L4D2
methodmap ZombieBotLocomotion < ILocomotion
#endif
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
};

#if defined GAME_TF2
methodmap NextBotGoundLocomotionCustom < NextBotGoundLocomotion
#elseif defined GAME_L4D2
methodmap ZombieBotLocomotionCustom < ZombieBotLocomotion
#endif
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

#if defined GAME_L4D2
	/*
	public native void SetAcceleration(float accel[3]);
	public native void SetVelocity(float accel[3]);

	public native void GetMoveVector(float vec[3]);
	public native void GetGroundAcceleration(float vec[3]);
	*/
#endif
};

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

	public native void MarkLastKnownPositionAsSeen();
	public native bool IsVisibleInFOVNow();
	public native bool IsVisibleRecently();
	public native bool IsObsolete();
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

methodmap IVision < INextBotComponent
{
#if defined GAME_TF2
	public native bool ForEachKnownEntity(nextbot_functions func, any data = 0);
	public native void CollectKnownEntities(ArrayList list);
	public native CKnownEntity GetPrimaryKnownThreat(bool onlyVisibleThreats = false);
	public native CKnownEntity GetClosestKnownTeam(int team);
	public native CKnownEntity GetClosestKnownFilter(nextbot_functions func, any data = 0);
	public native int GetKnownCount(int team, bool onlyVisible = false, float rangeLimit = -1.0);
	public native CKnownEntity GetKnown(int entity);
	public native void AddKnownEntity(int entity);
	public native void ForgetEntity(int entity);
	public native void ForgetAllKnownEntities();
	public native void CollectPotentiallyVisibleEntities(ArrayList list);
#elseif defined GAME_L4D2
	public native int GetPrimaryRecognizedThreat();
	public native int GetClosestRecognizedTeam(int team);
	public native int GetClosestRecognizedFilter(nextbot_functions func, any data = 0);
#endif
	public native float GetTimeSinceVisible(int team);
	public native bool IsAbleToSeeEntity(int entity, FieldOfViewCheckType checkFOV, float visibleSpot[3] = NULL_VECTOR);
	public native bool IsAbleToSeeVector(float pos[3], FieldOfViewCheckType checkFOV);
	public native bool IsIgnored(int entity);
#if defined GAME_L4D2
	public native bool IsNoticed(int entity);
#elseif defined GAME_TF2
	public native bool IsVisibleEntityNoticed(int entity);
#endif
	public native bool IsInFieldOfViewVector(float pos[3]);
	public native bool IsInFieldOfViewEntity(int entity);
	public native bool IsLineOfSightClear(float pos[3]);
	public native bool IsLineOfSightClearToEntity(int entity, float visibleSpot[3] = NULL_VECTOR);
	public native bool IsLookingAtVector(float pos[3], float cosTolerance = 0.95);
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
methodmap IVisionCustom < IVision
#elseif defined GAME_L4D2
methodmap ZombieBotVision < IVision
{
	
};

methodmap ZombieBotVisionCustom < ZombieBotVision
#endif
{
#if defined GAME_TF2
	property float MaxVisionRange
	{
		public native set(float height);
	}

	property float MinRecognizeTime
	{
		public native set(float height);
	}
#endif

	property float DefaultFieldOfView
	{
		public native set(float height);
	}
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

	property float SolidMask
	{
		public native get();
	}

#if defined GAME_TF2
	property float CollisionGroup
	{
		public native get();
	}
#endif

	public native bool IsPostureMobile();

	public native void GetHullMins(float vec[3]);
	public native void GetHullMaxs(float vec[3]);
};

methodmap IBodyCustom < IBody
{
	property float HullWidth
	{
		public native set(float height);
	}

	property float HullHeight
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

	public native void SetHullMins(float vec[3]);
	public native void SetHullMaxs(float vec[3]);
};

enum BehaviorResultType
{ 
	BEHAVIOR_CONTINUE,			// continue executing this action next frame - nothing has changed
	BEHAVIOR_CHANGE_TO,			// change actions next frame
	BEHAVIOR_SUSPEND_FOR,		// put the current action on hold for the new action
	BEHAVIOR_DONE,				// this action has finished, resume suspended action
	BEHAVIOR_SUSTAIN,			// for use with event handlers - a way to say "It's important to keep doing what I'm doing"
};

enum EventResultPriorityType
{
	RESULT_NONE,		// no result
	RESULT_TRY,			// use this result, or toss it out, either is ok
	RESULT_IMPORTANT,	// try extra-hard to use this result
	RESULT_CRITICAL		// this result must be used - emit an error if it can't be
};

methodmap BehaviorActionEntry < Handle
{
	public native BehaviorActionEntry(const char[] name);
	
	public native void set_function(const char[] name, nextbot_functions func);
	
	public native BehaviorAction create();
};

methodmap BehaviorAction
{
	property BehaviorActionEntry Entry
	{
		public native get();
	}
	
	public native void set_data(const char[] name, any data);
	public native any get_data(const char[] name);
};

methodmap IIntentionCustom < INextBotComponent
{
	public native void ResetBehavior(BehaviorAction action);
};

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

	public native bool BeginUpdate();
	public native void Update();
	public native void EndUpdate();

	public native void Reset();

	public bool DoThink(int entity)
	{
		static ConVar NextBotStop = null;
		if(NextBotStop == null) {
			NextBotStop = FindConVar("nb_stop");
		}

		if(this.BeginUpdate()) {

			UpdateEntityLastKnownArea(entity);

			if(!NextBotStop.BoolValue && !(GetEntityFlags(entity) & FL_FROZEN)) {
				this.Update();
			}

			this.EndUpdate();

			return true;
		}

		return false;
	}

	public native float IsRangeLessThanEntity(int entity, float range);
	public native float IsRangeLessThanVector(float vec[3], float range);

	public native float IsRangeGreaterThanEntity(int entity, float range);
	public native float IsRangeGreaterThanVector(float vec[3], float range);

	public native float GetRangeToEntity(int entity);
	public native float GetRangeToVector(float vec[3]);

	public native float GetRangeSquaredToEntity(int entity);
	public native float GetRangeSquaredToVector(float vec[3]);

#if defined GAME_L4D2
	public native float Get2DRangeToEntity(int entity);
	public native float Get2DRangeToVector(float vec[3]);
#endif

#if defined GAME_TF2
	public native NextBotGoundLocomotionCustom AllocateCustomLocomotion();
#elseif defined GAME_L4D2
	public native ZombieBotLocomotionCustom AllocateCustomLocomotion();
#endif

#if defined GAME_TF2
	public native IVisionCustom AllocateCustomVision();
#elseif defined GAME_L4D2
	public native ZombieBotVisionCustom AllocateCustomVision();
#endif

	public native IBodyCustom AllocateCustomBody();

	public native IIntentionCustom AllocateCustomIntention(nextbot_functions func);
	public native void StubIntention();
	
	public native bool IsDebugging(NextBotDebugType type);
};

#define INextBot_Null (view_as<INextBot>(Address_Null))

methodmap INextBotCustom < INextBot
{
	
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

	public native bool IsValid();

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
	public native void GetClosestPosition(float ret[3], float pos[3], Segment start = Segment_Null, float alongLimit = 0.0);

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

#if defined GAME_TF2
	public native bool ComputeVector(INextBot bot, float goal[3], nextbot_functions functor, any data = 0, float maxPathLength = 0.0, bool includeGoalIfPathFails = true);
	public native bool ComputeEntity(INextBot bot, int subject, nextbot_functions functor, any data = 0, float maxPathLength = 0.0, bool includeGoalIfPathFails = true);
#elseif defined GAME_L4D2
	public native bool ComputeVector(INextBot bot, float goal[3], nextbot_functions functor, any data = 0, float maxPathLength = 0.0);
	public native bool ComputeEntity(INextBot bot, int subject, nextbot_functions functor, any data = 0, float maxPathLength = 0.0);
#endif
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

	public native bool IsDiscontinuityAhead(INextBot bot, SegmentType type, float range = -1.0);

	public native bool IsAtGoal(INextBot bot);

	public native void Update(INextBot bot);
};

enum SubjectChaseType
{
	LEAD_SUBJECT,
	DONT_LEAD_SUBJECT
};

methodmap ChasePath < PathFollower
{
	public native ChasePath(SubjectChaseType chaseHow);

	public native void Update(INextBot bot, int entity, nextbot_functions functor, any data = 0, float predictedpos[3] = NULL_VECTOR);
};

methodmap DirectChasePath < ChasePath
{
	public native DirectChasePath(SubjectChaseType chaseHow);
};

methodmap RetreatPath < PathFollower
{
	public native RetreatPath();

	public native void Update(INextBot bot, int entity);
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

native bool EntityIsCombatCharacter(int entity);
native CNavArea GetEntityLastKnownArea(int entity);
native INextBotCustom MakeEntityNextBot(int entity);

#tryinclude <datamaps>

native Address AllocateNextBotCombatCharacter(int size_modifier);
native int GetNextBotCombatCharacterSize();

stock Address datamaps_allocatenextbot(int size_modifier)
{
	return AllocateNextBotCombatCharacter(size_modifier);
}

#if defined datamaps_included
stock CustomSendtable sendtable_from_nextbot_factory(CustomEntityFactory factory)
{
	return CustomSendtable.from_factory(factory, "NextBotCombatCharacter");
}

stock CustomEntityFactory register_nextbot_factory(const char[] classname)
{
	return EntityFactoryDictionary.register_function(classname, datamaps_allocatenextbot, GetNextBotCombatCharacterSize());
}
#endif

enum baseline_cost_flags
{
	cost_flags_none = 0,
	cost_flags_mod = (1 << 0),
	cost_flags_safest = (1 << 1),
	cost_flags_discrete = (1 << 2),
	cost_flags_nojumping = (1 << 3),
	cost_flags_noladders = (1 << 4),
	cost_flags_nocrouch = (1 << 5),
};

#define cost_flags_onlywalk (cost_flags_nojumping| \
							cost_flags_noladders| \
							cost_flags_nocrouch)

stock float baseline_path_cost(INextBot bot, CNavArea area, CNavArea fromArea, CNavLadder ladder, Address elevator, float length, baseline_cost_flags flags)
{
	if(fromArea == CNavArea_Null) {
		return 0.0;
	}

	ILocomotion locomotion = bot.LocomotionInterface;

	if(!locomotion.IsAreaTraversable(area)) {
		return -1.0;
	}

	float dist = 0.0;
	if(ladder != CNavLadder_Null) {
		if(flags & cost_flags_noladders) {
			return -1.0;
		}

		static const float ladderPenalty = 2.0;
		dist = ladder.Length * ladderPenalty;
	} else if(length > 0.0) {
		dist = length;
	} else {
		float pos1[3];
		area.GetCenter(pos1);

		float pos2[3];
		fromArea.GetCenter(pos2);

		float sub[3];
		SubtractVectors(pos1, pos2, sub);

		dist = GetVectorLength(sub);
	}

	float cost = fromArea.CostSoFar + dist;

	float deltaZ = fromArea.ComputeAdjacentConnectionHeightChange(area);
	if(deltaZ < -locomotion.DeathDropHeight ||
		deltaZ >= locomotion.MaxJumpHeight) {
		return -1.0;
	} else if(deltaZ >= locomotion.StepHeight ||
				area.HasAttributes(NAV_MESH_JUMP)) {
		if(flags & cost_flags_nojumping) {
			return -1.0;
		}

		static const float jumpPenalty = 5.0;
		cost *= jumpPenalty;
	}

	if(area.Underwater) {
		static const float underwaterPenalty = 20.0;
		cost *= underwaterPenalty;
	}

	if(area.HasAttributes(NAV_MESH_CROUCH)) {
		if(flags & cost_flags_nocrouch) {
			return -1.0;
		}

		static const float crouchPenalty = 5.0;
		cost *= crouchPenalty;
	}

	if(area.HasAttributes(NAV_MESH_AVOID)) {
		static const float avoidPenalty = 10.0;
		cost *= avoidPenalty;
	}

#if defined GAME_TF2
	if(area.HasAttributes(NAV_MESH_FUNC_COST)) {
		cost *= area.ComputeFuncNavCost(entity);
	}
#endif

	int entity = bot.Entity;

	int team = GetEntProp(entity, Prop_Data, "m_iTeamNum");
	
	bool safe = !!(flags & cost_flags_safest);
	bool discrete = !!(flags & cost_flags_discrete);

	if(safe || discrete) {
		int enemy_team = 0;
		switch(team) {
			case 2: { enemy_team = 3; }
			case 3: { enemy_team = 2; }
		}

		static const float dangerCost = 100.0;
		cost *= area.GetDanger(team) * dangerCost;

	#if defined GAME_TF2
		int obj = -1;
		while((obj = FindEntityByClassname(obj, "tf_obj_sentrygun")) != -1) {
			int obj_team = GetEntProp(entity, Prop_Data, "m_iTeamNum");
			if(obj_team == enemy_team) {
				UpdateEntityLastKnownArea(obj);

				if(GetEntityLastKnownArea(obj) == area) {
					static const float enemyBuildingCost = 10.0;
					cost *= enemyBuildingCost;
				}
			}
		}

		CTFNavArea tfarea = view_as<CTFNavArea>(area);

		if(tfarea.InCombat) {
			static const float combatDangerCost = 4.0;
			cost *= tfarea.CombatIntensity * combatDangerCost;
		}

		static const float enemySentryDangerCost = 5.0;
		switch(team) {
			case 2: {
				if(tfarea.HasAttributeTF(TF_NAV_BLUE_SENTRY_DANGER)) {
					cost *= enemySentryDangerCost;
				}
			}
			case 3: {
				if(tfarea.HasAttributeTF(TF_NAV_RED_SENTRY_DANGER)) {
					cost *= enemySentryDangerCost;
				}
			}
		}
	#endif
	}

	if(discrete) {
	#if defined GAME_TF2
		int obj = -1;
		while((obj = FindEntityByClassname(obj, "tf_obj_sentrygun")) != -1) {
			int obj_team = GetEntProp(entity, Prop_Data, "m_iTeamNum");
			if(obj_team == team) {
				UpdateEntityLastKnownArea(obj);

				if(GetEntityLastKnownArea(obj) == area) {
					static const float friendlyBuildingCost = 5.0;
					cost *= friendlyBuildingCost;
				}
			}
		}

		CTFNavArea tfarea = view_as<CTFNavArea>(area);

		static const float friendlySentryDangerCost = 2.5;
		switch(team) {
			case 3: {
				if(tfarea.HasAttributeTF(TF_NAV_BLUE_SENTRY_DANGER)) {
					cost *= friendlySentryDangerCost;
				}
			}
			case 2: {
				if(tfarea.HasAttributeTF(TF_NAV_RED_SENTRY_DANGER)) {
					cost *= friendlySentryDangerCost;
				}
			}
		}
	#endif

		static const float teammateCost = 10.0;
		cost *= area.GetPlayerCount(team) * teammateCost;
	}

	float preference = 1.0;

	if(flags & cost_flags_mod) {
		int timeMod = RoundToFloor( GetGameTime() / 10.0 ) + 1;
		preference = 1.0 + 50.0 * ( 1.0 + Cosine( float( entity * area.ID * timeMod ) ) );
	}

	cost *= preference;

	return cost;
}

#if !defined REQUIRE_EXTENSIONS
public void __ext_nextbot_SetNTVOptional()
{
	MarkNativeAsOptional("Path.Path");
	MarkNativeAsOptional("Path.ComputeVector");
	MarkNativeAsOptional("Path.ComputeEntity");
	MarkNativeAsOptional("Path.Memory.get");
	MarkNativeAsOptional("PathFollower.PathFollower");
	MarkNativeAsOptional("PathFollower.Update");
#if defined GAME_TF2
	MarkNativeAsOptional("CTFPathFollower.CTFPathFollower");
	MarkNativeAsOptional("NextBotGoundLocomotion.NextBotGoundLocomotion");
	MarkNativeAsOptional("NextBotGoundLocomotion.Memory.get");
#endif
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
