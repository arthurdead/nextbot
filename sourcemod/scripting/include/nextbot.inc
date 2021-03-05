#if defined nextbot_included
	#endinput
#endif
#define nextbot_included

#include <tf2>

#define TFTeam_Any view_as<TFTeam>(-2)

typeset nextbot_functions
{
	//IVision.ForEachKnownEntity
	function bool (CKnownEntity known, any data);

	//IVision.GetClosestKnownFilter
	function bool (int entity, any data);

	//Path.ComputeVector
	//Path.ComputeEntity
	function float (INextBot bot, CNavArea area, CNavArea fromArea, CNavLadder ladder, Address elevator, float length, any data);
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
	NAV_MESH_FUNC_COST		= 0x20000000,				// area has designer specified cost controlled by func_nav_cost entities
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

	public native bool HasAttributes(NavAttributeType bits);
	public native float ComputeFuncNavCost(int entity);
	public native void GetCenter(float pos[3]);
	public native float ComputeAdjacentConnectionHeightChange(CNavArea area);
	public native int GetPlayerCount(TFTeam team = TFTeam_Unassigned);
};

#define CNavArea_Null view_as<CNavArea>(Address_Null)

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

methodmap CNavMesh
{
	public native CNavArea GetNearestNavAreaVector(float pos[3], bool anyZ = false, float maxDist = 10000.0, bool checkLOS = false, bool checkGround = true, TFTeam team = TFTeam_Any);
	public native bool GetGroundHeight(float pos[3], float &height, float normal[3] = NULL_VECTOR);
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

methodmap INextBotComponent
{
	property INextBot Bot
	{
		public native get();
	}
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

	property float MaxAcceleration
	{
		public native get();
	}

	property float MaxDeceleration
	{
		public native get();
	}

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

methodmap NextBotGoundLocomotion < ILocomotion
{
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

	property float MaxYawRate
	{
		public native get();
	}
};

methodmap NextBotGoundLocomotionCustom < NextBotGoundLocomotion
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

	property float MaxAcceleration
	{
		public native set(float height);
	}

	property float MaxDeceleration
	{
		public native set(float height);
	}

	property float SpeedLimit
	{
		public native set(float height);
	}

	property float TraversableSlopeLimit
	{
		public native set(float height);
	}

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

	property float MaxYawRate
	{
		public native set(float height);
	}
};

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

enum FieldOfViewCheckType
{
	USE_FOV,
	DISREGARD_FOV
};

methodmap IVision < INextBotComponent
{
	public native bool ForEachKnownEntity(nextbot_functions func, any data = 0);
	public native void CollectKnownEntities(ArrayList list);
	public native CKnownEntity GetPrimaryKnownThreat(bool onlyVisibleThreats = false);
	public native float GetTimeSinceVisible(int team);
	public native CKnownEntity GetClosestKnownTeam(int team);
	public native CKnownEntity GetClosestKnownFilter(nextbot_functions func, any data = 0);
	public native int GetKnownCount(int team, bool onlyVisible = false, float rangeLimit = -1.0);
	public native CKnownEntity GetKnown(int entity);
	public native void AddKnownEntity(int entity);
	public native void ForgetEntity(int entity);
	public native void ForgetAllKnownEntities();
	public native void CollectPotentiallyVisibleEntities(ArrayList list);
	public native bool IsAbleToSeeEntity(int entity, FieldOfViewCheckType checkFOV, float visibleSpot[3] = NULL_VECTOR);
	public native bool IsAbleToSeeVector(float pos[3], FieldOfViewCheckType checkFOV);
	public native bool IsIgnored(int entity);
	public native bool IsVisibleEntityNoticed(int entity);
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

methodmap IVisionCustom < IVision
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

	property float CollisionGroup
	{
		public native get();
	}

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

	property int CollisionGroup
	{
		public native set(int height);
	}

	public native void SetHullMins(float vec[3]);
	public native void SetHullMaxs(float vec[3]);
};

methodmap INextBot
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

	//public native void RegisterComponent(INextBotComponent comp);
	//public native void UnregisterComponent(INextBotComponent comp);

	public native NextBotGoundLocomotionCustom AllocateCustomLocomotion();
	//public native IVisionCustom AllocateCustomVision();
	public native IBodyCustom AllocateCustomBody();
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
	}

	property Segment CurrentGoal
	{
		public native get();
	}

	public native void Invalidate();

	public native bool ComputeVector(INextBot bot, float goal[3], nextbot_functions functor, any data = 0, float maxPathLength = 0.0, bool includeGoalIfPathFails = true);
	public native bool ComputeEntity(INextBot bot, int subject, nextbot_functions functor, any data = 0, float maxPathLength = 0.0, bool includeGoalIfPathFails = true);
};

methodmap PathFollower < Path
{
	public native PathFollower();

	property float MinLookAheadDistance
	{
		public native get();
		public native set(float height);
	}

	property float GoalTolerance
	{
		public native get();
		public native set(float height);
	}

	property int Hindrance
	{
		public native get();
	}

	public native bool IsDiscontinuityAhead(INextBot bot, SegmentType type, float range = -1.0);

	public native void Update(INextBot bot);
};

methodmap CTFPathFollower < PathFollower
{
	public native CTFPathFollower();

	property float MinLookAheadDistance
	{
		public native get();
	}
};

native Address AllocateNextBotCombatCharacter();
native bool EntityIsCombatCharacter(int entity);
native CNavArea GetEntityLastKnownArea(int entity);
native void UpdateEntityLastKnownArea(int entity);

stock Address datamaps_allocatenextbot()
{
	return AllocateNextBotCombatCharacter();
}

enum baseline_cost_flags
{
	cost_flags_none = 0,
	cost_flags_mod = (1 << 0),
	cost_flags_safest = (1 << 1),
	cost_flags_discrete = (1 << 2),
};

stock float baseline_path_cost(INextBot bot, CNavArea area, CNavArea fromArea, CNavLadder ladder, Address elevator, float length, baseline_cost_flags flags)
{
	if(fromArea == CNavArea_Null) {
		return 0.0;
	} else {
		ILocomotion locomotion = bot.LocomotionInterface;

		if(!locomotion.IsAreaTraversable(area)) {
			return -1.0;
		}

		float dist = 0.0;
		if(ladder != CNavLadder_Null) {
			dist = ladder.Length;
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

		float deltaZ = fromArea.ComputeAdjacentConnectionHeightChange(area);
		if(deltaZ >= locomotion.StepHeight) {
			if(deltaZ >= locomotion.MaxJumpHeight) {
				return -1.0;
			}

			static const float jumpPenalty = 5.0;
			dist *= jumpPenalty * dist;
		} else if(deltaZ < -locomotion.DeathDropHeight) {
			return -1.0;
		}

		float preference = 1.0;

		int entity = bot.Entity;

		if(flags & cost_flags_mod) {
			int timeMod = RoundToFloor( GetGameTime() / 10.0 ) + 1;
			preference = 1.0 + 50.0 * ( 1.0 + Cosine( float( entity * area.ID * timeMod ) ) );
		}

		TFTeam team = view_as<TFTeam>(GetEntProp(entity, Prop_Data, "m_iTeamNum"));

		if(flags & cost_flags_safest) {
			CTFNavArea tfarea = view_as<CTFNavArea>(area);

			if(tfarea.InCombat) {
				static const float combatDangerCost = 4.0;
				dist *= combatDangerCost * tfarea.CombatIntensity;
			}

			static const float sentryDangerCost = 5.0;
			switch(team) {
				case TFTeam_Red: {
					if(tfarea.HasAttributeTF(TF_NAV_BLUE_SENTRY_DANGER)) {
						dist *= sentryDangerCost;
					}
				}
				case TFTeam_Blue: {
					if(tfarea.HasAttributeTF(TF_NAV_RED_SENTRY_DANGER)) {
						dist *= sentryDangerCost;
					}
				}
			}
		}

		if(flags & cost_flags_discrete) {
			TFTeam enemy_team = TFTeam_Unassigned;
			switch(team) {
				case TFTeam_Red: { enemy_team = TFTeam_Blue; }
				case TFTeam_Blue: { enemy_team = TFTeam_Red; }
			}

			int obj = -1;
			while((obj = FindEntityByClassname(obj, "tf_obj_sentrygun")) != -1) {
				TFTeam obj_team = view_as<TFTeam>(GetEntProp(entity, Prop_Data, "m_iTeamNum"));
				if(team == TFTeam_Unassigned ||
					enemy_team == TFTeam_Unassigned ||
					obj_team == TFTeam_Unassigned ||
					obj_team == enemy_team) {
					UpdateEntityLastKnownArea(obj);

					if(GetEntityLastKnownArea(obj) == area) {
						static const float enemyBuildingCost = 10.0;
						dist *= enemyBuildingCost;
					}
				}
			}

			static const float teammateCost = 10.0;
			dist += dist * teammateCost * area.GetPlayerCount(team);
		}

		float cost = dist * preference;

		if(area.HasAttributes(NAV_MESH_FUNC_COST)) {
			cost *= area.ComputeFuncNavCost(entity);
		}

		return cost + fromArea.CostSoFar;
	}
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
	MarkNativeAsOptional("CTFPathFollower.CTFPathFollower");
	MarkNativeAsOptional("NextBotGoundLocomotion.NextBotGoundLocomotion");
	MarkNativeAsOptional("NextBotGoundLocomotion.Memory.get");
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
