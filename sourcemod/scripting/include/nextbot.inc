#if defined nextbot_included
	#endinput
#endif
#define nextbot_included

#define TEAM_ANY -2

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

	public native void GetCenter(float pos[3]);
	public native float ComputeAdjacentConnectionHeightChange(CNavArea area);
};

methodmap CNavLadder
{
	property float Length
	{
		public native get();
	}
};

methodmap CFuncElevator
{

};

methodmap INextBotComponent
{

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

	public native bool IsAreaTraversable(CNavArea area);

	public native bool IsClimbingOrJumping();
	public native bool IsClimbingUpToLedge();
	public native bool IsJumpingAcrossGap();
	public native bool IsScrambling();
	public native bool IsRunning();
	public native bool IsStuck();

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

	property float SolidMask
	{
		public native set(float height);
	}

	property float CollisionGroup
	{
		public native set(float height);
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
	//public native IBodyCustom AllocateCustomBody();
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

	public native void GetPosition(float ret[3], float distanceFromStart, Segment start = Address_Null);
	public native void GetClosestPosition(float ret[3], float pos[3], Segment start = Address_Null, float alongLimit = 0.0);

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

enum nextbot_prop_type
{
	CustomPropInt,
	CustomPropFloat,
	CustomPropBool,
};

methodmap IEntityFactoryCustom
{

};

native void nextbot_custom_datamap(const char[] classname, const char[] name, nextbot_prop_type type);
native void nextbot_custom_datamap_ex(IEntityFactoryCustom fac, const char[] name, nextbot_prop_type type);
native IEntityFactoryCustom nextbot_register_classname(const char[] classname);

stock void SetEntPropCustom(int entity, const char[] name, any value)
{
	SetEntProp(entity, Prop_Data, name, value);
}

stock void SetEntPropFloatCustom(int entity, const char[] name, float value)
{
	SetEntPropFloat(entity, Prop_Data, name, value);
}

stock any GetEntPropCustom(int entity, const char[] name)
{
	return GetEntProp(entity, Prop_Data, name);
}

stock float GetEntPropFloatCustom(int entity, const char[] name)
{
	return GetEntPropFloat(entity, Prop_Data, name);
}

stock float baseline_path_cost(INextBot bot, CNavArea area, CNavArea fromArea, CNavLadder ladder, Address elevator, float length, any data)
{
	if(fromArea == Address_Null) {
		return 0.0;
	} else {
		ILocomotion locomotion = bot.LocomotionInterface;

		if(!locomotion.IsAreaTraversable(area)) {
			return -1.0;
		}

		float dist = 0.0;
		if(ladder != Address_Null) {
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
			dist += jumpPenalty * dist;
		} else if(deltaZ < -locomotion.DeathDropHeight) {
			return -1.0;
		}

		int timeMod = RoundToFloor( GetGameTime() / 10.0 ) + 1;
		int entity = bot.Entity;
		float preference = 1.0 + 50.0 * ( 1.0 + Cosine( float( entity * area.ID * timeMod ) ) );
		float cost = dist * preference;

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
	MarkNativeAsOptional("nextbot_register_classname");
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