#include <sourcemod>
#include <datamaps>
#include <nextbot>
#include <teammanager>
#include <rulestools>

public Plugin myinfo = 
{
	name = "nextbot",
	author = "Arthurdead",
	description = "",
	version = "0.1.2.0",
	url = ""
};

static Address datamaps_allocatenextbot(int size_modifier, any data)
{ return AllocateNextBotCombatCharacter(size_modifier); }

#if defined GAME_L4D2
static Address datamaps_allocateinfectednextbot(int size_modifier, int data)
{ return AllocateInfectedNextBotCombatCharacter(size_modifier, data); }
#endif

static int m_lifeState_offset = -1;
static int m_iHealth_offset = -1;
static int m_iMaxHealth_offset = -1;

public void OnPluginStart()
{
	m_lifeState_offset = FindSendPropInfo("CAI_BaseNPC", "m_lifeState");
	m_iHealth_offset = FindSendPropInfo("CTFRobotDestruction_Robot", "m_iHealth");
	m_iMaxHealth_offset = FindSendPropInfo("CTFRobotDestruction_Robot", "m_iMaxHealth");
}

public APLRes AskPluginLoad2(Handle myself, bool late, char[] error, int length)
{
	RegPluginLibrary("nextbot");

	CreateNative("baseline_path_cost_impl", native_baseline_path_cost_impl);

	CreateNative("register_nextbot_factory", native_register_nextbot_factory);
	CreateNative("register_basenpc_nextbot_factory", native_register_basenpc_nextbot_factory);
	CreateNative("register_tankboss_nextbot_factory", native_register_tankboss_nextbot_factory);
	CreateNative("register_robot_nextbot_factory", native_register_robot_nextbot_factory);

	return APLRes_Success;
}

stock CustomDatamap __register_nb_factory_based(Handle plugin, const char[] classname, const char[] name, const char[] clientnet, const char[] based, CustomSendtable &table = null, CustomEntityFactory &factory = null)
{
	char netname[64];
	netname[0] = 'C';
	strcopy(netname[1], sizeof(netname)-1, name);

	CustomEntityFactory old_factory = EntityFactoryDictionary.register_based_name(classname, based);
	factory = view_as<CustomEntityFactory>(CloneHandle(old_factory, plugin));
	delete old_factory;

	CustomSendtable old_table = CustomSendtable.from_factory(factory);
	table = view_as<CustomSendtable>(CloneHandle(old_table, plugin));
	delete old_table;

	CustomDatamap old_datamap = CustomDatamap.from_factory(factory);
	CustomDatamap datamap = view_as<CustomDatamap>(CloneHandle(old_datamap, plugin));
	delete old_datamap;

	if(!IsNullString(clientnet) && clientnet[0] != '\0') {
		table.override_with(clientnet);
	}

	table.set_network_name(netname);

	datamap.set_name(netname);

	return datamap;
}

stock CustomDatamap __register_nb_factory_func(Handle plugin, const char[] classname, const char[] name, const char[] clientnet, entityalloc_func_t alloc, int size, any data, CustomSendtable &table = null, CustomEntityFactory &factory = null)
{
	char netname[64];
	netname[0] = 'C';
	strcopy(netname[1], sizeof(netname)-1, name);

	CustomEntityFactory old_factory = EntityFactoryDictionary.register_function(classname, alloc, size, data);
	factory = view_as<CustomEntityFactory>(CloneHandle(old_factory, plugin));
	delete old_factory;

	CustomSendtable old_table = CustomSendtable.from_factory(factory);
	table = view_as<CustomSendtable>(CloneHandle(old_table, plugin));
	delete old_table;

	CustomDatamap old_datamap = CustomDatamap.from_factory(factory);
	CustomDatamap datamap = view_as<CustomDatamap>(CloneHandle(old_datamap, plugin));
	delete old_datamap;

	if(!IsNullString(clientnet) && clientnet[0] != '\0') {
		table.override_with(clientnet);
	}

	datamap.set_name(netname);

	return datamap;
}

#if defined GAME_L4D2
static any native_register_infected_nextbot_factory(Handle plugin, int params)
{
	int length = 0;
	GetNativeStringLength(1, length);
	char[] classname = new char[++length];
	GetNativeString(1, classname, length);

	int length = 0;
	GetNativeStringLength(2, length);
	char[] name = new char[++length];
	GetNativeString(2, name, length);

	int cls = GetNativeCell(3);

	CustomSendtable table = GetNativeCellRef(4);
	CustomEntityFactory factory = GetNativeCellRef(5);

	CustomDatamap datamap = __register_nb_factory_func(plugin, classname, name, NULL_STRING, datamaps_allocateinfectednextbot, GetInfectedSize(), cls, table, factory);

	SetNativeCellRef(4, table);
	SetNativeCellRef(5, factory);

	return datamap;
}

static any native_register_infected_factory(Handle plugin, int params)
{
	int length = 0;
	GetNativeStringLength(1, length);
	char[] classname = new char[++length];
	GetNativeString(1, classname, length);

	int length = 0;
	GetNativeStringLength(2, length);
	char[] name = new char[++length];
	GetNativeString(2, name, length);

	CustomSendtable table = GetNativeCellRef(3);
	CustomEntityFactory factory = GetNativeCellRef(4);

	CustomDatamap datamap = __register_nb_factory_based(plugin, classname, name, NULL_STRING, "infected", table, factory);

	SetNativeCellRef(3, table);
	SetNativeCellRef(4, factory);

	return datamap;
}
#endif

static any native_register_nextbot_factory(Handle plugin, int params)
{
	int length = 0;
	GetNativeStringLength(1, length);
	char[] classname = new char[++length];
	GetNativeString(1, classname, length);

	length = 0;
	GetNativeStringLength(2, length);
	char[] name = new char[++length];
	GetNativeString(2, name, length);

	CustomSendtable table = GetNativeCellRef(4);
	CustomEntityFactory factory = GetNativeCellRef(5);

	CustomDatamap datamap = null;

	if(!IsNativeParamNullString(3)) {
		length = 0;
		GetNativeStringLength(3, length);
		char[] clientnet = new char[++length];
		GetNativeString(3, clientnet, length);

		datamap = __register_nb_factory_func(plugin, classname, name, clientnet, datamaps_allocatenextbot, GetNextBotCombatCharacterSize(), 0, table, factory);
	} else {
		datamap = __register_nb_factory_func(plugin, classname, name, NULL_STRING, datamaps_allocatenextbot, GetNextBotCombatCharacterSize(), 0, table, factory);
	}

	SetNativeCellRef(4, table);
	SetNativeCellRef(5, factory);

	return datamap;
}

static any native_register_basenpc_nextbot_factory(Handle plugin, int params)
{
	int length = 0;
	GetNativeStringLength(1, length);
	char[] classname = new char[++length];
	GetNativeString(1, classname, length);

	length = 0;
	GetNativeStringLength(2, length);
	char[] name = new char[++length];
	GetNativeString(2, name, length);

	CustomSendtable table = GetNativeCellRef(3);
	CustomEntityFactory factory = GetNativeCellRef(4);

	CustomDatamap datamap = __register_nb_factory_func(plugin, classname, name, "CAI_BaseNPC", datamaps_allocatenextbot, GetNextBotCombatCharacterSize(), 0, table, factory);
	table.set_name("DT_AI_BaseNPC");
	table.set_network_name("CAI_BaseNPC");
	table.add_prop_int("m_iDeathPose", 4, 12);
	table.add_prop_int("m_iDeathFrame", 4, 5);
	table.add_prop_int("m_bImportanRagdoll", 1, 1, SPROP_UNSIGNED);
	table.add_prop_int("m_bPerformAvoidance", 1, 1, SPROP_UNSIGNED);
	table.add_prop_int("m_bIsMoving", 1, 1, SPROP_UNSIGNED);
	table.add_prop_int("m_bFadeCorpse", 1, 1, SPROP_UNSIGNED);
	table.add_prop_int("m_lifeState", 1, 3, SPROP_UNSIGNED, m_lifeState_offset);

	SetNativeCellRef(3, table);
	SetNativeCellRef(4, factory);

	return datamap;
}

static any native_register_tankboss_nextbot_factory(Handle plugin, int params)
{
	int length = 0;
	GetNativeStringLength(1, length);
	char[] classname = new char[++length];
	GetNativeString(1, classname, length);

	length = 0;
	GetNativeStringLength(2, length);
	char[] name = new char[++length];
	GetNativeString(2, name, length);

	CustomSendtable table = GetNativeCellRef(3);
	CustomEntityFactory factory = GetNativeCellRef(4);

	CustomDatamap datamap = __register_nb_factory_func(plugin, classname, name, "CTFTankBoss", datamaps_allocatenextbot, GetNextBotCombatCharacterSize(), 0, table, factory);
	table.set_name("DT_TFBaseBoss");
	table.set_network_name("CTFBaseBoss");
	table.add_prop_float("m_lastHealthPercentage", 0.0, 1.0, 11, SPROP_NOSCALE);

	SetNativeCellRef(3, table);
	SetNativeCellRef(4, factory);

	return datamap;
}

static any native_register_robot_nextbot_factory(Handle plugin, int params)
{
	int length = 0;
	GetNativeStringLength(1, length);
	char[] classname = new char[++length];
	GetNativeString(1, classname, length);

	length = 0;
	GetNativeStringLength(2, length);
	char[] name = new char[++length];
	GetNativeString(2, name, length);

	CustomSendtable table = GetNativeCellRef(3);
	CustomEntityFactory factory = GetNativeCellRef(4);

	CustomDatamap datamap = __register_nb_factory_func(plugin, classname, name, "CTFRobotDestruction_Robot", datamaps_allocatenextbot, GetNextBotCombatCharacterSize(), 0, table, factory);
	table.set_name("DT_TFRobotDestruction_Robot");
	table.set_network_name("CTFRobotDestruction_Robot");
	table.add_prop_int("m_iHealth", 4, -1, SPROP_VARINT, m_iHealth_offset);
	table.add_prop_int("m_iMaxHealth", 4, -1, SPROP_VARINT, m_iMaxHealth_offset);
	table.add_prop_int("m_eType", 4, -1, SPROP_VARINT, -1);

	SetNativeCellRef(3, table);
	SetNativeCellRef(4, factory);

	return datamap;
}

static any native_baseline_path_cost_impl(Handle plugin, int params)
{
	INextBot bot = GetNativeCell(1);
	CNavArea area = GetNativeCell(2);
	CNavArea fromArea = GetNativeCell(3);
	CNavLadder ladder = GetNativeCell(4);
	Address elevator = GetNativeCell(5);
	float length = GetNativeCell(6);
	baseline_cost_flags flags = GetNativeCell(7);

	if(fromArea == CNavArea_Null) {
		return 0.0;
	}

	ILocomotion locomotion = bot.LocomotionInterface;

	if(!locomotion.IsAreaTraversable(area)) {
		return -1.0;
	}
	
	int entity = bot.Entity;
	
	int team = GetEntProp(entity, Prop_Data, "m_iTeamNum");

	bool mvm = IsMannVsMachineMode();

	if(flags & cost_flags_noenemyspawn) {
#if defined GAME_TF2
		CTFNavArea tfarea = view_as<CTFNavArea>(area);
		switch(team) {
			case TF_TEAM_RED: {
				if(tfarea.HasAttributeTF(TF_NAV_SPAWN_ROOM_BLUE)) {
					return -1.0;
				}
			}
			case TF_TEAM_BLUE: {
				if(tfarea.HasAttributeTF(TF_NAV_SPAWN_ROOM_RED)) {
					return -1.0;
				}
			}
			case TEAM_UNASSIGNED, TF_TEAM_HALLOWEEN: {
				if(tfarea.HasAttributeTF(TF_NAV_SPAWN_ROOM_RED) ||
					tfarea.HasAttributeTF(TF_NAV_SPAWN_ROOM_BLUE)) {
					return -1.0;
				}
			}
			case TF_TEAM_PVE_INVADERS_GIANTS: {
				if(mvm) {
					if(tfarea.HasAttributeTF(TF_NAV_SPAWN_ROOM_RED)) {
						return -1.0;
					}
				}
			}
		}
#endif
	}

	float dist = 0.0;
	if(ladder != CNavLadder_Null) {
		if(flags & cost_flags_noladders) {
			return -1.0;
		}

		dist = ladder.Length;

		static const float ladderPenalty = 1.0;
		dist *= ladderPenalty;
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
	if(deltaZ >= locomotion.StepHeight ||
		area.HasAttributes(NAV_MESH_JUMP)) {
		if(flags & cost_flags_nojumping ||
			deltaZ >= locomotion.MaxJumpHeight) {
			return -1.0;
		}

		if(flags & cost_flags_fastest) {
			static const float jumpPenalty = 2.0;
			dist *= jumpPenalty;
		} else {
			static const float jumpPenalty = 2.0;
			dist *= jumpPenalty;
		}
	} else if(deltaZ < -locomotion.DeathDropHeight) {
		return -1.0;
	}

	if(area.Underwater) {
		if(flags & cost_flags_nowater) {
			return -1.0;
		}

		static const float underwaterPenalty = 20.0;
		dist *= underwaterPenalty;
	}

	if(area.HasAttributes(NAV_MESH_CROUCH)) {
		if(flags & cost_flags_nocrouch) {
			return -1.0;
		}

		if(flags & cost_flags_fastest) {
			static const float crouchPenalty = 20.0;
			dist *= crouchPenalty;
		} else {
			static const float crouchPenalty = 5.0;
			dist *= crouchPenalty;
		}
	}

	if(area.HasAttributes(NAV_MESH_WALK)) {
		if(flags & cost_flags_fastest) {
			static const float walkPenalty = 20.0;
			dist *= walkPenalty;
		} else {
			static const float walkPenalty = 5.0;
			dist *= walkPenalty;
		}
	}

	if(area.HasAttributes(NAV_MESH_AVOID)) {
		static const float avoidPenalty = 20.0;
		dist *= avoidPenalty;
	}

	if(area.Damaging) {
		static const float damagingPenalty = 100.0;
		dist *= damagingPenalty;
	}

	if(flags & cost_flags_safest) {
	#if defined GAME_TF2
		CTFNavArea tfarea = view_as<CTFNavArea>(area);
		if(tfarea.InCombat) {
			static const float combatDangerCost = 4.0;
			dist *= (combatDangerCost * tfarea.CombatIntensity);
		}

		if(!TeamManager_IsTruceActive()) {
			static const float enemySentryDangerCost = 5.0;

			switch(team) {
				case TF_TEAM_RED: {
					if(tfarea.HasAttributeTF(TF_NAV_BLUE_SENTRY_DANGER)) {
						dist *= enemySentryDangerCost;
					}
				}
				case TF_TEAM_BLUE: {
					if(tfarea.HasAttributeTF(TF_NAV_RED_SENTRY_DANGER)) {
						dist *= enemySentryDangerCost;
					}
				}
				case TEAM_UNASSIGNED, TF_TEAM_HALLOWEEN: {
					if(tfarea.HasAttributeTF(TF_NAV_RED_SENTRY_DANGER) ||
						tfarea.HasAttributeTF(TF_NAV_BLUE_SENTRY_DANGER)) {
						dist *= enemySentryDangerCost;
					}
				}
				case TF_TEAM_PVE_INVADERS_GIANTS: {
					if(mvm) {
						if(tfarea.HasAttributeTF(TF_NAV_RED_SENTRY_DANGER)) {
							dist *= enemySentryDangerCost;
						}
					}
				}
			}
		}
	#endif
	}

	if(flags & cost_flags_discrete) {
	#if defined GAME_TF2
		CTFNavArea tfarea = view_as<CTFNavArea>(area);

		static const float friendlySentryDangerCost = 2.5;
		if(!TeamManager_IsTruceActive()) {
			switch(team) {
				case TF_TEAM_BLUE: {
					if(tfarea.HasAttributeTF(TF_NAV_BLUE_SENTRY_DANGER)) {
						dist *= friendlySentryDangerCost;
					}
				}
				case TF_TEAM_RED: {
					if(tfarea.HasAttributeTF(TF_NAV_RED_SENTRY_DANGER)) {
						dist *= friendlySentryDangerCost;
					}
				}
				case TF_TEAM_PVE_INVADERS_GIANTS: {
					if(mvm) {
						if(tfarea.HasAttributeTF(TF_NAV_BLUE_SENTRY_DANGER)) {
							dist *= friendlySentryDangerCost;
						}
					}
				}
			}
		} else {
			if(tfarea.HasAttributeTF(TF_NAV_BLUE_SENTRY_DANGER)) {
				dist *= friendlySentryDangerCost;
			}
			if(tfarea.HasAttributeTF(TF_NAV_RED_SENTRY_DANGER)) {
				dist *= friendlySentryDangerCost;
			}
		}
	#endif
	}

	float cost = -1.0;

	if(flags & cost_flags_mod_small) {
		int timeMod = (RoundToFloor(GetGameTime() / NB_PATHCOST_MOD_PERIOD) + 1);
		int uniqueID = (view_as<int>(area) >> 7);
		int nRandomCost = ((entity * uniqueID * timeMod) % 293);
		cost += (1.0 + float(nRandomCost));
	} else if(flags & cost_flags_mod_heavy) {
		int timeMod = (RoundToFloor(GetGameTime() / NB_PATHCOST_MOD_PERIOD) + 1);
		float preference = 1.0 + 50.0 * ( 1.0 + Cosine( float( entity * area.ID * timeMod ) ) );
		cost = (dist * preference);
	} else {
		cost = dist;
	}

#if defined GAME_TF2
	if(area.HasAttributes(NAV_MESH_FUNC_COST)) {
		cost *= area.ComputeFuncNavCost(entity);
	}
#endif

	return cost + fromArea.CostSoFar;
}