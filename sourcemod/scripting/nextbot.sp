#include <sourcemod>
#include <nextbot>
#include <teammanager>
#include <rulestools>

public Plugin myinfo = 
{
	name = "nextbot",
	author = "Arthurdead",
	description = "",
	version = "0.1.1.6",
	url = ""
};

public APLRes AskPluginLoad2(Handle myself, bool late, char[] error, int length)
{
	RegPluginLibrary("nextbot");
	CreateNative("baseline_path_cost_impl", native_baseline_path_cost_impl);
	return APLRes_Success;
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