class_<RoboCupGameControlData>("RoboCupGameControlData")
   .def_readonly("header"        , &RoboCupGameControlData::header        )
   .def_readonly("version"       , &RoboCupGameControlData::version       )
   .def_readonly("playersPerTeam", &RoboCupGameControlData::playersPerTeam)
   .def_readonly("state"         , &RoboCupGameControlData::state         )
   .def_readonly("setPlay"       , &RoboCupGameControlData::setPlay       )
   .def_readonly("firstHalf"     , &RoboCupGameControlData::firstHalf     )
   .def_readonly("kickingTeam"   , &RoboCupGameControlData::kickingTeam   )
   .def_readonly("gamePhase"     , &RoboCupGameControlData::gamePhase     )
   .def_readonly("dropInTeam"    , &RoboCupGameControlData::dropInTeam    )
   .def_readonly("dropInTime"    , &RoboCupGameControlData::dropInTime    )
   .def_readonly("secsRemaining" , &RoboCupGameControlData::secsRemaining )
   .add_property("teams"         , &RoboCupGameControlData::teams         );

class_<PlayerInfo>("PlayerInfo")
   .def_readonly("penalty"              , &PlayerInfo::penalty            )
   .def_readonly("secsTillUnpenalised"  , &PlayerInfo::secsTillUnpenalised);

class_<TeamInfo>("TeamInfo")
   .def_readonly("teamNumber"           , &TeamInfo::teamNumber           )
   .def_readonly("score"                , &TeamInfo::score                )
   .add_property("players"              , &TeamInfo::players              );

   scope().attr("MAX_NUM_PLAYERS"                   ) = MAX_NUM_PLAYERS;
   scope().attr("TEAM_BLUE"                         ) = TEAM_BLUE;
   scope().attr("TEAM_CYAN"                         ) = TEAM_CYAN;
   scope().attr("TEAM_RED"                          ) = TEAM_RED;
   scope().attr("TEAM_MAGENTA"                      ) = TEAM_MAGENTA;
   scope().attr("STATE_INITIAL"                     ) = STATE_INITIAL;
   scope().attr("STATE_READY"                       ) = STATE_READY;
   scope().attr("STATE_SET"                         ) = STATE_SET;
   scope().attr("STATE_PLAYING"                     ) = STATE_PLAYING;
   scope().attr("STATE_FINISHED"                    ) = STATE_FINISHED;
   scope().attr("STATE_INVALID"                     ) = STATE_INVALID;
   scope().attr("STATE_PENALISED"                   ) = STATE_PENALISED;
   scope().attr("STATE_PENALISED_IN_SET"            ) = STATE_PENALISED_IN_SET;
   scope().attr("GAME_PHASE_NORMAL"                 ) = GAME_PHASE_NORMAL;
   scope().attr("GAME_PHASE_PENALTYSHOOT"           ) = GAME_PHASE_PENALTYSHOOT;
   scope().attr("GAME_PHASE_OVERTIME"               ) = GAME_PHASE_OVERTIME;
   scope().attr("GAME_PHASE_TIMEOUT"                ) = GAME_PHASE_TIMEOUT;
   scope().attr("SET_PLAY_NONE"                     ) = SET_PLAY_NONE;
   scope().attr("SET_PLAY_GOAL_FREE_KICK"           ) = SET_PLAY_GOAL_FREE_KICK;
   scope().attr("SET_PLAY_PUSHING_FREE_KICK"        ) = SET_PLAY_PUSHING_FREE_KICK;
   scope().attr("PENALTY_NONE"                      ) = PENALTY_NONE;
   scope().attr("PENALTY_SPL_ILLEGAL_BALL_CONTACT"  ) = PENALTY_SPL_ILLEGAL_BALL_CONTACT;
   scope().attr("PENALTY_SPL_PLAYER_PUSHING"        ) = PENALTY_SPL_PLAYER_PUSHING;
   scope().attr("PENALTY_SPL_ILLEGAL_MOTION_IN_SET" ) = PENALTY_SPL_ILLEGAL_MOTION_IN_SET;
   scope().attr("PENALTY_SPL_INACTIVE_PLAYER"       ) = PENALTY_SPL_INACTIVE_PLAYER;
   scope().attr("PENALTY_SPL_ILLEGAL_DEFENDER"      ) = PENALTY_SPL_ILLEGAL_DEFENDER;
   scope().attr("PENALTY_SPL_LEAVING_THE_FIELD"     ) = PENALTY_SPL_LEAVING_THE_FIELD;
   scope().attr("PENALTY_SPL_KICK_OFF_GOAL"         ) = PENALTY_SPL_KICK_OFF_GOAL;
   scope().attr("PENALTY_SPL_REQUEST_FOR_PICKUP"    ) = PENALTY_SPL_REQUEST_FOR_PICKUP;
   scope().attr("PENALTY_SPL_LOCAL_GAME_STUCK"      ) = PENALTY_SPL_LOCAL_GAME_STUCK;
   scope().attr("PENALTY_HL_KID_BALL_MANIPULATION"  ) = PENALTY_HL_KID_BALL_MANIPULATION;
   scope().attr("PENALTY_HL_KID_PHYSICAL_CONTACT"   ) = PENALTY_HL_KID_PHYSICAL_CONTACT;
   scope().attr("PENALTY_HL_KID_ILLEGAL_ATTACK"     ) = PENALTY_HL_KID_ILLEGAL_ATTACK;
   scope().attr("PENALTY_HL_KID_ILLEGAL_DEFENSE"    ) = PENALTY_HL_KID_ILLEGAL_DEFENSE;
   scope().attr("PENALTY_HL_KID_REQUEST_FOR_PICKUP" ) = PENALTY_HL_KID_REQUEST_FOR_PICKUP;
   scope().attr("PENALTY_HL_TEEN_BALL_MANIPULATION" ) = PENALTY_HL_TEEN_BALL_MANIPULATION;
   scope().attr("PENALTY_HL_TEEN_PHYSICAL_CONTACT"  ) = PENALTY_HL_TEEN_PHYSICAL_CONTACT;
   scope().attr("PENALTY_HL_TEEN_ILLEGAL_ATTACK"    ) = PENALTY_HL_TEEN_ILLEGAL_ATTACK;
   scope().attr("PENALTY_HL_TEEN_ILLEGAL_DEFENSE"   ) = PENALTY_HL_TEEN_ILLEGAL_DEFENSE;
   scope().attr("PENALTY_HL_TEEN_REQUEST_FOR_PICKUP") = PENALTY_HL_TEEN_REQUEST_FOR_PICKUP;
   scope().attr("PENALTY_MANUAL"                    ) = PENALTY_MANUAL;
