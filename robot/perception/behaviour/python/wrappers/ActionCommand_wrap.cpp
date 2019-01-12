class_<ActionCommand::Body>("BodyCommand")
   .def(init<ActionCommand::Body::ActionType,  /* Action type         */
             int, int,                         /* left right          */
             float,                            /* rotation (rads)     */
             float, float, float,              /* power bend speed    */
             float, ActionCommand::Body::Foot, /* kick direction/foot */
             bool,                             /* fast walk/dribble   */
             bool,                             /* whether to misalign the kick */
             bool,                             /* whether to shuffle  */
             bool,                             /* whether to put our left arm back */
             bool                             /* whether to put our right arm back */
            >())

   .def_readwrite("actionType"    , &ActionCommand::Body::actionType   )
   .def_readwrite("forward"       , &ActionCommand::Body::forward      )
   .def_readwrite("left"          , &ActionCommand::Body::left         )
   .def_readwrite("turn"          , &ActionCommand::Body::turn         )
   .def_readwrite("power"         , &ActionCommand::Body::power        )
   .def_readwrite("bend"          , &ActionCommand::Body::bend         )
   .def_readwrite("speed"         , &ActionCommand::Body::speed        )
   .def_readwrite("kickDirection" , &ActionCommand::Body::kickDirection)
   .def_readwrite("foot"          , &ActionCommand::Body::foot         )
   .def_readwrite("isFast"        , &ActionCommand::Body::isFast       )
   .def_readwrite("misalignedKick", &ActionCommand::Body::misalignedKick)
   .def_readwrite("useShuffle"    , &ActionCommand::Body::useShuffle   )
   .def_readwrite("leftArmLimp"   , &ActionCommand::Body::leftArmLimp  )
   .def_readwrite("rightArmLimp"  , &ActionCommand::Body::rightArmLimp );

class_<ActionCommand::Head>("HeadCommand")
   .def(init<float, float, bool, float, float>()                       )
   .def_readwrite("yaw"           , &ActionCommand::Head::yaw          )
   .def_readwrite("pitch"         , &ActionCommand::Head::pitch        )
   .def_readwrite("isRelative"    , &ActionCommand::Head::isRelative   )
   .def_readwrite("yawSpeed"      , &ActionCommand::Head::yawSpeed     )
   .def_readwrite("pitchSpeed"    , &ActionCommand::Head::pitchSpeed   );

class_<ActionCommand::rgb>("rgb")
   .def(init<bool, bool, bool>()                                       )
   .def_readwrite("red"           , &ActionCommand::rgb::red           )
   .def_readwrite("green"         , &ActionCommand::rgb::green         )
   .def_readwrite("blue"          , &ActionCommand::rgb::blue          );

class_<ActionCommand::LED>("LEDCommand")
   .def(init<ActionCommand::rgb,
             ActionCommand::rgb,
             ActionCommand::rgb,
             ActionCommand::rgb,
             ActionCommand::rgb
            >())

   .def_readwrite("leftEar"       , &ActionCommand::LED::leftEar       )
   .def_readwrite("rightEar"      , &ActionCommand::LED::rightEar      )
   .def_readwrite("leftEye"       , &ActionCommand::LED::leftEye       )
   .def_readwrite("rightEye"      , &ActionCommand::LED::rightEye      )
   .def_readwrite("chestButton"   , &ActionCommand::LED::chestButton   )
   .def_readwrite("leftFoot"      , &ActionCommand::LED::leftFoot      )
   .def_readwrite("rightFoot"     , &ActionCommand::LED::rightFoot     );

enum_<ActionCommand::Stiffen>("StiffenCommand")
   .value("STIFFEN"         , ActionCommand::STIFFEN      )
   .value("NONE"            , ActionCommand::NONE         );

class_<ActionCommand::All>("All")
   .def(init<ActionCommand::Head, ActionCommand::Body, ActionCommand::LED, float, ActionCommand::Stiffen>())

   .def_readwrite("head"          , &ActionCommand::All::head          )
   .def_readwrite("body"          , &ActionCommand::All::body          )
   .def_readwrite("leds"          , &ActionCommand::All::leds          )
   .def_readwrite("sonar"         , &ActionCommand::All::sonar         )
   .def_readwrite("stiffen"       , &ActionCommand::All::stiffen       );

enum_<ActionCommand::Body::ActionType>("ActionType")
   .value("NONE"                     , ActionCommand::Body::NONE                     )
   .value("STAND"                    , ActionCommand::Body::STAND                    )
   .value("WALK"                     , ActionCommand::Body::WALK                     )
   .value("DRIBBLE"                  , ActionCommand::Body::DRIBBLE                  )
   .value("TURN_DRIBBLE"             , ActionCommand::Body::TURN_DRIBBLE             )
   .value("GETUP_FRONT"              , ActionCommand::Body::GETUP_FRONT              )
   .value("GETUP_BACK"               , ActionCommand::Body::GETUP_BACK               )
   .value("TIP_OVER"                 , ActionCommand::Body::TIP_OVER                 )
   .value("KICK"                     , ActionCommand::Body::KICK                     )
   .value("INITIAL"                  , ActionCommand::Body::INITIAL                  )
   .value("DEAD"                     , ActionCommand::Body::DEAD                     )
   .value("REF_PICKUP"               , ActionCommand::Body::REF_PICKUP               )
   .value("GOALIE_SIT"               , ActionCommand::Body::GOALIE_SIT               )
   .value("GOALIE_DIVE_RIGHT"        , ActionCommand::Body::GOALIE_DIVE_RIGHT        )
   .value("GOALIE_DIVE_LEFT"         , ActionCommand::Body::GOALIE_DIVE_LEFT         )
   .value("GOALIE_CENTRE"            , ActionCommand::Body::GOALIE_CENTRE            )
   .value("GOALIE_UNCENTRE"          , ActionCommand::Body::GOALIE_UNCENTRE          )
   .value("GOALIE_INITIAL"           , ActionCommand::Body::GOALIE_INITIAL           )
   .value("GOALIE_AFTERSIT_INITIAL"  , ActionCommand::Body::GOALIE_AFTERSIT_INITIAL  )
   .value("MOTION_CALIBRATE"         , ActionCommand::Body::MOTION_CALIBRATE         )
   .value("STAND_STRAIGHT"           , ActionCommand::Body::STAND_STRAIGHT           )
   .value("DEFENDER_CENTRE"          , ActionCommand::Body::DEFENDER_CENTRE          )
   .value("LINE_UP"                  , ActionCommand::Body::LINE_UP                  )
   .value("NUM_ACTION_TYPES"         , ActionCommand::Body::NUM_ACTION_TYPES         );

enum_<ActionCommand::Body::Foot>("Foot")
   .value("LEFT"                     , ActionCommand::Body::LEFT                     )
   .value("RIGHT"                    , ActionCommand::Body::RIGHT                    );
