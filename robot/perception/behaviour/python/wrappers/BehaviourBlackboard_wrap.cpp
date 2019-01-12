class_<BehaviourBlackboard>("BehaviourBlackboard")
   .def_readonly("skill"              , &BehaviourBlackboard::skill)
   .def_readonly("behaviourSharedData", &BehaviourBlackboard::behaviourSharedData)
   .def_readonly("remoteStiffen", &BehaviourBlackboard::remoteStiffen);
