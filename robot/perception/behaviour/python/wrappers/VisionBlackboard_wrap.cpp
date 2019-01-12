class_<VisionBlackboard>("VisionBlackboard")
   .add_property("balls"    , &VisionBlackboard::balls    )
   .add_property("uncertain_balls"    , &VisionBlackboard::uncertain_balls    )
   .add_property("posts"    , &VisionBlackboard::posts    )
   .add_property("timestamp", &VisionBlackboard::timestamp)
   .add_property("homeMapSize"    , &VisionBlackboard::homeMapSize)
   .add_property("ballHint" , &VisionBlackboard::ballHint );
