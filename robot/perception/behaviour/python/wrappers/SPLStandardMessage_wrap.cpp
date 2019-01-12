class_<SPLStandardMessage>("SPLStandardMessage")
    .def_readonly("playerNum", &SPLStandardMessage::playerNum)
    .def_readonly("fallen", &SPLStandardMessage::fallen)
    .def_readonly("pose", &SPLStandardMessage::pose)
    .def_readonly("ballAge", &SPLStandardMessage::ballAge)
    .def_readonly("ball", &SPLStandardMessage::ball);
