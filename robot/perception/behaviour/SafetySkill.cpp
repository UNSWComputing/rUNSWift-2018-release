#include "perception/behaviour/SafetySkill.hpp"
#include "utils/basic_maths.hpp"
#include "utils/Logger.hpp"

SafetySkill::SafetySkill(bool use_getups) {
   useGetups = use_getups;
   filtered_fsr_sum = 5.0;  // assume we start standing
   blink = 0;
   cor_angular_velocity = 0;
   sag_angular_velocity = 0;
   prev_angles[0] = 0;
   prev_angles[1] = 0;
   llog(INFO) << "SafetySkill constructed" << std::endl;
}

SafetySkill::~SafetySkill() {
   llog(INFO) << "SafetySkill destroyed" << std::endl;
}

BehaviourRequest SafetySkill::wrapRequest(const BehaviourRequest &request, const SensorValues &s) {
	BehaviourRequest r = request;
	if(r.actions.body.actionType == ActionCommand::Body::MOTION_CALIBRATE) {
	   return r;
	}

	float fsr_sum = s.sensors[Sensors::LFoot_FSR_FrontLeft]
	               + s.sensors[Sensors::LFoot_FSR_FrontRight]
	               + s.sensors[Sensors::LFoot_FSR_RearLeft]
	               + s.sensors[Sensors::LFoot_FSR_RearRight]
	               + s.sensors[Sensors::RFoot_FSR_FrontLeft]
	               + s.sensors[Sensors::RFoot_FSR_FrontRight]
	               + s.sensors[Sensors::RFoot_FSR_RearLeft]
	               + s.sensors[Sensors::RFoot_FSR_RearRight];
	filtered_fsr_sum = filtered_fsr_sum + 0.2 * (fsr_sum - filtered_fsr_sum);
	blink = !blink;
	float ang[2] = {RAD2DEG(s.sensors[Sensors::InertialSensor_AngleX]),
	               RAD2DEG(s.sensors[Sensors::InertialSensor_AngleY])};

	// choose a safety skill
	if(ang[1] < -FALLEN_ANG) {
	  r.actions.leds.leftEye = ActionCommand::rgb(blink, 0);
	  r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
	  r.actions.body = (useGetups) ? ActionCommand::Body::GETUP_BACK : ActionCommand::Body::DEAD;
	} else if(ang[1] > FALLEN_ANG) {
	  r.actions.leds.leftEye = ActionCommand::rgb(blink, 0);
	  r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
	  r.actions.body = (useGetups) ? ActionCommand::Body::GETUP_FRONT : ActionCommand::Body::DEAD;
	} else if (ang[0] > FALLEN_ANG || ang[0] < -FALLEN_ANG) {
	  r.actions.leds.leftEye = ActionCommand::rgb(blink, 0);
	  r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
	  r.actions.body = ActionCommand::Body::TIP_OVER;
	} else if (ABS(ang[0]) > FALLING_ANG || ABS(ang[1]) > FALLING_ANG) {
	  r.actions.leds.leftEye = ActionCommand::rgb(blink, blink);
	  r.actions.leds.rightEye = ActionCommand::rgb(blink, blink);
	  r.actions.body = ActionCommand::Body::DEAD;
	} else if (filtered_fsr_sum < MIN_STANDING_WEIGHT) {
	  r.actions.leds.leftEye = ActionCommand::rgb(0, blink);
	  r.actions.leds.rightEye = ActionCommand::rgb(0, blink);
	  r.actions.body = ActionCommand::Body::REF_PICKUP;
	}

	return r;
}
