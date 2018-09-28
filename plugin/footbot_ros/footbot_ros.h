/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example controller to manually control the foot-bot.

 * To control a foot-bot, you need to Shift-Click on it in the OpenGL
 * visualization to select it.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/footbot_ros.argos
 */

#ifndef FOOTBOT_ROS_H
#define FOOTBOT_ROS_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

#include <ros/ros.h>
#include <string>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotros : public CCI_Controller {

public:

   struct SWheelTurningParams {
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;

      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

public:

   /* Class constructor. */
   CFootBotros();

   /* Class destructor. */
   virtual ~CFootBotros() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}

   virtual void Destroy() {}

   void Select();

   void Deselect();

   void SetControlVector(const CVector2& c_control);

   void cmdVelCallback(const geometry_msgs::Twist& twist);

   void GoalCallback(const geometry_msgs::Twist& twist);

protected:

   void SetWheelSpeedsFromVector(const CVector2& c_heading);

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_FootBotProximitySensor* m_pcProximity;
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcOmniCam;

   static constexpr Real HALF_BASELINE = 0.07f; // Half the distance between wheels
   static constexpr Real WHEEL_RADIUS = 0.029112741f;

   int stopWithoutSubscriberCount;

   int stepsSinceCallback;

   Real leftSpeed, rightSpeed;

   ros::Publisher puckListPub;

   ros::Publisher proximityPub;

   ros::Subscriber GoalSub;

   ros::Subscriber cmdVelSub;

   CCI_LEDsActuator* m_pcLEDs;

   SWheelTurningParams m_sWheelTurningParams;

   bool m_bSelected;

   CVector2 m_cControl;

public:
   static ros::NodeHandle* nodeHandle;
};

#endif
