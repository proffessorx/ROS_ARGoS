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
/* CCI_Controller class definition */
#include <argos3/core/control_interface/ci_controller.h>
/* differential steering actuator definition */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* LEDs actuator definition */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>

/* Sensors definitions */
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

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_ros_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
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

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_ros_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy() {}

   /*
    * Sets the selected flag on this robot.
    * When selected, a robot follows the control vector.
    */
   void Select();

   /*
    * Unsets the selected flag on this robot.
    * When unselected, a robot stays still.
    */
   void Deselect();

   /*
    * Sets the control vector.
    */
   void SetControlVector(const CVector2& c_control);

  /*
   * The callback method for getting new commanded speed on the cmd_vel topic.
   */
  void cmdVelCallback(const geometry_msgs::Twist& twist);

  void GoalCallback(const geometry_msgs::Twist& twist);


protected:

   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

private:

   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;

   /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;

   /* Flag to know whether this robot is selected */
   bool m_bSelected;

   /* The control vector */
   CVector2 m_cControl;

  CCI_DifferentialSteeringActuator* m_pcWheels;
  CCI_FootBotProximitySensor* m_pcProximity;
  CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcOmniCam;
//  CCI_FootBotGripperActuator* m_pcGripper;

  // The following constant values were copied from the argos source tree from
  // the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
  //const changed to constexpr in below two lines..
  static constexpr Real HALF_BASELINE = 0.07f; // Half the distance between wheels
  static constexpr Real WHEEL_RADIUS = 0.029112741f;
  
  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><argos_ros_bot_controller> section.
   */

  // The number of time steps from the time step of the last callback
  // after which leftSpeed and rightSpeed will be set to zero.  Useful to
  // shutdown the robot after the controlling code on the ROS side has quit.
  int stopWithoutSubscriberCount;

  // The number of time steps since the last callback.
  int stepsSinceCallback;

  // Most recent left and right wheel speeds, converted from the ROS twist
  // message.
  Real leftSpeed, rightSpeed;

  // The state of the gripper.
//  bool gripping;

  // Puck list publisher
  ros::Publisher puckListPub;

  // Proximity sensor publisher
  ros::Publisher proximityPub;

  // Goal list Subscriber
  ros::Subscriber GoalSub;

  // Subscriber for cmd_vel (Twist message) topic.
  ros::Subscriber cmdVelSub;

  // Subscriber for gripper (Bool message) topic.
//  ros::Subscriber gripperSub;

public:
  // We need only a single ROS node, although there are individual publishers
  // and subscribers for each instance of the class.
  static ros::NodeHandle* nodeHandle;

};

#endif
