package ca.mcgill.ecse211.canrecycler;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The RobotMovement class provides static public methods in order to move the robot with the two
 * mounted wheels used for navigation, including methods to convert angles and distances, move the
 * robot and find the best turn direction, set rotate and acceleration speeds of the motors, 
 * customize beeps, and initialize the can grasping claw.
 * 
 * @author Cyril Yared
 * @author MJ Tucker
 *
 */
public class RobotMovement {
  /**
   * EV3LargeRegulatedMotor Controls the left motor in port D.
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * EV3LargeRegulatedMotor Controls the right motor in port A.
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * Radius of wheels and tires (cm).
   */
  public static double WHEEL_RAD = 2.066;

  /**
   * Wheel base of robot measured from center of both tires (cm).
   */
  public static double TRACK = 12.94;

  /**
   * Wheel base of robot measured from center of both tires without a can (cm).
   */
  public static final double DEFAULT_WHEEL_TRACK = 12.94;

  /**
   * Wheel base of robot measured from center of both tires with a can (cm).
   */
  public static final double HEAVY_WHEEL_TRACK = 13.13;

  /**
   * Speed at which motors rotate when advancing (deg/sec).
   */
  public static final int FORWARD_SPEED = 250;
  
  /**
   * Speed at which motors rotate when advancing through tunnel (deg/sec).
   */
  public static final int FORWARD_TUNNEL_SPEED = 325;
  
  /**
   * Speed at which motors accelerate (deg/sec^2).
   */
  public static final int ACCELERATION = 2000;

  /**
   * Motor speed while the robot is rotating at turns during normal navigation (deg/sec).
   */
  public static final int ROTATE_SPEED = 130;

  /**
   * Size of tiles (cm).
   */
  public final static double TILE_SIZE = 30;

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius Radius of wheels in centimeters.
   * @param distance Distance to cover in centimeters.
   * @return Number of degrees that motor should rotate to proceed forward specified distance.
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * Finds number of degrees needed to rotate motor to achieve specified turn (assumes two motors
   * turning in opposite direction).
   * 
   * @param radius Radius of wheels in centimeters.
   * @param width Wheelbase of robot in centimeters.
   * @param angle Double for angle required to turn in degrees.
   * @return Integer for number of degrees that motor should rotate to achieve turn.
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * Turns robot by the degree specified in parameter. Positive for right turn, negative for left
   * turn.
   * 
   * @param degree Degree to turn by, positive for right turn, negative for left turn.
   */
  public static void turnByDegree(double degree) {
    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, degree), true);
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, degree), false);
  }

  /**
   * Moves robot forward by a specified number of centimeters. Positive for forward, negative for
   * backwards.
   * 
   * @param distance Distance to turn in centimeters.
   */
  public static void move(double distance) {
    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, distance), true);
    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, distance), false);
  }

  /**
   * Sets speed of motors to rotation speed.
   * 
   * @param rotateSpeed How fast to rotate in (deg/sec).
   */
  public static void setRotateSpeed(int rotateSpeed) {
    LEFT_MOTOR.setSpeed(rotateSpeed);
    RIGHT_MOTOR.setSpeed(rotateSpeed);
  }
  
  /**
   * Sets acceleration of motors.
   * 
   * @param acc How fast to accelerate in (deg/sec^2).
   */
  public static void setForwardAcceleration(int acc) {
    LEFT_MOTOR.setAcceleration(acc);
    RIGHT_MOTOR.setAcceleration(acc);
  }


  /**
   * Robot turns right at specified speed.
   */
  public static void turnRight() {
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.backward();
  }

  /**
   * Robot turns left at specified speed.
   */
  public static void turnLeft() {
    LEFT_MOTOR.backward();
    RIGHT_MOTOR.forward();
  }

  /**
   * Robot stops.
   */
  public static void stop() {
    LEFT_MOTOR.setSpeed(0);
    RIGHT_MOTOR.setSpeed(0);
    LEFT_MOTOR.stop();
    RIGHT_MOTOR.stop();
  }

  /**
   * Returns the best direction to turn (-180 to 180 degrees) to get from startAngle to targetAngle.
   * 
   * @param startAngle Integer representing starting orientation in degrees.
   * @param targetAngle Integer representing target orientation in degrees.
   * @return Integer for best direction to turn in degrees.
   */
  public static double bestDirTurn(double startAngle, double targetAngle) {
    double delta = (targetAngle - startAngle + 540) % 360 - 180;
    return delta;
  }

  /**
   * Beeps number amount of times.
   * 
   * @param number How many times to beep.
   * @param milliseconds Number of milliseconds to beep for.
   */
  public static void beep(int number, int milliseconds) {
    for (int i = 0; i < number; i++) {
      Sound.playTone(300, milliseconds);
      try {
        Thread.sleep(150);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    return;
  }

  /**
   * Stows arm on back of robot for navigation.
   */
  public static void canGraspInitialization() {
    Controller.CAN_MOTOR.setSpeed(40);
    Controller.CAN_MOTOR.rotate(-70);
    Controller.CAN_MOTOR.stop();

  }

}

