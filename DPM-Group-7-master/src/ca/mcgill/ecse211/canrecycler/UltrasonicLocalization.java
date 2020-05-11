package ca.mcgill.ecse211.canrecycler;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;

/**
 * This class localizes the robot's orientation using the ultrasonic sensor. It provides support for
 * both the falling edge and the rising edge methods in order to determine orientation and set the
 * robot's direction to the absolute 0.
 * 
 * @author Cyril Yared
 * @author Ayoub Bouhemhem
 * @author MJ Tucker
 *
 */
public class UltrasonicLocalization {

  /**
   * Threshold for ultrasonic sensor at which it takes marking.
   */
  public final static int THRESHOLD = 28;

  /**
   * Noise allowed for ultrasonic sensor when it takes marking.
   */
  public final static int ERROR = 2;

  /**
   * Orientation of corner of wall with respect to cardinal directions.
   */
  public final static int WALL_CORNER_ABS_ANGLE = 225;

  /**
   * Orientation of corner of wall with respect to cardinal directions.
   */
  public final static int VOID_MIDDLE_ABS_ANGLE = 45;

  /**
   * Speed at which motors rotate when turning when localizing with ultrasonic sensor (deg/sec).
   */
  public static final int US_ROTATE_SPEED = 170;


  /**
   * Uses rising edge technique in order to calibrate orientation of robot. If robot starts away
   * from wall, turns towards wall. Then uses two thresholds of rising wall in order to set
   * basepoint. Finally, calculates center of the two basepoints in order to calculate offset and
   * determine final orientation.
   * 
   * @throws OdometerExceptions Exception caused by Odometer.
   * @throws InterruptedException Exception caused by Thread sleeping.
   */
  public void risingEdge() throws OdometerExceptions, InterruptedException {

    // Get instance of odometer
    Odometer odometer = Odometer.getOdometer();
    RobotMovement.setRotateSpeed(US_ROTATE_SPEED);

    double orientationFirst; // Orientation of first marking based on current odometer readings
    double orientationSecond; // Orientation of second marking based on current odometer readings

    // We want to start with the ultrasonic sensor pointing into the wall to get the rising edge
    if (Controller.usDistance > THRESHOLD + ERROR) {
      RobotMovement.turnRight();

      // Case where ultrasonic sensor not pointing towards wall, if finally pointing towards wall
      // stops
      while (Controller.usDistance > THRESHOLD + ERROR) {
        Thread.sleep(30);
      }
      RobotMovement.stop();
      RobotMovement.setRotateSpeed(US_ROTATE_SPEED);

      // Turns right to even more to start more directly pointed towards the wall to get the first
      // readings
      RobotMovement.turnByDegree(45);
    }
    // Starts turning right to get rising wall readings
    RobotMovement.turnRight();

    // First baseline set
    orientationFirst = getRisingOrientationMarking(true, odometer);
    if (CanRecycler.DEBUG) {
      Sound.beep();
      System.out.println("First set: " + odometer.getXYT()[2] + " - " + orientationFirst);
    }

    // Flips direction to get other rising wall
    RobotMovement.turnLeft();
    Thread.sleep(2000); // Sleeps in order to avoid false readings

    // Second baseline set
    orientationSecond = getRisingOrientationMarking(false, odometer);
    if (CanRecycler.DEBUG) {
      Sound.twoBeeps();
      System.out.println("Second set: " + odometer.getXYT()[2] + " - " + orientationSecond);
    }

    // Angle to center from edge
    double centerAngle;

    // Determines center orientation of both baselines
    if (orientationFirst > orientationSecond) { // In order to avoid problems with negative angles
      centerAngle = ((orientationFirst - orientationSecond) / 2.0 + orientationSecond) % 360;
    } else {
      centerAngle = ((orientationFirst + 360 - orientationSecond) / 2.0 + orientationSecond) % 360;
    }

    if (CanRecycler.DEBUG) {
      System.out.println("Center angle: " + centerAngle);
    }


    // Calculates difference between the corner of the bricks and current orientation
    double difference = RobotMovement.bestDirTurn(odometer.getXYT()[2], centerAngle);

    // Sets theta, based on the fact that lower corner of bricks at specified angle
    odometer.setTheta((WALL_CORNER_ABS_ANGLE - difference + 360) % 360);
    if (CanRecycler.DEBUG) {
      System.out.println("New Theta at 2: " + odometer.getXYT()[2]);
    }

    // Turn to 0 at end
    RobotMovement.turnByDegree(RobotMovement.bestDirTurn(odometer.getXYT()[2], 0));

    if (CanRecycler.DEBUG) {
      System.out.println("Theta at 0: " + odometer.getXYT()[2]);
    }
  }

  /**
   * Uses falling edge technique in order to calibrate orientation of robot. If robot starts towards
   * wall, turns away from wall. Then uses two thresholds of falling wall in order to set basepoint.
   * Finally, calculates center of the two basepoints in order to calculate offset and determine
   * final orientation.
   * 
   * @throws OdometerExceptions Exception caused by Odometer.
   * @throws InterruptedException Exception caused by Thread sleeping.
   */
  public void fallingEdge() throws OdometerExceptions, InterruptedException {

    // Get instance of odometer
    Odometer odometer = Odometer.getOdometer();
    RobotMovement.setRotateSpeed(US_ROTATE_SPEED);

    double orientationFirst; // Orientation of first marking based on current odometer readings
    double orientationSecond; // Orientation of second marking based on current odometer readings

    // We want to start with the ultrasonic sensor pointing into the void to get the rising edge
    if (Controller.usDistance < THRESHOLD + ERROR) {
      RobotMovement.turnRight();

      // Case where ultrasonic sensor not pointing away from wall, if finally pointing away from
      // wall, stops
      while (Controller.usDistance < THRESHOLD + ERROR) {
        Thread.sleep(30);
      }
      RobotMovement.stop();
      RobotMovement.setRotateSpeed(US_ROTATE_SPEED);

      // Turns left to even more to start more directly pointed towards the wall to get the first
      // readings
      RobotMovement.turnByDegree(45);
    }
    // Starts turning right to get falling wall readings
    RobotMovement.turnRight();

    // Sets first baseline
    orientationFirst = getFallingOrientationMarking(true, odometer);

    if (CanRecycler.DEBUG) {
      Sound.beep();
      System.out.println("First set: " + odometer.getXYT()[2] + " - " + orientationFirst);
    }

    // Turns left to get other falling wall reading
    RobotMovement.turnLeft();
    Thread.sleep(2000); // Sleeps in order to avoid false readings

    // Sets second basline
    orientationSecond = getFallingOrientationMarking(false, odometer);
    if (CanRecycler.DEBUG) {
      Sound.twoBeeps();
      System.out.println("Second set: " + odometer.getXYT()[2] + " - " + orientationSecond);
    }

    // Angle to center from edge
    double centerAngle;

    // Determines center orientation of both baselines
    if (orientationFirst > orientationSecond) { // In order to avoid problems with negative angles
      centerAngle = ((orientationFirst - orientationSecond) / 2.0 + orientationSecond) % 360;
    } else {
      centerAngle = ((orientationFirst + 360 - orientationSecond) / 2.0 + orientationSecond) % 360;
    }

    if (CanRecycler.DEBUG) {
      System.out.println("Center angle: " + centerAngle);
    }


    // Calculates difference between the corner of the bricks and current orientation
    double difference = RobotMovement.bestDirTurn(odometer.getXYT()[2], centerAngle);

    // Sets theta, based on the fact that lower corner of bricks at specified angle
    odometer.setTheta((VOID_MIDDLE_ABS_ANGLE - difference + 360) % 360);

    if (CanRecycler.DEBUG) {
      System.out.println("New Theta: " + odometer.getXYT()[2]);
    }

    // Turn to 0
    RobotMovement.turnByDegree(RobotMovement.bestDirTurn(odometer.getXYT()[2], 0));

    if (CanRecycler.DEBUG) {
      System.out.println("Theta at 0: " + odometer.getXYT()[2]);
    }
  }

  /**
   * Returns the approximate orientation of the robot at the point that the ultrasonic sensor
   * detects that it is at a specified threshold from the blocks. In order to improve accuracy, it
   * takes the average of two readings. This is for the rising edge technique.
   * 
   * @param isFirstMarking boolean for if this is the first or second marking.
   * @param odometer Odometer instance.
   * @return double for approximate orientation of robot.
   * @throws InterruptedException caused by Thread sleep.
   */
  private double getRisingOrientationMarking(boolean isFirstMarking, Odometer odometer)
      throws InterruptedException {

    double lowerThresholdOrientation; // Orientation of lower threshold based on current odometer
                                      // readings
    double upperThresholdOrientation; // Orientation of upper threshold based on current odometer
                                      // readings
    // Gets reading of lower point of threshold
    while (Controller.usDistance < THRESHOLD - ERROR) {
      Thread.sleep(30);
    }
    lowerThresholdOrientation = odometer.getXYT()[2];
    // Gets reading of higher point of threshold
    while (Controller.usDistance < THRESHOLD + ERROR) {
      Thread.sleep(30);
    }
    RobotMovement.stop();
    RobotMovement.setRotateSpeed(US_ROTATE_SPEED);
    upperThresholdOrientation = odometer.getXYT()[2];
    // Calculates approximate orientation of robot when pointing towards marking
    if (isFirstMarking) {
      return (RobotMovement.bestDirTurn(lowerThresholdOrientation, upperThresholdOrientation) / 2.0
          + lowerThresholdOrientation) % 360;
    } else {
      return (RobotMovement.bestDirTurn(upperThresholdOrientation, lowerThresholdOrientation) / 2.0
          + upperThresholdOrientation) % 360;
    }
  }

  /**
   * Returns the approximate orientation of the robot at the point that the ultrasonic sensor
   * detects that it is at a specified threshold from the blocks. In order to improve accuracy, it
   * takes the average of two readings. This is for the falling edge technique.
   * 
   * @param isFirstMarking boolean for if this is the first or second marking.
   * @param odometer Odometer instance.
   * @return Approximate orientation of robot (degrees).
   * @throws InterruptedException If thread sleeping fails.
   */
  private double getFallingOrientationMarking(boolean isFirstMarking, Odometer odometer)
      throws InterruptedException {

    double lowerThresholdOrientation; // Orientation of lower threshold based on current odometer
                                      // readings
    double upperThresholdOrientation; // Orientation of upper threshold based on current odometer
                                      // readings
    // Gets reading of lower point of threshold
    while (Controller.usDistance > THRESHOLD + ERROR) {
      Thread.sleep(30);
    }
    lowerThresholdOrientation = odometer.getXYT()[2];
    // Gets reading of higher point of threshold
    while (Controller.usDistance > THRESHOLD - ERROR) {
      Thread.sleep(30);
    }
    RobotMovement.stop();
    RobotMovement.setRotateSpeed(US_ROTATE_SPEED);
    upperThresholdOrientation = odometer.getXYT()[2];
    // Calculates approximate orientation of robot when pointing towards marking
    if (isFirstMarking) {
      return (RobotMovement.bestDirTurn(lowerThresholdOrientation, upperThresholdOrientation) / 2.0
          + lowerThresholdOrientation) % 360;
    } else {
      return (RobotMovement.bestDirTurn(upperThresholdOrientation, lowerThresholdOrientation) / 2.0
          + upperThresholdOrientation) % 360;
    }
  }
}
