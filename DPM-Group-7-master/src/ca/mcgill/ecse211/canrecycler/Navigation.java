package ca.mcgill.ecse211.canrecycler;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * This class is used to calculate the shortest path to a specific WayPoint given current odometry
 * position and orientation as well as the location of the next WayPoint. It also controls the drive
 * motors to reach the next WayPoint. The Navigation class also implements object detection with the
 * Ultrasonic Sensor, and contains a method that invokes color and weight identification in order to
 * determine a cans attributes, and then further determines if the can will be collected or moved
 * out of the way depending on its total point value.
 * 
 * @author Cyril Yared
 * @author MJ Tucker
 * @author Donya Hojabr
 */
public class Navigation {

  /**
   * True if an object has been detected.
   */
  public static boolean objectDetected = false;

  /**
   * Distance threshold for ultrasonic sensor to detect can.
   */
  public static int DISTANCE_THRESH = 5;

  /**
   * Distance to move forward after an object has been detected.
   */
  public static int MOVE_FORWARD_AFTER_DETECTION = 10;

  /**
   * Distance to move backwards when dodging can.
   */
  public static int DODGE_CAN_MOVE_BACK = 10;

  /**
   * Distance to move forwards when dodging can.
   */
  public static int DODGE_CAN_MOVE_FORWARD = 40;

  /**
   * Distance to move laterally when dodging can.
   */
  public static int DODGE_CAN_MOVE_LATERAL = 15;

  /**
   * When reach threshold, will pickup can regardless of color.
   */
  public static int CAN_PICKUP_THRESHOLD = 0;

  /**
   * This method calculates that path and navigates the robot to the next WayPoint based on the
   * location of the next WayPoint and the current odometry position and orientation. Implements
   * obstacle navigation.
   * 
   * @param waypoint Defines the position of the next WayPoint for where to navigate.
   * @param detectCan If true, robot searches for cans while navigating.
   * @param inSearchZone True if navigating in search zone, false otherwise.
   * @throws Exception General exception that may be caused by Thread.sleep.
   */
  public static void navigateToWaypoint(WayPoint waypoint, boolean detectCan, boolean inSearchZone)
      throws Exception {

    // Get instance of odometer
    Odometer odoData = Odometer.getOdometer();

    // Finds the difference in x and y between previous WayPoint and next WayPoint
    double xDelta = waypoint.x - odoData.getXYT()[0] / RobotMovement.TILE_SIZE;
    double yDelta = waypoint.y - odoData.getXYT()[1] / RobotMovement.TILE_SIZE;;

    // Calculates distance to travel to reach next WayPoint
    double distance = Math.sqrt(xDelta * xDelta + yDelta * yDelta);

    // Calculates how much to turn by to reach next WayPoint
    double turnAngle = calculateTurn(yDelta, xDelta);

    // Turn to appropriate heading
    RobotMovement.setRotateSpeed(RobotMovement.ROTATE_SPEED);
    RobotMovement.turnByDegree(turnAngle);

    // Go forwards until reach next WayPoint, set speeds depending on location
    if (inSearchZone) {
      RobotMovement.setRotateSpeed(RobotMovement.FORWARD_SPEED);
    } else {
      RobotMovement.setForwardAcceleration(RobotMovement.ACCELERATION);
      RobotMovement.setRotateSpeed(RobotMovement.FORWARD_TUNNEL_SPEED);
    }
    RobotMovement.LEFT_MOTOR.rotate(RobotMovement.convertDistance(RobotMovement.WHEEL_RAD, distance)
        * (int) RobotMovement.TILE_SIZE, true);
    // We call rotate with detectCan as parameter, if true, returns immediately and executes while
    // loop below
    // Otherwise, finishes executing movement and breaks from method
    RobotMovement.RIGHT_MOTOR
        .rotate(RobotMovement.convertDistance(RobotMovement.WHEEL_RAD, distance)
            * (int) RobotMovement.TILE_SIZE, detectCan);

    if (detectCan) {
      // while the robot is navigating, must be checked since we returned from method
      while (isNavigating()) {

        if (Controller.usDistance <= DISTANCE_THRESH) { // object has been detected
          objectDetected = true;

          // Set rotate speed to lower speed to center can
          Thread.sleep(30);
          RobotMovement.setRotateSpeed(RobotMovement.ROTATE_SPEED);


          // Move forward, getting can in correct position for colour detection
          RobotMovement.move(MOVE_FORWARD_AFTER_DETECTION);
          RobotMovement.setRotateSpeed(RobotMovement.FORWARD_SPEED);

          determineCanAttributes();
        }
      }
    }
  }

  /**
   * Determines the color and weight of a can. Makes decision based on data. If finds blue or light
   * green can, disposes it by lifting can and rotating robot 180 degrees to move can out of way.
   * Otherwise, picks up can and returns to home zone to deposit can. If checked specified number of
   * cans already (CAN_PICKUP_THRESHOLD), will pick up next can regardless of color or weight.
   */
  private static void determineCanAttributes() {
    try {
      int colour = ColorIdentification.detectColor();
      if (colour == 0) { // Case where no color has been detected
        colour = ColorIdentification.detectColor(); // Try it once more
        if (colour == 0) { // No can detected, continue navigating
          return;
        }
      }

      // Prepares to lift up can
      Controller.canGraspSetUp();
      Controller.CAN_MOTOR.flt();
      // Must close since creating unregulated motor in WeightIdentification
      Controller.CAN_MOTOR.close();

      int weight = WeightIdentification.isHeavyCan();

      // If heavy can, then must adjust wheel track, beeps accordingly
      if (weight == 1) {
        RobotMovement.TRACK = RobotMovement.HEAVY_WHEEL_TRACK;
        RobotMovement.beep(colour, 1000);
      } else {
        RobotMovement.beep(colour, 500);
      }

      // Green and blue light can and detected cans is less than threshold
      if ((colour == 1) && Controller.canList.size() < CAN_PICKUP_THRESHOLD) {
        // In this case, will pick up can, flip robot 180 degrees and drop can
        Controller.pickUpCan();

        // Flip 180 degrees to move can out of way and backs up
        RobotMovement.turnByDegree(180);
        RobotMovement.move(-8); // Move out of can area

        Controller.canList.add(new CanMarker(Odometer.getOdometer().getXYT()[0],
            Odometer.getOdometer().getXYT()[1], colour, 0));

        // Drop can
        Controller.CAN_MOTOR.setSpeed(40);
        Controller.CAN_MOTOR.rotate(65);
        Controller.CAN_MOTOR.rotate(45);
        Controller.CAN_MOTOR.stop();

        RobotMovement.move(8); // Move out of can area
        Controller.CAN_MOTOR.rotate(-45); // Close the claws
        Navigation.objectDetected = false;

        // Move claw back up
        RobotMovement.canGraspInitialization();
        RobotMovement.TRACK = RobotMovement.DEFAULT_WHEEL_TRACK;

      } else {
        Controller.targetFound = true;
        Controller.pickUpCan();
      }
    } catch (Exception e) {
      e.printStackTrace();
    }

  }

  /**
   * Returns degree by which robot must turn to reach next WayPoint.
   * 
   * @param yDelta Number of tiles to traverse in y-vector.
   * @param xDelta Number of tiles to traverse in x-vector.
   * @return double for degree by which robot must turn to reach next WayPoint (negative is
   *         counterclockwise, positive is clockwise).
   * @throws OdometerExceptions Exception handled in OdometerExceptions class.
   */
  private static double calculateTurn(double yDelta, double xDelta) throws OdometerExceptions {
    // Get instance of odometer, since singleton, equivalent to just putting in method argument
    Odometer odoData = Odometer.getOdometer();

    double newTheta = Math.atan2(yDelta, xDelta); // Returns angle in radians to get from origin
    // (0,0) to coordinate position
    newTheta = newTheta * 180 / Math.PI; // Convert new theta to degrees

    newTheta = -newTheta + 90; // Converts from unit circle orientation to cardinal bearing
    // (counterclockwise to clockwise)
    // Adding 90 changes from unit circle 0 degree position to
    // cardinal direction 0 degree position

    if (newTheta < 0) { // Ensures new theta within 0 to 360 range
      newTheta = 360 + newTheta;
    }
    double curTheta = ((odoData.getXYT()[2])); // Current theta

    double turnAngle = RobotMovement.bestDirTurn(curTheta, newTheta); // Calculated angle needed to
                                                                      // turn to

    return turnAngle;
  }

  /**
   * Calculates absolute orientation to waypoint in degrees.
   * 
   * @param waypoint Waypoint that robot should travel to.
   * @return Orientation in degrees from current position to waypoint.
   */
  public static double calculateTurnToWaypoint(WayPoint waypoint) {
    try {
      // Finds the difference in x and y between previous WayPoint and next WayPoint
      double xDelta = waypoint.x - Odometer.getOdometer().getXYT()[0] / RobotMovement.TILE_SIZE;
      double yDelta = waypoint.y - Odometer.getOdometer().getXYT()[1] / RobotMovement.TILE_SIZE;
      double turn = calculateTurn(yDelta, xDelta);
      double absAngle = turn + Odometer.getOdometer().getXYT()[2];
      if (absAngle < 0) {
        absAngle += 360;
      }
      absAngle = absAngle % 360;
      return absAngle;
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    return 0;
  }

  /**
   * Returns if robot is moving or not.
   * 
   * @return boolean true if moving, false if not.
   */
  private static boolean isNavigating() {
    if (RobotMovement.LEFT_MOTOR.isMoving() || RobotMovement.RIGHT_MOTOR.isMoving()) {
      return true;
    } else {
      return false;
    }
  }
}
