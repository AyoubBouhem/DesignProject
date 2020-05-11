package ca.mcgill.ecse211.canrecycler;

import java.util.ArrayList;
import java.util.List;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3MediumRegulatedMotor;


/**
 * This class controls the flow of execution of the program after the sensors and actuators have
 * been initialized and the robot has localized. It handles navigating to and from the search zone,
 * relocalization, identifying cans and determining whether to track a can or bring it back to the
 * starting zone. It uses a state machine in order to keep track of the status of the robot and
 * updates the state machine if necessary. The state machine is composed of 3 main states, which
 * execute the standard sequence of actions to the robot follows in order to navigate to the search
 * zone, sweep the search zone, pick up a can, and navigate back to the home zone, before starting
 * again. It is a Moore state machine, however, the actions within the states are triggered only
 * when the state is entered.
 * 
 * @author Cyril Yared
 * @author MJ Tucker
 */
public class Controller {

  /**
   * Distance between any object and ultrasonic sensor (cm). Updated by Ultrasonic Poller.
   */
  public static int usDistance;

  /**
   * Ultrasonic distance considered inaccurate (cm).
   */
  public final static int BAD_READING = 200;

  /**
   * Number of iterations before BAD_READING accepted as accurate for ultrasonic distance.
   */
  public final static int FILTER_OUT = 3;

  /**
   * Counter which contains number of iterations of BAD_READING for ultrasonic sensor.
   */
  public static int filterControl;

  /**
   * List of locations of cans already found.
   */
  public static List<CanMarker> canList = new ArrayList<CanMarker>();;

  /**
   * Boolean if target can has been found.
   */
  public static boolean targetFound = false;

  /**
   * Motor that controls arm on aft of robot.
   */
  public static EV3MediumRegulatedMotor CAN_MOTOR =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

  /**
   * Describes the state of the robot and the current objective.
   * 
   * @author Cyril Yared
   * @author MJ Tucker
   *
   */
  public static enum RobotState {
    /**
     * State for navigating from start zone to search zone.
     */
    Navigate,

    /**
     * State for when arrived at start zone and sweeping for cans.
     */
    Search,

    /**
     * State for when picking up can, bringing can back to start zone and depositing can.
     */
    Retrieve,

    /**
     * State for when robot not moving, should not occur unless error.
     */
    Idle
  };

  /**
   * Current state of the robot.
   */
  private static RobotState currentState;

  /**
   * Controls the flow of execution of the robot in order to achieve objective depending on current
   * state. Navigates to search zone, retrieves and navigates through list of waypoints to search,
   * identifies can, determines can color and can weight and then decides whether to carry can and
   * bring it back to starting zone or move can. Relocalizes in order to optimize accuracy of
   * odometer. Updates the state if necessary.
   * 
   * @param liLocalization Instance of LightLocalization to call for relocalization.
   */
  public static void doMission(LightLocalization liLocalization) {
    currentState = RobotState.Navigate;

    while (currentState != RobotState.Idle) {
      switch (currentState) {
        case Navigate:
          navigateThroughIslandTunnel(liLocalization);
          navigateToSearchZone(liLocalization);
          currentState = RobotState.Search;
          break;
        case Search:
          sweep(liLocalization);
          currentState = RobotState.Retrieve;
          break;
        case Retrieve:
          navigateThroughHomeZoneTunnel(liLocalization);
          navigateToStartArea(liLocalization);
          try {
            dropCan();
          } catch (OdometerExceptions e) {
            e.printStackTrace();
          }
          currentState = RobotState.Navigate;
          break;
        case Idle:
          RobotMovement.stop();
          break;
      }
    }
  }

  /**
   * Drops the can at the starting zone.
   * 
   * @throws OdometerExceptions Exceptions caused by odometer.
   */
  private static void dropCan() throws OdometerExceptions {

    RobotMovement.setRotateSpeed(120);

    // Turns robot to face directly away from both walls
    switch (CanRecycler.startingCorner) {
      case 0:
        RobotMovement
            .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 45));
        break;
      case 1:
        RobotMovement
            .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 315));
        break;
      case 2:
        RobotMovement
            .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 225));
        break;
      case 3:
        RobotMovement
            .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 135));
        break;
    }

    // Drop can
    Controller.CAN_MOTOR.setSpeed(40);
    Controller.CAN_MOTOR.rotate(65);
    Controller.CAN_MOTOR.rotate(45);
    Controller.CAN_MOTOR.stop();

    // Reset wheel track
    RobotMovement.TRACK = RobotMovement.DEFAULT_WHEEL_TRACK;

    RobotMovement.beep(5, 500); // Required by assignment
    RobotMovement.move(8); // Move out of can area
    Controller.CAN_MOTOR.rotate(-45); // Close the claws
    targetFound = false;
    Navigation.objectDetected = false;
    RobotMovement.canGraspInitialization();
  }

  /**
   * Navigates back to start area to drop can.
   * 
   * @param liLocalization Instance of light localization to use for relocalization.
   */
  private static void navigateToStartArea(LightLocalization liLocalization) {
    WayPoint startCorner;
    switch (CanRecycler.startingCorner) {
      case 0:
        startCorner = new WayPoint(1, 1);
        break;
      case 1:
        startCorner = new WayPoint(14, 1);
        break;
      case 2:
        startCorner = new WayPoint(14, 8);
        break;
      case 3:
        startCorner = new WayPoint(1, 8);
        break;
      default:
        startCorner = new WayPoint(0, 0);
        break;
    }

    try {
      Navigation.navigateToWaypoint(startCorner, false, false);
    } catch (Exception e) {
      e.printStackTrace();
    }

  }

  /**
   * Navigates back through the tunnel to get to the home zone.
   * 
   * @param liLocalization Instance of light localization to use for relocalization.
   */
  private static void navigateThroughHomeZoneTunnel(LightLocalization liLocalization) {

    // Move light arm to front to deflect cans
    ColorIdentification.SENSOR_MOTOR.rotateTo(-115);


    boolean tunnelOrientationLongestX = determineTunnelOrientation(CanRecycler.tn_LL_x,
        CanRecycler.tn_LL_y, CanRecycler.tn_UR_x, CanRecycler.tn_UR_y);
    boolean isLLInHomeZone = isInHomeZone(CanRecycler.tn_LL_x, CanRecycler.tn_LL_y);

    WayPoint localizeTunnel;
    WayPoint beginningTunnel;
    WayPoint endTunnel;
    WayPoint postTunnelLocalizationObjective; // Since localization depends on next waypoint

    boolean tunnelIsOnEdge = false;
    int degreeOffset = 0;

    if (CanRecycler.DEBUG) {
      System.out.println("Tunnel Orientation X: " + tunnelOrientationLongestX);
      System.out.println("Is LL in Home Zone: " + isLLInHomeZone);
    }

    // For each case, checks which way the tunnel is oriented and if the LL is in home zone to
    // determine the start and end of the tunnel. Determines waypoints for localization and tunnel
    // traversal. Also determines degree direction to turn after exiting tunnel to avoid hitting
    // wall.
    if (tunnelOrientationLongestX && isLLInHomeZone) {
      localizeTunnel = new WayPoint(CanRecycler.tn_UR_x + 1.3, CanRecycler.tn_UR_y - 0.5);
      beginningTunnel = new WayPoint(CanRecycler.tn_UR_x + 0.5, CanRecycler.tn_UR_y - 0.5);
      endTunnel = new WayPoint(CanRecycler.tn_LL_x - .75, CanRecycler.tn_LL_y + 0.5);
      postTunnelLocalizationObjective =
          new WayPoint(CanRecycler.tn_LL_x - 1.5, CanRecycler.tn_LL_y + 0.5);

      if (CanRecycler.tn_UR_y > 8.5) { // Case where near upper wall, need to localize one tile down
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_UR_x + 1.3, CanRecycler.tn_UR_y - 1.5);
        degreeOffset = -30;
      } else if (CanRecycler.tn_UR_y < 1.5) { // Case where near lower wall, need to localize one
                                              // tile up
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_UR_x + 1.3, CanRecycler.tn_UR_y + 0.5);
        degreeOffset = 30;
      }

    } else if (tunnelOrientationLongestX && !isLLInHomeZone) {
      localizeTunnel = new WayPoint(CanRecycler.tn_LL_x - 1.3, CanRecycler.tn_LL_y + 0.5);
      beginningTunnel = new WayPoint(CanRecycler.tn_LL_x - 0.5, CanRecycler.tn_LL_y + 0.5);
      endTunnel = new WayPoint(CanRecycler.tn_UR_x + .75, CanRecycler.tn_UR_y - 0.5);
      postTunnelLocalizationObjective =
          new WayPoint(CanRecycler.tn_UR_x + 1.5, CanRecycler.tn_UR_y - 0.5);

      if (CanRecycler.tn_LL_y > 7.5) { // Case where near upper wall, need to localize one tile down
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_LL_x - 1.3, CanRecycler.tn_LL_y - 0.5);
        degreeOffset = 30;
      } else if (CanRecycler.tn_LL_y < 0.5) { // Case where near lower wall, need to localize one
                                              // tile up
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_LL_x - 1.3, CanRecycler.tn_LL_y + 1.5);
        degreeOffset = -30;
      }

    } else if (!tunnelOrientationLongestX && isLLInHomeZone) {
      localizeTunnel = new WayPoint(CanRecycler.tn_UR_x - 0.5, CanRecycler.tn_UR_y + 1.3);
      beginningTunnel = new WayPoint(CanRecycler.tn_UR_x - 0.5, CanRecycler.tn_UR_y + 0.5);
      endTunnel = new WayPoint(CanRecycler.tn_LL_x + 0.5, CanRecycler.tn_LL_y - .75);
      postTunnelLocalizationObjective =
          new WayPoint(CanRecycler.tn_LL_x + 0.5, CanRecycler.tn_LL_y - 1.5);

      if (CanRecycler.tn_UR_x > 14.5) { // Case where near right wall, need to localize one tile
                                        // left
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_UR_x - 1.5, CanRecycler.tn_UR_y + 1.3);
        degreeOffset = 30;
      } else if (CanRecycler.tn_UR_x < 1.5) { // Case where near left wall, need to localize one
                                              // tile right
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_UR_x + 0.5, CanRecycler.tn_UR_y + 1.3);
        degreeOffset = -30;
      }

    } else {
      localizeTunnel = new WayPoint(CanRecycler.tn_LL_x + 0.5, CanRecycler.tn_LL_y - 1.3);
      beginningTunnel = new WayPoint(CanRecycler.tn_LL_x + 0.5, CanRecycler.tn_LL_y - 0.5);
      endTunnel = new WayPoint(CanRecycler.tn_UR_x - 0.5, CanRecycler.tn_UR_y + 0.75);
      postTunnelLocalizationObjective =
          new WayPoint(CanRecycler.tn_UR_x - 0.5, CanRecycler.tn_UR_y + 1.5);

      if (CanRecycler.tn_LL_x > 13.5) { // Case where near right wall, need to localize one tile
                                        // left
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_LL_x - 0.5, CanRecycler.tn_LL_y - 1.3);
        degreeOffset = -30;
      } else if (CanRecycler.tn_LL_x < .5) { // Case where near left wall, need to localize one tile
                                             // right
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_LL_x + 1.5, CanRecycler.tn_LL_y - 1.3);
        degreeOffset = 30;
      }
    }
    if (CanRecycler.DEBUG) {
      System.out.println("Localize Tunnel: (" + localizeTunnel.x + ", " + localizeTunnel.y + ")");
    }

    try {
      // Navigate to localization zone, localize
      Navigation.navigateToWaypoint(localizeTunnel, false, false);

      // Deflect cans using arm at front
      ColorIdentification.SENSOR_MOTOR.rotateTo(-175);
      ColorIdentification.SENSOR_MOTOR.rotateTo(-55);
      ColorIdentification.SENSOR_MOTOR.rotateTo(-175);
      ColorIdentification.SENSOR_MOTOR.rotateTo(-55);
      ColorIdentification.SENSOR_MOTOR.rotateTo(-115);

      // Move light arm back to initial location
      ColorIdentification.SENSOR_MOTOR.rotateTo(115);
      ColorIdentification.SENSOR_MOTOR.flt();

      // Localize at specified waypoint
      liLocalization.localize(false, 0, Navigation.calculateTurnToWaypoint(beginningTunnel));

      // Navigate through tunnel
      Navigation.navigateToWaypoint(beginningTunnel, false, false);
      Navigation.navigateToWaypoint(endTunnel, false, false);

      // If tunnel is on edge, turns by degree offset to avoid hitting wall and moves forward, then
      // proceeds. In this case, relocalization does not occur. Otherwise, relocalizes.
      if (tunnelIsOnEdge) {
        RobotMovement.turnByDegree(degreeOffset);
        RobotMovement.move(18);
      } else {
        liLocalization.localize(false, 0,
            Navigation.calculateTurnToWaypoint(postTunnelLocalizationObjective));
      }

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * Pick up can after identification.
   */
  public static void pickUpCan() {
    Controller.CAN_MOTOR.setSpeed(200);
    Controller.CAN_MOTOR.rotate(-70);
    Controller.CAN_MOTOR.stop();
  }

  /**
   * Set up the robot so that it moves the can from the color identification side of robot to the
   * weight identification and arm side of the robot.
   * 
   * @throws Exception if Odometer or Thread sleeping fails.
   */
  public static void canGraspSetUp() throws Exception {
    double theta = 0;
    double currentY = 0;
    try {
      theta = Odometer.getOdometer().getXYT()[2];
      currentY = Odometer.getOdometer().getXYT()[1];
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }

    // Checks to ensure that not picking up can near wall, moves less if case
    int maxY = 270;
    if (maxY - currentY < 15) {

      if (theta > 45 && theta < 135) {
        RobotMovement.move((15 - (maxY - currentY)));
      }
    } else if (currentY < 15) {
      if ((theta > 270 && theta < 360) || (theta < 45)) {

        RobotMovement.move(15 - currentY);
      }
    }

    // Move back and turn 180 degrees to pick up can
    RobotMovement.setRotateSpeed(150);
    RobotMovement.move(-15);
    RobotMovement.setRotateSpeed(120);
    RobotMovement.turnByDegree(182);

    // Open arm and back up into can
    Controller.CAN_MOTOR.rotate(65);
    Controller.CAN_MOTOR.setSpeed(40);
    Controller.CAN_MOTOR.rotate(45);
    RobotMovement.setRotateSpeed(150);
    RobotMovement.move(-12);
  }

  /**
   * Sweeps the search zone for can.
   * 
   * @param liLocalization Instance of light localization to use for relocalization.
   */
  private static void sweep(LightLocalization liLocalization) {
    // Navigate through searchzone
    List<WayPoint> searchGridPoints = calculateSearchGridWaypoints(
        new WayPoint(CanRecycler.sz_LL_x + 0.7, CanRecycler.sz_LL_y + 0.7),
        new WayPoint(CanRecycler.sz_UR_x - 0.5, CanRecycler.sz_UR_y - 0.8));

    // boolean didRelocalizeAtWaypoint = false;

    for (int i = 0; i < searchGridPoints.size(); i++) {

      // If target can found, navigate directly to the UR corner
      if (targetFound) {
        return;
      } else {

        // If target not found, continue navigating through map
        try {
          Navigation.navigateToWaypoint(searchGridPoints.get(i), true, true);

          // If object detected, should continue navigating to next waypoint
          if (Navigation.objectDetected) {
            i--; // So that we don't skip the waypoint
            Navigation.objectDetected = false;
          } else {
            // didRelocalizeAtWaypoint = false;
          }
        } catch (Exception e) {
          e.printStackTrace();
        }
      }

    }
  }

  /**
   * Navigates from tunnel to search zone.
   * 
   * @param liLocalization Instance of light localization to use for relocalization.
   */
  private static void navigateToSearchZone(LightLocalization liLocalization) {
    try {

      boolean tunnelOrientationLongestX = determineTunnelOrientation(CanRecycler.tn_LL_x,
          CanRecycler.tn_LL_y, CanRecycler.tn_UR_x, CanRecycler.tn_UR_y);
      WayPoint relocalization;

      // Determines where should localize with respect to search zone depending on tunnel
      // orientation. If tunnel oriented in X direction, localizes to left of LL, otherwise,
      // localizes below LL
      if (tunnelOrientationLongestX) {
        relocalization = new WayPoint(CanRecycler.sz_LL_x - .4, CanRecycler.sz_LL_y + .7);
      } else {
        relocalization = new WayPoint(CanRecycler.sz_LL_x + .6, CanRecycler.sz_LL_y - .4);
      }
      
      if(!tunnelOrientationLongestX && CanRecycler.sz_LL_y == 0) {
        relocalization.y += 1.1;
      }
      if(tunnelOrientationLongestX && CanRecycler.sz_LL_x == 0) {
        relocalization.x += 1;
      }

      // Navigates to relocalization and creates waypoint for first spot in search zone
      Navigation.navigateToWaypoint(relocalization, false, false);
      WayPoint szwp = new WayPoint(CanRecycler.sz_LL_x + 0.7, CanRecycler.sz_LL_y + 0.7);

      // Localizes and proceeds to search zone, activates can sensing equipment
      liLocalization.localize(false, 0, Navigation.calculateTurnToWaypoint(szwp));
      RobotMovement.beep(3, 500); // Required 3 beeps when reach search zone
      Navigation.navigateToWaypoint(szwp, true, true);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * Navigates from starting block through tunnel to get to island.
   * 
   * @param liLocalization Instance of LightLocalization to call for relocalization.
   */
  private static void navigateThroughIslandTunnel(LightLocalization liLocalization) {
    boolean tunnelOrientationLongestX = determineTunnelOrientation(CanRecycler.tn_LL_x,
        CanRecycler.tn_LL_y, CanRecycler.tn_UR_x, CanRecycler.tn_UR_y);
    boolean isLLInHomeZone = isInHomeZone(CanRecycler.tn_LL_x, CanRecycler.tn_LL_y);

    WayPoint localizeTunnel;
    WayPoint beginningTunnel;
    WayPoint endTunnel;
    WayPoint postTunnelLocalizationObjective; // Since relocalization depends on direction of next
                                              // waypoint

    boolean tunnelIsOnEdge = false;
    int degreeOffset = 0;

    if (CanRecycler.DEBUG) {
      System.out.println("Tunnel Orientation X: " + tunnelOrientationLongestX);
      System.out.println("Is LL in Home Zone: " + isLLInHomeZone);
    }

    // For each case, checks which way the tunnel is oriented and if the LL is in home zone to
    // determine the start and end of the tunnel. Determines waypoints for localization and tunnel
    // traversal. Also determines degree direction to turn after exiting tunnel to avoid hitting
    // wall.
    if (tunnelOrientationLongestX && isLLInHomeZone) {
      localizeTunnel = new WayPoint(CanRecycler.tn_LL_x - 1.5, CanRecycler.tn_LL_y + 0.5);
      beginningTunnel = new WayPoint(CanRecycler.tn_LL_x - 0.5, CanRecycler.tn_LL_y + 0.5);
      endTunnel = new WayPoint(CanRecycler.tn_UR_x + 0.5, CanRecycler.tn_UR_y - 0.5);
      postTunnelLocalizationObjective =
          new WayPoint(CanRecycler.tn_UR_x + 1.5, CanRecycler.tn_UR_y - 0.5);

      if (CanRecycler.tn_LL_y > 7.5) { // If too close to upper wall, localizes one tile down
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_LL_x - 1.5, CanRecycler.tn_LL_y - 0.5);
        degreeOffset = 30;
      } else if (CanRecycler.tn_LL_y < 0.5) { // If too close to lower wall, localizes one tile up
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_LL_x - 1.5, CanRecycler.tn_LL_y + 1.5);
        degreeOffset = -30;
      }
    } else if (tunnelOrientationLongestX && !isLLInHomeZone) {
      localizeTunnel = new WayPoint(CanRecycler.tn_UR_x + 1.5, CanRecycler.tn_UR_y - 0.5);
      beginningTunnel = new WayPoint(CanRecycler.tn_UR_x + 0.5, CanRecycler.tn_UR_y - 0.5);
      endTunnel = new WayPoint(CanRecycler.tn_LL_x - 0.5, CanRecycler.tn_LL_y + 0.5);
      postTunnelLocalizationObjective =
          new WayPoint(CanRecycler.tn_LL_x - 1.5, CanRecycler.tn_LL_y + 0.5);

      if (CanRecycler.tn_UR_y > 8.5) { // If too close to upper wall, localizes one tile down
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_UR_x + 1.5, CanRecycler.tn_UR_y - 1.5);
        degreeOffset = -30;
      } else if (CanRecycler.tn_UR_y < 1.5) { // If too close to lower wall, localizes one tile up
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_UR_x + 1.5, CanRecycler.tn_UR_y + 0.5);
        degreeOffset = 30;
      }

    } else if (!tunnelOrientationLongestX && isLLInHomeZone) {
      localizeTunnel = new WayPoint(CanRecycler.tn_LL_x + 0.5, CanRecycler.tn_LL_y - 1.5);
      beginningTunnel = new WayPoint(CanRecycler.tn_LL_x + 0.5, CanRecycler.tn_LL_y - 0.5);
      endTunnel = new WayPoint(CanRecycler.tn_UR_x - 0.5, CanRecycler.tn_UR_y + 0.5);
      postTunnelLocalizationObjective =
          new WayPoint(CanRecycler.tn_UR_x - 0.5, CanRecycler.tn_UR_y + 1.5);

      if (CanRecycler.tn_LL_x > 13.5) { // If too close to right wall, localizes one tile left
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_LL_x - 0.5, CanRecycler.tn_LL_y - 1.5);
        degreeOffset = -30;
      } else if (CanRecycler.tn_LL_x < 0.5) { // If too close to left wall, localizes one tile right
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_LL_x + 1.5, CanRecycler.tn_LL_y - 1.5);
        degreeOffset = 30;
      }

    } else {
      localizeTunnel = new WayPoint(CanRecycler.tn_UR_x - 0.5, CanRecycler.tn_UR_y + 1.5);
      beginningTunnel = new WayPoint(CanRecycler.tn_UR_x - 0.5, CanRecycler.tn_UR_y + 0.5);
      endTunnel = new WayPoint(CanRecycler.tn_LL_x + 0.5, CanRecycler.tn_LL_y - 0.5);
      postTunnelLocalizationObjective =
          new WayPoint(CanRecycler.tn_LL_x + 0.5, CanRecycler.tn_LL_y - 1.5);

      if (CanRecycler.tn_UR_x > 14.5) { // If too close to right wall, localizes one tile left
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_UR_x - 1.5, CanRecycler.tn_UR_y + 1.5);
        degreeOffset = 30;
      } else if (CanRecycler.tn_UR_x < 1.5) { // If too close to left wall, localizes one tile right
        tunnelIsOnEdge = true;
        localizeTunnel = new WayPoint(CanRecycler.tn_UR_x + 0.5, CanRecycler.tn_UR_y + 1.5);
        degreeOffset = -30;
      }
    }
    if (CanRecycler.DEBUG) {
      System.out.println("Localize Tunnel: (" + localizeTunnel.x + ", " + localizeTunnel.y + ")");
    }

    try {
      // Navigate to localization zone, localize
      Navigation.navigateToWaypoint(localizeTunnel, false, false);
      if (CanRecycler.DEBUG) {
        System.out.println(
            "Turn to final orientation " + Navigation.calculateTurnToWaypoint(beginningTunnel));
      }
      liLocalization.localize(false, 0, Navigation.calculateTurnToWaypoint(beginningTunnel));

      // Navigate through tunnel
      Navigation.navigateToWaypoint(beginningTunnel, false, false);
      Navigation.navigateToWaypoint(endTunnel, false, false);

      // If tunnel is on edge, turns by degree offset to avoid hitting wall and moves forward, then
      // proceeds. In this case, relocalization does not occur. Otherwise, relocalizes.
      if (tunnelIsOnEdge) {
        RobotMovement.turnByDegree(degreeOffset);
        RobotMovement.move(18);
      } else {
        liLocalization.localize(false, 0,
            Navigation.calculateTurnToWaypoint(postTunnelLocalizationObjective));
      }

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * Determines whether the tunnel is oriented in the X or Y direction.
   * 
   * @param LL_x Lower left x of tunnel.
   * @param LL_y Lower left y of tunnel.
   * @param UR_x Upper right x of tunnel.
   * @param UR_y Upper right y of tunnel.
   * @return True if tunnel oriented in X direction, false otherwise.
   */
  private static boolean determineTunnelOrientation(int LL_x, int LL_y, int UR_x, int UR_y) {
    return Math.abs(LL_x - UR_x) > Math.abs(LL_y - UR_y);
  }

  /**
   * Determines if point is in home zone of robot.
   * 
   * @param x X coordinate to check if in home zone.
   * @param y Y coordinate to check if in home zone.
   * @return True if point in home zone of robot.
   */
  private static boolean isInHomeZone(int x, int y) {
    return x <= CanRecycler.start_UR_x && y <= CanRecycler.start_UR_y && x >= CanRecycler.start_LL_x
        && y >= CanRecycler.start_LL_y;
  }

  /**
   * Calculates path to search a grid of given dimensions. It selects a path that follows an S
   * pattern such that the robot with the funnel mechanism that centers the cans traverses every
   * single point in the search grid.
   * 
   * @param lowerLeftHand The lower left hand point of the search grid.
   * @param upperRightHand The upper right hand point of the search grid.
   * 
   * @return List of WayPoint to navigate search area following S pattern.
   */
  private static List<WayPoint> calculateSearchGridWaypoints(WayPoint lowerLeftHand,
      WayPoint upperRightHand) {

    int width = ((int) Math.round(upperRightHand.x - lowerLeftHand.x)) * 2;
    double x = lowerLeftHand.x;
    double xIncrease = (upperRightHand.x - lowerLeftHand.x) / width;

    // Finds bottom and top of s to sweep
    double lowY = lowerLeftHand.y;
    double highY = upperRightHand.y;
    double currentY = highY;

    List<WayPoint> searchGridPoints = new ArrayList<WayPoint>();

    // Number of waypoints in path
    int numWayPoints = ((width + 1) * 2) - 1;
    int i = 1;

    while (i <= numWayPoints) {
      WayPoint wayPoint;

      if (i % 2 == 0) { // if its even waypoint
        wayPoint = new WayPoint(x, currentY);
        // Checks if its at the bottom or top of the S search area
        if (currentY == lowY) {
          currentY = highY;
        } else {
          currentY = lowY;
        }
      } else { // if its an odd waypoint
        wayPoint = new WayPoint(x, currentY);
        x = x + xIncrease; // Increments x by half of tile size
      }
      // Adds calculated waypoint to list
      searchGridPoints.add(wayPoint);
      i++;
    }

    // Adds upper right hand waypoint as the last waypoint and returns list
    searchGridPoints.add(upperRightHand);
    return searchGridPoints;
  }

  /**
   * Sets the local distance variable from the ultrasonic poller. Uses basic filtering to filter out
   * large changes in distance.
   * 
   * @param distance Distance from ultrasonic poller (cm).
   */
  public static void processUSData(int distance) {
    if (usDistance >= BAD_READING && filterControl < FILTER_OUT) { // Case where ultrasonic
      // records
      // bad reading for fewer than FILTER_OUT iterations
      filterControl++;
    } else if (usDistance >= BAD_READING) { // Case where ultrasonic records bad reading
      // consistently,
      // assumes ultrasonic correct
      usDistance = distance;
    } else { // Case where reading in normal range
      filterControl = 0;
      usDistance = distance;
    }
  }
}
