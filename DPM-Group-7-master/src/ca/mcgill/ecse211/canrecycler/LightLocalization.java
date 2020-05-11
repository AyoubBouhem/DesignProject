package ca.mcgill.ecse211.canrecycler;


import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class uses the light sensor in order to localize both the position and orientation of the
 * robot. In order for this process to work, the orientation must be approximated by a prior
 * technique. If it is the initial localization, it can measure the distance from both walls to
 * estimate position before proceeding with a more accurate localization using the light sensors. Or
 * else, it uses an approximate of its position from the Odometer package. This method allows
 * localization to any starting corner. The relocalization method is able to relocalize the robot at
 * any point using the two light sensors. It decides which two lines to use to localize based on the
 * distance from the wall and the final orientation that it should point after localization.
 * 
 * @author Cyril Yared
 *
 */
public class LightLocalization {

  /**
   * Distance from center of rotation to light sensor in centimeters.
   */
  public final static double DIST_FROM_LS_TO_CENTER_OF_ROT = 5.1;

  /**
   * Differential threshold for light sensor to detect line.
   */
  public final static double LIGHT_SENSOR_DIFFERENTIAL_THRESHOLD = 0.022;

  /**
   * How far to move back to reset after completing first light sensor localization (cm).
   */
  public final static double MOVE_BACK_LOCALIZE = 5.5;

  /**
   * Speed at which to rotate forward for light sensor localization (deg/sec).
   */
  public static final int LIGHT_FORWARD_SPEED = 100;

  /**
   * Tiles in X direction of playing field.
   */
  public static final int GRID_X = 15;

  /**
   * Tiles in Y direction of playing field.
   */
  public static final int GRID_Y = 9;

  /**
   * Port for left light sensor used for localization.
   */
  private Port lsPortLeft = LocalEV3.get().getPort("S2");

  /**
   * Port for right light sensor used for localization.
   */
  private Port lsPortRight = LocalEV3.get().getPort("S1");

  /**
   * Buffer for data for left light sensor.
   */
  private float lsDataLeft[];

  /**
   * Buffer for data for right light sensor.
   */
  private float lsDataRight[];

  /**
   * Sample provider for left light sensor.
   */
  private SampleProvider lsColorLeft;

  /**
   * Sample provider for right light sensor.
   */
  private SampleProvider lsColorRight;

  /**
   * Initializes all sensors and buffers to store data.
   */
  public LightLocalization() {
    // Sets up light sensor in reflected light mode.
    @SuppressWarnings("resource")
    SensorModes lsLeftSensor = new EV3ColorSensor(lsPortLeft);
    @SuppressWarnings("resource")
    SensorModes lsRightSensor = new EV3ColorSensor(lsPortRight);
    lsColorLeft = lsLeftSensor.getMode("Red"); // Reflected light mode
    lsColorRight = lsRightSensor.getMode("Red"); // Reflected light mode
    lsDataLeft = new float[lsColorLeft.sampleSize()];
    lsDataRight = new float[lsColorRight.sampleSize()];
  }

  /**
   * Initiates the light sensor localization and controls the flow of execution of the entire
   * process. Handles different starting corners and applies offset between light sensor and center
   * of robot.
   * 
   * @param initializeLocalize True if this is the first localization.
   * @param startingCorner Represents starting corner as described in project description.
   * @param finalOrientation Final orientation after relocalization in degrees.
   * @throws Exception Odometer error or thread sleeping.
   */
  public void localize(Boolean initializeLocalize, int startingCorner, double finalOrientation)
      throws Exception {

    // Gets instance of odometer.
    Odometer odometer = Odometer.getOdometer();

    // Depending on starting corner, differences in how x and y should be set as well as theta
    // Note that offset applied since light sensors not exactly aligned to center of wheel
    if (initializeLocalize) {
      switch (startingCorner) {
        case 0:
          alignToAxis();
          odometer.setY(1 * RobotMovement.TILE_SIZE - DIST_FROM_LS_TO_CENTER_OF_ROT);
          odometer.setTheta(0);
          resetForSecondAlignment(90);
          alignToAxis();
          odometer.setX(1 * RobotMovement.TILE_SIZE - DIST_FROM_LS_TO_CENTER_OF_ROT);
          odometer.setTheta(90);
          break;
        case 1:
          alignToAxis();
          odometer.setX((GRID_X - 1) * RobotMovement.TILE_SIZE + DIST_FROM_LS_TO_CENTER_OF_ROT);
          odometer.setTheta(270);
          resetForSecondAlignment(0);
          alignToAxis();
          odometer.setY(1 * RobotMovement.TILE_SIZE - DIST_FROM_LS_TO_CENTER_OF_ROT);
          odometer.setTheta(0);
          break;
        case 2:
          alignToAxis();
          odometer.setY((GRID_Y - 1) * RobotMovement.TILE_SIZE + DIST_FROM_LS_TO_CENTER_OF_ROT);
          odometer.setTheta(180);
          resetForSecondAlignment(270);
          alignToAxis();
          odometer.setX((GRID_X - 1) * RobotMovement.TILE_SIZE + DIST_FROM_LS_TO_CENTER_OF_ROT);
          odometer.setTheta(270);
          break;
        case 3:
          alignToAxis();
          odometer.setX(1 * RobotMovement.TILE_SIZE - DIST_FROM_LS_TO_CENTER_OF_ROT);
          odometer.setTheta(90);
          resetForSecondAlignment(180);
          alignToAxis();
          odometer.setY((GRID_Y - 1) * RobotMovement.TILE_SIZE + DIST_FROM_LS_TO_CENTER_OF_ROT);
          odometer.setTheta(180);
          break;
      }
      if (CanRecycler.DEBUG) {
        System.out.println("X: " + odometer.getXYT()[0] / 30.0);
        System.out.println("Y: " + odometer.getXYT()[1] / 30.0);
        System.out.println("T: " + odometer.getXYT()[2]);
      }

    } else {
      RobotMovement.setRotateSpeed(RobotMovement.ROTATE_SPEED);

      // Determines best two lines to localize along
      // Takes into account how far robot is from both walls and final orientation objective
      // For instance, if robot final orientation north, will localize first against east and west
      // (whichever is farthest from wall), then localize on north line
      if (finalOrientation >= 315 || finalOrientation <= 45) { // North
        if (odometer.getXYT()[0] < 7.5 * RobotMovement.TILE_SIZE) { // Localize against east
          RobotMovement
              .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 90));
          alignToAxis();
          odometer.setTheta(90);
          odometer.setX(
              Math.round(odometer.getXYT()[0] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                  - DIST_FROM_LS_TO_CENTER_OF_ROT);
        } else { // Localize against west
          RobotMovement
              .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 270));
          alignToAxis();
          odometer.setTheta(270);
          odometer.setX(
              Math.round(odometer.getXYT()[0] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                  + DIST_FROM_LS_TO_CENTER_OF_ROT);
        }
        // Localizes back to final orientation of north
        resetForSecondAlignment(0);
        alignToAxis();
        odometer.setY(
            Math.round(odometer.getXYT()[1] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                - DIST_FROM_LS_TO_CENTER_OF_ROT);
        odometer.setTheta(0);
      } else if (finalOrientation > 45 && finalOrientation < 135) { // East
        if (odometer.getXYT()[1] < 4.5 * RobotMovement.TILE_SIZE) { // Localize against north
          RobotMovement
              .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 0));
          alignToAxis();
          odometer.setTheta(0);
          odometer.setY(
              Math.round(odometer.getXYT()[1] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                  - DIST_FROM_LS_TO_CENTER_OF_ROT);
        } else { // Localize against south
          RobotMovement
              .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 180));
          alignToAxis();
          odometer.setTheta(180);
          odometer.setY(
              Math.round(odometer.getXYT()[1] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                  + DIST_FROM_LS_TO_CENTER_OF_ROT);
        }
        // Localizes back to final orientation of east
        resetForSecondAlignment(90);
        alignToAxis();
        odometer.setX(
            Math.round(odometer.getXYT()[0] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                - DIST_FROM_LS_TO_CENTER_OF_ROT);
        odometer.setTheta(90);
      } else if (finalOrientation > 135 && finalOrientation < 225) { // South
        if (odometer.getXYT()[0] < 7.5 * RobotMovement.TILE_SIZE) { // Localize against east
          RobotMovement
              .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 90));
          alignToAxis();
          odometer.setTheta(90);
          odometer.setX(
              Math.round(odometer.getXYT()[0] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                  - DIST_FROM_LS_TO_CENTER_OF_ROT);
        } else { // Localize against west
          RobotMovement
              .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 270));
          alignToAxis();
          odometer.setTheta(270);
          odometer.setX(
              Math.round(odometer.getXYT()[0] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                  + DIST_FROM_LS_TO_CENTER_OF_ROT);
        }
        // Localizes back to final orientation of south
        resetForSecondAlignment(180);
        alignToAxis();
        odometer.setY(
            Math.round(odometer.getXYT()[1] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                + DIST_FROM_LS_TO_CENTER_OF_ROT);
        odometer.setTheta(180);
      } else { // West
        if (odometer.getXYT()[1] < 4.5 * RobotMovement.TILE_SIZE) { // Localize against north
          RobotMovement
              .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 0));
          alignToAxis();
          odometer.setTheta(0);
          odometer.setY(
              Math.round(odometer.getXYT()[1] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                  - DIST_FROM_LS_TO_CENTER_OF_ROT);
        } else { // Localize against south
          RobotMovement
              .turnByDegree(RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], 180));
          alignToAxis();
          odometer.setTheta(180);
          odometer.setY(
              Math.round(odometer.getXYT()[1] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                  + DIST_FROM_LS_TO_CENTER_OF_ROT);
        }
        // Localizes back to final orientation of west
        resetForSecondAlignment(270);
        alignToAxis();
        odometer.setX(
            Math.round(odometer.getXYT()[0] / RobotMovement.TILE_SIZE) * RobotMovement.TILE_SIZE
                + DIST_FROM_LS_TO_CENTER_OF_ROT);
        odometer.setTheta(270);
      }


      if (CanRecycler.DEBUG) {
        System.out.println("X: " + Odometer.getOdometer().getXYT()[0]);
        System.out.println("Y: " + Odometer.getOdometer().getXYT()[1]);
        System.out.println("Theta: " + Odometer.getOdometer().getXYT()[2]);
      }
    }
  }

  /**
   * Resets position of robot so that it can localize on second axis.
   * 
   * @param finalOrientation Final orientation of robot after localizing in degrees.
   * @throws OdometerExceptions if Odometer fails.
   */
  private void resetForSecondAlignment(int finalOrientation) throws OdometerExceptions {
    RobotMovement.setRotateSpeed(RobotMovement.ROTATE_SPEED);
    RobotMovement.move(-MOVE_BACK_LOCALIZE);
    RobotMovement.turnByDegree(
        RobotMovement.bestDirTurn(Odometer.getOdometer().getXYT()[2], finalOrientation));
  }

  /**
   * Moves robot forward until both light sensors detect gridline. If one light sensor detects the
   * gridline first, the respective motor stops and the other continues until that light sensor has
   * detected the gridline. Uses a differential light sensor filter.
   * 
   * @throws InterruptedException caused by thread sleeping.
   */
  private void alignToAxis() throws InterruptedException {
    // Sets up light sensors in reflected light mode.

    // Sets rotation speed to slower speed for light sensor to get accurate readings
    RobotMovement.setRotateSpeed(LIGHT_FORWARD_SPEED);
    RobotMovement.LEFT_MOTOR.forward();
    RobotMovement.RIGHT_MOTOR.forward();

    // Gets previous reading in order to calculate differential
    double prevReadingL = lsDataLeft[0];
    double prevReadingR = lsDataRight[0];

    // Booleans for if light sensor detected lines already
    boolean leftLineReached = false;
    boolean rightLineReached = false;

    // Continue forwards until both light sensor has reached line
    while (!leftLineReached || !rightLineReached) {
      if (!leftLineReached) {
        // Checks if left sensor has reached line
        if (prevReadingL - lsDataLeft[0] > LIGHT_SENSOR_DIFFERENTIAL_THRESHOLD) {
          RobotMovement.LEFT_MOTOR.setSpeed(0);
          leftLineReached = true;
          if (CanRecycler.DEBUG) {
            Sound.beep();
          }
        }
        prevReadingL = lsDataLeft[0];
        lsColorLeft.fetchSample(lsDataLeft, 0);
      }

      if (!rightLineReached) {
        // Checks if right sensor has reached line
        if (prevReadingR - lsDataRight[0] > LIGHT_SENSOR_DIFFERENTIAL_THRESHOLD) {
          RobotMovement.RIGHT_MOTOR.setSpeed(0);
          rightLineReached = true;
          if (CanRecycler.DEBUG) {
            Sound.beep();
          }
        }
        prevReadingR = lsDataRight[0];
        lsColorRight.fetchSample(lsDataRight, 0);
      }
      Thread.sleep(30);
    }
    // Robot is now aligned to axis.
    if (CanRecycler.DEBUG) {
      Sound.beepSequenceUp();
    }
  }
}
