package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.canrecycler.*;


/**
 * This class implements the odometer functionality. Using tachometers on both the left and right
 * motors, it is able to estimate the robots current (x, y) position in centimeters relative to the
 * origin (or starting position if no origin is specified). It is also able to determine the robots
 * orientation relative to its starting orientation. It subsequently calls the appropriate methods
 * in OdometerData in order to update the odometry data across the entire system. This class follows
 * a singleton design pattern.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 * 
 * @author Ayoub Bouhemhem
 * @author Cyril Yared
 */
public class Odometer extends OdometerData implements Runnable {

  /**
   * Instance of OdometerData to ensure that class is singleton.
   */
  private OdometerData odoData;

  /**
   * Saves instance of odometer to ensure that class is singleton.
   */
  private static Odometer odo = null;

  /**
   * Tachomoter count of left motor.
   */
  private int leftMotorTachoCount;

  /**
   * Tachomoter count of right motor.
   */
  private int rightMotorTachoCount;

  /**
   * Count of the left tachometer in the previous iteration.
   */
  private int pastLeftTachoCount;

  /**
   * Count of the right tachometer in the previous iteration.
   */
  private int pastRightTachoCount;

  /**
   * Estimated distance traveled by left wheel since last iteration (cm).
   */
  private double distL;

  /**
   * Estimated distance traveled by right wheel since last iteration (cm).
   */
  private double distR;

  /**
   * Estimated change in the x-position since the last iteration (cm).
   */
  private double dX;

  /**
   * Estimated change in the y-position since the last iteration (cm).
   */
  private double dY;

  /**
   * Estimated change in the theta since the last iteration (degrees).
   */
  private double dTheta;

  /**
   * The new theta derived from the previous theta and dTheta (radians).
   */
  private double theta;

  /**
   * Stores the position and orientation as specified in OdometerData.
   */
  private double[] position;

  /**
   * Odometer update period in ms.
   */
  private static final long ODOMETER_PERIOD = 25;

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @throws OdometerExceptions if odometer fails
   */
  private Odometer() throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods


    // Reset the values of x, y and Theta to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    this.pastLeftTachoCount = 0;
    this.pastRightTachoCount = 0;
  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @return new or existing Odometer Object
   * @throws OdometerExceptions if odometer fails.
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer();
      return odo;
    }
  }

  /**
   * This method is where the logic for the odometer runs. It uses the information in OdometerData
   * in addition to information from the tachometer counts in order to estimate changes in the
   * position and orientation of the robot.
   */
  public void run() {
    long updateStart, updateEnd; // Used to time thread

    while (true) {
      updateStart = System.currentTimeMillis();

      // Gets the tacho counts from both motors
      leftMotorTachoCount = RobotMovement.LEFT_MOTOR.getTachoCount();
      rightMotorTachoCount = RobotMovement.RIGHT_MOTOR.getTachoCount();

      position = odo.getXYT();

      distL = 3.14159 * RobotMovement.WHEEL_RAD * (leftMotorTachoCount - pastLeftTachoCount) / 180; // If not
      // optimal,
      // then use
      // old + new
      // Tacho
      distR = 3.14159 * RobotMovement.WHEEL_RAD * (rightMotorTachoCount - pastRightTachoCount) / 180;

      dTheta = ((distL - distR) / RobotMovement.TRACK) * 180 / 3.14159; // Approximates the arcsin and
                                                               // converts to
      // degrees
      theta = (dTheta + position[2]) * 3.14159 / 180; // Converts from degrees to radians
      dX = 0.5 * (distR + distL) * Math.sin(theta); // Both math functions take rads
      dY = 0.5 * (distR + distL) * Math.cos(theta);

      // Sets past tachometer counts for next iteration
      pastLeftTachoCount = leftMotorTachoCount;
      pastRightTachoCount = rightMotorTachoCount;

      // Updates odometer values with new calculated values
      odo.update(dX, dY, dTheta);


      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }
}
