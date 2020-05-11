package ca.mcgill.ecse211.canrecycler;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class is used in order to determine the color of a can that the robot encounters. When a
 * robot encounters a can, the detectColor() method is called. The robot uses the stepper motor in
 * order to take multiple readings of the can and determine the can color.
 * 
 * @author Cyril Yared
 * @author Ayoub Bouhemhem
 *
 */
public class ColorIdentification {

  /**
   * Red can: previously recorded normalized red value mean.
   */
  public static final double R_MEAN_RED = 0.99215;

  /**
   * Red can: previously recorded red value standard deviation.
   */
  public static final double R_SD_RED = 0.05;

  /**
   * Red can: previously recorded normalized green value mean.
   */
  public static final double R_MEAN_GREEN = 0.10277;

  /**
   * Red can: previously recorded green value standard deviation.
   */
  public static final double R_SD_GREEN = 0.05;

  /**
   * Red can: previously recorded normalized blue value mean.
   */
  public static final double R_MEAN_BLUE = 0.07112;

  /**
   * Red can: previously recorded blue value standard deviation.
   */
  public static final double R_SD_BLUE = 0.05;

  /**
   * Yellow can: previously recorded normalized red value mean.
   */
  public static final double Y_MEAN_RED = 0.87887;

  /**
   * Yellow can: previously recorded red value standard deviation.
   */
  public static final double Y_SD_RED = 0.05;

  /**
   * Yellow can: previously recorded normalized green value mean.
   */
  public static final double Y_MEAN_GREEN = 0.45759;

  /**
   * Yellow can: previously recorded green value standard deviation.
   */
  public static final double Y_SD_GREEN = 0.05;

  /**
   * Yellow can: previously recorded normalized blue value mean.
   */
  public static final double Y_MEAN_BLUE = 0.13479;

  /**
   * Yellow can: previously recorded blue value standard deviation.
   */
  public static final double Y_SD_BLUE = 0.05;

  /**
   * Green can: previously recorded normalized red value mean.
   */
  public static final double G_MEAN_RED = 0.32592; // 0.32592

  /**
   * Green can: previously recorded red value standard deviation.
   */
  public static final double G_SD_RED = 0.1;

  /**
   * Green can: previously recorded normalized green value mean.
   */
  public static final double G_MEAN_GREEN = 0.93830;

  /**
   * Green can: previously recorded green value standard deviation.
   */
  public static final double G_SD_GREEN = 0.1;

  /**
   * Green can: previously recorded normalized blue value mean.
   */
  public static final double G_MEAN_BLUE = 0.26170;

  /**
   * Green can: previously recorded blue value standard deviation.
   */
  public static final double G_SD_BLUE = 0.1;

  /**
   * Blue can: previously recorded normalized red value mean.
   */
  public static final double B_MEAN_RED = 0.24507;

  /**
   * Blue can: previously recorded red value standard deviation.
   */
  public static final double B_SD_RED = 0.1;

  /**
   * Blue can: previously recorded normalized green value mean.
   */
  public static final double B_MEAN_GREEN = 0.75943;

  /**
   * Blue can: previously recorded green value standard deviation.
   */
  public static final double B_SD_GREEN = 0.1;

  /**
   * Blue can: previously recorded normalized blue value mean.
   */
  public static final double B_MEAN_BLUE = 0.60261;

  /**
   * Blue can: previously recorded blue value standard deviation.
   */
  public static final double B_SD_BLUE = 0.1;

  /**
   * Arc size of the color sensor as it sweeps to take RGB values (deg).
   */
  public static final int STEPPER_MOTOR_OSCILLATE = 180;

  /**
   * Initial amount to move stepper motor from "home" location to where the color sensor first
   * obtains RGB readings (deg).
   */
  public static final int INITIAL_HEADING = -30;

  /**
   * Speed for the stepper motor to rotate with the color sensor attached while obtaining values
   * (deg/sec).
   */
  public static final int STEPPER_MOTOR_ROTATE_SPEED = 60;

  /**
   * Speed for the stepper motor to rotate it as it returns to its home position after obtaining all
   * readings (deg/sec).
   */
  public static final int STEPPER_MOTOR_RETURN_SPEED = 180;

  /**
   * Number of times that the stepper motor stops along arc in order to take readings.
   */
  public static final int LIGHT_SENSOR_DETECTION_STOPS = 20;

  /**
   * Stepper motor with the color sensor attached.
   */
  public static final EV3MediumRegulatedMotor SENSOR_MOTOR =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * Rotates the stepper motor in order to take multiple readings at multiple locations of the can.
   * For each reading, it is compared with experimental values to see if it falls within a standard
   * deviation to the mean of any of the can colors for all RGB values. If this is true for a can
   * color, it immediately returns the color. Or else, it continues scanning until this is the case
   * or until the scanning is completed.
   * 
   * @return Color of can (None = 0, Blue = 1, Green = 2, Yellow = 3, Red = 4).
   * @throws InterruptedException If thread sleeping throws exception.
   */
  public static int detectColor() throws InterruptedException {
    // Resets stepper motor and rotates it to initial starting location to start
    // taking readings
    SENSOR_MOTOR.setSpeed(STEPPER_MOTOR_ROTATE_SPEED);
    SENSOR_MOTOR.resetTachoCount();
    SENSOR_MOTOR.rotateTo(INITIAL_HEADING);

    int color = 0;
    double red = 0;
    double blue = 0;
    double green = 0;

    // Takes a specified number of readings per stop and moves to the next stop
    for (int i = 0; i < LIGHT_SENSOR_DETECTION_STOPS; i++) {
      // Fetches sample and records readings
      CanRecycler.lsColor.fetchSample(CanRecycler.lsRGBData, 0);
      red = CanRecycler.lsRGBData[0];
      blue = CanRecycler.lsRGBData[2];
      green = CanRecycler.lsRGBData[1];

      // Obtains the normalized values for all of the medians
      float normalDenom = (float) Math.sqrt(red * red + green * green + blue * blue);
      red = (float) (red / normalDenom);
      green = (float) (green / normalDenom);
      blue = (float) (blue / normalDenom);

      // Check if in color distribution for red
      if (isInColorDistribution(red, green, blue, R_MEAN_RED, R_SD_RED, R_MEAN_GREEN, R_SD_GREEN,
          R_MEAN_BLUE, R_SD_BLUE)) {
        color = 4;
        if (CanRecycler.DEBUG) {
          System.out.println("Red can detected");
        }
        break; // If it is in distribution, breaks from for loop, no need to continue

        // Check if in color distribution for green
      } else if (isInColorDistribution(red, green, blue, G_MEAN_RED, G_SD_RED, G_MEAN_GREEN,
          G_SD_GREEN, G_MEAN_BLUE, G_SD_BLUE)) {
        if (CanRecycler.DEBUG) {
          System.out.println("Green can detected");
        }
        color = 2;
        break; // If it is in distribution, breaks from for loop, no need to continue

        // Check if in color distribution for blue
      } else if (isInColorDistribution(red, green, blue, B_MEAN_RED, B_SD_RED, B_MEAN_GREEN,
          B_SD_GREEN, B_MEAN_BLUE, B_SD_BLUE)) {
        if (CanRecycler.DEBUG) {
          System.out.println("Blue can detected");
        }
        color = 1;
        break; // If it is in distribution, breaks from for loop, no need to continue

        // Check if in color distribution for yellow
      } else if (isInColorDistribution(red, green, blue, Y_MEAN_RED, Y_SD_RED, Y_MEAN_GREEN,
          Y_SD_GREEN, Y_MEAN_BLUE, Y_SD_BLUE)) {
        if (CanRecycler.DEBUG) {
          System.out.println("Yellow can detected");
        }
        color = 3;
        break; // If it is in distribution, breaks from for loop, no need to continue
      }
      // Rotates to next stop, note that stepper motor turns to absolute location
      SENSOR_MOTOR.rotateTo(
          -((i + 1) * STEPPER_MOTOR_OSCILLATE / LIGHT_SENSOR_DETECTION_STOPS) + INITIAL_HEADING,
          false); // Counterclockwise rotation
    }
    // Returns to starting location
    SENSOR_MOTOR.setSpeed(STEPPER_MOTOR_RETURN_SPEED);
    SENSOR_MOTOR.rotateTo(0, false); // Clockwise rotation back to center
    SENSOR_MOTOR.flt();

    if (CanRecycler.DEBUG) {
      System.out.println("R: " + String.valueOf(red));
      System.out.println("G: " + String.valueOf(green));
      System.out.println("B: " + String.valueOf(blue));
    }


    return color;
  }

  /**
   * Returns true if falls in Gaussian distribution for entire spectrum of RGB values.
   * 
   * @param rValue Normalized red value.
   * @param gValue Normalized green value.
   * @param bValue Normalized blue value.
   * @param rMean Normalized red mean for specific color.
   * @param rStd Red standard deviation for specific color.
   * @param gMean Normalized green mean for specific color.
   * @param gStd Green standard deviation for specific color.
   * @param bMean Normalized blue mean for specific color.
   * @param bStd Blue standard deviation for specific color.
   * @return True if falls in Gaussian distribution of specific color, false otherwise.
   */
  private static boolean isInColorDistribution(double rValue, double gValue, double bValue,
      double rMean, double rStd, double gMean, double gStd, double bMean, double bStd) {
    return isInDistribution(rValue, rMean, rStd) && isInDistribution(gValue, gMean, gStd)
        && isInDistribution(bValue, bMean, bStd);
  }

  /**
   * Returns true if value falls within standard deviation of mean.
   * 
   * @param value Value to be checked.
   * @param mean Mean to be compared against.
   * @param std Standard deviation.
   * @return True if value is within standard deviation of mean, false otherwise.
   */
  private static boolean isInDistribution(double value, double mean, double std) {
    return (value <= (mean + std)) && (value >= (mean - std));
  }
}
