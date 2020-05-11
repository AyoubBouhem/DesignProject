package ca.mcgill.ecse211.canrecycler;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;

/**
 * This class uses the large unregulated motor in to determine the weight of the can. It contains
 * all fields and methods necessary for this calculation and a single public method which can be
 * accessed in order to determine the weight of a can that has already been grabbed by the robot.
 * 
 * @author Cyril Yared
 *
 */
public class WeightIdentification {

  /**
   * Power setting which is unable to lift light or heavy can (closes arms).
   */
  public static final int POWER_CLOSE = 10;

  /**
   * Power setting which is able to lift light can but unable to lift heavy can.
   */
  public static final int POWER_LIFT = 35;

  /**
   * Threshold angle at which if it goes above, it is a light can and if it is under, it is a heavy
   * can.
   */
  public static final int THRESHOLD_ANGLE = 30;

  /**
   * Instance of unregulated motor that controls robot's arm.
   */
  private static UnregulatedMotor canGrasper;

  /**
   * Detects whether the can is heavy or light. Uses a large unregulated motor in order to determine
   * the weight of the can. It sets the unregulated motor to a specified power, if it is able to
   * lift the can (as detected by the tachocount going above the THRESHOLD_ANGLE), then it is a
   * light can, otherwise, it is a heavy can.
   * 
   * @return 1 if heavy can detected, 0 if light can detected, -1 if unable to detect weight.
   */
  public static int isHeavyCan() {
    canGrasper = new UnregulatedMotor(LocalEV3.get().getPort("C"));

    // Close around the can
    canGrasper.setPower(POWER_CLOSE);
    canGrasper.backward();
    delay(1500);
    canGrasper.stop();

    // Attempts to lift can
    int firstDeg = canGrasper.getTachoCount();
    canGrasper.setPower(POWER_LIFT);
    canGrasper.backward();
    delay(2000);
    int secondDeg = canGrasper.getTachoCount();

    // For Debugging, prints angle
    if (CanRecycler.DEBUG) {
      System.out.println("Angle lift " + Math.abs(firstDeg - secondDeg));
    }

    // If above threshold, light can, or else, heavy can
    int result = 0;
    if (Math.abs(firstDeg - secondDeg) > THRESHOLD_ANGLE) {
      result = 0;
    } else {
      result = 1;
    }

    // Drops can if light can
    canGrasper.flt();
    delay(1000);

    // Close around the can again so that ready to lift
    canGrasper.setPower(POWER_CLOSE);
    canGrasper.backward();
    delay(500);

    // Sets unregulated motor to float mode and closes instance, regenerates regulated motor
    canGrasper.flt();
    canGrasper.close();
    Controller.CAN_MOTOR = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
    return result;
  }

  /**
   * Delays robot by specified number of milliseconds by sleeping thread.
   * 
   * @param milliseconds Number of milliseconds to sleep robot.
   */
  private static void delay(int milliseconds) {
    try {
      Thread.sleep(milliseconds);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }
  }
}
