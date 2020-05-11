package ca.mcgill.ecse211.canrecycler;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This is the main starting point for the Can Recycler Project. Here, all sensors and parameters
 * will be initialized. Furthermore, all instances are created to enable localization. Additionally,
 * game-specific parameters will be obtained from the Wifi class. To determine whether the robot is
 * green or red team, the Team Number is compared to both teams numbers received by the wifi, and
 * depending which number is equal to our assigned team number, the appropriate values will be
 * assigned to the game parameters defined. The robot performs initial ultrasonic and light
 * localization to orient itself, and issues 3 beeps, and then the flow of execution is handed to
 * the Controller to execute the specific objectives of the project The Controller is built on a
 * state machine further described in the respective class. The robot begins by navigating to the
 * tunnel entrance, performing relocalization before entering. After the robot has navigated through
 * the tunnel, it performs relocalization, before navigating to the lower left corner of its search
 * zone, where it performs relocalization again, and issues 3 beeps. The robot then starts sweeping
 * the search zone, using the ultrasonic sensor for obstacle detection. Once a can has been
 * detected, the robot performs color identification, turns around, and then performs weight
 * identification, and the appropriate sequence of beeps follows depending on the attributes of the
 * can. If the can is deemed a 'valuable' can through the implemented optimization algorithm, the
 * robot picks up the can, and starts navigation back towards the tunnel. If it is not deemed a
 * 'valuable' can, the robot moves the can out of the way and continues along its search path
 * through the grid. If it is retrieving a can, relocalization is performed before entering the
 * tunnel and after it has navigated through. Once through the tunnel, the robot navigates to its
 * starting corner, turns around and places the can down in the starting corner of the robot and
 * issues a sequence of 5 beeps. Once the can is dropped, the robot starts navigating towards the
 * tunnel to repeat the can retrieval process again, until trial time runs out.
 * 
 * @author Cyril Yared
 * @author MJ Tucker
 */
public class CanRecycler {

  /**
   * Server IP Address.
   */
  public static final String SERVER_IP = "192.168.2.4";

  /**
   * Team Number of group.
   */
  public static final int TEAM_NUMBER = 7;

  /**
   * If true, outputs extra information to help debugging process.
   */
  public final static boolean DEBUG = false;

  /**
   * Enable/disable printing of debug info from the WiFi class.
   */
  public static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  /**
   * Team number with the specified color, should be same as TEAM_NUMBER.
   */
  public static int team;

  /**
   * Starting corner of robot.
   */
  public static int startingCorner;

  /**
   * Lower left hand corner x of starting zone.
   */
  public static int start_LL_x;

  /**
   * Lower left hand corner y of starting zone.
   */
  public static int start_LL_y;

  /**
   * Upper right hand corner x of starting zone.
   */
  public static int start_UR_x;

  /**
   * Upper right hand corner y of starting zone.
   */
  public static int start_UR_y;

  /**
   * Lower left hand corner x of island zone.
   */
  public static int island_LL_x;

  /**
   * Lower left hand corner y of island zone.
   */
  public static int island_LL_y;

  /**
   * Upper right hand corner x of island zone.
   */
  public static int island_UR_x;

  /**
   * Upper right hand corner y of island zone.
   */
  public static int island_UR_y;

  /**
   * Lower left hand corner of x tunnel footprint.
   */
  public static int tn_LL_x;

  /**
   * Lower left hand corner of y tunnel footprint.
   */
  public static int tn_LL_y;

  /**
   * Upper right hand corner of x tunnel footprint.
   */
  public static int tn_UR_x;

  /**
   * Upper right hand corner of y tunnel footprint.
   */
  public static int tn_UR_y;

  /**
   * Lower left hand corner x of player search zone.
   */
  public static int sz_LL_x;

  /**
   * Lower left hand corner y of player search zone.
   */
  public static int sz_LL_y;

  /**
   * Upper right hand corner x of player search zone.
   */
  public static int sz_UR_x;

  /**
   * Upper right hand corner of y player search zone.
   */
  public static int sz_UR_y;

  /**
   * EV3 port at which the ultrasonic sensor is located.
   */
  public static final Port US_PORT = LocalEV3.get().getPort("S3");

  /**
   * Samples from the color sensor attached to arm.
   */
  public static SampleProvider lsColor;

  /**
   * Data buffer for color sensor attached to arm.
   */
  public static float lsRGBData[];

  /**
   * Port for light sensor to detect can.
   */
  public final static String LS_COLOR_PORT = "S4";

  /**
   * Starting point of Can Recycler program. Receives wifi parameters, initialize sensors and start
   * threads and hands flow of execution to the Controller class.
   * 
   * @param args String arguments. Not being used.
   * @throws OdometerExceptions Exception handled in OdometerExceptions class.
   * @throws InterruptedException Exception handled if thread sleeping fails.
   */
  @SuppressWarnings("rawtypes")
  public static void main(String[] args) throws OdometerExceptions, InterruptedException {

    // Get instance of ultrasonic and light localizer and setup sensor
    initializeUltrasonicSensor();
    final UltrasonicLocalization usLocalizer = new UltrasonicLocalization();
    final LightLocalization liLocalizer = new LightLocalization();

    // Wait until user decides to end program
    // Gets singleton odometer instance
    Odometer odometer = Odometer.getOdometer();

    // Starts odometer thread
    Thread odoThread = new Thread(odometer);
    odoThread.start();

    // initialize the light sensor arm
    initializeLightSensorOnArm();

    (new Thread() { // Starts main thread for localization and navigation
      public void run() {

        try {
          if (DEBUG) {
            System.out.println("Running..");
          }

          // Initialize WifiConnection class
          WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);


          // Connect to server and get the data, catching any errors that might occur
          try {
            /*
             * getData() will connect to the server and wait until the user/TA presses the "Start"
             * button in the GUI on their laptop with the data filled in. Once it's waiting, you can
             * kill it by pressing the upper left hand corner button (back/escape) on the EV3.
             * getData() will throw exceptions if it can't connect to the server (e.g. wrong IP
             * address, server not running on laptop, not connected to WiFi router, etc.). It will
             * also throw an exception if it connects but receives corrupted data or a message from
             * the server saying something went wrong. For example, if TEAM_NUMBER is set to 1 above
             * but the server expects teams 17 and 5, this robot will receive a message saying an
             * invalid team number was specified and getData() will throw an exception letting you
             * know.
             */
            Map data = conn.getData();

            initWifiValues(data);

          } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
          }

          // Moves claw up to prepare robot for travel
          RobotMovement.canGraspInitialization();

          try {
            localize(startingCorner, usLocalizer, liLocalizer);
          } catch (Exception e) {
            System.out.println(e.getMessage());
          }

          // call navigation from controller
          Controller.doMission(liLocalizer);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }).start();

    // If escape button pressed, exits.
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }

  /**
   * Initialize the light sensor on arm of robot in RGB mode.
   */
  private static void initializeLightSensorOnArm() {
    lejos.hardware.port.Port lsPort = LocalEV3.get().getPort(LS_COLOR_PORT);
    @SuppressWarnings("resource")
    SensorModes lsSensor = new EV3ColorSensor(lsPort);
    lsColor = lsSensor.getMode("RGB");
    lsRGBData = new float[lsColor.sampleSize()];
  }


  /**
   * Localizes robot using ultrasonic sensor (optionally) to measure orientation and light sensor in
   * order to determine position and refine orientation. This can be used at the start or in the
   * middle of the track.
   * 
   * @param startCorner Start corner (see requirements for details).
   * @param usLocalizer Instance of UltrasonicLocalization to call.
   * @param liLocalizer Instance of LightLocalization to call.
   * @throws Exception if odometer or thread sleeping fails.
   */
  private static void localize(int startCorner, UltrasonicLocalization usLocalizer,
      LightLocalization liLocalizer) throws Exception {

    if (Controller.usDistance > UltrasonicLocalization.THRESHOLD
        + 3 * UltrasonicLocalization.ERROR) {
      usLocalizer.fallingEdge();
    } else if (Controller.usDistance < UltrasonicLocalization.THRESHOLD
        - 3 * UltrasonicLocalization.ERROR) {
      usLocalizer.risingEdge();
    } else {
      RobotMovement.setRotateSpeed(RobotMovement.ROTATE_SPEED);
      RobotMovement.turnByDegree(90);
      if (Controller.usDistance > UltrasonicLocalization.THRESHOLD) {
        usLocalizer.fallingEdge();
      } else {
        usLocalizer.risingEdge();
      }
    }

    // Creates instance of light localization and starts localization
    liLocalizer.localize(true, startCorner, 0);
    RobotMovement.beep(3, 500);
  }

  /**
   * Initialize ultrasonic sensor and starts ultrasonic poller.
   */
  private static void initializeUltrasonicSensor() {
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT); // usSensor is the instance

    // usDistance provides samples from this instance
    SampleProvider usDistance = usSensor.getMode("Distance");

    // usData is the buffer in which data are returned
    float[] usData = new float[usDistance.sampleSize()];

    // Creates and starts ultrasonic poller
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);
    usPoller.start();
  }

  /**
   * Initialize all game specific values received from wifi class.
   * 
   * @param data, to map values received from Wifi Connection.
   * @throws Exception if wifi connection fails.
   */
  private static void initWifiValues(Map<?, ?> data) throws Exception {

    int redTeam = ((Long) data.get("RedTeam")).intValue();

    int greenTeam = ((Long) data.get("GreenTeam")).intValue();

    int redCorner = ((Long) data.get("RedCorner")).intValue();

    int greenCorner = ((Long) data.get("GreenCorner")).intValue();

    int red_LL_x = ((Long) data.get("Red_LL_x")).intValue();

    int red_LL_y = ((Long) data.get("Red_LL_y")).intValue();

    int red_UR_x = ((Long) data.get("Red_UR_x")).intValue();

    int red_UR_y = ((Long) data.get("Red_UR_y")).intValue();

    int green_LL_x = ((Long) data.get("Green_LL_x")).intValue();

    int green_LL_y = ((Long) data.get("Green_LL_y")).intValue();

    int green_UR_x = ((Long) data.get("Green_UR_x")).intValue();

    int green_UR_y = ((Long) data.get("Green_UR_y")).intValue();

    island_LL_x = ((Long) data.get("Island_LL_x")).intValue();

    island_LL_y = ((Long) data.get("Island_LL_y")).intValue();

    island_UR_x = ((Long) data.get("Island_UR_x")).intValue();

    island_UR_y = ((Long) data.get("Island_UR_y")).intValue();

    int tnr_LL_x = ((Long) data.get("TNR_LL_x")).intValue();

    int tnr_LL_y = ((Long) data.get("TNR_LL_y")).intValue();

    int tnr_UR_x = ((Long) data.get("TNR_UR_x")).intValue();

    int tnr_UR_y = ((Long) data.get("TNR_UR_y")).intValue();

    int tng_LL_x = ((Long) data.get("TNG_LL_x")).intValue();

    int tng_LL_y = ((Long) data.get("TNG_LL_y")).intValue();

    int tng_UR_x = ((Long) data.get("TNG_UR_x")).intValue();

    int tng_UR_y = ((Long) data.get("TNG_UR_y")).intValue();

    int szr_LL_x = ((Long) data.get("SZR_LL_x")).intValue();

    int szr_LL_y = ((Long) data.get("SZR_LL_y")).intValue();

    int szr_UR_x = ((Long) data.get("SZR_UR_x")).intValue();

    int szr_UR_y = ((Long) data.get("SZR_UR_y")).intValue();

    int szg_LL_x = ((Long) data.get("SZG_LL_x")).intValue();

    int szg_LL_y = ((Long) data.get("SZG_LL_y")).intValue();

    int szg_UR_x = ((Long) data.get("SZG_UR_x")).intValue();

    int szg_UR_y = ((Long) data.get("SZG_UR_y")).intValue();

    // Checks which team is team number and sets appropriate parameter
    if (redTeam == TEAM_NUMBER) {
      team = redTeam;
      startingCorner = redCorner;
      start_LL_x = red_LL_x;
      start_LL_y = red_LL_y;
      start_UR_x = red_UR_x;
      start_UR_y = red_UR_y;

      tn_LL_x = tnr_LL_x;
      tn_LL_y = tnr_LL_y;
      tn_UR_x = tnr_UR_x;
      tn_UR_y = tnr_UR_y;

      sz_LL_x = szr_LL_x;
      sz_LL_y = szr_LL_y;
      sz_UR_x = szr_UR_x;
      sz_UR_y = szr_UR_y;

    } else if (greenTeam == TEAM_NUMBER) {
      team = greenTeam;
      startingCorner = greenCorner;
      start_LL_x = green_LL_x;
      start_LL_y = green_LL_y;
      start_UR_x = green_UR_x;
      start_UR_y = green_UR_y;

      tn_LL_x = tng_LL_x;
      tn_LL_y = tng_LL_y;
      tn_UR_x = tng_UR_x;
      tn_UR_y = tng_UR_y;

      sz_LL_x = szg_LL_x;
      sz_LL_y = szg_LL_y;
      sz_UR_x = szg_UR_x;
      sz_UR_y = szg_UR_y;

    } else {
      System.out.println("TEAM NUMBER ERROR");
    }
  }
}
