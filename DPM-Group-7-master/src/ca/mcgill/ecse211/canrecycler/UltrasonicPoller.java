package ca.mcgill.ecse211.canrecycler;


import lejos.robotics.SampleProvider;

/**
 * Class to implement a periodic UltrasonicPoller thread. The while loop at the bottom executes in a
 * loop. Assuming that the us.fetchSample, and cont.processUSData methods operate in about 20mS, and
 * that the thread sleeps for 50 mS at the end of each loop, then one cycle through the loop is
 * approximately 70 mS. This corresponds to a sampling rate of 1/70mS or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {

  /**
   * Instance of sample provider for ultrasonic sensor.
   */
  private SampleProvider us;

  /**
   * Array of float containing ultrasonic data.
   */
  private float[] usData;

  /**
   * Constructor for ultrasonic poller. Initializes fields.
   * 
   * @param us SampleProvider to acquire data from ultrasonic sensor.
   * @param usData Float array which contains ultrasonic sensor buffer.
   */
  public UltrasonicPoller(SampleProvider us, float[] usData) {
    this.us = us;
    this.usData = usData;
  }

  /**
   * Run method to return the sensor values as floats using a uniform protocol, and converted US
   * result to an integer.
   * 
   */
  public void run() {
    int distance;
    while (true) {
      us.fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int, multiply to CM
      Controller.processUSData(distance);
      try {
        Thread.sleep(100);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
  }

}

