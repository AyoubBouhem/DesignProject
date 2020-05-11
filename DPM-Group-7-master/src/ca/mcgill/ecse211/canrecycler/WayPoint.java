package ca.mcgill.ecse211.canrecycler;

/**
 * Contains x and y data for a single waypoint.
 * 
 * @author Cyril Yared
 * @author Ayoub Bouhemhem
 *
 */
public class WayPoint {

  /**
   * X-coordinate for waypoint as defined in Cartesian system (tiles)
   */
  public double x;

  /**
   * Y-coordinate for waypoint as defined in Cartesian system (tiles).
   */
  public double y;

  /**
   * Constructor for waypoint.
   * 
   * @param x X-coordinate for waypoint.
   * @param y Y-coordinate for waypoint.
   */
  public WayPoint(double x, double y) {
    this.x = x;
    this.y = y;
  }
}
