package ca.mcgill.ecse211.canrecycler;

/**
 * Contains the defining features of a can including its x and y position on the grid, its color
 * (integer value based on a preset distinction from 1-4, and weight (1 - heavy can, 0 - light can)). 
 * Used to store information about a can that has been assessed for its position, color and
 * weight. 
 * 
 * @author Cyril Yared
 * @author MJ Tucker
 *
 */
public class CanMarker extends WayPoint {

  /**
   * Colour of the can, specified by an integer corresponding to a specific colour.
   */
  public int colour;

  /**
   * Weight of the can, specificed by -1 (null), 0 (light), 1 (heavy).
   */
  public int weight;

  /**
   * Constructor for canMarker.
   * 
   * @param x X-coordinate for can.
   * @param y Y-coordinate for can.
   * @param colour integer representing colour of the can
   * @param weight integer representing if can is light/heavy
   */
  public CanMarker(double x, double y, int colour, int weight) {
    super(x, y);
    this.colour = colour;
    this.weight = weight;

  }

}
