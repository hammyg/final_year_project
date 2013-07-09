package main.java.misc;

/**
 * This class holds 3D coordinates
 *
 * @author hxs830
 */
public class Vector {

    private double x = 0, y = 0, z = 0;

    /**
     * Constructor to set all three values for the vector
     */
    public Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * @return The x value
     */
    public double getX() {
        return x;
    }

    /**
     * @param d The new value for x
     */
    public void setX(double d) {
        x = d;
    }

    /**
     * @return The y value
     */
    public double getY() {
        return y;
    }

    /**
     * @param d The new value for y
     */
    public void setY(double d) {
        y = d;
    }

    /**
     * @return The z value
     */
    public double getZ() {
        return z;
    }

    /**
     * @param d The new value for z
     */
    public void setZ(double d) {
        z = d;
    }

    /**
     * @return A string representation of all three values (x, y, & z)
     */
    @Override
    public String toString() {
        return "x: " + x + " y: " + y + " z: " + z;
    }

    /**
     * Checks whether another vector is the equal
     *
     * @param obj The object to check
     * @return If the object is a vector that has the same values
     */
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Vector)) {
            return false;
        }
        Vector v = (Vector) obj;
        return getX() == v.getX()
                && getY() == v.getY()
                && getZ() == v.getZ();
    }

    /**
     * Used to test the vector
     */
    public static void main(String[] args) {
        System.out.println(new Vector(1, 2, 3).toString());
    }
}
