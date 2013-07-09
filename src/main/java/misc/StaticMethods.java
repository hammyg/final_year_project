package main.java.misc;

import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageFactory;

/**
 * This class has many commonly used static methods
 *
 * @author hxs830
 */
public class StaticMethods {

    //message factor for creating ros objects
    public static MessageFactory messageFactory;

    /**
     * Method written by Mark Rowan. Checks whether the cell is occupied
     *
     * @param x: the x position
     * @param y: the y position
     * @param map The occupancy grid
     * @return: true if location is occupied
     */
    public static boolean cellIsOccupied(double x, double y, OccupancyGrid map) {
        ChannelBuffer data = map.getData();

        // Map data
        long map_width = map.getInfo().getWidth();
        long map_height = map.getInfo().getHeight();
        float map_resolution = map.getInfo().getResolution(); // in m per pixel

        // Particle position
        double x_orig = x / map_resolution;
        double y_orig = y / map_resolution;

        //Check the indexes
        int index;
        if ((int) Math.round(x_orig) < 0 || (int) Math.round(y_orig) < 0 || x > (int) map_width || y > (int) map_height) {
            // If requested location is out of the bounds of the map
            index = -1;
            System.out.println("Invalid co-ordines. Cannot calculate map index");
            System.out.println(x_orig);
            System.out.println(y_orig);
            System.out.println(map_width);
            System.out.println(map_height);
        } else {
            index = ((int) Math.round(y_orig) * (int) map_width) + (int) Math.round(x_orig);
        }

        if (index > 0 && index < data.capacity()) {
            Byte cellData = data.getByte(index);
            if (cellData.byteValue() < 0 || cellData.byteValue() > 65) {
                //System.out.println("Occupied: Cell data value is " + cellData.byteValue());
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }

    /**
     * Method originally written by Mark Rowan. Checks to see if a line does not
     * go through an occupied cell on the map or a dynamic object
     *
     * @param ox The x position of the line on the map
     * @param oy The y position of the line on the map
     * @param bearing The bearing to turn
     * @param max_range The length of the line to check
     * @param map The occupancy grid
     * @return If the line is clear
     */
    public static boolean legalLine(double ox, double oy, double bearing, double max_range, OccupancyGrid map) {
        ChannelBuffer data = map.getData();

        // As the ROS angle system has 0 deg = east and increases anti-clockwise
        // but the Quaternion trigonometry and particle raytracing assume 0 deg = north / clockwise,
        // we must first convert to 0 deg = north / clockwise by subtracting PI/2
        //Printer.println("Bearing Before: " + bearing);
        bearing -= Math.PI / 2;
        bearing *= -1;

        // Map data
        float map_resolution = map.getInfo().getResolution(); // in m per pixel

        // Find gradient of the line of sight in x,y plane, assuming 0 deg = north
        double grad_x = Math.sin(bearing);
        double grad_y = Math.cos(bearing);

        // Particle position
        double x_orig = ox / map_resolution;
        double y_orig = oy / map_resolution;
        // Max range position relative to the current position
        double x_max_offset = max_range * grad_x / map_resolution;
        double y_max_offset = max_range * grad_y / map_resolution;

        double x = x_orig;
        double y = y_orig;

        // Stop travelling away from the robot when we reach max range of
        // laser or an occupied cell
        while (Math.abs(x - x_orig) < Math.abs(x_max_offset)
                && Math.abs(y - y_orig) < Math.abs(y_max_offset)) {

            x += grad_x;
            y += grad_y;

            int index = getMapIndex((int) Math.round(x), (int) Math.round(y), map);

            if (index > 0 && index < data.capacity()) {
                // If we are on the map to begin with...
                Byte cellData = data.getByte(index);

                if (cellData.byteValue() < 0 || cellData.byteValue() > 65) {
                    // If we're on the map, but the map has no data, or there is an obstacle...
                    //or if it is within a objecct detected by the laser readings
                    return false;
                }
            } else {
                return false;
            }
        }
        return true;
    }

    /**
     * Method written by Mark Rowan. Finds the index in the map date of a
     * location on the map
     *
     * @param x The x location on the map
     * @param y The y location on the map
     * @param map The occupancy grid
     * @return An index within the map data
     */
    public static int getMapIndex(int x, int y, OccupancyGrid map) {
        int map_width = map.getInfo().getWidth();
        int map_height = map.getInfo().getHeight();
        if (x < 0 || y < 0 || x > map_width || y > map_height) {
            // If requested location is out of the bounds of the map
            return -1;
        } else {
            return (y * map_width) + x;
        }
    }

    /**
     * Checks to see if a specific location on the map is valid for the robot to
     * be at
     *
     * @param x The robots x location on the map
     * @param y The robots y location on the map
     * @param robot_radius The radius of the robot
     * @param map The occupancy grid
     * @return If the robot can be at this location on the map
     */
    public static boolean legalAreaForRobot(double x, double y, double robot_radius, OccupancyGrid map) {
        for (int i = 0; i < 360; i++) {
            //checkRobotSpace is false because this it the method that actually checks for valid robot spaces.
            //if set to true, it will cause an infinite recursive call.
            if (!legalLine(x, y, Math.toRadians(i), robot_radius, map)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Calculates the euclidean distance between two Poses
     *
     * @param p1 Pose 1
     * @param p2 Pose 2
     * @return The euclidean distance
     */
    public static double distanceBetweenPoses(Pose p1, Pose p2) {
        double x = p1.getPosition().getX() - p2.getPosition().getX();
        double y = p1.getPosition().getY() - p2.getPosition().getY();
        return Math.sqrt((x * x) + (y * y));
    }

    /**
     * Convert float array to double array
     */
    public static double[] floatArrToDoubleArr(float[] arr) {
        double[] doubleArr = new double[arr.length];
        for (int i = 0; i < doubleArr.length; i++) {
            doubleArr[i] = (double) arr[i];
        }
        return doubleArr;
    }

    /**
     * Convert double array to float array
     */
    public static float[] doubleArrToFloatArr(double[] arr) {
        float[] floatArr = new float[arr.length];
        for (int i = 0; i < arr.length; i++) {
            floatArr[i] = (float) arr[i];
        }
        return floatArr;
    }

    /**
     * Gets the 3D euclidean distance between two vectors
     *
     * @param v1 Vector 1
     * @param v2 Vector 2
     * @return The euclidean distance between v1 and v2
     */
    public static double euclideanDistance(Vector v1, Vector v2) {
        double dX = v1.getX() - v2.getX();
        double dY = v1.getY() - v2.getY();
        double dZ = v1.getZ() - v2.getZ();
        return Math.sqrt(
                (dX * dX)
                + (dY * dY)
                + (dZ * dZ));
    }
}
