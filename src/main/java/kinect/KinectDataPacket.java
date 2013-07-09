package main.java.kinect;

import java.util.Collection;
import java.util.EnumMap;
import main.java.misc.Vector;

/**
 * This class holds all the relevant information to represent a person being
 * tracked by the Kinect It holds all the vectors (3d positions) for each joint
 * of the person
 *
 * @author hxs830
 */
public class KinectDataPacket {

    //all the skeleton points and their vectors
    private EnumMap<KinectPointType.Type, Vector> data = new EnumMap<KinectPointType.Type, Vector>(KinectPointType.Type.class);
    private long creationTime = 0;

    /**
     * Empty constructor
     */
    public KinectDataPacket() {
    }

    /**
     * Used create a packet with existing data, for copying a packet object
     *
     * @param data A map with the joint types and the vectors
     * @param time The creation time
     */
    private KinectDataPacket(EnumMap<KinectPointType.Type, Vector> data, long time) {
        this.data = data;
        this.creationTime = time;
    }

    /**
     * @return A collection of all the vectors available
     */
    public Collection<Vector> getVectors() {
        return data.values();
    }

    /**
     * Checks to see if the skeleton point has been added already
     *
     * @param type The type of skeleton point
     * @return true if it already exists
     */
    public boolean exists(KinectPointType.Type type) {
        return data.containsKey(type);
    }

    /**
     * Adds a new skeleton point with its location vector
     *
     * @param type The type of skeleton point
     * @param vector The location vector
     */
    public void add(KinectPointType.Type type, Vector vector) {
        data.put(type, vector);
    }

    /**
     * Checks to see if this packet has all the required skeleton points
     *
     * @return True if packet is complete
     */
    public boolean isComplete() {
        return data.size() == KinectPointType.count();
    }

    /**
     * Gets the location vector for a specific skeleton point type
     *
     * @param type The skeleton point type
     * @return The location vector
     * @throws KinectException Throws a KinectException if the type does not
     * exist
     */
    public Vector get(KinectPointType.Type type) {
        Vector v = data.get(type);
        return v;
    }

    /**
     * Get the time the packet was completed
     *
     * @return The time
     */
    public long getTime() {
        return creationTime;
    }

    /**
     * Set the time the packet was completed
     *
     * @param time The time
     */
    public void setTime(long time) {
        this.creationTime = time;
    }

    /**
     * @return A new data packet with the same vectors as this and the same
     * create time
     */
    public KinectDataPacket copy() {
        return new KinectDataPacket(data, creationTime);
    }

    /**
     * Used for testing the packet
     *
     * @param args
     */
    public static void main(String[] args) {
        KinectDataPacket packet = new KinectDataPacket();
        for (int i = 0; i < KinectPointType.asArray().size(); i++) {
            System.out.println("adding: " + KinectPointType.asArray().get(i).name());
            packet.add(KinectPointType.asArray().get(i), new Vector(1, 2, 3));
            System.out.println("isComplete: " + packet.isComplete());

        }
        System.out.println(packet.get(KinectPointType.Type.HEAD));
    }
}
