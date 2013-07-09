package main.java.kinect;

/**
 * A data stream for new kinect packets
 *
 * @author hxs830
 */
public class KinectDataStream {

    //temprory store the last kinect packet
    private KinectDataPacket last = null;

    /**
     * Called when a new packet is ready
     *
     * @param dataPacket The new data packet
     */
    public void newData(KinectDataPacket dataPacket) {
        last = dataPacket;
    }

    /**
     * @return If there is a packet available to retrieve
     */
    public boolean packetAvailable() {
        return last != null;
    }

    /**
     * Blocking call to get the next kinect packet
     *
     * @return the next available packet
     */
    public KinectDataPacket getPacket() {
        KinectDataPacket toReturn = peek();
        last = null;
        return toReturn;
    }

    /**
     * Blocking call to peek at next packet
     *
     * @return the next available packet
     */
    public KinectDataPacket peek() {
        try {
            while (last == null) {
                Thread.sleep(1);
            }
        } catch (Exception ex) {
        }
        KinectDataPacket toReturn = last;
        return toReturn;
    }
}
