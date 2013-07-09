package main.java.kinect;

import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import java.util.List;
import main.java.kinect.KinectPointType.Type;
import main.java.misc.Vector;

/**
 * Handles data from the ros topic and creates a data stream for new packets
 *
 * @author hxs830
 */
public class KinectDataSubscriber {

    //an array of all the kinect joints available
    private Type[] pointTypes = KinectPointType.Type.values();
    private KinectDataStream stream;
    private KinectDataPacket packet;
    private String personToTrack = "0";

    /**
     * Constructor. Creates the data stream and a new packet
     */
    public KinectDataSubscriber() {
        stream = new KinectDataStream();
        packet = new KinectDataPacket();
    }

    /**
     * Called when a new message from the ros topic is available
     *
     * @param message The message containing all tranforms
     * @return If the person that supposed to be tracked, has tranform data in
     * the message
     */
    public boolean onNewMessage(tf.tfMessage message) {
        boolean transformForPersonFound = false;
        List<TransformStamped> transforms = message.getTransforms();
        //iterate through each item in the transforms
        for (int i = 0; i < transforms.size(); i++) {
            //loop through every transformation it just receieved
            for (int j = 0; j < pointTypes.length; j++) {
                //check which type of point it just received
                if (transforms.get(i).getChildFrameId().startsWith(pointTypes[j].toString())
                        && transforms.get(i).getChildFrameId().endsWith(personToTrack)) {
                    Transform transform = transforms.get(i).getTransform();
                    //add the type of skeleton point and its vector
                    packet.add(pointTypes[j],
                            new Vector(
                            transform.getTranslation().getX(),
                            transform.getTranslation().getY(),
                            transform.getTranslation().getZ()));
                    transformForPersonFound = true;
                    break;
                }
            }
            //if the packet is complete
            if (packet.isComplete()) {
                //give the packet to the stream
                packet.setTime(System.currentTimeMillis());
                stream.newData(packet);
                //create a new packet
                packet = new KinectDataPacket();
            }
        }
        return transformForPersonFound;
    }

    /**
     * Update the person that is supposed to be tracked
     *
     * @param personNumber The person number as a string
     */
    public void setPersonToTrack(String personNumber) {
        personToTrack = personNumber;
        packet = new KinectDataPacket();
        System.out.println("Now tracking person number " + personToTrack);
    }

    /**
     * @return The person number that is supposed to be tracked
     */
    public String getPersonNumberToTrack() {
        return personToTrack;
    }

    /**
     * @return The kinect data stream
     */
    public KinectDataStream getStream() {
        return stream;
    }
}
