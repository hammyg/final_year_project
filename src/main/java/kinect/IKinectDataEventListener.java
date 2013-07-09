package main.java.kinect;

import java.util.EventListener;

/**
 * Listener interface for the buffer to use when Kinect packets are available
 *
 * @author hxs830
 */
public interface IKinectDataEventListener extends EventListener {

    //fired when the kinect buffer is full
    void newKinectData(KinectDataPacket[] packets);
}
