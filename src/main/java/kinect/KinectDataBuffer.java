package main.java.kinect;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * This class acts as a buffer for the Kinect data packets and runs in a
 * separate thread
 *
 * @author hxs830
 */
public class KinectDataBuffer extends Thread {

    //the stream from where to get the kinect data packets
    private KinectDataStream stream;
    //event listeners when the buffer is ready
    private List<IKinectDataEventListener> eventListeners = new ArrayList<IKinectDataEventListener>();
    //the size of the buffer
    private int bufferSize;
    //the buffer data stored in a fixed sized aray
    private KinectDataPacket[] lastPacketsArray;
    //the index for the of the pointer in the array
    private int pointerIndex = 0;
    //the buffer data stored in an array list which behaves as a queue
    private ArrayList<KinectDataPacket> lastPacketsQueue = new ArrayList<KinectDataPacket>();
    //which method of storing the data to use
    private boolean useQueueBuffer = false;
    //the last time of new kienct data
    private long lastDataTime = -1;

    /**
     * Constructor for the buffer
     *
     * @param stream The Kinect stream to get the Kinect data from
     * @param bufferSize The size of the buffer
     */
    public KinectDataBuffer(KinectDataStream stream, int bufferSize) {
        this.stream = stream;
        this.bufferSize = bufferSize;
        lastPacketsArray = new KinectDataPacket[bufferSize];
    }

    /**
     * @return The size of the buffer
     */
    public int getBufferSize() {
        return bufferSize;
    }

    /**
     * Adds to the list of event listener that will be notified when data is
     * ready
     *
     * @param listener The listener to be notified
     */
    public synchronized void addEventListener(IKinectDataEventListener listener) {
        eventListeners.add(listener);
    }

    /**
     * Notifies all event listeners that buffer is ready with data
     *
     * @param packets The buffer data
     */
    private synchronized void fireNewKinectPacketsEvent(KinectDataPacket[] packets) {
        Iterator i = eventListeners.iterator();
        while (i.hasNext()) {
            ((IKinectDataEventListener) i.next()).newKinectData(packets);
        }
    }

    /**
     * Thread to handle the packets from the kinect data stream
     */
    @Override
    public void run() {
        while (true) {
            handleNewSkeletonData();
        }
    }

    /**
     * Used to set which method to use for the buffer
     *
     * @param useQueueBuffer Use queue behaviour
     */
    public void setUseQueueBuffer(boolean useQueueBuffer) {
        this.useQueueBuffer = useQueueBuffer;
        //reset the buffer
        reset();
    }

    /**
     * Method is called when there is a new kinect packet available in the
     * stream
     */
    private void handleNewSkeletonData() {
        lastDataTime = System.currentTimeMillis();
        //if using the queue behaviour
        if (useQueueBuffer) {
            //waits for queue to populate with data, before firing.
            //once overflowing, front items are removed.
            lastPacketsQueue.add(stream.getPacket());


            //if the queue is full
            if (lastPacketsQueue.size() > bufferSize) {
                //remoe the first packet
                lastPacketsQueue.remove(0);
            }

            //if the queue is full, then notify the listeners
            if (lastPacketsQueue.size() == bufferSize) {
                fireNewKinectPacketsEvent(lastPacketsQueue.toArray(new KinectDataPacket[lastPacketsQueue.size()]));
            }
        } else {
            //waits for the array to be populated with new data before firing.
            KinectDataPacket packet = stream.getPacket();

            //updates the item at the current pointer position
            lastPacketsArray[pointerIndex] = packet;
            //move the pointer to the next position
            pointerIndex++;
            //if the pointer has exceeded the size, then notify listeners and reset pointer position
            if (pointerIndex == bufferSize) {
                fireNewKinectPacketsEvent(lastPacketsArray);
                pointerIndex = 0;
            }
        }
    }

    /**
     * @return The last time Kinect data was received
     */
    public long getLastDataTime() {
        return lastDataTime;
    }

    /**
     * Clears the buffer
     */
    public void reset() {
        pointerIndex = 0;
        lastPacketsQueue.clear();
        System.out.println("DataHandler buffer cleared (reset).");
    }
}
