package main.java.main;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.LaserScan;
import std_msgs.Bool;

/**
 * This is a ros node that takes two laser scans and publishes one combined
 * laser scan
 *
 * @author hxs830
 */
public class LaserCombinerNode implements NodeMain {

    //subscribers
    private Subscriber<LaserScan> bLaserSub;
    private Subscriber<LaserScan> kLaserSub;
    private Subscriber<Bool> userLaserSub;
    //publishers
    private Publisher<LaserScan> laserPub;
    //other
    private LaserScan lastBLaserScan;
    private LaserScan lastKLaserScan;
    private boolean useKLaserScan = false;

    /**
     * @return The name of the node
     */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("laser_combiner_node");
    }

    /**
     * Called when the node is started
     *
     * @param node The node used to create subscribers and publishers
     */
    @Override
    public void onStart(ConnectedNode node) {
        System.out.println("Node Started");

        //publisher for the combined laser scans
        laserPub = node.newPublisher("combined_laser_scan", LaserScan._TYPE);

        //subscriber for laser scan
        bLaserSub = node.newSubscriber("/scan", LaserScan._TYPE);
        bLaserSub.addMessageListener(new MessageListener<LaserScan>() {
            /**
             * Called when a new base laser scan is available
             */
            @Override
            public void onNewMessage(LaserScan message) {
                if (lastBLaserScan == null) {
                    //store the data
                    lastBLaserScan = message;
                    //call the combine and publish method
                    combineAndPublish();
                }
            }
        });
        //subscriber for kinect to laser scan
        kLaserSub = node.newSubscriber("/kinect_to_laser_scan", LaserScan._TYPE);
        kLaserSub.addMessageListener(new MessageListener<LaserScan>() {
            /**
             * Called when a new kinect laser scan is available
             */
            @Override
            public void onNewMessage(LaserScan message) {
                if (lastKLaserScan == null) {
                    //store the data
                    lastKLaserScan = message;
                }
            }
        });

        // subscriber to a boolean topic to choose whether or not the
        //laser scans should be combined
        userLaserSub = node.newSubscriber("/use_kinect_laser_scan", Bool._TYPE);
        userLaserSub.addMessageListener(new MessageListener<Bool>() {
            @Override
            public void onNewMessage(Bool message) {
                useKLaserScan = message.getData();
            }
        });
    }

    /* Assuming both laser scans that will be combined
     * have the same min/max angles 
     * and the same angle increment
     * This method is always called when only a new base scan is receieved,
     * and not when a kinect laser scan is received (slower scan time)
     */
    public synchronized void combineAndPublish() {
        int readings = lastBLaserScan.getRanges().length - 1; //base laser gives 361 readings
        float minRange = 0;
        float maxRange = 0;
        float[] ranges = new float[readings];
        //check if there is a kinect laser scan and that it should be combined
        if (lastKLaserScan == null || !useKLaserScan) {
            //only use the base laser data
            ranges = lastBLaserScan.getRanges();
        } else {
            //iterate through all the readings
            for (int i = 0; i < readings; i++) {
                //select the minimum values from both laser scans
                float range = Math.min(
                        lastBLaserScan.getRanges()[i],
                        lastKLaserScan.getRanges()[i]);
                ranges[i] = range;
                //store the min and max readings
                minRange = Math.min(minRange, range);
                maxRange = Math.max(maxRange, range);
            }
            lastBLaserScan.setRangeMax(maxRange);
            lastBLaserScan.setRangeMin(minRange);
        }
        //publish the laser scan
        lastBLaserScan.setRanges(ranges);
        laserPub.publish(lastBLaserScan);
        lastBLaserScan = null;
        lastKLaserScan = null;
    }

    /**
     * Called when node is shutting down
     *
     * @param node The node shutting down
     */
    @Override
    public void onShutdown(Node node) {
    }

    /**
     * Called when shutdown has completed
     *
     * @param node The node that has been shutdown
     */
    @Override
    public void onShutdownComplete(Node node) {
    }

    /**
     * Called when an error with a node has occurred
     *
     * @param node The error node
     * @param throwable The error
     */
    @Override
    public void onError(Node node, Throwable throwable) {
    }
}
