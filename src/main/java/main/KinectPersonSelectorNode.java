package main.java.main;

import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import main.java.misc.StaticMethods;
import main.java.misc.Vector;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * This is a ros node that selects which person should be selected to be tracked
 * The person selected is published for other nodes to use a common person
 *
 * @author hxs830
 */
public class KinectPersonSelectorNode implements NodeMain {

    //subscribers
    private Subscriber<tf.tfMessage> tfSub;
    //publishers
    private Publisher<std_msgs.String> selectedPersonPub;
    //other
    private boolean run = true;
    private String currentPerson = "-1";
    private long lastUpdate = 0;
    //time to wait before selecting another person to track
    private int duration = 1000;

    /**
     * @return A name given to this node
     */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("kinect_person_selector_node");
    }

    /**
     * Called when the node is started by ros
     *
     * @param node provides the node to allow new publishers and subscribers to
     * be created
     */
    @Override
    public void onStart(ConnectedNode node) {
        System.out.println("Node Started");

        //create a string publisher for the person selectd
        selectedPersonPub = node.newPublisher("tracking_person_number", std_msgs.String._TYPE);

        //subscriber for transformations
        tfSub = node.newSubscriber("/tf", tf.tfMessage._TYPE);
        tfSub.addMessageListener(new MessageListener<tf.tfMessage>() {
            //one transform is selected
            String transformName = "/torso";
            int firstIndex = 7; // "torso_XX
            Vector lastTorsoLocation;
            boolean printNoOne = true;

            //code to run on new tranform message from the ros topic
            @Override
            public void onNewMessage(tf.tfMessage message) {
                //iterate through all the transforms
                for (TransformStamped tr : message.getTransforms()) {
                    //check for torso transform (can be any tr from skeleton)
                    if (tr.getChildFrameId().startsWith(transformName)) {
                        //check what person number is on the transform string
                        String person = tr.getChildFrameId().substring(firstIndex, tr.getChildFrameId().length());

                        //check if time of last person id has expired
                        //and no suspecion of lost person
                        if (currentPerson.equals(person)) {

                            //convert the tranform to a vector
                            Transform transform = tr.getTransform();
                            Vector torsoLocation = new Vector(
                                    transform.getTranslation().getX(),
                                    transform.getTranslation().getY(),
                                    transform.getTranslation().getZ());

                            /* check if the torso location is exactly as the previous
                             * when person is lost, usually data lingers around
                             * so it's not immediately known that the person has disappeared.
                             * but since no one stays exactly in the same location in space,
                             * it can be detected.
                             */

                            if (lastTorsoLocation != null && (lastTorsoLocation.equals(torsoLocation)
                                    || StaticMethods.euclideanDistance(torsoLocation, lastTorsoLocation) > 0.05)) {
                                //System.out.println("Person " + currentPerson + " Not Moving!!!");
                                if (!lastTorsoLocation.equals(torsoLocation)) {
                                    System.out.println("Fast Detection: " + StaticMethods.euclideanDistance(torsoLocation, lastTorsoLocation));
                                }
                            } else {
                                //if the person is still being tracked
                                //update the last update time
                                lastUpdate = System.currentTimeMillis();
                            }
                            lastTorsoLocation = torsoLocation;
                            break;
                        } else if (System.currentTimeMillis() - lastUpdate > duration) {
                            //update person id + last update time
                            currentPerson = person;
                            lastTorsoLocation = null;
                            lastUpdate = System.currentTimeMillis();
                            System.out.println("Person " + person + " has now been selected to be tracked");
                            break;
                        }
                    }
                }
                if (System.currentTimeMillis() - lastUpdate > duration) {
                    //indicates person has disappeared and there is no one else
                    if (lastUpdate != 0
                            && printNoOne) {
                        printNoOne = false;
                        System.out.println("Person " + currentPerson + " has disappeared. Cannot see anyone else.");
                    }
                } else {
                    printNoOne = true;
                }
            }
        });

        try {
            //initial loop to wait for first transform data
            while ("-1".equals(currentPerson)) {
                System.out.println("Waiting for tf data...");
                Thread.sleep(1000);
            }
            //continous loop to publish the selected person
            while (run) {
                std_msgs.String newMessage = selectedPersonPub.newMessage();
                if (System.currentTimeMillis() - lastUpdate > duration) {
                    newMessage.setData("-1");
                } else {
                    newMessage.setData(currentPerson);
                }
                selectedPersonPub.publish(newMessage);
                Thread.sleep(50);
            }
        } catch (InterruptedException ex) {
            //do nothing
        }
    }

    /**
     * Called when node is shutting down
     *
     * @param node The node shutting down
     */
    @Override
    public void onShutdown(Node node) {
        run = false;
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
