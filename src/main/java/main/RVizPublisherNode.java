package main.java.main;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import java.util.ArrayList;
import main.java.kinect.KinectDataPacket;
import main.java.kinect.KinectDataSubscriber;
import main.java.kinect.KinectPointType;
import main.java.kinect.KinectPointType.Type;
import main.java.misc.Vector;
import main.java.neuralNetwork.main.GestureNeuralNetwork;
import main.java.neuralNetwork.main.GestureNeuralNetwork.GestureType;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

/**
 * This is a ros node that subscribes to data (such as Kinect) and publishes 3D
 * markers to be used in RViz to represent the data
 *
 * @author hxs830
 */
public class RVizPublisherNode implements NodeMain {

    //subscribers
    private Subscriber<std_msgs.Float32MultiArray> gestureResultSub;
    private Subscriber<tf.tfMessage> tfSub;
    private Subscriber<std_msgs.String> personToTrackSub;
    //publishers
    private Publisher<MarkerArray> skeletonMarkerPub;
    private Publisher<MarkerArray> pointingTargetsPub;
    //other
    private ArrayList<Type> typesCanGlow = new ArrayList<Type>(); //the types that can glow
    private ArrayList<Boolean> glowValues = new ArrayList<Boolean>(); //if th types should glow
    private MessageFactory messageFactory;
    private KinectDataSubscriber kinectSubscriber;
    private boolean run = true;
    private Duration markerLifeTime = new Duration(1.0);

    /**
     * Constructor Adds the markers that can change colour (right & left hand)
     */
    public RVizPublisherNode() {
        typesCanGlow.add(Type.LEFT_HAND);
        glowValues.add(false);
        typesCanGlow.add(Type.RIGHT_HAND);
        glowValues.add(false);
    }

    /**
     * @return Name of this node
     */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rviz_publisher_node");
    }

    /**
     * Called when the node is started
     *
     * @param node The node used to create subscribers and publishers
     */
    @Override
    public void onStart(ConnectedNode node) {
        System.out.println("Node Started");
        //message factory is used to create ros objects
        messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
        //create a subsciber to handle textract the kinect data from /tf
        kinectSubscriber = new KinectDataSubscriber();

        //setup publishers for the markers
        skeletonMarkerPub = node.newPublisher("kinect_skeleton_markers", MarkerArray._TYPE);
        pointingTargetsPub = node.newPublisher("target_pointing_markers", MarkerArray._TYPE);

        //subscriber for transformations
        tfSub = node.newSubscriber("/tf", tf.tfMessage._TYPE);
        tfSub.addMessageListener(new MessageListener<tf.tfMessage>() {
            @Override
            public void onNewMessage(tf.tfMessage message) {
                kinectSubscriber.onNewMessage(message);
            }
        });

        //subscriber to check which person to track from kinect
        personToTrackSub = node.newSubscriber("/tracking_person_number", std_msgs.String._TYPE);
        personToTrackSub.addMessageListener(new MessageListener<std_msgs.String>() {
            String lastMessage;

            @Override
            public void onNewMessage(std_msgs.String message) {
                //check if the person that should be tracked has changed
                if (lastMessage == null || !message.getData().equals(lastMessage)) {
                    lastMessage = message.getData();
                    kinectSubscriber.setPersonToTrack(lastMessage);
                }
            }
        });

        //setup a subscriber for the neural network output
        gestureResultSub = node.newSubscriber("/gesture_result", std_msgs.Float32MultiArray._TYPE);
        gestureResultSub.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
            @Override
            public void onNewMessage(std_msgs.Float32MultiArray message) {
                //convert float array to double array
                double[] doubleMessage = new double[message.getData().length];
                for (int i = 0; i < doubleMessage.length; i++) {
                    doubleMessage[i] = (double) message.getData()[i];
                }
                //call to update any colour changes in the skeleton
                updateGlowList(doubleMessage);
            }
        });

        //constantly publish the markers
        while (run) {
            //get the latest kinect skeleton packet
            KinectDataPacket packet = kinectSubscriber.getStream().getPacket().copy();
            publishSkeleton(packet);
        }
    }

    /**
     * Publishes the markers that represent a skeleton based on a Kinect data
     * packet
     *
     * @param packet The Kinect data to base the visualisation on
     */
    private void publishSkeleton(KinectDataPacket packet) {
        //create a marker array to store all the markers
        MarkerArray markers = skeletonMarkerPub.newMessage();
        //this array will store pairs of vectors that require a line between them
        ArrayList<Vector> vectors = new ArrayList<Vector>();
        int id = 0;
        //iterate through every possible Kinect joint (head, neck, shoulder...)
        for (Type t : KinectPointType.asArray()) {
            //get the vector from the packet
            Vector v = packet.get(t);
            //create a marker from the vector
            Marker sphere = createSphere(v, id++);
            //alter the size if it is a head, hand or foot
            if (t == Type.HEAD) {
                sphere.getScale().setX(0.15);
                sphere.getScale().setY(0.15);
                sphere.getScale().setZ(0.15);
            } else if (t == Type.LEFT_HAND
                    || t == Type.RIGHT_HAND
                    || t == Type.LEFT_FOOT
                    || t == Type.RIGHT_FOOT) {
                sphere.getScale().setX(0.1);
                sphere.getScale().setY(0.1);
                sphere.getScale().setZ(0.1);
            }
            //alter the colour depending on the neural network result
            if (typesCanGlow.contains(t)) {
                if (glowValues.get(typesCanGlow.indexOf(t))) {
                    sphere.getColor().setR(0.0f);
                    sphere.getColor().setG(1.0f);
                } else {
                    sphere.getColor().setR(1.0f);
                    sphere.getColor().setG(0.0f);
                }
            }

            //add the sphere to the marker array
            markers.getMarkers().add(sphere);

            //add the vectors that require a line between them.
            //e.g: left shoulder -> torso
            switch (t) {
                case HEAD:
                    vectors.add(v);
                    vectors.add(packet.get(Type.NECK));
                    break;
                case LEFT_ELBOW:
                    vectors.add(v);
                    vectors.add(packet.get(Type.LEFT_SHOULDER));
                    vectors.add(v);
                    vectors.add(packet.get(Type.LEFT_HAND));
                    break;
                case LEFT_HIP:
                    vectors.add(v);
                    vectors.add(packet.get(Type.LEFT_KNEE));
                    break;
                case LEFT_KNEE:
                    vectors.add(v);
                    vectors.add(packet.get(Type.LEFT_FOOT));
                    break;
                case NECK:
                    vectors.add(v);
                    vectors.add(packet.get(Type.LEFT_SHOULDER));
                    vectors.add(v);
                    vectors.add(packet.get(Type.RIGHT_SHOULDER));
                    break;
                case RIGHT_ELBOW:
                    vectors.add(v);
                    vectors.add(packet.get(Type.RIGHT_SHOULDER));
                    vectors.add(v);
                    vectors.add(packet.get(Type.RIGHT_HAND));
                    break;
                case RIGHT_HIP:
                    vectors.add(v);
                    vectors.add(packet.get(Type.RIGHT_KNEE));
                    vectors.add(v);
                    vectors.add(packet.get(Type.LEFT_HIP));
                    break;
                case RIGHT_KNEE:
                    vectors.add(v);
                    vectors.add(packet.get(Type.RIGHT_FOOT));
                    break;
                case TORSO:
                    vectors.add(v);
                    vectors.add(packet.get(Type.LEFT_HIP));
                    vectors.add(v);
                    vectors.add(packet.get(Type.RIGHT_HIP));
                    vectors.add(v);
                    vectors.add(packet.get(Type.RIGHT_SHOULDER));
                    vectors.add(v);
                    vectors.add(packet.get(Type.LEFT_SHOULDER));
                    break;
            }
        }

        //add markers for the lines that join each point
        markers.getMarkers().add(lineArray(vectors, id++));
        //publish the marker array to the ros topic
        skeletonMarkerPub.publish(markers);
    }

    /**
     * This method creates a cylinder marker with all the properties set
     * (colour, size...)
     *
     * @param p The location of the marker
     * @param id The unique id of the marker
     * @return The marker object
     */
    private Marker createCylinder(Pose p, int id) {
        Marker marker = messageFactory.newFromType(Marker._TYPE);
        marker.setId(id);
        marker.getPose().getPosition().setX(p.getPosition().getX());
        marker.getPose().getPosition().setY(p.getPosition().getY());
        marker.getPose().getPosition().setZ(p.getPosition().getZ());
        marker.setType(Marker.CYLINDER);
        double radius = 0.1;
        marker.getScale().setX(radius);
        marker.getScale().setY(radius);
        marker.getScale().setZ(0.05f);
        marker.getColor().setA(0.7f);
        marker.getColor().setR(0.0f);
        marker.getColor().setG(1.0f);
        marker.getColor().setB(0.0f);
        marker.getHeader().setFrameId("/openni_depth_frame");
        marker.setLifetime(markerLifeTime);
        return marker;
    }

    /**
     * This method creates a sphere marker with all the properties set (colour,
     * size...)
     *
     * @param v The location of the marker
     * @param id The unique id of the marker
     * @return The marker object
     */
    private Marker createSphere(Vector v, int id) {
        Marker marker = messageFactory.newFromType(Marker._TYPE);
        marker.setId(id);
        marker.getPose().getPosition().setX(v.getX());
        marker.getPose().getPosition().setY(v.getY());
        marker.getPose().getPosition().setZ(v.getZ());
        marker.getPose().getOrientation().setX(0.0);
        marker.getPose().getOrientation().setY(0.0);
        marker.getPose().getOrientation().setZ(0.0);
        marker.getPose().getOrientation().setW(1.0);
        marker.setType(Marker.SPHERE);
        marker.getScale().setX(0.075);
        marker.getScale().setY(0.075);
        marker.getScale().setZ(0.075);
        marker.getColor().setA(0.9f);
        marker.getColor().setR(0.5f);
        marker.getColor().setG(0.5f);
        marker.getColor().setB(0.0f);
        marker.getHeader().setFrameId("/openni_depth_frame");
        marker.setLifetime(markerLifeTime);
        return marker;
    }

    /**
     * This method returns a marker that contains all lines for the skeleton
     *
     * @param vectors Includes pairs of vectors to draw lines between
     * @param id Unique i for this marker
     * @return The marker for all the lines in the skeleton
     */
    private Marker lineArray(ArrayList<Vector> vectors, int id) {
        Marker marker = messageFactory.newFromType(Marker._TYPE);
        marker.setId(id);
        marker.setType(Marker.LINE_LIST);
        marker.getScale().setX(0.01);
        marker.getScale().setZ(0.05f);
        marker.getColor().setA(0.9f);
        marker.getColor().setR(0.7f);
        marker.getColor().setG(0.7f);
        marker.getColor().setB(0.0f);
        marker.getHeader().setFrameId("/openni_depth_frame");
        marker.setLifetime(markerLifeTime);

        //iterate through the vectors in pairs (increment by 2)
        for (int i = 0; i < vectors.size() - 1; i += 2) {
            //create a line marker starting at vector 1 to vector 2
            Point point1 = messageFactory.newFromType(Point._TYPE);
            point1.setX(vectors.get(i).getX());
            point1.setY(vectors.get(i).getY());
            point1.setZ(vectors.get(i).getZ());
            Point point2 = messageFactory.newFromType(Point._TYPE);
            point2.setX(vectors.get(i + 1).getX());
            point2.setY(vectors.get(i + 1).getY());
            point2.setZ(vectors.get(i + 1).getZ());
            marker.getPoints().add(point1);
            marker.getPoints().add(point2);
        }
        return marker;
    }

    /**
     * This method updates colours of the points on the skeleton
     *
     * @param gestureData The result from the neural network
     */
    private void updateGlowList(double[] gestureData) {
        GestureType approxGesture = GestureNeuralNetwork.approxDoubleArrToGesture(gestureData);
        /*
         * Left: Left = green, Right = red
         * Right: Left = red, Right = green
         * Not: Left = red, Right = red
         */
        switch (approxGesture) {
            case LEFTPOINTING:
                glowValues.set(typesCanGlow.indexOf(Type.LEFT_HAND), true);
                glowValues.set(typesCanGlow.indexOf(Type.RIGHT_HAND), false);
                break;
            case RIGHTPOINTING:
                glowValues.set(typesCanGlow.indexOf(Type.LEFT_HAND), false);
                glowValues.set(typesCanGlow.indexOf(Type.RIGHT_HAND), true);
                break;
            default:
            case NOTPOINTING:
                glowValues.set(typesCanGlow.indexOf(Type.LEFT_HAND), false);
                glowValues.set(typesCanGlow.indexOf(Type.RIGHT_HAND), false);
                break;
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
