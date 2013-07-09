package main.java.main;

import actionlib_msgs.GoalStatus;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Twist;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;
import javax.vecmath.Point3d;
import main.java.kinect.KinectDataPacket;
import main.java.kinect.KinectDataSubscriber;
import main.java.kinect.KinectPointType;
import main.java.misc.PTUController;
import main.java.misc.StaticMethods;
import main.java.misc.Vector;
import main.java.neuralNetwork.main.GestureNeuralNetwork;
import main.java.neuralNetwork.main.GestureNeuralNetwork.GestureType;
import main.java.tfjava.StampedTransform;
import main.java.tfjava.TFListener;
import nav_msgs.OccupancyGrid;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.JointState;
import std_msgs.Bool;

/**
 *
 * @author hxs830
 */
public class FSMNode implements NodeMain {

    //all the possible states in the machine
    enum State {

        INIT,
        COMPLETEDINIT,
        SCANNINGFORPERSON,
        SCANNINGPAUSED,
        COMPLETEDSCANSECTION,
        COMPLETEDWHOLESCAN,
        PERSONFOUND,
        PERSONLOST,
        WAITINGFORINPUT,
        NAVIGATINGTOGOAL,
        REACHEDGOAL
    }

    enum Hand {

        LEFT, RIGHT
    }
    //parameters
    //duration of not seeing someone before PERSONLOST
    private final int PERSONFOUNDTIMEOUT = 2000;
    //how long to wait between each scan turn section
    private final int TURNINGPAUSETIMEOUT = 5000;
    //number of times to pause rotation (4 = 90 degrees)
    private final int SCANNINGSECTIONS = 7;
    //time to wait before accepting gestures. Reduces erros when person comes into frame
    private final int DELAYBEFOREREADINGINPUT = 3000;
    //the time after pointing to stop reading
    private final int DELAYBEFOREENDINGGESTURE = 500;
    private final double ROBOTRADIUS = 0.26;
    private final double DISTANCEFROMGOALMARGIN = ROBOTRADIUS;
    private final double SCANNINGPTUTILTANGLE = -0.2;
    private final double SCANNINGPTUSPEED = 2;
    private final double[] NAVIGATINGPANTILTANGLES = new double[]{0.0, -0.5};
    //number of complete scans when no one is found before moving to previous location
    private final int COMPLETESCANSBEFORERETURNING = 1;
    //subscribers
    private Subscriber<tf.tfMessage> tfSub;
    private Subscriber<std_msgs.String> personToTrackSub;
    private Subscriber<OccupancyGrid> mapSub;
    private Subscriber<std_msgs.Float32MultiArray> gestureResultSub;
    private Subscriber<PoseWithCovarianceStamped> amclPoseSub;
    private Subscriber<actionlib_msgs.GoalStatusArray> moveBaseStatusSub;
    private Subscriber<PoseStamped> moveBaseGoalSub;
    private Subscriber<JointState> ptuSub;
    private Subscriber<Odometry> odomSub;
    //publishers
    private Publisher<PoseStamped> moveBaseGoalPub;
    private Publisher<actionlib_msgs.GoalID> moveBaseCancelPub;
    private Publisher<JointState> ptuPub;
    private Publisher<Bool> useKLaserScanPub;
    private Publisher<Twist> cmdVelPub;
    //other
    private ConnectedNode node;
    private MessageFactory messageFactory;
    private State currentState;
    private OccupancyGrid map;
    private KinectDataSubscriber kinectSubscriber;
    private long lastTimePersonFound = 0;
    private PTUController ptuController;
    private long timeWhenTurningPaused = 0;
    private long firstTimePersonFound = 0;
    private Pose lastEstimatedAMCLPose;
    private TFListener tfListener;
    private GestureNeuralNetwork.GestureType gestureResult;
    private Pose lastMoveBaseGoal;
    private int scanSectionCount = 0; //how many sections it has covered
    private double scanningTurnAngle = PTUController.PANANGLERANGE / (SCANNINGSECTIONS - 1);//Math.toRadians(60);//Math.PI * 2 / SCANNINGSECTIONS;
    private boolean moveBaseFailed = false;
    private boolean trackPerson = false;
    private List<Pose> posesWherePeopleWhereFound;
    private long timeBeforeStopReadingGestures = 0;
    private static String lastSpeech = "";
    private static BufferedWriter talkOut;

    /**
     * Empty constructor
     */
    public FSMNode() {
    }

    /**
     * @return Name of the node
     */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("fsm_node");
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
        talk("Starting.");
        //set the initial state
        changeState(State.INIT);

        this.node = node;
        //message factory used to create ros objects
        messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
        StaticMethods.messageFactory = messageFactory;
        posesWherePeopleWhereFound = new ArrayList<Pose>();

        //publisher for the move_base navigation goal pose
        moveBaseGoalPub = node.newPublisher("/move_base_simple/goal", PoseStamped._TYPE);
        //publisher to cancel move base navigation to goal
        moveBaseCancelPub = node.newPublisher("/move_base/cancel", actionlib_msgs.GoalID._TYPE);
        //publisher for ptu commands
        ptuPub = node.newPublisher("/ptu/cmd", JointState._TYPE);
        //publisher to tell laser combiner if it should combine base laser with kinect laser
        useKLaserScanPub = node.newPublisher("/use_kinect_laser_scan", Bool._TYPE);
        //publisher to stop any movement
        cmdVelPub = node.newPublisher("/b21/cmd_vel", Twist._TYPE);

        //kinect subscriber to extract kinect data from tf
        kinectSubscriber = new KinectDataSubscriber();
        //instantiate the ptu controller
        ptuController = new PTUController(ptuPub);
        //tf listener to allow for conversions between tf frames
        tfListener = new TFListener(messageFactory);


        //set up subscribers and anything else
        //subscriber for amcl estimated pose
        amclPoseSub = node.newSubscriber("/amcl_pose", PoseWithCovarianceStamped._TYPE);
        amclPoseSub.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
            @Override
            public void onNewMessage(PoseWithCovarianceStamped message) {
                lastEstimatedAMCLPose = message.getPose().getPose();
            }
        });

        //subscriber for map
        mapSub = node.newSubscriber("/map", OccupancyGrid._TYPE);
        mapSub.addMessageListener(new MessageListener<OccupancyGrid>() {
            @Override
            public void onNewMessage(OccupancyGrid message) {
                map = message;
            }
        });

        //subscriber for ptu states
        ptuSub = node.newSubscriber("/ptu/state", JointState._TYPE);
        ptuSub.addMessageListener(new MessageListener<JointState>() {
            @Override
            public void onNewMessage(JointState message) {
                ptuController.onNewMessage(message);
            }
        });

        //subsciber to odom
        odomSub = node.newSubscriber("/b21/odom", Odometry._TYPE);
        odomSub.addMessageListener(new MessageListener<Odometry>() {
            @Override
            public void onNewMessage(Odometry t) {
                /*
                 * move base can be in a state where it turns 360 degrees
                 * without any goals.
                 * This counter acts this undeterministic behaviour
                 */
                if (currentState != State.NAVIGATINGTOGOAL
                        && Math.abs(t.getTwist().getTwist().getAngular().getZ()) > 0.1) {
                    stopCMDVel();
                    //System.out.println("ODOM STOP!");
                }
            }
        });

        //subscriber for transformations
        tfSub = node.newSubscriber("/tf", tf.tfMessage._TYPE);
        tfSub.addMessageListener(new MessageListener<tf.tfMessage>() {
            @Override
            public void onNewMessage(tf.tfMessage message) {
                tfListener.call(message);
                if (currentState == State.INIT) {
                    return;
                }
                boolean personFound = kinectSubscriber.onNewMessage(message);
                if (personFound) {
                    lastTimePersonFound = System.currentTimeMillis();
                    if (currentState == State.SCANNINGFORPERSON
                            || currentState == State.COMPLETEDSCANSECTION
                            || currentState == State.SCANNINGPAUSED) {
                        changeState(State.PERSONFOUND);
                    }
                    if (trackPerson && kinectSubscriber.getStream().packetAvailable()) {
                        //disabled
                        //ptuTrackPerson(kinectSubscriber.getStream().peek());
                    }
                }
            }
        });

        //subscriber for the gesture result from the neural network
        gestureResultSub = node.newSubscriber("/gesture_result", std_msgs.Float32MultiArray._TYPE);
        gestureResultSub.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
            private GestureType lastGesture;

            @Override
            public void onNewMessage(std_msgs.Float32MultiArray message) {
                if (currentState == State.INIT) {
                    return;
                }
                //convert float array to double array
                double[] doubleMessage = StaticMethods.floatArrToDoubleArr(message.getData());
                gestureResult = GestureNeuralNetwork.approxDoubleArrToGesture(doubleMessage);

                if (gestureResult != GestureType.NOTPOINTING) {
                    if (gestureResult != lastGesture) {
                        if (currentState == State.WAITINGFORINPUT) {
                            System.out.println("Ignoring " + gestureResult + " as it is an anomily");
                        }
                        lastGesture = gestureResult;
                        gestureResult = GestureType.NOTPOINTING;
                    } else {
                        lastGesture = gestureResult;
                    }
                } else {
                    lastGesture = gestureResult;
                }
            }
        });

        //subscriber to check which person to track from kinect
        personToTrackSub = node.newSubscriber("/tracking_person_number", std_msgs.String._TYPE);
        personToTrackSub.addMessageListener(new MessageListener<std_msgs.String>() {
            String lastMessage;

            @Override
            public void onNewMessage(std_msgs.String message) {
                //check if the person to be tracked has changed
                if (lastMessage == null || !message.getData().equals(lastMessage)) {
                    lastMessage = message.getData();
                    kinectSubscriber.setPersonToTrack(lastMessage);
                }
            }
        });

        //subscriber for move base status to know if it fails to find a plan
        moveBaseStatusSub = node.newSubscriber("/move_base/status", actionlib_msgs.GoalStatusArray._TYPE);
        moveBaseStatusSub.addMessageListener(new MessageListener<actionlib_msgs.GoalStatusArray>() {
            byte lastMoveBaseStatus = Byte.MAX_VALUE;
            byte FAILEDSTATUS = 4;
            byte NAVIGATINGSTATUS = 1;
            byte GOALACCEPTEDSTATUS = 6;
            byte STANDBYSTATUS = 2;
            byte realLastValue = 0;

            @Override
            public void onNewMessage(actionlib_msgs.GoalStatusArray message) {
                List<GoalStatus> statusList = message.getStatusList();
                //check if there are any statuses
                if (!statusList.isEmpty()) {
                    //get the state code
                    byte moveBaseStatus = statusList.get(statusList.size() - 1).getStatus();
                    //check if it has changed
                    if (realLastValue != moveBaseStatus) {
                        realLastValue = moveBaseStatus;
                        System.out.println("*** move base status updated to: "
                                + moveBaseStatus + " - " + statusList.get(statusList.size() - 1).getText());
                    }
                    //check if move base failed
                    if (moveBaseStatus == FAILEDSTATUS) {
                        if (lastMoveBaseStatus != FAILEDSTATUS) {
                            //failed
                            lastMoveBaseStatus = moveBaseStatus;
                            moveBaseFailed = true;
                        }
                    } else {
                        //not failed
                        lastMoveBaseStatus = moveBaseStatus;
                        moveBaseFailed = false;
                    }
                    //if fsm statue is not supposed to be moving,
                    //but move base is moving, then stop
                    if (currentState != State.WAITINGFORINPUT
                            && currentState != State.NAVIGATINGTOGOAL
                            && moveBaseStatus != STANDBYSTATUS
                            && moveBaseStatus != FAILEDSTATUS) {
                        stopMoveBaseNavigation();
                        stopCMDVel();
                        System.out.println("Move base still navigating! Stopping move base navigation!");
                    }
                }
                if (currentState != State.INIT && currentState != State.NAVIGATINGTOGOAL) {
                    //stop any movement if the robot is not in the 
                    //navigation state
                    stopCMDVel();
                }
            }
        });

        //subscriber for the move base goal
        moveBaseGoalSub = node.newSubscriber("/move_base_simple/goal", PoseStamped._TYPE);
        moveBaseGoalSub.addMessageListener(new MessageListener<PoseStamped>() {
            @Override
            public void onNewMessage(PoseStamped message) {
                lastMoveBaseGoal = message.getPose();
            }
        });

        //stop the move base naviation
        stopMoveBaseNavigation();
        //put the ptu at the navigation position
        ptuNavPosition();
        //set laser combiner to start combining
        useKinectLaserScan(true);

        //wait for the map
        while (map == null) {
            System.out.println("Waiting for map...");
            talkOnce("Waiting for the map...");
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                //do nothing
            }
        }

        //wait for the amcl pose while moving in attempt to trigger amcl
        int rotatationState = 1;
        while (lastEstimatedAMCLPose == null) {
            System.out.println("Waiting for first amcl pose...");
            talkOnce("Waiting for first a.m.c.l pose...");
            if (rotatationState == -1) {
                cmdVelRotate(-0.1);
                rotatationState = 1;
            } else {
                cmdVelRotate(0.1);
                rotatationState = -1;
            }
            try {
                Thread.sleep(2500);
            } catch (InterruptedException ex) {
                //do nothing
            }
        }

        //stop rotating
        cmdVelRotate(0);

        changeState(State.COMPLETEDINIT);
        int numberOfCompleteScans = 0;
        Hand hand = null;
        long lastPTUErrorWarning = 0;
        while (true) {
            //first thing to do
            if (currentState == State.COMPLETEDINIT) {
                //scan
                scan(true);
            }
            //change state to completed section scanning when ptu reaches target
            if (currentState == State.SCANNINGFORPERSON && ptuController.targetReached()) {
                changeState(State.COMPLETEDSCANSECTION);
            }

            //if error has been detected with the ptu, then notify user
            if (ptuController.errorDetected() && System.currentTimeMillis() - lastPTUErrorWarning > 5000) {
                FSMNode.talk("Warning. Detected error with pan tilt unit.");
                System.out.println("Error with Pan Tilt Unit. Reset PTU & restart PTU server.");
                lastPTUErrorWarning = System.currentTimeMillis();
            }

            //if ptu has covered a section without finding anyone
            if (currentState == State.COMPLETEDSCANSECTION) {
                //completed scan without finding anyone
                scanSectionCount++;
                System.out.println("Scanned " + scanSectionCount + " out of " + SCANNINGSECTIONS);
                timeWhenTurningPaused = System.currentTimeMillis();
                changeState(State.SCANNINGPAUSED);
            }

            //pause before moving to next section to scan
            if (currentState == State.SCANNINGPAUSED) {
                if (System.currentTimeMillis() - timeWhenTurningPaused
                        > TURNINGPAUSETIMEOUT) {
                    if (scanSectionCount < SCANNINGSECTIONS) {
                        scan(false);
                    } else {
                        changeState(State.COMPLETEDWHOLESCAN);
                    }
                }
            }

            //if all sections have been covered
            if (currentState == State.COMPLETEDWHOLESCAN) {
                numberOfCompleteScans++;
                int lastPersonPoseIndex = posesWherePeopleWhereFound.size() - 1;
                //if completed maximum number of scans then navigate to previous location
                if (numberOfCompleteScans >= COMPLETESCANSBEFORERETURNING
                        && lastPersonPoseIndex != -1) {
                    if (StaticMethods.distanceBetweenPoses(posesWherePeopleWhereFound.get(lastPersonPoseIndex),
                            lastEstimatedAMCLPose)
                            >= DISTANCEFROMGOALMARGIN) {
                        navigate(posesWherePeopleWhereFound.get(lastPersonPoseIndex));
                        numberOfCompleteScans = 0;
                        talkOnce("No one found! Returning to previous location");
                        System.out.println("Completed full scan - no one found - Returning to previous location...");
                    }
                    posesWherePeopleWhereFound.remove(lastPersonPoseIndex);
                } else {
                    //else, rescan
                    scan(true);
                    talkOnce("No one found! Re-scanning...");
                    System.out.println("Completed full scan - no one found - Rescanning...");
                }

            }

            //if person has been lost, then continue scanning
            if (currentState == State.PERSONLOST) {
                scan(false);
            }

            //if person has been found, then change state to waiting for input
            if (currentState == State.PERSONFOUND) {
                ptuController.stop();
                talkOnce("Found person " + kinectSubscriber.getPersonNumberToTrack() + ".");
                firstTimePersonFound = System.currentTimeMillis();
                trackPerson = true;
                changeState(State.WAITINGFORINPUT);
            }

            //if state is waiting for input, then wiat for nn or if lost person
            if (currentState == State.WAITINGFORINPUT) {
                if (System.currentTimeMillis() - lastTimePersonFound > PERSONFOUNDTIMEOUT) {
                    talk("Lost Person.");
                    trackPerson = false;
                    //return to previous section to rescan for person
                    if (scanSectionCount > 0) {
                        scanSectionCount--;
                    }
                    changeState(State.PERSONLOST);
                    continue;
                }

                //add delay as a person coming into view could be seen as a gesture
                if (System.currentTimeMillis() - firstTimePersonFound < DELAYBEFOREREADINGINPUT) {
                    continue;
                }

                //if the ptu is moving, then ignore
                if (ptuController.isMoving()) {
                    System.out.println("Ignoring gestures as PTU is moving!");
                    hand = null;
                    //lastTimePersonFound = System.currentTimeMillis();
                    firstTimePersonFound = System.currentTimeMillis() - (DELAYBEFOREREADINGINPUT / 2);
                    continue;
                }

                //if nn is predicting not pointing
                if (gestureResult == GestureNeuralNetwork.GestureType.NOTPOINTING) {
                    if (hand == null) {
                        //notify user about gesturing
                        talkOnce("I am waiting for you to gesture...");
                    } else {
                        //when user has stopped pointing, then select the correct hand
                        if (System.currentTimeMillis() - timeBeforeStopReadingGestures > DELAYBEFOREENDINGGESTURE) {
                            if (hand == Hand.LEFT) {
                                talkOnce("You pointed with your RIGHT hand.");
                            } else {
                                talkOnce("You pointed with your LEFT hand.");
                            }

                            //calculate the target location to navigate to
                            Pose newMoveBaseGoal = validTargetMapPose(
                                    kinectSubscriber.getStream().getPacket().copy(), hand);

                            //if the target location is where the robot currently is, then dismiss it
                            if (StaticMethods.distanceBetweenPoses(newMoveBaseGoal, lastEstimatedAMCLPose)
                                    < DISTANCEFROMGOALMARGIN) {
                                talk("I am already at that location...");
                            } else {
                                talk("Navigating to that location.");
                                navigate(newMoveBaseGoal);
                                posesWherePeopleWhereFound.add(lastEstimatedAMCLPose);
                            }
                            hand = null;
                        }
                    }
                } else {
                    timeBeforeStopReadingGestures = System.currentTimeMillis();
                    if (gestureResult == GestureNeuralNetwork.GestureType.LEFTPOINTING) {
                        hand = Hand.LEFT;
                    } else {
                        hand = Hand.RIGHT;
                    }
                }
            }


            if (currentState == State.NAVIGATINGTOGOAL) {
                //if navigating and move base has failed
                if (moveBaseFailed) {
                    talk("Failed to find a valid plan.");
                    stopMoveBaseNavigation();
                    ptuController.goToPanTiltAngle(0, SCANNINGPTUTILTANGLE);
                    ptuController.waitTillReachedTarget();
                    scan(true);
                    continue;
                }
                //if navigating and has reached target location
                if (StaticMethods.distanceBetweenPoses(lastMoveBaseGoal, lastEstimatedAMCLPose)
                        < DISTANCEFROMGOALMARGIN) {
                    stopMoveBaseNavigation();
                    talk("I have reached the target location.");
                    ptuController.goToPanTiltAngle(0, SCANNINGPTUTILTANGLE);
                    ptuController.waitTillReachedTarget();
                    scan(true);
                }
            }
        }
    }

    /**
     * Publishes location for the move base
     *
     * @param newMoveBaseGoal The new location to navigate to
     */
    private void navigate(Pose newMoveBaseGoal) {
        trackPerson = false;
        scanSectionCount = 0;
        ptuController.setSpeed(PTUController.MAXSPEED);
        ptuNavPosition();
        //wait till ptu is in correct position
        ptuController.waitTillReachedTarget();
        //publish the move base goal
        publishMoveBaseGoal(newMoveBaseGoal);
        lastMoveBaseGoal = newMoveBaseGoal;
        changeState(State.NAVIGATINGTOGOAL);
    }

    /**
     * Controls the ptu to centre the person currently being tracked
     *
     * @param kinectPacket The kinect data packet
     */
    private void ptuTrackPerson(KinectDataPacket kinectPacket) {
        if (!ptuController.targetReached()) {
            return;
        }

        //calculate pan and tilt angles to centre the person
        Vector pointV = kinectPacket.get(KinectPointType.Type.NECK);
        double targetPanAngle = ptuController.getSubscribedPanAngle() + (Math.toRadians(20) * pointV.getY());
        double targetTiltAngle = ptuController.getSubscribedTiltAngle() + (Math.toRadians(15) * pointV.getZ());
        boolean adjustPan = Math.abs(pointV.getY()) > 0.3; //0.3
        boolean adjustTilt = Math.abs(pointV.getZ()) > 0.1 && targetTiltAngle < 0.1 && targetTiltAngle > -0.3; //not too high/low
        if (targetPanAngle > PTUController.MAXPANANGLE || targetPanAngle < PTUController.MINPANANGLE) {
            adjustPan = false;
        }
        if (targetTiltAngle > PTUController.MAXTILTANGLE || targetTiltAngle < PTUController.MINTILTANGLE) {
            adjustTilt = false;
        }

        //this speed does not loose track of person too easily
        ptuController.setSpeed(0.25);
        if (adjustPan && adjustTilt) {
            ptuController.goToPanTiltAngle(targetPanAngle, targetTiltAngle);
        } else if (adjustTilt) {
            ptuController.goToTiltAngle(targetTiltAngle);
        } else if (adjustPan) {
            ptuController.goToPanAngle(targetPanAngle);
        } else {
            ptuController.stop();
        }
    }

    /**
     * Set the ptu to the ideal angle for using its 3d point cloud as laser
     */
    private void ptuNavPosition() {
        ptuController.goToPanTiltAngle(NAVIGATINGPANTILTANGLES[0], NAVIGATINGPANTILTANGLES[1]);
    }

    /**
     * Scans a section for people in the robots surround area
     *
     * @param resetCounter If the section that should be scanned should be reset
     */
    private void scan(boolean resetCounter) {
        if (resetCounter) {
            scanSectionCount = 0;
        }
        changeState(State.SCANNINGFORPERSON);
        ptuController.setSpeed(SCANNINGPTUSPEED);
        int midSection = (int) (SCANNINGSECTIONS / 2.0);
        double panAngle;
        //allow the first pan angle to be infront,
        //then scan right to left (skipping middle section)
        if (scanSectionCount == 0) {
            panAngle = PTUController.MINPANANGLE + (midSection * scanningTurnAngle);
        } else if (scanSectionCount > midSection) {
            //if it's the middle section or beyond it, skip and goto next angle (centre to left)
            panAngle = PTUController.MINPANANGLE + (scanSectionCount * scanningTurnAngle);
        } else {
            //turn to angle of section in reverse order. (centre to right)
            panAngle = PTUController.MINPANANGLE + ((midSection - scanSectionCount) * scanningTurnAngle);
        }
        //make sure it reaches the correct tilt before scanning
        //removes errors when tracking begins before reaching correct tilt angle
        ptuController.goToTiltAngle(SCANNINGPTUTILTANGLE);
        ptuController.waitTillReachedTarget();
        ptuController.goToPanAngle(panAngle);
        talkOnce("Looking for someone.");
    }

    /**
     * Sends a cancel message for the move base to stop current navigation
     */
    private void stopMoveBaseNavigation() {
        moveBaseCancelPub.publish(moveBaseCancelPub.newMessage());
        System.out.println("Cancelling any move_base navigation goals");
    }

    /**
     * Publishes twist commands to stop the robot
     */
    private void stopCMDVel() {
        cmdVelPub.publish(cmdVelPub.newMessage());
    }

    /**
     * Publishes twist commands to rotate the robot
     *
     * @param radiansPerSec The rate of rotation
     */
    private void cmdVelRotate(double radiansPerSec) {
        Twist twist = cmdVelPub.newMessage();
        twist.getAngular().setZ(radiansPerSec);
        cmdVelPub.publish(twist);
    }

    /**
     * Publishes the pose to the move base topic
     *
     * @param pose the new target location
     */
    private void publishMoveBaseGoal(Pose pose) {
        PoseStamped poseStamped = moveBaseGoalPub.newMessage();
        poseStamped.setPose(pose);
        poseStamped.getHeader().setFrameId("/map");
        moveBaseGoalPub.publish(poseStamped);
        moveBaseFailed = false;
    }

    /**
     * Calculates the target pose on a map given the location of the elbow and
     * hand taking occupied cells into consideration
     *
     * @param packet The kinect data packet containing all positions of skeleton
     * @param hand Calculate for left hand or right hand
     * @return A valid target location on the map
     */
    private Pose validTargetMapPose(KinectDataPacket packet, Hand hand) {
        Vector handV;
        Vector elbowV;
        if (hand == Hand.LEFT) {
            handV = packet.get(KinectPointType.Type.LEFT_HAND);
            elbowV = packet.get(KinectPointType.Type.LEFT_ELBOW);
        } else {
            handV = packet.get(KinectPointType.Type.RIGHT_HAND);
            elbowV = packet.get(KinectPointType.Type.RIGHT_ELBOW);
        }
        Pose validTargetMapPose = validTargetMapPose(elbowV, handV);
        return validTargetMapPose;
    }

    /**
     * Ray tracing to target location using the elbow and hand
     *
     * @param elbow Vector of elbow
     * @param handV Vector of hand
     * @return A valid location on the map the person is pointing to
     */
    private Pose validTargetMapPose(Vector elbowV, Vector handV) {
        double targetZval = -1; //the Z depth of the map floor
        double rayStep = map.getInfo().getResolution();//0.5;

        Vector targetV;
        double rayDistance = 0;
        //ray trace until Z reaches specified value
        //create new pose and set coordinates
        Pose targetP = messageFactory.newFromType(Pose._TYPE);
        Pose targetPOnMap;
        boolean backTrace = false;
        do {
            if (!backTrace) {
                //increment the ray trace distance
                rayDistance += rayStep;
            } else {
                //decrement the ray trace distance
                rayDistance -= rayStep;
            }
            //get the position of the distance
            targetV = vectorDistanceFromHand(elbowV, handV, rayDistance);

            //Vector to Pose
            targetP.getPosition().setX(targetV.getX());
            targetP.getPosition().setY(targetV.getY());
            targetP.getPosition().setZ(targetV.getZ());
            //transform the Pose onto the map
            targetPOnMap = transformToMapFrame(targetP);
            if (targetPOnMap != null) {
                //first check the target cell is not occupied
                //then backtrace until suitable area for robot to be in is found
                if (!backTrace) {
                    //check if ray has hit the floor
                    //or if hit an occupied cell.
                    if (targetV.getZ() <= targetZval
                            || StaticMethods.cellIsOccupied(
                            targetPOnMap.getPosition().getX(),
                            targetPOnMap.getPosition().getY(),
                            map)) {
                        backTrace = true;
                    }
                } else {
                    //check if not backtracing to behind hand
                    //or backtrace suitable area for robot is found.
                    if (rayDistance <= 0
                            || StaticMethods.legalAreaForRobot(
                            targetPOnMap.getPosition().getX(),
                            targetPOnMap.getPosition().getY(),
                            ROBOTRADIUS + 0.1,
                            map)) {
                        break;
                    }
                }
            } else {
                System.out.println("Target not on map!");
            }
        } while (true);

        //reset the Z dimension (3D ray tracing position to 2D position on map)
        targetPOnMap.getPosition().setZ(0);
        //TODO: set heading
        targetPOnMap.getOrientation().setW(1);
        return targetPOnMap;
    }

    /**
     * Calculates the 3D location a certain distance from the hand
     *
     * @param elbow Vector of elbow
     * @param hand Vector of hand
     * @param distance Distance from hand
     * @return A vector location
     */
    private Vector vectorDistanceFromHand(Vector elbow, Vector hand, double distance) {
        double tX = (hand.getX() - elbow.getX()) * distance;
        double tY = (hand.getY() - elbow.getY()) * distance;
        double tZ = (hand.getZ() - elbow.getZ()) * distance;
        tX += hand.getX();
        tY += hand.getY();
        tZ += hand.getZ();

        return new Vector(tX, tY, tZ);
    }

    /**
     * Transforms an openni_depth Pose to a map Pose
     *
     * @param p openni_depth Pose
     * @return p transformed on to the map
     */
    private Pose transformToMapFrame(Pose p) {
        StampedTransform lookupTransform = tfListener.lookupTransform("/map", "/openni_depth_frame", node.getCurrentTime());
        if (lookupTransform == null) {
            System.out.println("Transform is null!");
            return null;
        } else {

            Point3d newPoint = new Point3d();
            lookupTransform.transformPoint(
                    new Point3d(
                    p.getPosition().getX(),
                    p.getPosition().getY(),
                    p.getPosition().getZ()),
                    newPoint);
            //Point3d to Pose
            Pose transformedPose = messageFactory.newFromType(Pose._TYPE);
            transformedPose.setOrientation(p.getOrientation());
            transformedPose.getPosition().setX(newPoint.x);
            transformedPose.getPosition().setY(newPoint.y);
            transformedPose.getPosition().setZ(newPoint.z);
            return transformedPose;
        }
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

    /**
     * Changes the state of the FSMNode
     *
     * @param newState The new state of the FSMNode
     */
    private void changeState(State newState) {
        if (newState != currentState) {
            System.out.println("---- State updated to: " + newState + " ----");
        }
        currentState = newState;

        //don't publish when state is initialising as the publishers are not ready yet
        if (currentState != State.INIT) {
            useKinectLaserScan(currentState == State.NAVIGATINGTOGOAL);
        }
    }

    /**
     * Publishes on the topic to notify the laser combiner node to combine both
     * laser scans
     *
     * @param use Boolean to combine
     */
    private void useKinectLaserScan(boolean use) {
        Bool newMessage = useKLaserScanPub.newMessage();
        newMessage.setData(use);
        useKLaserScanPub.publish(newMessage);
    }

    /**
     * Writes to the DoubleTalk device on the B21
     *
     * @param toSpeak The text to say
     * @return If it was successful
     */
    public static boolean talk(String toSpeak) {
        try {
            System.out.println("Talk: " + toSpeak);
            lastSpeech = toSpeak;
            if (talkOut == null) {
                Process p = Runtime.getRuntime().exec("/bin/bash");
                talkOut = new BufferedWriter(new OutputStreamWriter(p.getOutputStream()));
            }
            talkOut.write("echo " + toSpeak + " > /dev/ttyR0\n");
            talkOut.flush();
            return true;
        } catch (IOException ex) {
            System.out.println("Could not talk: " + ex.getMessage());
            return false;
        }
    }

    /**
     * If last speech was the same, then it will not say it
     *
     * @param toSpeak The text to speak
     * @return If it was successful
     */
    public static boolean talkOnce(String toSpeak) {
        if (toSpeak.equalsIgnoreCase(lastSpeech)) {
            return true;
        }
        return talk(toSpeak);
    }

    /**
     * used to test the ptu
     */
    private void testPTU() {
        int i = 0;
        while (i == 0) {
            try {
                //set pan and tilt angles to test the ptu controller
                System.out.println("Speed: " + ptuController.getSpeed());
                System.out.println("MAX...");
                ptuController.goToPanTiltAngle(PTUController.MAXPANANGLE, PTUController.MAXTILTANGLE);
                System.out.println("1 Waiting...");
                while (!ptuController.targetReached()) {
                    System.out.println("1 target not reached");
                    Thread.sleep(100);
                }
                System.out.println("1 Target reached!");
                Thread.sleep(1000);
                System.out.println("MIN");
                ptuController.goToPanTiltAngle(PTUController.MINPANANGLE, PTUController.MINTILTANGLE);
                System.out.println("2 Waiting...");
                while (!ptuController.targetReached()) {
                    System.out.println("2 target not reached");
                    Thread.sleep(100);
                }
                System.out.println("2 Target reached!");
                Thread.sleep(1000);
                ptuController.setSpeed(ptuController.getSpeed() + 1);
            } catch (InterruptedException ex) {
                //do nothing
            }
        }
    }
}
