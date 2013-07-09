package main.java.misc;

import org.ros.node.topic.Publisher;
import sensor_msgs.JointState;

/**
 * This class controls the PTU on the robot by subscribing to the ptu state
 * topic and publishing to the ptu cmd topic
 *
 * @author hxs830
 */
public class PTUController {

    //final variables of limitations for the PTU
    public static final double MAXPANANGLE = 2.76;
    public static final double MINPANANGLE = -2.76;
    public static final double MAXTILTANGLE = 0.53;
    public static final double MINTILTANGLE = -0.8;
    public static final double MAXPANVEL = 3.0;
    public static final double MAXSPEED = 2.6;
    public static final double PANANGLERANGE = MAXPANANGLE - MINPANANGLE;
    public static final double TILTANGLERANGE = MAXTILTANGLE - MINTILTANGLE;
    //publisher for the ptu
    private Publisher<JointState> jointStatePub;
    //the set speed
    private double speed = 1;
    //target angles
    private double targetPanAngle = 0;
    private double targetTiltAngle = 0;
    //safe margin for ptu to be in from the target
    private double targetMargin = 0.01;
    //the last ptu data
    private JointState lastMessage;
    //if the target has been reached
    private boolean targetReached = true;
    private boolean allowStopCall = true;
    //if the ptu is currently moving
    private boolean isMoving = false;
    //used for error detection
    private long errorPTUWarningTimeout = 0;

    /**
     * Constructor
     *
     * @param jointStatePub The publisher to send JointStates to
     */
    public PTUController(Publisher<JointState> jointStatePub) {
        this.jointStatePub = jointStatePub;
    }

    /**
     * Called when a new JointState from the ptu is received Checks whether the
     * target has been reached
     *
     * @param message The jointState
     */
    public void onNewMessage(JointState message) {
        if (lastMessage != null) {
            if (lastMessage.getPosition()[0] != message.getPosition()[0]
                    || lastMessage.getPosition()[1] != message.getPosition()[1]) {
                isMoving = true;
                errorPTUWarningTimeout = System.currentTimeMillis();
            } else {
                isMoving = false;
            }
        }
        lastMessage = message;

        //check if pan & tilt are within angle margin
        if (Math.abs(message.getPosition()[0] - targetPanAngle) <= targetMargin
                && Math.abs(message.getPosition()[1] - targetTiltAngle) <= targetMargin) {
            targetReached = true;
        }
    }

    /**
     * @param speed The speed the ptu should move at
     */
    public void setSpeed(double speed) {
        if (speed > MAXSPEED) {
            speed = MAXSPEED;
        }
        if (speed < 0) {
            speed = 0;
        }
        this.speed = speed;
    }

    /*
     * Gets the current speed
     */
    public double getSpeed() {
        return speed;
    }

    /**
     * @return The current pan angle received from the ptu
     */
    public double getSubscribedPanAngle() {
        return lastMessage.getPosition()[0];
    }

    /*
     * @return The current tilt angle received from the ptu
     */
    public double getSubscribedTiltAngle() {
        return lastMessage.getPosition()[1];
    }

    /**
     * @return If the ptu is currently moving
     */
    public boolean isMoving() {
        return isMoving;
    }

    /**
     * @param panAngle The pan angle for the ptu to move to
     */
    public void goToPanAngle(double panAngle) {
        setPanAngle(panAngle);
        targetReached = false;
        allowStopCall = true;
        publish(targetPanAngle, targetTiltAngle);
    }

    /**
     * @param tiltAngle The tilt angle for the ptu to move to
     */
    public void goToTiltAngle(double tiltAngle) {
        setTiltAngle(tiltAngle);
        targetReached = false;
        allowStopCall = true;
        publish(targetPanAngle, targetTiltAngle);
    }

    /**
     * Goes to the specified pan and tilt angles
     *
     * @param panAngle The pan angle
     * @param tiltAngle The tilt angle
     */
    public void goToPanTiltAngle(double panAngle, double tiltAngle) {
        setPanAngle(panAngle);
        setTiltAngle(tiltAngle);
        targetReached = false;
        allowStopCall = true;
        publish(targetPanAngle, targetTiltAngle);
    }

    /**
     * Stops the ptu
     */
    public void stop() {
        if (lastMessage == null) {
            return;
        }
        if (allowStopCall) {
            allowStopCall = false;
            publish(lastMessage);
        }
    }

    /**
     * Detects if there is an error with the PTU.
     *
     * @return true if stuck
     */
    public boolean errorDetected() {
        return !targetReached && !isMoving && System.currentTimeMillis() - errorPTUWarningTimeout > 5000;
    }

    /**
     * Blocking method until ptu reaches target pan/tilt angles
     */
    public void waitTillReachedTarget() {
        while (!targetReached) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException ex) {
                //do nothing
            }
        }
    }

    /**
     * @return True if the ptu has reached the target pan & tilt angles
     */
    public boolean targetReached() {
        return targetReached;
    }

    /**
     * Checks the angle values
     *
     * @param panAngle The pan angle
     */
    private void setPanAngle(double panAngle) {
        if (panAngle > MAXPANANGLE) {
            panAngle = MAXPANANGLE;
        }
        if (panAngle < MINPANANGLE) {
            panAngle = MINPANANGLE;
        }
        targetPanAngle = panAngle;
    }

    /**
     * Checks the angle values
     *
     * @param tiltAngle The tilt angle
     */
    private void setTiltAngle(double tiltAngle) {
        if (tiltAngle > MAXTILTANGLE) {
            tiltAngle = MAXTILTANGLE;
        }
        if (tiltAngle < MINTILTANGLE) {
            tiltAngle = MINTILTANGLE;
        }
        targetTiltAngle = tiltAngle;
    }

    /**
     * Publishes a JointState onto the topic
     *
     * @param pan The pan angle
     * @param tilt The title Angle
     */
    private void publish(double pan, double tilt) {
        JointState newMessage = jointStatePub.newMessage();
        newMessage.getName().add("pan");
        newMessage.getName().add("tilt");
        newMessage.setPosition(new double[]{pan, tilt});
        newMessage.setVelocity(new double[]{speed, speed});
        publish(newMessage);
    }

    /**
     * Publishes a JointState onto the topic
     *
     * @param jointState The JointState to publish
     */
    private void publish(JointState jointState) {
        jointStatePub.publish(jointState);
        errorPTUWarningTimeout = System.currentTimeMillis();
    }
}
