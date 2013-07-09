package main.java.main;

import java.io.BufferedReader;
import java.io.FileReader;
import main.java.gui.NeuralNetworkGui;
import main.java.kinect.KinectDataBuffer;
import main.java.kinect.KinectDataSubscriber;
import main.java.misc.StaticMethods;
import main.java.neuralNetwork.main.GestureNeuralNetwork;
import main.java.neuralNetwork.main.IPointingEventListener;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import std_msgs.Float32MultiArray;

/**
 * This is a ros node that runs the neural network & gui to be compatible with
 * ros, subscribing to kinect data and publishing the networks output and
 * expected gestures
 *
 * @author hxs830
 */
public class NeuralNetworkNode implements NodeMain, IPointingEventListener {

    //subscribers
    private Subscriber<tf.tfMessage> tfSub;
    private Subscriber<std_msgs.String> personToTrackSub;
    private Subscriber<std_msgs.Float32MultiArray> gestureTrueResultSub;
    //publishers
    private Publisher<std_msgs.Float32MultiArray> gestureResultPub;
    private Publisher<std_msgs.Float32MultiArray> gestureTrueResultPub;
    //other
    private KinectDataSubscriber kinectSubscriber;
    private KinectDataBuffer dataBuffer;
    public static MessageFactory messageFactory;
    private final int TARGETPACKETCOUNT = 21; //number of frames to capture for NN
    public static int SKELETONNODESCOUNT = 2; //number of joints for the NN: 2 = left & right hand

    /**
     * @return Name of this ros node
     */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("neural_network_node");
    }

    /**
     * Called when the node is started
     *
     * @param node The node used to create subscribers and publishers
     */
    @Override
    public void onStart(ConnectedNode node) {
        System.out.println("Node Started");
        messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

        //create the Kinect data subscriber
        kinectSubscriber = new KinectDataSubscriber();
        dataBuffer = new KinectDataBuffer(kinectSubscriber.getStream(), TARGETPACKETCOUNT);
        //start thread to handle kinect data
        dataBuffer.start();

        //create the neurl network for gestures
        GestureNeuralNetwork network = new GestureNeuralNetwork();
        //adds the input layer with the correct number of input neurons
        network.addLayer((TARGETPACKETCOUNT - 1) * 3 * SKELETONNODESCOUNT); //3: xyz

        //read the parameters from the params.txt file
        String[] params = readParamsFile();
        //check if there are any parameters
        boolean hasParameters = params.length > 0;
        //either training or classifying
        boolean paramTraining = false;

        //if there are parameters then set the neural network according to the values
        if (hasParameters) {
            System.out.println("** Found parameters in file. Setting them.");
            //if it should be training or classifying
            paramTraining = ("t".equals(params[0]));
            //if training
            if (paramTraining) {
                System.out.println("** Training");
                System.out.println("Setting new neural network to " + params[2]);
                //add the hidden layers according to the parameter values
                for (String neuronCount : params[2].split("-")) {
                    network.addLayer(Integer.parseInt(neuronCount));
                }
            } else {
                System.out.println("** Classifying");
            }
        } else {
            //no parameter file, then add the best/test number of hidden layers
            //i.e: one hidden layer with 30 neurons
            System.out.println("No parameters found. Loading default.");
            network.addLayer(30);
        }

        //create the GUI for the neural network, providing the neural network and the kinect data buffer
        final NeuralNetworkGui gui = new NeuralNetworkGui(network, dataBuffer);
        gui.setVisible(true);
        //add the gui as an event listener
        dataBuffer.addEventListener(gui);

        //to publish ros boolean topic when is pointing event is fired
        network.addEventListener(this);
        //output layer
        network.addLayer(GestureNeuralNetwork.GestureType.values().length);

        //subscriber for transforms
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
                //check if the person to focus on has changed
                if (lastMessage == null || !message.getData().equals(lastMessage)) {
                    lastMessage = message.getData();
                    kinectSubscriber.setPersonToTrack(lastMessage);
                    //reset the buffer queue so that it starts again for the new person
                    dataBuffer.reset();
                }
            }
        });

        //setup  a subscriber for the gesture topic from recorded ros bag files
        gestureTrueResultSub = node.newSubscriber("/gesture_true_result", std_msgs.Float32MultiArray._TYPE);
        gestureTrueResultSub.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
            @Override
            public void onNewMessage(std_msgs.Float32MultiArray message) {
                if (gui.subscribeToTrueGestureValue) {
                    double[] doubleMessage = StaticMethods.floatArrToDoubleArr(message.getData());
                    gui.changeGestureType(GestureNeuralNetwork.doubleArrToGesture(doubleMessage));
                }
            }
        });

        //setup the publisher for the expected (true) gesture for when playing  ros bag files
        gestureTrueResultPub = node.newPublisher("/gesture_true_result", std_msgs.Float32MultiArray._TYPE);
        gestureResultPub = node.newPublisher("/gesture_result", std_msgs.Float32MultiArray._TYPE);

        if (hasParameters) {
            if (paramTraining) {
                try {
                    //load inputs
                    System.out.println("-Loading");
                    gui.loadFromFile("./saved_states/Experiments4/15TrainingInputs", false);
                    //output training error values
                    network.experiment = true;
                    network.trainingExperimentOutputFileName = "./saved_states/Experiments4/results/" + params[3];
                    //train
                    System.out.println("-Training");
                    gui.train();
                    //save
                    System.out.println("-Saving");
                    gui.saveToFile("./saved_states/Experiments4/results/" + params[1]);
                    //exit
                    System.exit(0);
                } catch (Exception ex) {
                    System.out.println("Unable to run training experiment! " + ex.getMessage());
                }
            } else {
                try {
                    dataBuffer.setUseQueueBuffer(true);
                    //load
                    System.out.println("-Loading");
                    gui.loadFromFile("./saved_states/Experiments4/results/" + params[1], true);
                    //set experiment variable to true -> write to file
                    gui.experiment = true;
                    gui.classificationExperimentOutputFileName = "./saved_states/Experiments4/results/" + params[2];
                    gui.subscribeToTrueGestureValue = true;
                    gui.classify = true;
                } catch (Exception ex) {
                    System.out.println("Unable to run classification experiment!" + ex.getMessage());
                }
            }
        }

        while (true) {
            if (gui.publishTrueGestureValue) {
                //publish true value
                publishGestureValue(gui.learningTypeArr, gestureTrueResultPub);
            }
            //if running classifier experiment and no data detected -> exit
            if (hasParameters
                    && !paramTraining
                    && dataBuffer.getLastDataTime() != -1
                    && (System.currentTimeMillis() - dataBuffer.getLastDataTime()) > 7000) {
                System.out.println("-No data detected. Exitting (experiment mode)");
                System.exit(0);
            }
            try {
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                //do nothing
            }
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
     * Called when the neural network has predicted a result
     *
     * @param value The output from th neural network
     */
    @Override
    public void networkResultComplete(double[] value) {
        publishGestureValue(value, gestureResultPub);
    }

    /**
     * This method publishes on the real gesture
     *
     * @param value The double array representation of the gesture
     * @param publisher The topic to publish on
     */
    private void publishGestureValue(double[] value, Publisher<std_msgs.Float32MultiArray> publisher) {
        Float32MultiArray message = publisher.newMessage();
        message.setData(StaticMethods.doubleArrToFloatArr(value));
        publisher.publish(message);
    }

    /**
     * This method reads the params.txt file, if it exists, that holds values
     * for the neural network
     *
     * @return A string array of all the values from the param file
     */
    private String[] readParamsFile() {
        String firstLine = "";
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader("./saved_states/Experiments4/params.txt"));
            firstLine = reader.readLine();
        } catch (Exception ex) {
            //do nothing
        }
        if ("".equals(firstLine)) {
            return new String[0];
        }
        return firstLine.split(" ");
    }
}
