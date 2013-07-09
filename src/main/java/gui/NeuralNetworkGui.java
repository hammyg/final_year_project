package main.java.gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.HeadlessException;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.text.DateFormat;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.DataLine;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import main.java.kinect.IKinectDataEventListener;
import main.java.kinect.KinectDataBuffer;
import main.java.kinect.KinectDataPacket;
import main.java.kinect.KinectPointType;
import main.java.main.NeuralNetworkNode;
import main.java.misc.SaveState;
import main.java.misc.Vector;
import main.java.neuralNetwork.NeuralNetworkLayer;
import main.java.neuralNetwork.main.GestureNeuralNetwork;
import main.java.neuralNetwork.main.GestureNeuralNetwork.GestureType;
import main.java.neuralNetwork.main.IPointingEventListener;

/**
 * This class provides a graphical user interface for the neural network
 *
 * @author hxs830
 */
public class NeuralNetworkGui extends JFrame implements ActionListener, KeyListener, IKinectDataEventListener {

    //status label
    private JLabel status = new JLabel();
    //the gesture neural network that will be used
    private GestureNeuralNetwork gestureNetwork;
    //if back propagation has been applied to train the neural network
    private boolean networkTrained = false;
    //the inputs and outputs for the training set
    private ArrayList<double[]> inputs = new ArrayList<double[]>();
    private ArrayList<double[]> outputs = new ArrayList<double[]>();
    //the current selected gesture
    private GestureNeuralNetwork.GestureType learningType =
            GestureNeuralNetwork.GestureType.NOTPOINTING;
    //the double array representation of the selected gesture
    public double[] learningTypeArr = GestureNeuralNetwork.gestureToDoubleArr(learningType);
    //the last gesture
    private GestureNeuralNetwork.GestureType lastLearningType = learningType;
    //if kinect data should be recorded and added to the training set
    private boolean record = false;
    //if the kinect data should be used for classification by the neural network
    public boolean classify = false;
    //the kinect buffer
    private KinectDataBuffer dataBuffer;
    //the record and slected gesture buttons
    private JButton recordButton;
    private JButton learnTypeButton;
    //this publishes the gesture type to ros
    public boolean publishTrueGestureValue = false;
    //this gets the gesture type from ros
    public boolean subscribeToTrueGestureValue = false;
    //if experimentation code should be run
    public boolean experiment = false;
    //the file name to store experiment data to
    public String classificationExperimentOutputFileName;
    //variables for the back prop
    private double trainingLearningRate = 0.2;
    private double trainingMomentum = 0.8;
    private int maxTrainingIterations = 20000;
    private int minTrainingError = -1;//-1: not checked

    /**
     * Creates a JFrame for th GUI for the provided network
     *
     * @param network A Gesture Neural Network for the GUI to use
     */
    public NeuralNetworkGui(GestureNeuralNetwork network, KinectDataBuffer dataHandler) {
        //sets properties for the JFrame
        setSize(600, 450);
        setLocationRelativeTo(null);
        setLayout(new GridBagLayout());
        setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        setTitle("Neural Network GUI");

        //sets the layout for the controls
        GridBagConstraints gc = new GridBagConstraints();

        //adds the load and save buttons
        gc.fill = GridBagConstraints.BOTH;
        //add North buttons
        gc.anchor = GridBagConstraints.NORTH;
        gc.gridy = 0;
        gc.weighty = 1;
        JPanel topPanel = new JPanel(new FlowLayout());
        topPanel.add(newButton("Load NN"));
        topPanel.add(newButton("Save NN"));
        getContentPane().add(topPanel, gc);

        //sets and adds status label
        gc.gridy = 1;
        gc.weighty = 1;
        gc.weightx = 1;
        gc.anchor = GridBagConstraints.CENTER;
        JPanel centerPanel = new JPanel();
        centerPanel.setLayout(new BorderLayout());
        status.setFont(new Font("Serif", Font.BOLD, 32));
        status.setHorizontalAlignment(SwingConstants.CENTER);
        centerPanel.add(status, BorderLayout.CENTER);
        setStatusText("Ready...", Color.RED);
        getContentPane().add(centerPanel, gc);

        //adds remaining buttons at the bottom of the JFrame
        gc.gridy = 2;
        gc.weighty = 1;
        gc.anchor = GridBagConstraints.SOUTH;
        JPanel bottomPanel = new JPanel(new FlowLayout());
        recordButton = newButton("Record");
        bottomPanel.add(recordButton);
        bottomPanel.add(newButton("Remove Last Input"));
        bottomPanel.add(newButton("Clear Buffer"));
        bottomPanel.add(newButton("Publish G"));
        bottomPanel.add(newButton("Subscribe G"));
        learnTypeButton = newButton(learningType.name());
        learnTypeButton.setActionCommand("learnTypeButton");
        bottomPanel.add(learnTypeButton);
        bottomPanel.add(newButton("Train NN"));
        bottomPanel.add(newButton("Classify"));
        getContentPane().add(bottomPanel, gc);

        //registers keyboard press event for the whole JFrame and not individual objects
        KeyboardFocusManager manager = KeyboardFocusManager.getCurrentKeyboardFocusManager();
        manager.addKeyEventDispatcher(new MyDispatcher(this));

        this.gestureNetwork = network;
        this.gestureNetwork.resetNetwork();
        this.dataBuffer = dataHandler;
    }

    /**
     * @return Returns the Kinect Data Buffer
     */
    public KinectDataBuffer getKinectDataBuffer() {
        return dataBuffer;
    }

    /**
     * Creates a JButton with the action command listener set
     *
     * @param text The text of the button and the action command
     * @return A JButton
     */
    public JButton newButton(String text) {
        JButton button = new JButton(text);
        button.addActionListener(this);
        button.setActionCommand(text);
        return button;
    }

    /**
     * Updates the font colour of the status label
     *
     * @param forecolour The new colour to change to
     */
    public void setStatusForecolour(Color forecolour) {
        status.setForeground(forecolour);
    }

    /**
     * Updates the text of the status label
     *
     * @param text The new text. Can contain \n for new lines
     */
    public void setStatusText(String text) {
        // \n only works if html tags are added
        if (text.contains("\n")) {
            status.setText("<html>" + text.replace("\n", "<br>") + "</html>");
        } else {
            status.setText(text);
        }
    }

    /**
     * Updates the text and colour of the status label
     *
     * @param text The new text to update to
     * @param forecolour the new colour to update to
     */
    public void setStatusText(String text, Color forecolour) {
        setStatusForecolour(forecolour);
        setStatusText(text);
    }

    /**
     * Method is called for every button created by the newButton method
     *
     * @param ae The event that called this method. Action command can be
     * extracted from the event
     */
    @Override
    public void actionPerformed(ActionEvent ae) {
        //get the JButton that called this method
        JButton button = (JButton) ae.getSource();
        //check if the load button was clicked
        if (ae.getActionCommand().equals("Load NN")) {
            open();
            //check if the save button was clicked
        } else if (ae.getActionCommand().equals("Save NN")) {
            save();
            //check if the record button was clicked
        } else if (ae.getActionCommand().equals("Record")) {
            record();
            //check if the stop recording button was clicked
        } else if (ae.getActionCommand().equals("Stop Recording")) {
            stopRecord();
            //check if the remove last input button was clicked
        } else if (ae.getActionCommand().equals("Remove Last Input")) {
            //check if there are inputs before removing
            if (inputs.size() > 0) {
                //remove last item from input and output array
                inputs.remove(inputs.size() - 1);
                outputs.remove(outputs.size() - 1);
                setStatusText("Removed last input\nInput size: " + inputs.size());
                printAllInputs();
            } else {
                //update status to warn user
                setStatusText("Nothing to remove.");
            }
            //check if the clear buffer button was clicked
        } else if (ae.getActionCommand().equals("Clear Buffer")) {
            dataBuffer.reset();
            //check if the publish gesture button was clicked
        } else if (ae.getActionCommand().equals("Publish G")) {
            publishTrueGestureValue = true;
            //change button text and action command to stop
            button.setText("Stop Pub G");
            button.setActionCommand("Stop Pub G");
            //check if the stop publishing gesture button was clicked
        } else if (ae.getActionCommand().equals("Stop Pub G")) {
            publishTrueGestureValue = false;
            //change button text and action command to original
            button.setText("Publish G");
            button.setActionCommand("Publish G");
            //check if subscribe to gesture button was clicked
        } else if (ae.getActionCommand().equals("Subscribe G")) {
            subscribeToTrueGestureValue = true;
            //change button text and action command to stop subscribing
            button.setText("Stop Sub G");
            button.setActionCommand("Stop Sub G");
            //check if the stop subscribing button was clicked
        } else if (ae.getActionCommand().equals("Stop Sub G")) {
            subscribeToTrueGestureValue = false;
            //revert button text and action command to its original
            button.setText("Subscribe G");
            button.setActionCommand("Subscribe G");
            //check if the gesture type (output) button was clicked
        } else if (ae.getActionCommand().equals("learnTypeButton")) {
            //update gesture output to next one
            changeGestureType(true);
            //check if the train neural network button was clicked
        } else if (ae.getActionCommand().equals("Train NN")) {
            //if still recording, inform user
            if (record) {
                setStatusText("Stop recording first!");
                //check if network has not already been trained
            } else if (!networkTrained) {
                //check if there are inputs to train on
                if (inputs.size() == 0) {
                    setStatusText("Add inputs first!");
                } else {
                    train();
                }
            }
            //check if the classify gesture button was clicked
        } else if (ae.getActionCommand().equals("Classify")) {
            //check if the neural network is not trained, inform user
            if (!networkTrained) {
                setStatusText("Network not trained!");
            } else {
                classify = true;
                if (dataBuffer != null) {
                    //reset the buffer and change the method to use the queue
                    System.out.println("Buffer changed to queue.");
                    dataBuffer.setUseQueueBuffer(true);
                }
                setStatusText("Classifying");
                //change button to stop classifying
                button.setText("Stop Classifying");
                button.setActionCommand("Stop Classifying");
            }
            //check if the stop classifying button was clicked
        } else if (ae.getActionCommand().equals("Stop Classifying")) {
            classify = false;
            //change text and action command to its original
            button.setText("Classify");
            button.setActionCommand("Classify");
        }
    }

    /**
     * Starts record inputs for the training set.
     */
    private void record() {
        if (!networkTrained) {
            setStatusText("Recording");
            record = true;
            if (dataBuffer != null) {
                //uses array as the buffer method
                //also resets buffer
                dataBuffer.setUseQueueBuffer(false);
            }
            recordButton.setText("Stop Recording");
            recordButton.setActionCommand("Stop Recording");
        }
    }

    /**
     * Stops adding data for the training set
     */
    private void stopRecord() {
        if (record) {
            setStatusText("Recording Stopped\nInput size: " + inputs.size());
            record = false;
            recordButton.setText("Record");
            recordButton.setActionCommand("Record");
        }
    }

    /**
     * Updates the gesture type output
     *
     * @param next Updates it to the next/previous gesture on the list
     */
    private void changeGestureType(boolean next) {
        ArrayList<GestureNeuralNetwork.GestureType> types = new ArrayList<GestureType>(Arrays.asList(GestureNeuralNetwork.GestureType.values()));
        int nextIndex;
        if (next) {
            nextIndex = (types.indexOf(learningType) + 1) % types.size();
        } else {
            int i = (types.indexOf(learningType) - 1);
            if (i < 0) {
                i = types.size() - 1;
            }
            nextIndex = i % types.size();
        }
        changeGestureType(types.get(nextIndex));
    }

    /**
     * Updates the learning gesture type and updates the double array
     * representation of the gesture
     *
     * @param type The gesture to update to
     */
    public void changeGestureType(GestureNeuralNetwork.GestureType type) {
        if (learningType != GestureNeuralNetwork.GestureType.NOTPOINTING) {
            lastLearningType = learningType;
        }
        learningType = type;
        //updates the double array representation
        learningTypeArr = GestureNeuralNetwork.gestureToDoubleArr(type);
        learnTypeButton.setText(learningType.name());
    }

    /**
     * Converts two vectors into 3 inputs (x, y, z) for a neural network
     *
     * @param v1 Vector 1
     * @param v2 Vector 2
     * @return A double array (x, y, z)
     */
    private double[] convertToNNInputs(Vector v1, Vector v2) {
        double[] data = new double[3]; //x, y, z
        //calculate the magnitude and direction for each dimesion
        data[0] = v2.getX() - v1.getX();
        data[1] = v2.getY() - v1.getY();
        data[2] = v2.getZ() - v1.getZ();
        return data;
    }

    /**
     * Given a list of data packets, a double array is created containing the
     * vectors between two data packets, for both hands
     *
     * @param data An array of data packets to use for creating the inputs for
     * the neural network
     * @return A double array for the neural network inputs
     */
    private double[] convertToNNInputs(KinectDataPacket[] data) {
        //how many nodes (joints) from the data packet to use
        //both hands = 2
        int typesCount = NeuralNetworkNode.SKELETONNODESCOUNT;
        //create a double array with the correct size to store everything
        double[] nnInputData = new double[(data.length - 1) * 3 * typesCount]; //3: x,y,z
        //loop through all but one of the of the data packets
        for (int i = 0; i < data.length - 1; i++) {
            //retrieve the double array for the vectors of the left hand button the current
            //packet and the next
            double[] nnLHInputs = convertToNNInputs(
                    data[i].get(KinectPointType.Type.LEFT_HAND),
                    data[i + 1].get(KinectPointType.Type.LEFT_HAND));
            //place them in to the double array that needs to be returned
            nnInputData[(i * typesCount * 3)] = nnLHInputs[0];
            nnInputData[(i * typesCount * 3) + 1] = nnLHInputs[1];
            nnInputData[(i * typesCount * 3) + 2] = nnLHInputs[2];

            //apply the same for the right hand but place the 
            //values after the left hand vector values
            double[] nnRHInputs = convertToNNInputs(
                    data[i].get(KinectPointType.Type.RIGHT_HAND),
                    data[i + 1].get(KinectPointType.Type.RIGHT_HAND));
            nnInputData[(i * typesCount * 3) + 3] = nnRHInputs[0];
            nnInputData[(i * typesCount * 3) + 4] = nnRHInputs[1];
            nnInputData[(i * typesCount * 3) + 5] = nnRHInputs[2];
        }
        return nnInputData;
    }
    //Decimal for used to show a formatted value of the output
    private DecimalFormat df = new DecimalFormat("#.##");

    /**
     * Called when the data buffer is ready/filled
     *
     * @param packets The content of the data buffer
     */
    @Override
    public void newKinectData(KinectDataPacket[] packets) {
        //convert the whole array to a double array of value
        //suitable for neural network inputs
        double[] nNInputs = convertToNNInputs(packets);
        //if it is set to record and the network is not trained,
        //then add the double aray to the training set
        if (!networkTrained && record) {
            inputs.add(nNInputs);
            //add the gesture type to the output
            outputs.add(learningTypeArr);
            setStatusText("Input size: " + inputs.size()
                    + "\nLearning: " + learningType.name());
            printAllInputs();
        }

        //if the network has been trained and is set to classify (predict the gesture)
        if (networkTrained && classify) {
            //use the converted inputs to calculate the output from the neural network
            //the double array (output) will be of length 3, with values 0-1 for each gesture
            double[] output = gestureNetwork.getOutput(nNInputs);
            //logical output is the output with some conditions applied
            double[] logicalOutput;

            //apply logic to select the predicted gesture to be published
            double leftPointingValue = -1;
            double rightPointingValue = -1;
            GestureType logicalOutputType = GestureType.NOTPOINTING;
            double threshold = 0.20;
            //extract the left and right values from the output array
            for (int i = 0; i < GestureType.values().length; i++) {
                GestureType t = GestureType.values()[i];
                if (t == GestureType.LEFTPOINTING) {
                    leftPointingValue = output[i];
                } else if (t == GestureType.RIGHTPOINTING) {
                    rightPointingValue = output[i];
                }
            }
            //check if the left or right value is above or equal to the threshold
            if (Math.max(leftPointingValue, rightPointingValue) >= threshold) {
                //select the highest value and set the gesture type
                if (leftPointingValue > rightPointingValue) {
                    logicalOutputType = GestureType.LEFTPOINTING;
                } else {
                    logicalOutputType = GestureType.RIGHTPOINTING;
                }
            }
            //set the display text and highlight the gesture
            String displayString = "";
            String printString = "";
            for (int i = 0; i < GestureType.values().length; i++) {
                GestureType t = GestureType.values()[i];
                if (t == logicalOutputType) {
                    //change colour to green for the predicted gesture
                    displayString += "<font color=green>" + t.name() + "</font>\n";
                } else {
                    displayString += t.name() + "\n";
                }
                printString += t.name() + ": " + df.format(output[i]) + "\n";
            }
            setStatusText(displayString);
            //print the classifcation
            System.out.println("Classification: " + logicalOutputType.name()
                    + "\nData:\n" + printString);
            //convert the logical output gesture type to a double array
            logicalOutput = GestureNeuralNetwork.gestureToDoubleArr(logicalOutputType);
            //notify the listener about the output
            gestureNetwork.fireNetworkResultComplete(logicalOutput);
            //if experimenting, then write output to csv file
            if (experiment) {
                writeDataToFile(learningTypeArr, logicalOutput);
            }
        }
    }

    /**
     * Trains the neural network using back propagation On completion, sound is
     * played and window is set to the front
     */
    public void train() {
        setStatusText("Training...");
        gestureNetwork.setTrainginSet(inputs.toArray(new double[inputs.size()][inputs.get(0).length]), //same as (TARGETLENGTH - 1) * 3
                outputs.toArray(new double[outputs.size()][outputs.get(0).length]) //same as GestureNeuralNetwork.GestureType.values().length
                );
        gestureNetwork.train(trainingLearningRate, trainingMomentum, maxTrainingIterations, minTrainingError);
        networkTrained = true;
        setStatusText("Completed Training");
        System.out.println("Completed Training");
        System.out.println("Time Taken: " + gestureNetwork.getTimeTakenToTrain() + "ms");
        //bring jframe to the front
        toFront();
        repaint();
        //play audio clip
        playCompletedAudio();
    }

    /**
     * Opens a file chooser for the user to select a directory to save the
     * neural network to
     */
    private void save() {
        JFileChooser fc = new JFileChooser("./saved_states/");
        if (fc.showSaveDialog(this) == JFileChooser.APPROVE_OPTION) {
            try {
                saveToFile(fc.getSelectedFile().getAbsolutePath());
            } catch (Exception ex) {
                Logger.getLogger(NeuralNetworkGui.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    /**
     * Serialises and save the neural network to a file
     *
     * @param fileName The file path and name to save to
     * @throws IOException Thrown is an error occurs during serialisation and
     * saving
     */
    public void saveToFile(String fileName) throws IOException {
        //create a new object to store all the relevant objects required for the neural network
        SaveState state = new SaveState();
        //set the relevant objects
        state.setNetwork(gestureNetwork.getNetwork());
        state.setBackprop(gestureNetwork.getTrainer());
        state.setInputs(inputs);
        state.setOutputs(outputs);
        state.setIsTrained(networkTrained);
        state.setDate(Calendar.getInstance().getTime());
        state.setDescription("Data Handler Packet Count: " + dataBuffer.getBufferSize()
                + ", Learning Data Sets: " + inputs.size()
                + ", NN Inputs: " + gestureNetwork.getNetwork().getLayers().get(0).getNeuronCount()
                + ", NN Outputs: " + gestureNetwork.getNetwork().getLayers().get(gestureNetwork.getNetwork().getLayers().size() - 1).getNeuronCount());
        //Creates a file output and object stream to write the Save State to
        FileOutputStream fout = new FileOutputStream(fileName);
        ObjectOutputStream oos = new ObjectOutputStream(fout);
        oos.writeObject(state);
        oos.close();

        setStatusText("Saved to\n" + fileName);
        System.out.println("Saved to:");
        System.out.println(fileName);
        System.out.println("Description:");
        System.out.println(state.getDescription());
    }

    /**
     * Displays a file chooser to the user for a location of a stored neural
     * network
     */
    private void open() {
        JFileChooser fc = new JFileChooser("./saved_states/");
        fc.setFileSelectionMode(JFileChooser.FILES_AND_DIRECTORIES);
        //if the user did not cancel the file chooser
        if (fc.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
            try {
                //gets the path of the selected file
                String fileName = fc.getSelectedFile().getAbsolutePath();
                boolean replaceNN = false;
                //ask the user if only the training set should be loaded
                if (JOptionPane.showConfirmDialog(this,
                        "Inputs and results have been restored.\n"
                        + "However, would you also like to replace the network?",
                        "Replace Network?",
                        JOptionPane.YES_NO_OPTION,
                        JOptionPane.QUESTION_MESSAGE) == JOptionPane.YES_OPTION) {
                    replaceNN = true;
                }
                //loads the neural network from the speficied path
                //and whether or not to only load the training set
                loadFromFile(fileName, replaceNN);
            } catch (Exception ex) {
                Logger.getLogger(NeuralNetworkGui.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    /**
     * Loads the saved state for a neural network
     *
     * @param fileName The path of the saved state
     * @param includeNN If the neural network in the saved state should be
     * loaded
     * @throws FileNotFoundException Thrown if the file does not exist
     * @throws IOException Thrown if an error occurred while opening the file
     * @throws ClassNotFoundException Thrown if unable to cast the read in
     * object
     */
    public void loadFromFile(String fileName, boolean includeNN) throws FileNotFoundException, IOException, ClassNotFoundException {
        FileInputStream fin = new FileInputStream(fileName);
        ObjectInputStream ois = new ObjectInputStream(fin);
        //reads the object from the file and casts it to a save state
        SaveState state = (SaveState) ois.readObject();
        ois.close();
        networkTrained = false;
        //restores the training set
        inputs = state.getInputs();
        outputs = state.getOutputs();

        //if the neural network shouldbe restored
        if (includeNN) {
            //temproraily store the event listener of the current neural network
            List<IPointingEventListener> previousEventListeners = gestureNetwork.getEventListeners();
            //restore the neural network from the save state
            gestureNetwork = new GestureNeuralNetwork(state.getNetwork(), state.getBackprop());

            //event listeners are not stored in the saved state.
            //it must be stored before and set after creating a new GestureNeuralNetwork
            gestureNetwork.getEventListeners().addAll(previousEventListeners);

            //restore if the neural network is trained
            networkTrained = state.isTrained();
            System.out.println("Network replaced.");
        } else {
            System.out.println("Network not over-written.");
        }

        //print the layer structure
        System.out.println("--- Layers ---");
        for (int i = 0; i < gestureNetwork.getNetwork().getLayers().size(); i++) {
            NeuralNetworkLayer layer = gestureNetwork.getNetwork().getLayers().get(i);
            System.out.println("Layer " + i + ": " + layer.getNeuronCount());
        }

        System.out.println("------------");
        //print the error of the neural network if it was restored
        if (includeNN) {
            System.out.println("Final Error:" + gestureNetwork.getTrainer().getCurrentNetowrkError());
        }
        setStatusText("Loaded:\n" + fileName);
        System.out.println("Loaded:");
        System.out.println(fileName);
        System.out.println("Description:");
        System.out.println(state.getDescription());
        DateFormat dateFormat = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
        System.out.println("Date:");
        System.out.println(dateFormat.format(state.getDate()));
        printAllInputs();
    }

    @Override
    public void keyTyped(KeyEvent ke) {
    }

    @Override
    public void keyPressed(KeyEvent ke) {
    }

    /**
     * Method is called when a key is pressed from the keyboard. This is used to
     * interface with the wireless usb presenter
     *
     * @param ke They key press event
     */
    @Override
    public void keyReleased(KeyEvent ke) {
        switch (ke.getKeyCode()) {
            case 34:
                //right: change gesture type to the next 
                changeGestureType(true);
                break;
            case 33:
                //left: change the gesture type to the previous
                changeGestureType(false);
                break;
            case 66:
                //topL: toggle gesture type with not pointing
                if (learningType == GestureNeuralNetwork.GestureType.NOTPOINTING) {
                    changeGestureType(lastLearningType);
                } else {
                    changeGestureType(GestureNeuralNetwork.GestureType.NOTPOINTING);
                }
                break;
            case 116:
            case 27:
                //topR: start and stop recording for the training set
                if (record) {
                    stopRecord();
                } else {
                    record();
                }
                break;
        }
    }

    /**
     * Plays an audio file. Used to inform the user when the neural network has
     * completed training as it is time consuming
     */
    public void playCompletedAudio() {
        AudioInputStream soundIn = null;
        try {
            File soundFile = new File("./training_complete.wav");
            soundIn = AudioSystem.getAudioInputStream(soundFile);
            AudioFormat format = soundIn.getFormat();
            DataLine.Info info = new DataLine.Info(Clip.class, format);
            Clip clip = (Clip) AudioSystem.getLine(info);
            clip.open(soundIn);
            clip.start();
            while (clip.isRunning()) {
                Thread.yield();
            }
        } catch (Exception ex) {
            Logger.getLogger(NeuralNetworkGui.class.getName()).log(Level.SEVERE, null, ex);
        } finally {
            try {
                soundIn.close();
            } catch (IOException ex) {
                Logger.getLogger(NeuralNetworkGui.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    /**
     * Class is used as a global key press event listener All event key presses
     * are the directed to the GUI class
     */
    private class MyDispatcher implements KeyEventDispatcher {
        //right: pressed & released: 34
        //left: pressed & released: 33
        //topL: typed:0, released: 66
        //topR: typed:0 , released: 116, 27

        KeyListener keyListenerClass;

        /**
         * Constructor
         *
         * @param keyListenerClass Takes a KeyListener to direct the key press
         * events to
         */
        public MyDispatcher(KeyListener keyListenerClass) {
            this.keyListenerClass = keyListenerClass;
        }

        /**
         * Called when a key press event happens
         *
         * @param e The key press event
         * @return always returns false to abort the key press from continuing
         */
        @Override
        public boolean dispatchKeyEvent(KeyEvent e) {
            //key press type is checked and the correct method in the key listener is called
            if (e.getID() == KeyEvent.KEY_PRESSED) {
                keyListenerClass.keyPressed(e);
            } else if (e.getID() == KeyEvent.KEY_RELEASED) {
                keyListenerClass.keyReleased(e);
            } else if (e.getID() == KeyEvent.KEY_TYPED) {
                keyListenerClass.keyTyped(e);
            }
            return false;
        }
    }

    /**
     * Prints all the inputs and outputs in the training set
     */
    private void printAllInputs() {
        System.out.println("---Input/Results---");
        for (int i = 0; i < inputs.size(); i++) {
            System.out.println(i + ": "
                    + GestureNeuralNetwork.doubleArrToGesture(outputs.get(i)));
        }
    }
    //buffered writer for the experiment file data
    private BufferedWriter out;
    //if the file has been created
    private boolean fileCreated = false;

    /**
     * Creates the experimentation CSV file with a file name current time and
     * data
     */
    private void createFile() {
        FileWriter fstream = null;
        try {
            if ("".equals(classificationExperimentOutputFileName)) {
                Date date = new Date();
                SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH-mm-ss");
                classificationExperimentOutputFileName = "./saved_states/Experiments3/" + dateFormat.format(date) + "-classifications.csv";
            }
            //create the file
            File file = new File(classificationExperimentOutputFileName);
            fstream = new FileWriter(file);
            out = new BufferedWriter(fstream);
            fileCreated = true;
            //write the headings for the file in comma seperated format
            out.write("\n\n,Expected,Result,,,isPointingExcepted,isPointingResult,TP,FP,TN,FN\n");
        } catch (IOException ex) {
            Logger.getLogger(NeuralNetworkGui.class.getName()).log(Level.SEVERE, null, ex);
            try {
                fstream.close();
            } catch (IOException ex1) {
                //do nothing
            }
        }
    }
    //keeps track of the line number
    private int lineCount = 0;

    /**
     * Writes data to the experimentation file in CSV format
     *
     * @param expectedGesture The expected gesture double array
     * @param resultGesture The predicted gesture double array
     */
    private void writeDataToFile(double[] expectedGesture, double[] resultGesture) {
        //check if the file has not been created
        if (!fileCreated) {
            createFile();
        }
        try {
            //the data to write
            String line = ""; //isPointingExcepted,isPointingResult,TP,FP,TN,FN
            //convert the double arrays to gesture types
            GestureType expectedGestureType = GestureNeuralNetwork.approxDoubleArrToGesture(expectedGesture);
            GestureType resultGestureType = GestureNeuralNetwork.approxDoubleArrToGesture(resultGesture);

            //write the appropiate value by comparing the two gesture types
            //values determine if they are true or false, and
            //true/false positive, true/false negative
            if (expectedGestureType == GestureType.NOTPOINTING) {
                line += "0";
            } else {
                line += "1";
            }
            if (resultGestureType == GestureType.NOTPOINTING) {
                line += ",0";
            } else {
                line += ",1";
            }
            if (resultGestureType == GestureType.NOTPOINTING) {
                if (expectedGestureType == resultGestureType) {
                    //TN
                    line += ",0,0,1,0";
                } else {
                    //FN
                    line += ",0,0,0,1";
                }
            } else {
                //is pointing
                if (expectedGestureType == GestureType.LEFTPOINTING
                        || expectedGestureType == GestureType.RIGHTPOINTING) {
                    //TP
                    line += ",1,0,0,0";
                } else {
                    //FP
                    line += ",0,1,0,0";
                }
            }
            //write the line with the line number before it
            out.write(lineCount++
                    + ","
                    + expectedGestureType.name()
                    + ","
                    + resultGestureType.name()
                    + ",,," + line + "\n");
            out.flush();
        } catch (IOException ex) {
            Logger.getLogger(NeuralNetworkGui.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}
