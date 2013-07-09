package main.java.neuralNetwork.main;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import main.java.neuralNetwork.BackpropTrainer;
import main.java.neuralNetwork.NeuralNetwork;
import main.java.neuralNetwork.NeuralNetworkLayer;

/**
 * Brings the neural network and the back propagation together, with the event
 * listeners
 *
 * @author hxs830
 */
public class GestureNeuralNetwork {

    //the neural network
    private NeuralNetwork network;
    //the back propagation trainer
    private BackpropTrainer trainer;
    //the event listeners when the neural network outputs
    private List<IPointingEventListener> eventListeners = new ArrayList<IPointingEventListener>();
    public long timeTakenToTrain = 0;
    //whether or not to run experimenation code
    public boolean experiment = false;
    //output file for experiment data
    public String trainingExperimentOutputFileName;
    //if the experiment file has been created
    private boolean experimentOutputFileCreated = false;
    //the buffered writer
    private BufferedWriter out;
    //the training set inputs and outputs
    private double inputs[][];
    private double outputs[][];

    //the available gestures
    public enum GestureType {

        LEFTPOINTING, RIGHTPOINTING, NOTPOINTING,}

    /**
     * Constructor. Creates the neural network
     */
    public GestureNeuralNetwork() {
        network = new NeuralNetwork();
    }

    /**
     * Constructor used when restoring from save state
     *
     * @param network The neural network
     * @param trainer The back propagation trainer
     */
    public GestureNeuralNetwork(NeuralNetwork network, BackpropTrainer trainer) {
        this.network = network;
        this.trainer = trainer;
    }

    /**
     * @return The neural network
     */
    public NeuralNetwork getNetwork() {
        return network;
    }

    /**
     * @return The back propagation trainer
     */
    public BackpropTrainer getTrainer() {
        return trainer;
    }

    /**
     * @param listener The listener to add
     */
    public synchronized void addEventListener(IPointingEventListener listener) {
        eventListeners.add(listener);
    }

    /**
     * @return All the listeners
     */
    public List<IPointingEventListener> getEventListeners() {
        return eventListeners;
    }

    /**
     * Notifies all the listeners about the output
     *
     * @param output The neural network output
     */
    public synchronized void fireNetworkResultComplete(double[] output) {
        Iterator i = eventListeners.iterator();
        while (i.hasNext()) {
            ((IPointingEventListener) i.next()).networkResultComplete(output);
        }
    }

    /**
     * Adds a layer to the neural network
     *
     * @param neurons Number of neurons for this layer
     */
    public void addLayer(int neurons) {
        network.addNewLayer(new NeuralNetworkLayer(neurons));
    }

    /**
     * Resets all neural network layers
     */
    public void resetNetwork() {
        network.resetAllLayers();
    }

    /**
     * @return The time taken to apply back propagation
     */
    public long getTimeTakenToTrain() {
        return timeTakenToTrain;
    }

    /**
     * Set the training set for the back propagation
     *
     * @param inputs The training set inputs
     * @param outputs The training set outputs
     */
    public void setTrainginSet(double inputs[][], double outputs[][]) {
        this.inputs = inputs;
        this.outputs = outputs;
    }

    /**
     * Trains the neural network by applying back propagation
     *
     * @param learningRate The learning rate to apply
     * @param momentum The momentum to apply
     * @param maxIterations The maximum number of iterations
     * @param maxErrorValue The maximum error value allowed: -1 means error is
     * not checked
     */
    public void train(double learningRate, double momentum, double maxIterations,
            double maxErrorValue) {
        trainer = new BackpropTrainer(network);
        trainer.setLearningRate(learningRate);
        trainer.setMomentum(momentum);
        trainer.setTrainginSet(inputs, outputs);

        //if running experiments
        if (experiment) {
            if (!experimentOutputFileCreated) {
                createFile();
            }
            //output to the file
            try {
                out.write("Number of inputs: " + inputs.length + "\n"
                        + "Size of inputs: " + inputs[0].length + "\n"
                        + "Learning rate: " + learningRate + "\n"
                        + "Momentum: " + momentum + "\n");
                out.write("\nIteration,Error\n");
                out.flush();
            } catch (IOException ex) {
                //do nothing
            }
        }

        long startTime = System.currentTimeMillis();
        int iteration = 1;
        do {
            //apply one back propagation step
            trainer.TrainStep();
            //only print every 250 steps
            if (iteration == 1 || iteration % 250 == 0) {
                System.out.println("Iteration #" + iteration + " Error:" + trainer.getCurrentNetowrkError());
                if (experiment) {
                    writeErrorToFile(iteration, trainer.getCurrentNetowrkError());
                }
            }

            iteration++;
            //check the condition on when to stop applying back propagation
        } while ((trainer.getCurrentNetowrkError()
                > maxErrorValue && ((maxIterations == -1 || iteration < maxIterations))));
        System.out.println("Final Iteration #" + iteration + " Error:" + trainer.getCurrentNetowrkError());

        if (experiment) {
            //write to experimentation file
            writeErrorToFile(iteration, trainer.getCurrentNetowrkError());
        }

        timeTakenToTrain = System.currentTimeMillis() - startTime;

        if (experiment) {
            try {
                out.write("\nTime taken: " + timeTakenToTrain + " ms");
                out.flush();
            } catch (IOException ex) {
                //do nothing
            }
        }
    }

    /**
     * Writes data to the experimentation file
     *
     * @param iteration The iteration number
     * @param error The neural networks current error
     */
    private void writeErrorToFile(double iteration, double error) {
        try {
            out.write(iteration + ","
                    + trainer.getCurrentNetowrkError() + "\n");
        } catch (IOException ex) {
            System.out.println("Error writing to file." + ex.getMessage());
        }
    }

    /**
     * Calculates the output from the input on the neural network
     *
     * @param input The input gesture to predict
     * @return The neural networks predicted output gesture
     */
    public double[] getOutput(double[] input) {
        double[] output = network.calculateOutputs(input);
        return normalise(output);
        //  return output;
    }

    /**
     * Normalises the outputs so the summation = 1
     *
     * @param data The neural networks output
     * @return Normalised output
     */
    public double[] normalise(double[] data) {
        double[] normalisedData = new double[data.length];
        double sum = 0;
        for (int i = 0; i < data.length; i++) {
            sum += data[i];
        }
        for (int i = 0; i < data.length; i++) {
            normalisedData[i] = data[i] / sum;
        }
        return normalisedData;
    }

    /**
     * used for testing the neural network
     */
    public static void main(final String args[]) {
        //traingin set inputs
        double GestureInputs[][] = {
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {2.0, 5.0, 0.0},};

        //training set outputs
        double GestureResults[][] = {
            gestureToDoubleArr(GestureType.LEFTPOINTING),
            gestureToDoubleArr(GestureType.LEFTPOINTING),
            gestureToDoubleArr(GestureType.LEFTPOINTING),
            gestureToDoubleArr(GestureType.NOTPOINTING),};

        //set up the neural network
        GestureNeuralNetwork gestureNetwork = new GestureNeuralNetwork();
        gestureNetwork.addLayer(GestureInputs[0].length);
        gestureNetwork.addLayer(10);
        gestureNetwork.addLayer(GestureType.values().length);
        gestureNetwork.resetNetwork();

        System.out.println("Start");
        // train the neural network
        //iterations = 5000
        gestureNetwork.setTrainginSet(GestureInputs, GestureResults);
        gestureNetwork.train(0.7, 0.9, -1, 0.001);
        System.out.println("End training");
        // test the neural network
        System.out.println("Neural Network Results:");
        for (int i = 0;
                i < GestureResults.length;
                i++) {
            final double result[] = gestureNetwork.getOutput(GestureInputs[i]);
            GestureType resultGesture = approxDoubleArrToGesture(result);
            GestureType actualGesture = doubleArrToGesture(GestureResults[i]);
            System.out.println(GestureInputs[i][0] + "," + GestureInputs[i][1]
                    + ", cal.result=" + resultGesture.name() + ", real.result=" + actualGesture.name()
                    + " Result=" + (resultGesture.equals(actualGesture) ? "TRUE" : " !!!---FALSE----!!!"));
        }
    }

    /**
     * Converts a gesture to its double array representation
     *
     * @param gesture The gesture to convert
     * @return A double array for the gesture
     */
    public static double[] gestureToDoubleArr(GestureType gesture) {
        GestureType[] gestures = GestureType.values();
        double[] result = new double[gestures.length];
        for (int i = 0; i < gestures.length; i++) {
            if (gestures[i].equals(gesture)) {
                result[i] = 1.0;
            } else {
                result[i] = 0.0;
            }
        }
        return result;
    }

    /**
     * Converts a double array to the exact gesture it represents
     *
     * @param input Array of 0.0 and 1.0
     * @return A gesture type
     */
    public static GestureType doubleArrToGesture(double[] input) {
        GestureType[] gestures = GestureType.values();
        if (input.length != gestures.length) {
            throw new RuntimeException("Input length is not equal to gesture length");
        }
        for (int i = 0; i < input.length; i++) {
            if (input[i] == 1.0) { //must be exactly 1 to match the gesture
                return gestures[i];
            }
        }
        throw new RuntimeException("No gesture matched the array data. Is this an aproximate?");
    }

    /**
     * Converts a double array to the approximate gesture it represents
     *
     * @param input Array of 0.0 to 1.0
     * @return A gesture type
     */
    public static GestureType approxDoubleArrToGesture(double[] input) {
        GestureType[] gestures = GestureType.values();
        if (input.length != gestures.length) {
            throw new RuntimeException("Input length is not equal to gesture length");
        }
        double highestValue = Double.MIN_VALUE;
        int indexOfValue = 0;

        //finds the highest value in the input and gets the gesture for it
        for (int i = 0; i < input.length; i++) {
            if (input[i] >= highestValue) {
                highestValue = input[i];
                indexOfValue = i;
            }
        }
        return gestures[indexOfValue];
    }

    /**
     * Converts a double array to the approximate string output
     *
     * @param input Array of 0.0 to 1.0
     * @return A string output
     */
    public static String approxDoubleArrToDisplayString(double[] input) {
        GestureType[] gestures = GestureType.values();
        if (input.length != gestures.length) {
            throw new RuntimeException("Input length is not equal to gesture length");
        }
        double highestValue = Double.MIN_VALUE;
        int indexOfValue = 0;

        for (int i = 0; i < input.length; i++) {
            if (input[i] >= highestValue) {
                highestValue = input[i];
                indexOfValue = i;
            }
        }
        return ((int) Math.round(highestValue * 100.0)) + "% " + gestures[indexOfValue];
        //return ((int) Math.round(highestValue * 100.0)) + "% sure you are\n" + gestures[indexOfValue];
    }

    /**
     * Creates a CSV file for experimentation output
     */
    private void createFile() {
        FileWriter fstream = null;
        try {
            if ("".equals(trainingExperimentOutputFileName)) {
                Date date = new Date();
                SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH-mm-ss");
                trainingExperimentOutputFileName = "./saved_states/Experiments3/" + dateFormat.format(date) + "-trainingerror.csv";
            }
            File file = new File(trainingExperimentOutputFileName);
            fstream = new FileWriter(file);
            out = new BufferedWriter(fstream);
            experimentOutputFileCreated = true;

        } catch (IOException ex) {
            System.out.println("Error creating file. " + ex.getMessage());
            try {
                fstream.close();
            } catch (IOException ex1) {
                //do nothing
            }
        }
    }
}
