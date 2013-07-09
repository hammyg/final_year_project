package main.java.neuralNetwork;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * This class is used for the neural network to store the neural network layers
 * and error
 *
 * @author hxs830
 */
//FeedForward Network
public class NeuralNetwork implements Serializable {

    private static final long serialVersionUID = 3859422143925541410L;
    //all layers within the network
    private List<NeuralNetworkLayer> layers = new ArrayList<NeuralNetworkLayer>();
    private double totalError;
    private int trainingSetSize;

    /**
     * Empty constructor
     */
    public NeuralNetwork() {
    }

    /**
     * Calculates the error of the network for the training set
     *
     * @param inputs Th training set inputs
     * @param outputs The training set outputs
     * @return The error
     */
    public double calculateError(double inputs[][], double outputs[][]) {
        resetError();

        //iterate through all inputs
        for (int i = 0; i < inputs.length; i++) {
            //feed input into network
            calculateOutputs(inputs[i]);
            //update the error using the network result and the
            //output from the training set
            updateError(layers.get(layers.size() - 1).getOutputArray(),
                    outputs[i]);
        }
        //the output using root mean square
        return Math.sqrt(totalError / trainingSetSize);
    }

    /**
     * Resets the error and the training set size
     */
    private void resetError() {
        totalError = 0;
        trainingSetSize = 0;
    }

    /**
     * Update the error using the predicted and expected gestures
     *
     * @param predicted The predicted gesture
     * @param expected The expected gesture
     */
    private void updateError(double predicted[], double expected[]) {
        for (int i = 0; i < predicted.length; i++) {
            double error = expected[i] - predicted[i];
            totalError += (error * error);
        }
        trainingSetSize += expected.length;
    }

    /**
     * Gets the outputs from the network
     *
     * @param input The input gesture
     * @return The output gesture
     */
    public double[] calculateOutputs(double input[]) {
        //iterate through all layers and feed input into input layer
        for (NeuralNetworkLayer layer : layers) {
            if (layer.getPreviousLayer() == null) {
                layer.calculateOutputs(input);
            } else if (layer.getNextLayer() != null
                    && layer.getPreviousLayer() != null) {
                layer.calculateOutputs();
            }
        }
        //gets the output from output layer
        return layers.get(layers.size() - 1).getOutputArray();
    }

    /**
     * Adds a new layer to the layers array
     *
     * @param layer The new layer
     */
    public void addNewLayer(NeuralNetworkLayer layer) {
        //check if there are existing arrays
        if (!layers.isEmpty()) {
            //set the new layers previous layer
            layer.setPreviousLayer(layers.get(layers.size() - 1));
            //set the last layer the next layer as the new layer
            layers.get(layers.size() - 1).setNextLayer(layer);
        }
        //add the new layer to the layers array
        layers.add(layer);
    }

    /**
     * @return All the layers in this neural network
     */
    public List<NeuralNetworkLayer> getLayers() {
        return layers;
    }

    /**
     * Resets all neural network layers
     */
    public void resetAllLayers() {
        for (NeuralNetworkLayer layer : layers) {
            layer.reset();
        }
    }
}
