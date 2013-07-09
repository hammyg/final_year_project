package main.java.neuralNetwork;

import java.io.Serializable;
import java.util.HashMap;
import java.util.Map;

/**
 * Class is used to train the neural network weights
 *
 * @author hxs830
 */
public class BackpropTrainer implements Serializable {

    private static final long serialVersionUID = 8424005957443402336L;
    //2d array for the inputs
    private double inputs[][];
    //2d array for the expected outputs
    private double outputs[][];
    //the learning rate: default value from trial-&-error
    private double learningRate = 0.2;
    //the momentum: default value from trial-&-error
    private double momentum = 0.8;
    //the neural network that will be trained
    private NeuralNetwork network;
    //the current error value
    private double currentNetworkerror;
    //a map to pair each network and back propagation layer
    private Map<NeuralNetworkLayer, BackpropLayer> networkMap;

    /**
     * Constructor to train the neural network with a given training set,
     * learning rate and momentum
     *
     * @param networkToTrain The neural network that will be trained
     */
    public BackpropTrainer(NeuralNetwork networkToTrain) {
        network = networkToTrain;
        networkMap = new HashMap<NeuralNetworkLayer, BackpropLayer>(network.getLayers().size());

        /* iterate through all the neural network layers
         * and create a back prop layer for each one
         */
        for (NeuralNetworkLayer networkLayer : networkToTrain.getLayers()) {
            BackpropLayer backpropLayer = new BackpropLayer(networkLayer);
            backpropLayer.setBackPropTrainer(this);
            //place neural network and back prop layer in the map
            networkMap.put(networkLayer, backpropLayer);
        }
    }

    /**
     * Clears all errors on in every layer
     */
    private void clearAllErrors() {
        for (NeuralNetworkLayer layer : network.getLayers()) {
            getBackpropLayer(layer).clearError();
        }
    }

    /**
     * Calculates the errors for the expected output
     *
     * @param expected The target output
     */
    private void calculateError(double expected[]) {
        clearAllErrors();

        //iterate backwards through the network layers
        for (int i = network.getLayers().size() - 1; i >= 0; i--) {
            NeuralNetworkLayer networkLayer = network.getLayers().get(i);
            //if it's the output layer, then provide the expected output
            if (networkLayer.getNextLayer() == null) {
                getBackpropLayer(networkLayer).calculateError(expected);
            } else {
                getBackpropLayer(networkLayer).calculateError();
            }
        }
    }

    /**
     * Gets the paired back prop layer for the network layer
     *
     * @param networkLayer The network layer
     * @return The back propagation layer for this network layer
     */
    public BackpropLayer getBackpropLayer(NeuralNetworkLayer networkLayer) {
        return networkMap.get(networkLayer);
    }

    /**
     * @return The current error of the network
     */
    public double getCurrentNetowrkError() {
        return currentNetworkerror;
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
     * @param learningRate The training learning rate
     */
    public void setLearningRate(double learningRate) {
        this.learningRate = learningRate;
    }

    /**
     * @param momentum The training momentum
     */
    public void setMomentum(double momentum) {
        this.momentum = momentum;
    }

    /**
     * Applies back prop once
     */
    public void TrainStep() {
        //calculates the errors using the inputs and outputs
        for (int j = 0; j < inputs.length; j++) {
            network.calculateOutputs(inputs[j]);
            calculateError(outputs[j]);
        }
        //updates the weights
        for (NeuralNetworkLayer layer : network.getLayers()) {
            getBackpropLayer(layer).updateWeights(learningRate, momentum);
        }
        //set the error
        currentNetworkerror = network.calculateError(inputs, outputs);
    }
}
