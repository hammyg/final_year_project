package main.java.neuralNetwork;

import java.io.Serializable;

/**
 * This class represents a layer within the neural network
 *
 * @author hxs830
 */
public class NeuralNetworkLayer implements Serializable {

    private Matrix weights;
    private NeuralNetworkLayer nextLayer;
    private NeuralNetworkLayer previousLayer;
    private double outputArray[];

    /**
     * Constructor to create the layer with a set neuron count
     *
     * @param neuronCount The number of neurons in this layer
     */
    public NeuralNetworkLayer(int neuronCount) {
        outputArray = new double[neuronCount];
    }

    /**
     * Calculators the outputs for this layer from the last output
     *
     * @return double array for the outputs (neurons)
     */
    public double[] calculateOutputs() {
        Matrix inputMatrix = createMatrixUsingInput(outputArray);

        for (int i = 0; i < nextLayer.getNeuronCount(); i++) {
            Matrix column = weights.getColumn(i);
            double sum = dotProduct(column, inputMatrix);
            nextLayer.setOutput(i, sigmoidActivationFunc(sum));
        }

        return outputArray;
    }

    /**
     * Calculates the dot product from two matrixes
     *
     * @return The dot product of matrix1 and matrix2
     */
    private double dotProduct(Matrix matrix1, Matrix matrix2) {
        double m1Array[] = matrix1.to1DArray();
        double m2Array[] = matrix2.to1DArray();
        double result = 0;
        for (int i = 0; i < m1Array.length; i++) {
            //multiple both values from the matrix and add to the result
            result += m1Array[i] * m2Array[i];
        }
        return result;
    }

    /**
     * Calculates the outputs using an input (used for the input layer)
     *
     * @param input The input
     * @return The output for this layer
     */
    public double[] calculateOutputs(double input[]) {
        for (int i = 0; i < getNeuronCount(); i++) {
            setOutput(i, input[i]);
        }

        return calculateOutputs();
    }

    /**
     * The sigmoid activation function
     *
     * @param input The input value
     * @return The activation value
     */
    private double sigmoidActivationFunc(double input) {
        return 1.0 / (1 + Math.exp(-1.0 * input));
    }

    /**
     * Creates a matrix using inputs
     *
     * @param input The input
     * @return A matrix with one row
     */
    private Matrix createMatrixUsingInput(double input[]) {
        Matrix matrix = new Matrix(1, input.length + 1);
        for (int i = 0; i < input.length; i++) {
            matrix.setValue(0, i, input[i]);
        }
        matrix.setValue(0, input.length, 1);
        return matrix;
    }

    /**
     * @return The output for this layer
     */
    public double[] getOutputArray() {
        return outputArray;
    }

    /**
     * @return The weight matrix
     */
    public Matrix getWeights() {
        return weights;
    }

    /**
     * @return The number of neurons in this layer
     */
    public int getNeuronCount() {
        return outputArray.length;
    }

    /**
     * @return Gets the next neural network layer
     */
    public NeuralNetworkLayer getNextLayer() {
        return nextLayer;
    }

    /**
     * @return Gets the previous neural network layer
     */
    public NeuralNetworkLayer getPreviousLayer() {
        return previousLayer;
    }

    /**
     * Resets the weights to random
     */
    public void reset() {
        if (weights != null) {
            weights.setRandomValues();
        }
    }

    /**
     * Sets a value for an output neuron
     *
     * @param index The output to set
     * @param value The value to set it to
     */
    private void setOutput(int index, double value) {
        outputArray[index] = value;
    }

    /**
     * @param weights The new weights to set it to
     */
    public void setWeights(Matrix weights) {
        this.weights = weights;
        if (weights != null) {
            outputArray = new double[weights.getRowCount()];
        }
    }

    /**
     * Set the next layer and creates a new weight matrix
     *
     * @param nextLayer the next neural network layer
     */
    public void setNextLayer(NeuralNetworkLayer nextLayer) {
        this.nextLayer = nextLayer;
        weights = new Matrix(getNeuronCount(), nextLayer
                .getNeuronCount());
    }

    /**
     * @param previousLayer The previous neural network layer
     */
    public void setPreviousLayer(NeuralNetworkLayer previousLayer) {
        this.previousLayer = previousLayer;
    }
}
