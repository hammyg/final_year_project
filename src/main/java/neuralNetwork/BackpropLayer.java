package main.java.neuralNetwork;

import java.io.Serializable;

/**
 * This class is paired with each neural network layer and updates the weights
 * on that layer
 *
 * @author hxs830
 */
public class BackpropLayer implements Serializable {

    private static final long serialVersionUID = -5399777437205815887L;
    //the error for this layer
    private double error[];
    //the delta error for this layer
    private double errorDelta[];
    //the accumalitve
    private Matrix addedMatrixDelta;
    private Matrix matrixDelta;
    //the trainer
    private BackpropTrainer backPropTrainer;
    //the network layer for this back prop layer
    private NeuralNetworkLayer networkLayer;

    /**
     * Constructor for the back prop layer
     *
     * @param layer The neural network layer that will be updated
     */
    public BackpropLayer(NeuralNetworkLayer layer) {
        networkLayer = layer;

        //instantiate the arrays to the correct size
        error = new double[layer.getNeuronCount()];
        errorDelta = new double[layer.getNeuronCount()];

        //check if there is a next layer to create the matrixes
        if (layer.getNextLayer() != null) {
            addedMatrixDelta = new Matrix(layer.getNeuronCount(), layer
                    .getNextLayer().getNeuronCount());
            matrixDelta = new Matrix(layer.getNeuronCount(), layer
                    .getNextLayer().getNeuronCount());
        }
    }

    /**
     * @param backprop The back prop trainer
     */
    public void setBackPropTrainer(BackpropTrainer backprop) {
        backPropTrainer = backprop;
    }

    /**
     * Used to add delta values
     *
     * @param row The row the delta value is at
     * @param column The column the delta value is at
     * @param value The value to add
     */
    public void addMatrixDelta(int row, int column,
            double value) {
        addedMatrixDelta.addValue(row, column, value);
    }

    /**
     * Calculates the error for this layer and the next
     */
    public void calculateError() {
        BackpropLayer next = backPropTrainer
                .getBackpropLayer(networkLayer.getNextLayer());

        //iterate through all neurons in the next layer
        for (int i = 0; i < networkLayer.getNextLayer().getNeuronCount(); i++) {
            //iterate through all neurons in this layer
            for (int j = 0; j < networkLayer.getNeuronCount(); j++) {
                addMatrixDelta(j, i, next.getErrorDeltaArray()[i]
                        * networkLayer.getOutputArray()[j]);
                setError(j, getErrorArray()[j] + networkLayer.getWeights().getValue(j, i)
                        * next.getErrorDeltaArray()[i]);
            }
        }

        if (networkLayer.getPreviousLayer() != null &&
                networkLayer.getNextLayer() != null) {
            for (int i = 0; i < networkLayer.getNeuronCount(); i++) {
                setErrorDelta(i, delta(i));
            }
        }
    }

    /**
     * Calculates the error using the expected output for this layer
     *
     * @param expected The expected gesture
     */
    public void calculateError(double expected[]) {
        for (int i = 0; i < networkLayer.getNeuronCount(); i++) {
            setError(i, expected[i] - networkLayer.getOutputArray()[i]);
            setErrorDelta(i, delta(i));
        }
    }

    /**
     * Calculates the delta value using the derivative of the activation
     * function
     *
     * @param index The index value
     * @return The delta value
     */
    private double delta(int index) {
        return getErrorArray()[index]
                * sigmoidDerivFun(networkLayer.getOutputArray()[index]);
    }

    /**
     * The derivative of the sigmoid activation function
     *
     * @param input The input value
     * @return The derivative applied to the input
     */
    private double sigmoidDerivFun(double input) {
        return input * (1.0 - input);
    }

    /**
     * Resets all error values to 0
     */
    public void clearError() {
        for (int i = 0; i < networkLayer.getNeuronCount(); i++) {
            error[i] = 0;
        }
    }

    /**
     * @return The error array
     */
    public double[] getErrorArray() {
        return error;
    }

    /**
     * @return The delta error array
     */
    public double[] getErrorDeltaArray() {
        return errorDelta;
    }

    /**
     * Update the weights
     *
     * @param learnRate Learning rate
     * @param momentum Momentum
     */
    public void updateWeights(double learnRate, double momentum) {
        if (networkLayer.getWeights() != null) {
            Matrix updatedDelta = multiply(matrixDelta, momentum);
            Matrix updatedAddedDelta = multiply(addedMatrixDelta,
                    learnRate);
            matrixDelta = add(updatedAddedDelta, updatedDelta);
            networkLayer.setWeights(add(networkLayer.getWeights(),
                    matrixDelta));
            addedMatrixDelta.clearAllValues();
        }
    }

    /**
     * Adds two matrixes together
     *
     * @return A new matrix formed by added martix1 and matrix2
     */
    private Matrix add(Matrix matrix1, Matrix matrix2) {
        final double result[][] = new double[matrix1.getRowCount()][matrix1.getColumnCount()];
        for (int resultRow = 0; resultRow < matrix1.getRowCount(); resultRow++) {
            for (int resultCol = 0; resultCol < matrix1.getColumnCount(); resultCol++) {
                result[resultRow][resultCol] = matrix1.getValue(resultRow, resultCol)
                        + matrix2.getValue(resultRow, resultCol);
            }
        }
        return new Matrix(result);
    }

    /**
     * Updates the value of the error
     *
     * @param index The error index
     * @param value The new error value
     */
    public void setError(int index, double value) {
        error[index] = value;
    }

    /**
     * Updates the delta error value
     *
     * @param index The delta error index
     * @param value The new delta error value
     */
    public void setErrorDelta(int index, double value) {
        errorDelta[index] = value;
    }

    /**
     * Multiplies tow matrixes
     *
     * @return A new matrix formed by multiplying matrix1 with matrix2
     */
    private Matrix multiply(Matrix matrix1, double matrix2) {
        final double result[][] = new double[matrix1.getRowCount()][matrix1.getColumnCount()];
        for (int row = 0; row < matrix1.getRowCount(); row++) {
            for (int column = 0; column < matrix1.getColumnCount(); column++) {
                result[row][column] = matrix1.getValue(row, column) * matrix2;
            }
        }
        return new Matrix(result);
    }
}
