package main.java.neuralNetwork;

import java.io.Serializable;

/**
 * A class to represent a mathematical matrix in Java
 *
 * @author hxs830
 */
public class Matrix implements Serializable {

    private static final long serialVersionUID = 6416282986589694861l;
    //the matrix values in a 2d double array
    private double matrixValues[][];

    /**
     * Constructor
     *
     * @param matrixValues The initial matrix values
     */
    public Matrix(double matrixValues[][]) {
        //create the matrix with the same matrix values
        this.matrixValues = matrixValues;
    }

    /**
     * Constructor Creates an empty matrix
     *
     * @param rows Row size
     * @param columns Column size
     */
    public Matrix(int rows, int columns) {
        matrixValues = new double[rows][columns];
    }

    /**
     * Adds a value to the current value in the matrix
     *
     * @param row The row the value is in
     * @param column The colum the value is in
     * @param value The value to add to the existing value
     */
    public void addValue(int row, int column, double value) {
        setValue(row, column, getValue(row, column) + value);
    }

    /**
     * Resets all values in the matrix to 0
     */
    public void clearAllValues() {
        for (int i = 0; i < getRowCount(); i++) {
            for (int j = 0; j < getColumnCount(); j++) {
                setValue(i, j, 0);
            }
        }
    }

    /**
     * Gets the value from the matrix
     *
     * @param row The row the value is in
     * @param column The column the value is in
     * @return The value at that location
     */
    public double getValue(int row, int column) {
        return matrixValues[row][column];
    }

    /**
     * Gets a 1 dimensional matrix of the specified column
     *
     * @param column The column from the matrix
     * @return A 1D matrix with those values from the specified column
     */
    public Matrix getColumn(int column) {
        //create 1 column matrix
        double newMatrix[][] = new double[getRowCount()][1];
        //copy the values from the specified column
        for (int row = 0; row < getRowCount(); row++) {
            newMatrix[row][0] = matrixValues[row][column];
        }

        return new Matrix(newMatrix);
    }

    /**
     * @return The number of columns in the matrix
     */
    public int getColumnCount() {
        return matrixValues[0].length;
    }

    /**
     * @return The number of rows in the matrix
     */
    public int getRowCount() {
        return matrixValues.length;
    }

    /**
     * Sets all the values in the matrix to random values between -1 and 1
     */
    public void setRandomValues() {
        for (int i = 0; i < getRowCount(); i++) {
            for (int j = 0; j < getColumnCount(); j++) {
                setValue(i, j, (Math.random() * 2d) - 1d);
            }
        }
    }

    /**
     * Sets a specific value in the matrix
     *
     * @param row The row the value to set is in
     * @param column The column the value to set is in
     * @param value The new value
     */
    public void setValue(int row, int column, double value) {
        matrixValues[row][column] = value;
    }

    /**
     * Converts the matrix to a one dimensional array with size rows * columns
     *
     * @return A double array with all the values from the matrix
     */
    public double[] to1DArray() {
        //create a double array to store all the values from the matrix
        double result[] = new double[getRowCount() * getColumnCount()];
        //iterate through all rows and columns and place value in the double array
        int pointer = 0;
        for (int i = 0; i < getRowCount(); i++) {
            for (int j = 0; j < getColumnCount(); j++) {
                result[pointer++] = matrixValues[i][j];
            }
        }
        return result;
    }
}
