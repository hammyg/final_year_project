package main.java.misc;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Date;
import main.java.neuralNetwork.BackpropTrainer;
import main.java.neuralNetwork.NeuralNetwork;

/**
 * This class is used to hold all relevant information for saving and restoring
 * a neural network
 *
 * @author hxs830
 */
public class SaveState implements Serializable {

    private static final long serialVersionUID = -6174120377121717143L;
    /*
     * the variables that will be stored for 
     * the neural network to be saved and loaded
     */
    private NeuralNetwork network;
    private BackpropTrainer backProp;
    private boolean isTrained;
    private String description;
    private Date date;
    private ArrayList<double[]> inputs = new ArrayList<double[]>();
    private ArrayList<double[]> outputs = new ArrayList<double[]>();

    /**
     * Empty constructor
     */
    public SaveState() {
    }

    /**
     * @param date The date to set it to
     */
    public void setDate(Date date) {
        this.date = date;
    }

    /**
     * @param description The description of this saved state
     */
    public void setDescription(String description) {
        this.description = description;
    }

    /**
     * @param inputs The inputs for the training set
     */
    public void setInputs(ArrayList<double[]> inputs) {
        this.inputs = inputs;
    }

    /**
     * @param output The outputs to the training set
     */
    public void setOutputs(ArrayList<double[]> output) {
        this.outputs = output;
    }

    /**
     * @param isTrained If the neural network is trained or not
     */
    public void setIsTrained(boolean isTrained) {
        this.isTrained = isTrained;
    }

    /**
     * @param network The neural network
     */
    public void setNetwork(NeuralNetwork network) {
        this.network = network;
    }

    /**
     * @param backProp The back-propagation
     */
    public void setBackprop(BackpropTrainer backProp) {
        this.backProp = backProp;
    }

    /**
     * @return The description for this state
     */
    public String getDescription() {
        return description;
    }

    /**
     * @return If this network is trained
     */
    public boolean isTrained() {
        return isTrained;
    }

    /**
     * @return The neural network
     */
    public NeuralNetwork getNetwork() {
        return network;
    }

    /**
     * @return The back-propagation
     */
    public BackpropTrainer getBackprop() {
        return backProp;
    }

    /**
     * @return The date the save state was created
     */
    public Date getDate() {
        return date;
    }

    /**
     * @return The array of inputs for the training set
     */
    public ArrayList<double[]> getInputs() {
        return inputs;
    }

    /**
     * @return The array of outputs for the training set
     */
    public ArrayList<double[]> getOutputs() {
        return outputs;
    }
}
