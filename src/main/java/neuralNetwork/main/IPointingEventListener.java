package main.java.neuralNetwork.main;

/**
 * Listener interface used for the neural network
 * @author hxs830
 */
public interface IPointingEventListener {

    /**
     * Called when the neural network has predicted the gesture
     * @param output The output gesture from the neural network
     */
    public void networkResultComplete(double[] output);
}
