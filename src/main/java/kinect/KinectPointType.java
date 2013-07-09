package main.java.kinect;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Stores all the available person points (joints) that can be tracked
 *
 * @author hxs830
 */
public class KinectPointType {

    //the types in an array
    private static ArrayList<Type> asArray = null;

    /* All the possible kinect skeleton points
     * with the string representation for matching on in the ros topic
     * person number is not included as this can change
     * format of string for left elbow of person 2: /left_elbow_2
     */
    public enum Type {

        HEAD {
            public String toString() {
                return "/head";
            }
        },
        NECK {
            public String toString() {
                return "/neck";
            }
        },
        TORSO {
            public String toString() {
                return "/torso";
            }
        },
        LEFT_SHOULDER {
            public String toString() {
                return "/left_shoulder";
            }
        },
        LEFT_ELBOW {
            public String toString() {
                return "/left_elbow";
            }
        },
        LEFT_HAND {
            public String toString() {
                return "/left_hand";
            }
        },
        RIGHT_SHOULDER {
            public String toString() {
                return "/right_shoulder";
            }
        },
        RIGHT_ELBOW {
            public String toString() {
                return "/right_elbow";
            }
        },
        RIGHT_HAND {
            public String toString() {
                return "/right_hand";
            }
        },
        LEFT_HIP {
            public String toString() {
                return "/left_hip";
            }
        },
        LEFT_KNEE {
            public String toString() {
                return "/left_knee";
            }
        },
        LEFT_FOOT {
            public String toString() {
                return "/left_foot";
            }
        },
        RIGHT_HIP {
            public String toString() {
                return "/right_hip";
            }
        },
        RIGHT_KNEE {
            public String toString() {
                return "/right_knee";
            }
        },
        RIGHT_FOOT {
            public String toString() {
                return "/right_foot";
            }
        }
    };

    /**
     * Creates an array of all the points/joints
     *
     * @return The array holding all the points/joints
     */
    public static ArrayList<Type> asArray() {
        if (asArray == null) {
            asArray = new ArrayList<Type>(Arrays.asList(Type.values()));
        }
        return asArray;
    }

    /**
     * @return Total number of kinect points/joints
     */
    public static int count() {
        return asArray().size();
    }
}
