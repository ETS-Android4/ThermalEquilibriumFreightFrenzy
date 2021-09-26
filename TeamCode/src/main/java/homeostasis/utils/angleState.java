package homeostasis.utils;

import static homeostasis.utils.AngleTools.angleWrap;

/**
 * some states are in the form of a radians angle, whi
 */
public class angleState extends state {

    /**
     * assuming this state is desired, the oth
     *
     * @param other is the state we are using to calculate error from
     * @return the error between the two states
     */
    public state stateError(state other) {
        return new state(angleWrap(this.position - other.position), this.velocity - other.velocity);
    }

}
