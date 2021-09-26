package homeostasis.systems;

import homeostasis.utils.constraints;
import homeostasis.utils.state;

/**
 * in control theory, the plant is the system we would like to control
 */
public class plant {


    protected constraints constraints = new constraints(0, 0, 0);

    /**
     * systems state
     *
     * @return the system state
     */
    public state getState() {
        return null;
    }

    /**
     * input to the plant
     *
     * @param input to the plant
     */
    public void input(double input) {

    }

    /**
     * obtain the plants constraints on velocity, acceleration, and minimum motor power
     *
     * @return constraints object
     */
    public homeostasis.utils.constraints getConstraints() {
        return constraints;
    }
}
