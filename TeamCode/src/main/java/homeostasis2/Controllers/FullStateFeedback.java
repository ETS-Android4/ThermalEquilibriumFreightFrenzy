package homeostasis2.Controllers;

import homeostasis2.Utils.Vector;

public class FullStateFeedback {

	protected Vector K;

	/**
	 * controller gains
	 * @param K Gain Vector K
	 */
	public FullStateFeedback(Vector K) {
		this.K = K;
	}

	public double calculate(Vector reference, Vector State) throws Exception {
		return reference.minus(State).dot(K);
	}



}
