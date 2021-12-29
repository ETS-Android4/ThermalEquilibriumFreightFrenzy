package homeostasis2.Parameters;

public class FeedforwardCoefficientsEx extends FeedforwardCoefficients{

	public double Kg;
	public double Kcos;

	public FeedforwardCoefficientsEx(double Kv, double Ka, double Ks, double Kg, double Kcos) {
		super(Kv, Ka, Ks);
		this.Kg = Kg;
		this.Kcos = Kcos;
	}
}
