package org.firstinspires.ftc.teamcode.Filter;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {

    private SimpleMatrix F, Q, H;

    // system state estimate
    private SimpleMatrix x, P;

    public void configure(DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H) {
        this.F = new SimpleMatrix(F);
        this.Q = new SimpleMatrix(Q);
        this.H = new SimpleMatrix(H);
    }

    public void setState(DMatrixRMaj x, DMatrixRMaj P) {
        this.x = new SimpleMatrix(x);
        this.P = new SimpleMatrix(P);
    }

    public void predict() {

        x = F.mult(x);

        P = F.mult(P).mult(F.transpose()).plus(Q);

    }

    public void update(DMatrixRMaj _z, DMatrixRMaj _R) {
        SimpleMatrix z = SimpleMatrix.wrap(_z);
        SimpleMatrix R = SimpleMatrix.wrap(_R);

        SimpleMatrix product = H.mult(x);
        System.out.println("z is " + z);
        System.out.println("H is " + H);
        System.out.println("X is " + x);
        System.out.println("product is " + product);
        SimpleMatrix y = z.minus(product.transpose());


        SimpleMatrix S = H.mult(P);
        System.out.println("S is" + S);
        S = S.mult(H.transpose());
        System.out.println("S is now " + S);

        S = S.plus(R);

        SimpleMatrix K = P.mult(H.transpose().mult(S.invert()));

        System.out.println("K is" + K);
        System.out.println("y is " + y);

        SimpleMatrix sensorFusion = K.mult(y.transpose());
        x = x.plus(sensorFusion);


        P = P.minus(K.mult(H).mult(P));


    }

    public DMatrixRMaj getState() {
        return x.getMatrix();
    }

    public DMatrixRMaj getCovariance() {
        return P.getMatrix();
    }
}
