package tools;

import Jama.Matrix;

public class Kalmanfilter {

    private Matrix X = new Matrix(2, 3);
    private Matrix F = new Matrix(2, 2);
    private Matrix B = new Matrix(2, 1);
    private Matrix Theta = new Matrix(1, 3);
    private Matrix P = new Matrix(2, 2);
    private Matrix Q = new Matrix(2, 2);
    private Matrix Y = new Matrix(1, 3);
    private Matrix Z = new Matrix(1, 3);
    private Matrix H = new Matrix(1, 2);
    private Matrix I = new Matrix(2, 2);
    private Matrix S = new Matrix(1, 1);
    private Matrix R = new Matrix(1, 1);
    private Matrix K = new Matrix(2, 1);

    public Kalmanfilter(double deltaT, double QTheta, double QThetaBias, double gx, double gy, double gz, double zx, double zy, double zz) {
        initial(deltaT, QTheta, QThetaBias);
        iteration(deltaT, QTheta, QThetaBias, gx, gy, gz, zx, zy, zz);
    }

    private void initial(double deltaT, double QTheta, double QThetaBias) {

        X.set(0, 0, 0);
        X.set(0, 1, 0);
        X.set(0, 2, 0);
        X.set(1, 0, 0);
        X.set(1, 1, 0);
        X.set(1, 2, 0);

        P.set(0, 0, 0);
        P.set(0, 1, 0);
        P.set(1, 0, 0);
        P.set(1, 1, 0);

        H.set(0, 0, 1);
        H.set(0, 1, 0);

        I.set(0, 0, 1);
        I.set(0, 1, 0);
        I.set(1, 0, 0);
        I.set(1, 1, 1);
    }

    public Matrix iteration(double deltaT, double QTheta, double QThetaBias, double gx, double gy, double gz, double zx, double zy, double zz) {

        F.set(0, 0, 1);
        F.set(0, 1, -deltaT);
        F.set(1, 0, 0);
        F.set(1, 1, 1);

        B.set(0, 0, deltaT);
        B.set(1, 0, 0);

        Theta.set(0, 0, gx);
        Theta.set(0, 1, gy);
        Theta.set(0, 2, gz);

        Q.set(0, 0, QTheta);
        Q.set(0, 1, 0);
        Q.set(1, 0, 0);
        Q.set(1, 1, QThetaBias);
        Q = Q.times(deltaT);

        X = F.times(X).plus(B.times(Theta));
        P = F.times(P).times(F.transpose()).plus(Q);
        Y = Z.minus(H.times(X));
        S = H.times(P).times(H.transpose()).plus(R);
        K = P.times(H.transpose()).times(S.inverse());
        X = X.plus(K.times(Y));
        P = I.minus(K.times(H)).times(P);

        return X;
    }

    public Matrix getP() {
        return P;
    }
}
