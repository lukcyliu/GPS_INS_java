package tools;

public class AngularVelocity {

    public double x, y, z;
    public long t;

    public AngularVelocity(double avx, double avy, double avz) {
        this.x = avx;
        this.y = avy;
        this.z = avz;
    }

    public AngularVelocity(double avx, double avy, double avz, long timeStamp) {
        this(avx, avy, avz);
        this.t = timeStamp;
    }

    public AngularVelocity() {
        this(0.0, 0.0, 0.0, 0);
    }

    public AngularVelocity(double[] av, long timeStamp) {
        this(av[0], av[1], av[2], timeStamp);
    }

    public AngularVelocity(AngularVelocity av) {
        this(av.x, av.y, av.z, av.t);
    }
}
