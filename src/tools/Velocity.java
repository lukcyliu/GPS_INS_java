package tools;

public class Velocity {

    public double x, y, z;
    public long t;

    public Velocity(double vx, double vy, double vz, long timeStamp) {
        this.x = vx;
        this.y = vy;
        this.z = vz;
        this.t = timeStamp;
    }

    public Velocity(double vx, double vy, double vz) {
        this.x = vx;
        this.y = vy;
        this.z = vz;
    }

    public Velocity() {
        this(0.0, 0.0, 0.0, 0);
    }

    public Velocity(double[] v, long timeStamp) {
        this(v[0], v[1], v[2], timeStamp);
    }

    public Velocity(Velocity v) {
        this(v.x, v.y, v.z, v.t);
    }
}
