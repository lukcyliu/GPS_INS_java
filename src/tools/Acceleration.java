package tools;

public class Acceleration {

    public double x, y, z;
    public long t;

    public Acceleration(double ax, double ay, double az) {
        x = ax;
        y = ay;
        z = az;
    }

    public Acceleration(double ax, double ay, double az, long timeStamp) {
        this(ax, ay, az);
        t = timeStamp;
    }

    public Acceleration() {
        this(0.0, 0.0, 0.0, 0);
    }

    public Acceleration(double[] a, long timeStamp) {
        this(a[0], a[1], a[2], timeStamp);
    }

    public Acceleration(Acceleration a) {
        this(a.x, a.y, a.z, a.t);
    }
}
