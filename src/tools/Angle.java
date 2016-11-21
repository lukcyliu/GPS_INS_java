package tools;

public class Angle {

    public double x, y, z;

    public Angle(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Angle() {
        this(0, 0, 0);
    }

    public Angle(Angle a) {
        this(a.x, a.y, a.z);
    }
}
