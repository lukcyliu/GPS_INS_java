package tools;

public class Position {

    public double x, y, z;
    public long t;

    public Position(double x, double y, double z, long timeStamp) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.t = timeStamp;
    }

    public Position() {
        this(0.0, 0.0, 0.0, 0);
    }

    public Position(double[] p, long timeStamp) {
        this(p[0], p[1], p[2], timeStamp);
    }

    public Position(Position p) {
        this(p.x, p.y, p.z, p.t);
    }
}
