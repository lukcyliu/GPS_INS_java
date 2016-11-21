package tools;

/**
 * Created by Administrator on 2016/10/17 0017.
 */
public class Omega {
    private double L;
    private double h;
    private double VN;
    private double VE;
    private static final double e = 1 / 298.257;
    private static final double Re = 6377830;
    private static final double w = 0.00007292;

    private double Rm;
    private double Rn;

    private double[] wie = new double[3];
    private double[] wep = new double[3];
    private double[][] wadd = new double[3][3];

    public Omega() {
        L = 0;
        h = 0;
        VN = 0;
        VE = 0;
    }

    public Omega(double L, double h, double VN, double VE) {
        this.L = L;
        this.h = h;
        this.VN = VN;
        this.VE = VE;
    }


    public double getRm() {
        this.Rm = Re * (1 - e * e) * Math.pow(1 - e * e * Math.sin(L * Math.PI / 180), 1.5);
        return Rm;
    }

    public double getRn() {
        this.Rn = Re * Math.sqrt(1 - e * e * Math.sin(L * Math.PI / 180) * Math.sin(L * Math.PI / 180));
        return Rn;
    }

    public double[] getWie() {
        wie[0] = 0;
        wie[1] = w * Math.cos(L * Math.PI / 180);
        wie[2] = w * Math.sin(L * Math.PI / 180);
        return wie;
    }

    public double[] getWep() {
        wep[0] = -VN / (getRm() + h);
        wep[1] = VE / (getRn() + h);
        wep[2] = VE / (getRn() + h) * Math.tan(L * Math.PI / 180);
        return wep;
    }

    public double[][] getWadd() {
        wie = getWie();
        wep = getWep();
        wadd[0][0] = 0;
        wadd[0][1] = -(2 * wie[2] + wep[2]);
        wadd[0][2] = 2 * wie[1] + wep[1];
        wadd[1][0] = 2 * wie[2] + wep[2];
        wadd[1][1] = 0;
        wadd[1][2] = -(2 * wie[0] + wep[0]);
        wadd[2][0] = -(2 * wie[1] + wep[1]);
        wadd[2][1] = 2 * wie[0] + wep[0];
        wadd[2][2] = 0;
        return wadd;
    }
}
