package smooth;

import Jama.Matrix;
import tools.Kalman_GPS_INS;
import tools.MahonyAHRS;
import tools.Position;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by lukcy on 2017/9/18.
 */
public class INSTest {

    //------------------------郎朗转弯代码变量----------------------------------------//
    static double[] stepP = new double[3];
    static double[] gyoP = new double[3];
    static double[] magP = new double[3];
    static double[] lastAv = new double[3];
    static boolean initAngle = false;
    static ArrayList<Double> initGyrMagList = new ArrayList<Double>();
    static double initGyrHeading;
    static double lastGyrOrientation;
    private static double lastUncalibratedGyrOrientation;
    public final static boolean useGyr = true;
    public static double thetaCThreshold = 12.45;
    public static double thetaMThreshold = 11.110; // 11.115
    private static double lastMagOrientation;
    private static int flag = 0;
    private static double lastStepOrientation;
    public final static double[][] Weights = {
            // lastStepOrientation, magOrientation, gyrOrientation
            {0.004, 0.488, 0.508},  // walk straight, when thetaC <= thetaCThreshold && thetaM <= thetaMThreshold
            {0.008, 0.730, 0.262},  // turning, when thetaC <= thetaCThreshold && thetaM > thetaMThreshold
            {0.904, 0.073, 0.023},  // when thetaC > thetaCThreshold && thetaM <= thetaMThreshold
            {0.850, 0.064, 0.086},
            {0.170, 0.015, 0.815},    // magnetometer disturbed, when thetaC > thetaCThreshold && thetaM > thetaMThreshold
            {0.000, 0.000, 1.000}

//            {0.004, 0.488, 0.508},  // walk straight, when thetaC <= thetaCThreshold && thetaM <= thetaMThreshold
//            {0.008, 0.730, 0.262},  // turning, when thetaC <= thetaCThreshold && thetaM > thetaMThreshold
//            {0.904, 0.073, 0.023},  // when thetaC > thetaCThreshold && thetaM <= thetaMThreshold
//            {0.850, 0.064, 0.086},
//            {0.170, 0.015, 0.815},    // magnetometer disturbed, when thetaC > thetaCThreshold && thetaM > thetaMThreshold
//            {0.000, 0.000, 1.000}

//     	{0.01, 0.5, 0.49},  // walk straight, when thetaC <= thetaCThreshold && thetaM <= thetaMThreshold
//        {  0, 0.722, 0.278},  // turning, when thetaC <= thetaCThreshold && thetaM > thetaMThreshold
//        {0.95, 0.05, 0},
//        {0.86, 0.06, 0.08},  // when thetaC > thetaCThreshold && thetaM <= thetaMThreshold
//        {0.178,   0, 0.822},   // magnetometer disturbed, when thetaC > thetaCThreshold && thetaM > thetaMThreshold
//        {0.1, 0.5, 0.4}
    };
    private static long[] countMethod = new long[5];
    public final static boolean calibrateGyr = true;
    private static int calibrateGyrStepCount;
    public final static boolean checkMag = true;
    public final static double viewWidth = 960;
    public final static double midAccHeight = 120, midGyrHeight = 120, midMagHeight = 120, midPathHeight = 240;

    public final static double STEP_LENGTH = 0.6;
    private static int magCheckStepCount;
    private static double lastThetaC;

    //---------------------------------------------------------------------------------//
    int firstSmooth = 0;
    int width = 12;
    static double accCalErr_X = -0.09092, accCalErr_Y = 0.081208, accCalErr_Z = 0.015632;
    double[][] acc = new double[3][width];
    double[][] gyo = new double[3][width];
    double[][] mag = new double[3][width];
    double[][] angle = new double[3][width];
    double[][] gps = new double[2][width];


    static MahonyAHRS mahonyAHRS;
    static Matrix mahonyR;
    static Matrix jyR;
    static double deg2rad = 3.1415926 / 180;
    static double rad2deg = 180 / 3.1415926;


    //循环队列滑动滤波算法测试
    double[] value_buf = new double[width];
    int slideN = 0;


    public double slideFilter(double x) {
        int count = 0;
        double sum = 0;
        value_buf[slideN++] = x;
        if (slideN == width) slideN = 0;
        for (count = 0; count < width; count++)
            sum += value_buf[count];
        return sum / width;
    }


    public ArrayList<double[][]> slideWindows(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, double GPSyaw, double GPSv) {

        ArrayList<double[][]> queue = new ArrayList<double[][]>();
        if (firstSmooth == 0) {
            for (int i = 0; i < width; i++) {
                acc[0][i] = ax;
                acc[1][i] = ay;
                acc[2][i] = az;
                gyo[0][i] = gx;
                gyo[1][i] = gy;
                gyo[2][i] = gz;
                mag[0][i] = mx;
                mag[1][i] = my;
                mag[2][i] = mz;
                gps[0][i] = GPSyaw;
                gps[1][i] = GPSv;
            }
            firstSmooth = 1;
        } else {
            for (int i = 1; i < width; i++) {
                acc[0][i - 1] = acc[0][i];
                acc[1][i - 1] = acc[1][i];
                acc[2][i - 1] = acc[2][i];
                gyo[0][i - 1] = gyo[0][i];
                gyo[1][i - 1] = gyo[1][i];
                gyo[2][i - 1] = gyo[2][i];
                mag[0][i - 1] = mag[0][i];
                mag[1][i - 1] = mag[1][i];
                mag[2][i - 1] = mag[2][i];
                gps[0][i - 1] = gps[0][i];
                gps[1][i - 1] = gps[1][i];
            }
            acc[0][width - 1] = ax;
            acc[1][width - 1] = ay;
            acc[2][width - 1] = az;
            gyo[0][width - 1] = gx;
            gyo[1][width - 1] = gy;
            gyo[2][width - 1] = gz;
            mag[0][width - 1] = mx;
            mag[1][width - 1] = my;
            mag[2][width - 1] = mz;
            gps[0][width - 1] = GPSyaw;
            gps[1][width - 1] = GPSv;
        }
        queue.add(acc);
        queue.add(gyo);
        queue.add(mag);
        queue.add(gps);
        return queue;
    }

    public static void main(String[] args) {

        //acc高通低通滤波
        double acc_mod = 0, acc_low = 0, acc_high = 0;
        double low_value = 0.2;

        double hx = 0, hy = 0, hAy = 0, hAx = 0, hVy = 0, hVx = 0, hSx = 0, hSy = 0, hV2 = 0, hS2 = 0, hSy2 = 0, hSx2 = 0, lasthA2 = 0, lasthV2 = 0;
        double lasthAy = 0, lasthAx = 0, lastlastAy = 0, lastlastAx = 0, lasthVy = 0, lasthVx = 0, lastlastVy = 0, lastlastVx = 0;

        double yaw = 90;
        double hubuYaw = 90;
        int count = 0;
        SmoothTest smoothTest = new SmoothTest();
        //io read csv.
        CsvHelper csvHelper = new CsvHelper();
        csvHelper.creatOutputFile();
        csvHelper.write("smoothAx" + "," + "smoothAy" + "," + "smoothAz" + "," +
                "smoothGx" + "," + "smoothGy" + "," + "smoothGz" + "," +
                "smoothMx" + "," + "smoothMy" + "," + "smoothMz" + "," +
                "smoothGPSYaw" + "," + "smoothGPSv" + "," + "GPSVe" + "," + "GPSVn" + "," + "mahonyRoll" + "," + "mahonyPitch" + "," + "mahonyYaw"
                + "," + "VccqX,VccqY,VccqZ" + "," + "Vx,Vy,Vz" + "," + "E,L,h,Px,Py" + "," +
                "gyoOrientation,uncalGyoOrientation,magOrientation,setpOrientation" + "," +
                "stepPx,stepPy,gyoPx , gyoPy ,magPx , magPy" + "\n");
        ArrayList<double[]> input = csvHelper.read(20);
        System.out.println(input.size());
        double[] data = new double[20];
        ArrayList<double[][]> smoothRes;
        double ax, ay, az, gx, gy, gz, mx, my, mz, jyYaw, GPSYaw = 0, GPSv = 0, GPSLongitude, GPSLattitude, GPSHeight, GPS_SN;
        float[] q;
        double smoothAx = 0, smoothAy = 0, smoothAz = 0, smoothGx = 0, smoothGy = 0, smoothGz = 0, smoothMx = 0, smoothMy = 0, smoothMz = 0, smoothGPSYaw = 0, smoothGPSv = 0;
        double GPSVn = 0, GPSVe = 0, GPSVu = 0;
        double lastGPSLongtitude = 0, lastGPSLattitude = 0, lastGPSh = 51.2;
        double last_L = 0, last_lamb = 0, last_h = 0;
        int firstGPSOff = 0;
        int firstGPSVcc_in = 0;
        int firstGPSVcc_out = 0;
        int GPSOff = 0;
        double speedE = 0, speedN = 0, speedH = 0;
        double pre_lamb = 0, pre_L = 0, pre_h = 0;
        double INS_lamb = 0, INS_L = 0, INS_h = 0;
        double lastpVx = 0, lastpVy = 0, lastpVz = 0;
        double pVx = 0, pVy = 0, pVz = 0;
        double lastAx = 0, lastAy = 0, lastAz = 0;
        double Px = 0, Py = 0;
        Position AddS = new Position();
        int insCnt = 0;
        double Pitch0 = 0, Roll0 = 0, Yaw0 = 0;
        double Pitch = 0, Roll = 0, Yaw = 0;
        double firstPitch = 0, firstRoll = 0, firstYaw = 0;
        float[] eInt = {0, 0, 0};
        //Re长半轴 r短半轴 f椭球扁率 e椭球偏心率 wie地球自转角速率
        double earthRe = 6378137, earthr = 6356752.3142, earthf = 1 / 298.257, earthe = 0.0818, earthwie = 7.292e-5;
        double G0 = 9.8015;

        double Rm = 0, Rn = 0, R0 = 0, lastRm = 0, lastRn = 0;
        double WieE = 0, WinE = 0, WWX = 0;
        Matrix Fn = new Matrix(3, 1);
        double[] Vccq = {0, 0, 0};
        double L = 39.980793 * deg2rad, E = 116.321083 * deg2rad, h = 51.2;
        Kalman_GPS_INS kalman_gps_ins = new Kalman_GPS_INS();
        //融合迭代补偿tao
        double tao = 0;

        //初始化定姿
        double[] data0 = input.get(0);
        ax = -(data0[0] - accCalErr_X);
        ay = -(data0[1] - accCalErr_Y);
        az = -(data0[2] - accCalErr_Z);
        gx = data0[3];
        gy = data0[4];
        gz = data0[5];
        mx = data0[6];
        my = data0[7];
        mz = -data0[8];
        //求初始的姿态角
        Pitch0 = Math.atan2(-ay, -az);
        Roll0 = Math.atan2(ax, -az);
        Yaw0 = Math.atan2(-my * Math.cos(Roll0) + mz * Math.sin(Roll0), mx * Math.cos(Pitch0) + my * Math.sin(Pitch0) * Math.sin(Roll0) - mz * Math.sin(Pitch0) * Math.cos(Roll0));
        firstPitch = Pitch0 * 57.29578;
        firstRoll = Roll0 * 57.29578;
        firstYaw = -Yaw0 * 57.29578;
        //计算初始四元数
        float[] q0 = EularToQuaternion(Yaw0, Pitch0, Roll0);
        mahonyAHRS = new MahonyAHRS(0.2f, 2, 0.01f, eInt);
        mahonyAHRS.setQuaternion(q0);
        E = data0[14] * deg2rad;
        L = data0[15] * deg2rad;


        for (int i = 1; i < input.size(); i++) {
            data = input.get(i);
            ax = -(data[0] - accCalErr_X);
            ay = -(data[1] - accCalErr_Y);
            az = -(data[2] - accCalErr_Z);
            gx = data[3];
            gy = data[4];
            gz = data[5];
            mx = data[6];
            my = data[7];
            mz = -data[8];
            jyYaw = data[13];
            GPSLongitude = data[14];
            GPSLattitude = data[15];
            GPSHeight = data[16];
            GPSYaw = data[17];
            GPSv = data[18];
            GPS_SN = data[19];

//---------------------------------------------滑动滤波-----------------------------------------------------------------//
            //循环队列方法
            double slidejyYaw = smoothTest.slideFilter(jyYaw);
            //暴力法
            smoothRes = smoothTest.slideWindows(ax, ay, az, gx, gy, gz, mx, my, mz, GPSYaw, GPSv);
            for (int j = 0; j < smoothTest.width; ++j) {
                smoothAx += smoothRes.get(0)[0][j];
                smoothAy += smoothRes.get(0)[1][j];
                smoothAz += smoothRes.get(0)[2][j];
                smoothGx += smoothRes.get(1)[0][j];
                smoothGy += smoothRes.get(1)[1][j];
                smoothGz += smoothRes.get(1)[2][j];
                smoothMx += smoothRes.get(2)[0][j];
                smoothMy += smoothRes.get(2)[1][j];
                smoothMz += smoothRes.get(2)[2][j];
                smoothGPSYaw += smoothRes.get(3)[0][j];
                smoothGPSv += smoothRes.get(3)[1][j];
            }
            smoothAx /= smoothTest.width;
            smoothAy /= smoothTest.width;
            smoothAz /= smoothTest.width;
            smoothGx /= smoothTest.width;
            smoothGy /= smoothTest.width;
            smoothGz /= smoothTest.width;
            smoothMx /= smoothTest.width;
            smoothMy /= smoothTest.width;
            smoothMz /= smoothTest.width;
            smoothGPSYaw /= smoothTest.width;
            smoothGPSv /= smoothTest.width;

            GPSVn = smoothGPSv * Math.cos(smoothGPSYaw * 3.1415926 / 180) / 3.6;
            GPSVe = smoothGPSv * Math.sin(smoothGPSYaw * 3.1415926 / 180) / 3.6;
            GPSVu = GPSHeight - lastGPSh;

//-----------------------------------------------滑动滤波结束-------------------------------------------------------------------//

//-------------------------------------------------以下是乱七八糟的测试部分------------------------------------------------//
            //用mahony互补滤波更新四元数并计算出姿态角
            mahonyAHRS.update((float) (gx * deg2rad), (float) (gy * deg2rad), (float) (gz * deg2rad), (float) ax, (float) ay, (float) az, (float) smoothMx, (float) smoothMy, (float) smoothMz);
            q = mahonyAHRS.getQuaternion();
            Yaw = -Math.atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[2] * q[2] - 1) * rad2deg;
            Pitch = Math.asin(2 * q[2] * q[3] + 2 * q[0] * q[1]) * rad2deg;
            Roll = -Math.atan2(2 * q[1] * q[3] + 2 * q[0] * q[2], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * rad2deg;
            //有更新四元数得到更新旋转矩阵Cnb
            mahonyR = quternionToCnbMatrix(q);
            //转置得到Cbn
            mahonyR.transpose();

            //测试朗朗转弯方法-------------------------------------------------------------------------------------------
            double[] resultOrientation = TurnningTest(gx, gy, gz, mx, my, mz);
            resultOrientation[2] = 90 - resultOrientation[2];
            stepP[0] += Math.sin(resultOrientation[3] * 3.1415926 / 180);
            stepP[1] += Math.cos(resultOrientation[3] * 3.1415926 / 180);
            gyoP[0] += Math.sin(resultOrientation[0] * 3.1415926 / 180);
            gyoP[1] += Math.cos(resultOrientation[0] * 3.1415926 / 180);
            magP[0] += Math.sin(resultOrientation[2] * 3.1415926 / 180);
            magP[1] += Math.cos(resultOrientation[2] * 3.1415926 / 180);

            //测试单步长匀速路径
            Px += Math.cos(Yaw * 3.1415926 / 180);
            Py += Math.sin(Yaw * 3.1415926 / 180);
            //------------------------------------------end---------------------------------------------------------------

            //计算更新的子午曲率半径Rm和卯酉曲率半径Rn以及曲率平均半径R0
            Rm = earthRe * (1 - 2 * earthf + 3 * earthf * Math.sin(last_L) * Math.sin(last_L));
            Rn = earthRe * (1 + earthf * Math.sin(last_L) * Math.sin(last_L));
            R0 = Math.sqrt(Rm * Rm + Rn * Rn);

//-----------------------------------------------------积分测试部分--------------------------------------------------------//
            //投影到东北天坐标系下计算绝对加速度比力和计算绝对速度微分Vccq
            Fn = accToFn(mahonyR, ax, ay, az).times(G0);
            Vccq[0] = -Fn.get(0, 0);
            Vccq[1] = Fn.get(1, 0);
            Vccq[2] = Fn.get(2, 0) + G0;

            //用转弯策略测试航位推算算法，取y轴加速度为汽车前进加速度进行航向分解
            Vccq[0] = ay * G0 * Math.sin(resultOrientation[3] * 3.1415926 / 180);
            Vccq[1] = ay * G0 * Math.cos(resultOrientation[3] * 3.1415926 / 180);

            pVx += (Vccq[0]) * 0.2;
            pVy += (Vccq[1]) * 0.2;
            pVz += (Vccq[2]) * 0.2;

            L = L + (lastpVy / (Rm + last_h)) * 0.2;
            E = E + (lastpVx / (Math.cos(last_L) * (Rn + last_h))) * 0.2;
            h = h - lastpVz * 0.2;

            lastAx = Vccq[0];
            lastAy = Vccq[1];
            lastAz = Vccq[2];
            lastpVx = pVx;
            lastpVy = pVy;
            lastpVz = pVz;
            last_L = L;
            last_h = h;

            lastGPSLongtitude = GPSLongitude;
            lastGPSLattitude = GPSLattitude;
            lastGPSh = GPSHeight;

//---------------------------------------------------------融合------------------------------
            if (GPS_SN >= 4) {
                tao += 0.2;
                double[][] Dpv = {{L * rad2deg - GPSLattitude}, {E * rad2deg - GPSLongitude}, {h - GPSHeight}, {pVx - GPSVe}, {pVy - GPSVn}, {pVz - GPSVu}};
                Matrix XX = kalman_gps_ins.kalman_GPS_INS_pv(Dpv, pVx, pVy, pVz, last_L, last_h, mahonyR, Fn, tao, Rm, Rn);
                pVx = pVx - XX.get(3, 0);
                pVy = pVy - XX.get(4, 0);
                pVz = pVz - XX.get(5, 0);
                L = L - 0.29 * XX.get(6, 0);
                E = E - 0.32 * XX.get(7, 0);
                h = h - XX.get(8, 0);
            }

            csvHelper.write(smoothAx + "," + smoothAy + "," + smoothAz + "," +
                    smoothGx + "," + smoothGy + "," + smoothGz + "," +
                    smoothMx + "," + smoothMy + "," + smoothMz + "," +
                    smoothGPSYaw + "," + smoothGPSv + "," + GPSVe + "," + GPSVn + "," +
                    Roll + "," + Pitch + "," + Yaw + "," +
                    Vccq[0] + "," + Vccq[1] + "," + Vccq[2] + "," +
                    pVx + "," + pVy + "," + pVz + "," +
                    E * rad2deg + "," + L * rad2deg + "," + h + "," + Px + "," + Py + "," +
                    resultOrientation[0] + "," + resultOrientation[1] + "," + resultOrientation[2] + "," + resultOrientation[3] +
                    "," + stepP[0] + "," + stepP[1] + "," + gyoP[0] + "," + gyoP[1] + "," + magP[0] + "," + magP[1] + "\n");

            smoothAx = 0;
            smoothAy = 0;
            smoothAz = 0;
            smoothGx = 0;
            smoothGy = 0;
            smoothGz = 0;
            smoothMx = 0;
            smoothMy = 0;
            smoothMz = 0;
            smoothGPSYaw = 0;
            smoothGPSv = 0;
        }

    }


    //由jy四元数构造旋转矩阵R
    private static void q2R(double[] q) {
        double q0q0 = q[0] * q[0];
        double q1q1 = q[1] * q[1];
        double q2q2 = q[2] * q[2];
        double q3q3 = q[3] * q[3];
        double q0q1 = q[0] * q[1];
        double q0q2 = q[0] * q[2];
        double q0q3 = q[0] * q[3];
        double q1q2 = q[1] * q[2];
        double q1q3 = q[1] * q[3];
        double q2q3 = q[2] * q[3];
        jyR = new Matrix(3, 3);
        jyR.set(0, 0, q0q0 + q1q1 - q2q2 - q3q3);
        jyR.set(0, 1, 2 * (q1q2 - q0q3));
        jyR.set(0, 2, 2 * (q1q3 + q0q2));
        jyR.set(1, 0, 2 * (q1q2 + q0q3));
        jyR.set(1, 1, q0q0 - q1q1 + q2q2 - q3q3);
        jyR.set(1, 2, 2 * (q2q3 - q0q1));
        jyR.set(2, 0, 2 * (q1q3 - q0q2));
        jyR.set(2, 1, 2 * (q2q3 + q0q1));
        jyR.set(2, 2, q0q0 - q1q1 - q2q2 + q3q3);

    }

    // 通过四元数构造转换矩阵R，然后acc = R · acc完成转换。
    private static Matrix quternionToCnbMatrix(float[] quaternion) {
        Matrix mahonyR;
        float[] q2 = quaternion;        // 四元数

        float q0q0 = q2[0] * q2[0];
        float q1q1 = q2[1] * q2[1];
        float q2q2 = q2[2] * q2[2];
        float q3q3 = q2[3] * q2[3];
        float q0q1 = q2[0] * q2[1];
        float q0q2 = q2[0] * q2[2];
        float q0q3 = q2[0] * q2[3];
        float q1q2 = q2[1] * q2[2];
        float q1q3 = q2[1] * q2[3];
        float q2q3 = q2[2] * q2[3];
        //定义Cnb
        mahonyR = new Matrix(3, 3);
        mahonyR.set(0, 0, q0q0 + q1q1 - q2q2 - q3q3);
        mahonyR.set(0, 1, 2 * (q1q2 - q0q3));
        mahonyR.set(0, 2, 2 * (q1q3 + q0q2));
        mahonyR.set(1, 0, 2 * (q1q2 + q0q3));
        mahonyR.set(1, 1, q0q0 - q1q1 + q2q2 - q3q3);
        mahonyR.set(1, 2, 2 * (q2q3 - q0q1));
        mahonyR.set(2, 0, 2 * (q1q3 - q0q2));
        mahonyR.set(2, 1, 2 * (q2q3 + q0q1));
        mahonyR.set(2, 2, q0q0 - q1q1 - q2q2 + q3q3);

        return mahonyR;
    }

    private static Matrix accToFn(Matrix R, double x, double y, double z) {

        Matrix m = new Matrix(3, 1);
        Matrix result = new Matrix(3, 1);
        m.set(0, 0, x);
        m.set(1, 0, y);
        m.set(2, 0, z);
        result = R.times(m);
//		m = R.times(m);

        return result;
    }

    private static float[] EularToQuaternion(double yaw, double pitch, double roll) {
        float[] Qresult = new float[4];
        double cosRoll = Math.cos(roll * 0.5);
        double sinRoll = Math.sin(roll * 0.5);
        double cosPitch = Math.cos(pitch * 0.5);
        double sinPitch = Math.sin(pitch * 0.5);
        double cosYaw = Math.cos(yaw * 0.5);
        double sinYaw = Math.sin(yaw * 0.5);

        Qresult[0] = (float) (cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
        Qresult[1] = (float) (sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw);
        Qresult[2] = (float) (cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw);
        Qresult[3] = (float) (cosRoll * cosPitch * sinYaw - sinRoll * sinRoll * cosYaw);

        return Qresult;
    }

    public static double[] TurnningTest(double gyoX, double gyoY, double gyoZ, double magX, double magY, double magZ) {
        double[] resultOrientation = new double[4];
        double[] gyoTemp = {gyoX, gyoY, gyoZ};
        double magOrientation;
        double magNorm = Math.sqrt((float) (magX * magX + magY * magY));
        if (magY > 0) {
            magOrientation = Math.asin((float) (magX / magNorm)) * rad2deg;
        } else {
            magOrientation = (Math.PI - Math.asin((float) (magX / magNorm))) * rad2deg;
        }
        if (magOrientation > 180) magOrientation -= 360;
        // 上面算出的实际上是磁北相对于手机的角度，而我们需要的是手机相对于磁北的角度，故返回值为相反数
        magOrientation = -magOrientation;

        double[] avSum = new double[3];
        double[] temp1 = lastAv;
        double[] temp2 = gyoTemp;
        avSum[0] += (temp1[0] + temp2[0]) * 0.05;
        avSum[1] += (temp1[1] + temp2[1]) * 0.05;
        avSum[2] += (temp1[2] + temp2[2]) * 0.05;

        // 陀螺仪的值似乎是北向转西向为正，故返回相反数
        double gyrOrientationDiff = -avSum[2];
        lastAv = gyoTemp;

        double gyrOrientation = 720;
        double uncalibratedGyrOrientation = 720;
        double calibratedMagOrientation = 720;
        if (!initAngle) {
            if (Math.abs(gyrOrientationDiff) > 20) {
                initGyrMagList.clear();
            } else {
                initGyrMagList.add(magOrientation);
            }
            if (initGyrMagList.size() > 10) {
                initAngle = true;
                initGyrHeading = calculateAngle325(initGyrMagList);
                initGyrMagList.clear();
                gyrOrientation = initGyrHeading;
                uncalibratedGyrOrientation = initGyrHeading;
            }
        } else {
            gyrOrientation = lastGyrOrientation + gyrOrientationDiff;
            uncalibratedGyrOrientation = lastUncalibratedGyrOrientation + gyrOrientationDiff;
            while (gyrOrientation > 180) gyrOrientation -= 360;
            while (gyrOrientation < -180) gyrOrientation += 360;
            while (uncalibratedGyrOrientation > 180) uncalibratedGyrOrientation -= 360;
            while (uncalibratedGyrOrientation < -180) uncalibratedGyrOrientation += 360;
        }

        if (!useGyr) {
            gyrOrientation = 720;
        }

        double stepOrientation;
        double thetaC = Math.abs(magOrientation - gyrOrientation);
        if (thetaC > 180) thetaC = 360 - thetaC;
        double thetaM = Math.abs(magOrientation - lastMagOrientation);
        if (thetaM > 180) thetaM = 360 - thetaM;
        if (gyrOrientation == 720) {
            if (lastStepOrientation == 720) {
                stepOrientation = magOrientation;
            } else {
                if (lastMagOrientation != 720 && thetaM > 15) {
                    if (Math.abs(gyrOrientationDiff) < 10) {
                        double[] orientations = {lastStepOrientation, magOrientation};
                        double[] weights = {0.8, 0.2};
                        stepOrientation = fuseOrientations(orientations, weights);
                    } else {
                        double[] orientations = {lastStepOrientation, magOrientation};
                        double[] weights = {0.2, 0.8};
                        stepOrientation = fuseOrientations(orientations, weights);
                    }
                } else {
                    double[] orientations = {lastStepOrientation, magOrientation};
                    double[] weights = {0.5, 0.5};
                    stepOrientation = fuseOrientations(orientations, weights);
                }
            }
            gyrOrientation = stepOrientation;
            uncalibratedGyrOrientation = stepOrientation;
        } else {
            double[] orientations = {lastStepOrientation, magOrientation, gyrOrientation};
            if (thetaC <= thetaCThreshold && thetaM <= thetaMThreshold) {
                double[] weights = Weights[0];
//                double[] weights = {0.3, 0.4, 0.3};
                stepOrientation = fuseOrientations(orientations, weights);
                flag = 1;
                countMethod[0]++;
            } else if (thetaC <= thetaCThreshold && thetaM > thetaMThreshold) {
                double[] weights = Weights[1];
                stepOrientation = fuseOrientations(orientations, weights);
                flag = 2;
                countMethod[1]++;
            } else if (thetaC > thetaCThreshold && thetaM <= thetaMThreshold) {
                if (Math.abs(gyrOrientationDiff) < 2.5) {
                    flag = 3;
                    countMethod[2]++;
                    double[] weights = Weights[2];
//                    double[] weights = {0.7, 0.055, 0.245};
                    stepOrientation = fuseOrientations(orientations, weights);
                } else {
                    flag = 4;
                    countMethod[3]++;
                    double[] weights = Weights[3];
                    stepOrientation = fuseOrientations(orientations, weights);
                }
            } else {
                if (Math.abs(gyrOrientationDiff) < 3.6) {
                    flag = 5;
                    countMethod[4]++;
                    double[] weights = Weights[4];
                    stepOrientation = fuseOrientations(orientations, weights);
                    calibratedMagOrientation = stepOrientation;
                } else {
                    flag = 6;
                    countMethod[5]++;
                    double[] weights = Weights[5];
                    stepOrientation = fuseOrientations(orientations, weights);
                }
            }
        }

        if (calibrateGyr) {
            double calibrateMagOri = 720;
            int magFlag = 0;
            double calibrateStepOri = 720;
            int stepFlag = 0;
            double stepDiff = Math.abs(lastStepOrientation - stepOrientation);
            if (stepDiff > 180) stepDiff = 360 - stepDiff;
            if (lastStepOrientation != 720 && stepDiff < 5) {
                calibrateGyrStepCount++;
            } else {
                calibrateGyrStepCount = 0;
            }
            if (calibrateGyrStepCount == 18) {
                calibrateStepOri = stepOrientation; // 取当前一步的方向作为绝对方向来校准陀螺仪的方向，可以改成求前几步的加权平均，类似下面地磁的处理。
                stepFlag = 1;
                calibrateGyrStepCount = 0;
                magCheckStepCount = 0;
            }

//            if (gyrOrientationDiff < 5) {
//                calibrateGyrMagList.add(magOrientation);
//            } else {
//                calibrateGyrMagList.clear();
//            }
//            if (calibrateGyrMagList.size() == 40) {
//                calibrateMagOri = AngleUtils.calculateAngle325(calibrateGyrMagList); // 取前几步地磁的平均作为绝对方向来校准陀螺仪的方向。
//                magFlag = 1;
//                calibrateGyrMagList.clear();
//            }

            if (magFlag + stepFlag == 1) {
                gyrOrientation = calibrateMagOri * magFlag + calibrateStepOri * stepFlag;
            } else if (magFlag + stepFlag == 2) {
                ArrayList<Double> list = new ArrayList<Double>();
                list.add(calibrateMagOri);
                list.add(calibrateStepOri);
                gyrOrientation = calculateAngle325(list);
            }
        }

        if (checkMag) {
            double thetaCDiff = Math.abs(thetaC - lastThetaC);
            if (thetaCDiff < 1.2) {
                magCheckStepCount++;
            } else {
                magCheckStepCount = 0;
            }
            if (magCheckStepCount == 5) {
                gyrOrientation = magOrientation;
                stepOrientation = magOrientation;
                System.out.println("checkmag");
                magCheckStepCount = 0;
            }
        }

        lastGyrOrientation = gyrOrientation;
        lastUncalibratedGyrOrientation = uncalibratedGyrOrientation;
        lastMagOrientation = magOrientation;
        lastStepOrientation = stepOrientation;
        lastThetaC = thetaC;

        resultOrientation[0] = gyrOrientation;
        resultOrientation[1] = uncalibratedGyrOrientation;
        resultOrientation[2] = magOrientation;
        resultOrientation[3] = stepOrientation;

        return resultOrientation;

    }

    private static double fuseOrientations(double[] orientations, double[] weights) {
        ArrayList<Double> list = new ArrayList<>();
        for (int i = 0; i < weights.length; i++) {
            int num = (int) Math.round(weights[i] * 1000);
            for (int j = 0; j < num; j++) {
                list.add(orientations[i]);
            }
        }
        return calculateAngle325(list);
    }

    public static double calculateAngle325(ArrayList<Double> list) {
        int count = 0;// 锟斤拷锟斤拷统锟狡达拷锟斤拷135锟斤拷小锟斤拷-135锟饺的革拷锟斤拷
        if (list.size() != 0) {

            ArrayList<Double> positiveList = new ArrayList<Double>();
            ArrayList<Double> negativeList = new ArrayList<Double>();
            for (int i = 0; i < list.size(); i++) {
                if (list.get(i) >= 0) {
                    positiveList.add(list.get(i));
                } else {
                    negativeList.add(list.get(i));
                }
                if (list.get(i) > 90 || list.get(i) < -90) {
                    count++;
                }
            }
            double positive = positiveList.size();
            double sum = list.size();
            double negative = negativeList.size();
            double percent = (positive / sum);
            // System.out.println(percent);
            if (positive == 0 || positive == sum) {
                return calculateAvg(list);
            }
            if (count > sum * 0.3) {
                // Log.d(tag, "姝ｈ礋閫傞噺鐩稿綋");
                double avgPositive = calculateAvg(positiveList);
                double avgNegative = calculateAvg(negativeList);

                double angleDifferenceABS = Math.abs(avgPositive - avgNegative);
                double angleDifference = 0;

                if (angleDifferenceABS >= 180) {
                    angleDifference = 360 - angleDifferenceABS;
                } else {
                    angleDifference = angleDifferenceABS;
                }
                double avgPiece = angleDifference / sum;

                if (Math.abs(avgNegative) + Math.abs(avgPositive) > 180) {
                    double num = avgNegative - positiveList.size() * avgPiece;
                    if (num < -180) {
                        return num + 360;
                    } else {
                        return num;
                    }
                } else {
                    double num = avgNegative + positiveList.size() * avgPiece;
                    return num;
                }
            } else {
                return calculateAvg(list);
            }
        } else {
            return 0;
        }
    }

    public static double calculateAvg(ArrayList<Double> list) {
        double sum = 0.0;
        for (int i = 0; i < list.size(); i++) {
            sum = sum + list.get(i);
        }
        return sum / list.size();

    }

}
