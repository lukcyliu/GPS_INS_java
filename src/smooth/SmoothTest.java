package smooth;

import Jama.Matrix;
import javafx.geometry.Pos;
import tools.Acceleration;
import tools.MahonyAHRS;
import tools.Position;

import java.util.ArrayList;


/**
 * Created by lukcy on 2017/7/14.
 */
public class SmoothTest {
    int firstSmooth = 0;
    int width = 50;
    static double accCalErr_X = -0.07, accCalErr_Y = 0.075, accCalErr_Z = 0.03;
    double[][] acc = new double[3][width];
    double[][] gyo = new double[3][width];
    double[][] mag = new double[3][width];
    double[][] angle = new double[3][width];
    double[][] gps = new double[2][width];


    static Matrix mahonyR;
    static Matrix jyR;

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

    public void display() {

    }

    public static void main(String[] args) {

        //acc高通低通滤波
        double acc_mod = 0, acc_low = 0, acc_high = 0;
        double low_value = 0.2;

        double  hx = 0, hy = 0, hAy = 0, hAx = 0, hVy = 0, hVx = 0, hSx = 0, hSy = 0, hV2 = 0, hS2 = 0, hSy2 = 0, hSx2 = 0, lasthA2 = 0, lasthV2 = 0;
        double lasthAy = 0, lasthAx = 0, lastlastAy = 0, lastlastAx = 0, lasthVy = 0, lasthVx = 0, lastlastVy = 0, lastlastVx = 0;

        double yaw = 90;
        double hubuYaw = 90;
        int count = 0;
        SmoothTest smoothTest = new SmoothTest();
        //io read csv.
        CsvHelper csvHelper = new CsvHelper();
        csvHelper.creatOutputFile();
        csvHelper.write("smoothAx" + "," + "smoothAy" + "," + "smoothAz,acc_mod" + "," +
                "smoothGx" + "," + "smoothGy" + "," + "smoothGz" + "," +
                "smoothMx" + "," + "smoothMy" + "," + "smoothMz" + "," +
                "smoothGPSYaw" + "," + "smoothGPSv" + "," + "GPSVe" + "," + "GPSVn" + "," + "Px" + "," + "Py" + "," + "slideYaw" + "," + "mahanyYaw,jyYaw" + "," + "hx" + "," + "hy" + "," + "hAx,hAy,hVx,hVy,hSx,hSy,hSx2,hSxy2" + "\n");
        ArrayList<double[]> input = csvHelper.read(20);
        System.out.println(input.size());
        double[] data = new double[20];
        ArrayList<double[][]> smoothRes;
        double ax, ay, az, gx, gy, gz, mx, my, mz, jyYaw, GPSYaw = 0, GPSv = 0, GPSLongitude, GPSLattitude, GPSHeight, GPS_SN;
        double[] q = new double[4];
        double smoothAx = 0, smoothAy = 0, smoothAz = 0, smoothGx = 0, smoothGy = 0, smoothGz = 0, smoothMx = 0, smoothMy = 0, smoothMz = 0, smoothGPSYaw = 0, smoothGPSv = 0;
        double GPSVn = 0, GPSVe = 0;

        double last_L = 0, last_lamb = 0, last_h = 0;
        int firstGPSOff = 0;
        int firstGPSVcc_in = 0;
        int firstGPSVcc_out = 0;
        int GPSOff = 0;
        double speedE=0, speedN=0, speedH=0;
        double pre_lamb=0, pre_L=0, pre_h=0;
        double INS_lamb=0,INS_L=0,INS_h=0;
        double lastpVx = 0, lastpVy = 0;
        double pVx = 0,pVy = 0;
        double lastAx=0,lastAy = 0;
        double Px = 0, Py = 0;
        Position AddS = new Position();
        int insCnt=0;

        //滑动滤波
        for (int i = 0; i < input.size(); i++) {
            data = input.get(i);
            ax = data[0] - accCalErr_X;
            ay = data[1] - accCalErr_Y;
            az = data[2] - accCalErr_Z;
            gx = data[3];
            gy = data[4];
            gz = data[5];
            mx = data[6];
            my = data[7];
            mz = data[8];
            q[0] = data[9];
            q[1] = data[10];
            q[2] = data[11];
            q[3] = data[12];
            jyYaw = data[13];
            GPSLongitude = data[14];
            GPSLattitude = data[15];
            GPSHeight = data[16];
            GPSYaw = data[17];
            GPSv = data[18];
            GPS_SN = data[19];
            //设置低通高通滤波器，剥离重力分量得到静止检测算子
            acc_mod = Math.sqrt(ax * ax + ay * ay + az * az);
//            //低通滤波提出重力分量影响
//            acc_low = ay * (1 - low_value)  + low_value * acc_low;
//            //高通滤波剥离重力分量
//            acc_high = ay - acc_low;
//            if(GPSYaw > 180)
//                GPSYaw -= 360;
//            else if (GPSYaw < -180)
//                GPSYaw += 360;
            //循环队列方法
            double slidejyYaw = smoothTest.slideFilter(jyYaw);
            //暴力方法
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

            //低通滤波提出重力分量影响
            acc_low = smoothAy * (1 - low_value)  + low_value * acc_low;
            //高通滤波剥离重力分量
            acc_high = smoothAy - acc_low;

            if (firstGPSOff == 0){
                if (GPS_SN < 4){
                    System.out.println("Waiting GPS working......................................");
                    continue;
                }
                System.out.println("first location..................................seesion 0 ");
                last_lamb = GPSLongitude;
                last_L = GPSLattitude;
                last_h = GPSHeight;
                INS_lamb = last_lamb;
                INS_L = last_L;
                INS_h = last_h;

                firstGPSOff = 1;
            }
            else if(firstGPSOff ==1) {
                if(GPS_SN < 4){

                }
                else if (GPS_SN >= 4) {
                    if (GPSLattitude == last_L && GPSLongitude == last_lamb) {
                        System.out.println("---------------------------------------session 1");
                        if (firstGPSVcc_in == 0) {
                            pre_lamb = 0;
                            pre_L = 0;
                            pre_h = 0;
                            speedE = GPSv * Math.cos(GPSYaw) / 3.6;
                            speedN = GPSv * Math.sin(GPSYaw) / 3.6;
                            speedH = GPSHeight - last_h;
                            lastpVx = speedE;
                            lastpVy = speedN;
                            System.out.println("---------------------------------------session 1_1");
                        }
                        firstGPSVcc_in = 1;
                        q2R(q);
                        Matrix accMatrix = transWithRMatrix(jyR, smoothAx, smoothAy, smoothAz);
                        pVx = lastpVx + 0.5 * (accMatrix.get(0, 0) + lastAx) * 0.2;
                        pVy = lastpVy + 0.5 * (accMatrix.get(1, 0) + lastAy) * 0.2;
                        Px = 0.5 * (pVx + lastpVx) * 0.2;
                        Py = 0.5 * (pVy + lastpVy) * 0.2;
                        lastAx = accMatrix.get(0, 0);
                        lastAy = accMatrix.get(1, 0);
                        lastpVx = pVx;
                        lastpVy = pVy;
                        pre_lamb += Px / (111000 * Math.cos(INS_L * 3.1415926 / 180));
                        pre_L += Py / 111000;
                    } else {
                        System.out.println("---------------------------------------------session 2");
                        speedE = GPSv * Math.cos(GPSYaw) / 3.6;
                        speedN = GPSv * Math.sin(GPSYaw) / 3.6;
                        speedH = GPSHeight - last_h;

                        last_lamb = GPSLongitude;
                        last_L = GPSLattitude;
                        last_h = GPSHeight;

                        q2R(q);
                        Matrix accMatrix = transWithRMatrix(jyR, smoothAx, smoothAy, smoothAz);
                        pVx += 0.5 * (accMatrix.get(0, 0) + lastAx) * 0.2;
                        pVy += 0.5 * (accMatrix.get(1, 0) + lastAy) * 0.2;
                        Px += 0.5 * (pVx + lastpVx) * 0.2;
                        Py += 0.5 * (pVy + lastpVy) * 0.2;
                        lastAx = accMatrix.get(0, 0);
                        lastAy = accMatrix.get(1, 0);
                        lastpVx = pVx;
                        lastpVy = pVy;
                        INS_lamb += pre_lamb + Px / (111000 * Math.cos(INS_L * 3.1415926 / 180));
                        INS_L += pre_L + Py / 111000;
                        //-------------卡尔曼融合部分---------------------------------//
                        //------------------------------------------------------------//
                        AddS.x = INS_lamb;
                        AddS.y = INS_L;
                        firstGPSVcc_in = 0;
                    }
                }
            }
//-------------------------------------------------以下是乱七八糟的测试部分------------------------------------------------//
 //----------------------------------------------------角度测试部分--------------------------------------------------------//
//            //旋转矩阵积分法,前左上
//            updateRMatrix((float) smoothAy, (float) -smoothAx, -(float) smoothAz, (float) smoothGx, (float) smoothGy, (float) smoothGz);
           //右前上
            updateRMatrix((float)smoothAx,(float)smoothAy,(float)smoothAz,(float)smoothGx,(float)smoothGy,(float)smoothGz);
            mahonyR = mahonyR.transpose();
            q2R(q);
            jyR = jyR.transpose();
            //在旋转矩阵Cbn中提出欧拉角（弧度）
            double pitch = Math.asin(mahonyR.get(2, 1));
            double roll = Math.atan(-mahonyR.get(0, 1) / mahonyR.get(1, 1));
            hubuYaw += Math.atan(-mahonyR.get(0, 1) / mahonyR.get(1, 1)) * 180 / 3.1416 / 50;
            yaw = -Math.atan2(-jyR.get(0, 1), jyR.get(1, 1)) * 180 / 3.1416;
//            yaw = Math.atan2(2*(q[0]*q[3]+q[1]*q[2]) , (1-2*q[2]*q[2]-2*q[3]*q[3])) *180/3.1416;

//-----------------------------------------------------积分测试部分--------------------------------------------------------//
//用求得的分加速度进行最小二乘训练以及积分两种方式的测试

            //航位推算法
            //1.GPSV+GPSYaw
            hx += GPSVe * 0.2;
            hy += GPSVn * 0.2;
            //2.1将汽车前进y轴加速度分解并梯形积分到东北向速度
//            if(Math.abs(acc_high) < 0.05 || Math.abs(acc_high) > 0.5)
//                smoothAy = 0;
            hAx = 9.8015 * smoothAy * Math.sin(smoothGPSYaw * 3.1415926 / 180);
            hAy = 9.8015 * smoothAy * Math.cos(smoothGPSYaw * 3.1415926 / 180);

            //梯形积分
//            hVx += 0.5 * (hAx + lasthAx) * 0.2;
//            hVy += 0.5 * (hAy + lasthAy) * 0.2;
            //辛普森积分
            hVx += (lastlastAx + 4 * lasthAx + hAx) * 0.2 / 3;
            hVy += (lastlastAy + 4 * lasthAy + hAy) * 0.2 / 3;
            hSx += (lastlastVx + 4 * lasthVx + hVx) * 0.2 / 3;
            hSy += (lastlastVy + 4 * lasthVy + hVy) * 0.2 / 3;
            lastlastAx = lasthAx;
            lastlastAy = lasthAy;
            lasthAx = hAx;
            lasthAy = hAy;
            lastlastVx = lasthVx;
            lastlastVy = lasthVy;
            lasthVx = hVx;
            lasthVy = hVy;
            //2.2若先积分出全部的位移，再乘上航向角
            hV2 += 0.5 * (lasthA2 + smoothAy) * 9.8015 * 0.2;
//            if(Math.abs(acc_high) < 0.05 || Math.abs(acc_high) > 0.6) hV2=0;
            hS2 += 0.5 * (lasthV2 + hV2) * 0.2;
            hSx2 = hS2 * Math.sin(smoothGPSYaw * 3.1415926 / 180);
            hSy2 = hS2 * Math.cos(smoothGPSYaw * 3.1415926 / 180);
            lasthA2 = smoothAy;
            lasthV2 = hV2;

            csvHelper.write(smoothAx + "," + smoothAy + "," + smoothAz + "," + acc_high + "," +
                    smoothGx + "," + smoothGy + "," + smoothGz + "," +
                    smoothMx + "," + smoothMy + "," + smoothMz + "," +
                    smoothGPSYaw + "," + smoothGPSv + "," + GPSVe + "," + GPSVn + "," + AddS.x + "," + AddS.y + "," + slidejyYaw + "," + hubuYaw + "," + yaw + "," + hx + "," + hy +
                    "," + hAx + "," + hAy + "," + hVx + "," + hVy + "," + hSx + "," + hSy + "," + hSx2 + "," + hSy2 + "," + hV2 + "," + hS2 + "\n");
//            System.out.println((count++)+"smoothAcc = "+(float)smoothAx+" "+(float)smoothAy+" "+(float)smoothAz+" ; "+
//                                "smoothGyo = "+(float)smoothGx+" "+(float)smoothGy+" "+(float)smoothGz+" ; "+
//                                "smoothMag = "+(float)smoothMx+" "+(float)smoothMy+" "+(float)smoothMz);
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
    private static void updateRMatrix(float aX, float aY, float aZ, float gX, float gY, float gZ) {
        MahonyAHRS mahonyAHRS = new MahonyAHRS((float) 0.2, (float) 2, (float) 0.05);
        mahonyAHRS.update(gX, gY, gZ, aX, aY, aZ);

        float[] q2 = mahonyAHRS.Quaternion;        // 四元数

        double q0q0 = q2[0] * q2[0];
        double q1q1 = q2[1] * q2[1];
        double q2q2 = q2[2] * q2[2];
        double q3q3 = q2[3] * q2[3];
        double q0q1 = q2[0] * q2[1];
        double q0q2 = q2[0] * q2[2];
        double q0q3 = q2[0] * q2[3];
        double q1q2 = q2[1] * q2[2];
        double q1q3 = q2[1] * q2[3];
        double q2q3 = q2[2] * q2[3];
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
    }

    private static Matrix transWithRMatrix(Matrix R, double x, double y, double z) {

        Matrix m = new Matrix(3, 1);
        Matrix result = new Matrix(3, 1);
        m.set(0, 0, x);
        m.set(1, 0, y);
        m.set(2, 0, z);
        result = R.times(m);
//		m = R.times(m);

        return result;
    }
}
