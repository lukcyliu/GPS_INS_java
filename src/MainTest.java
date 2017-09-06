/*
 * 程序主入口。
 */

import java.util.ArrayList;
import java.util.Scanner;

import Jama.Matrix;
import tools.*;

public class
MainTest {
    //git测试
    private static MadgwickAHRS madgwickAHRS;
    private static MahonyAHRS mahonyAHRS;
    private static Matrix madgwickR;
    private static Matrix mahonyR;
    //预设陀螺仪的过零阈值
    //预设滑动窗口开关以及长度
    private static double GYRO_THRE = Config.GYRO_THRESHOLD;
    private static boolean SMOOTHEN_ON = Config.SMOOTHEN_ON;
    private static int WINDOW = Config.SMOOTHEN_WINDOW;

    private static int Start = Config.DATA_START;
    private static int End = Config.DATA_END;

    private static int Timecount = 0;

    private static boolean kalman_flag = false;
    // 跟个什么线性加速度相关的
//	private static double[] gravity = new double[3];
//	private static double[] linearAcceleration = new double[3];
//	private final static double alpha = 0.8;

    @SuppressWarnings({"unused", "unchecked"})
    public static void main(String[] args) {


        IOAdapter.createOutputFile();
        NewKalmanFilter kalmanFilter = new NewKalmanFilter();

        // 所有输入数据均在input中，每个点为一个float数组，0-5分别为加速度和陀螺仪三轴数值ax,ay,az,gx,gy,gz。
        ArrayList<float[]> input = IOAdapter.input(6);
        ArrayList<Integer> input_time = IOAdapter.getTime();
        ArrayList<float[]> input_GPS = IOAdapter.getGPSdata(10);

        // 平滑处理
        if (SMOOTHEN_ON) {
            input = smoothen(input);
        }

        // 计算静止加速度
//        Acceleration staticAcc = new Acceleration();
//        for (int i = 1250; i < 1300; i++) {
//            staticAcc.x += input.get(i)[0];
//            staticAcc.y += input.get(i)[1];
//            staticAcc.z += input.get(i)[2];
//        }
//        staticAcc.x = staticAcc.x * (Config.DATA_SOURCE == Config.IMU ? 9.8 : 1) / 10;
//        staticAcc.y = staticAcc.y * (Config.DATA_SOURCE == Config.IMU ? 9.8 : 1) / 10;
//        staticAcc.z = staticAcc.z * (Config.DATA_SOURCE == Config.IMU ? 9.8 : 1) / 10;
        //显示静止加速度
//		System.out.println(staticAcc.x + ", " + staticAcc.y + ", " + staticAcc.z);
//        EngineINS.setStaticAcceleration(staticAcc);//by 陈一曲 咱们之前用的那个是从1300开始的数据，那这段静止加速度是不是有问题？

        float samplePeriod;
        samplePeriod = Config.DATA_SOURCE == Config.IMU ? Config.SAMPLE_PERIOD_IMU : Config.SAMPLE_PERIOD_PHONE;

        // 两个算法
        madgwickAHRS = new MadgwickAHRS((float) samplePeriod, 0.041f);// 实例化，传入的参数第一个是采样的时间间隔，千万不能错，第二个应该是跟梯度下降算法的收敛速度有关的。。。
        mahonyAHRS = new MahonyAHRS((float) samplePeriod);

//		ArrayList SampleVx =new ArrayList();//用来存放所有时刻的速度
//		ArrayList SampleVy =new ArrayList();
//		ArrayList SampleVz =new ArrayList();
        double[] SampleVx = new double[input.size()];//用来存放所有时刻的速度
        double[] SampleVy = new double[input.size()];
        double[] SampleVz = new double[input.size()];
//        boolean first = true;
        double L = 0.0, h = 0.0;
        Matrix LastSampleVx = new Matrix(3, 1);
        Matrix w = new Matrix(3, 3);
        double[][] G = {{0}, {0}, {9.8}};
        Matrix g = new Matrix(G);
        Acceleration templeA = new Acceleration();
        Velocity templeV = new Velocity();
        Position templeS = new Position();
        Matrix X = new Matrix(15,1);
        for (int i = Start; i < input.size(); i++) {
//			in.nextLine();

//			// 用根据四元数得出的转换矩阵，将测得的加速度从载体坐标系转换到导航坐标系
//			// 先用加速度和陀螺仪的数据更新R矩阵
//			float[] sampleToR = input.get(i); 
//			updateRMatrix(sampleToR[0], sampleToR[1], sampleToR[2], sampleToR[3], sampleToR[4], sampleToR[5]);
//			
//			mahonyR.print(3, 8);
            // 进行单位转换
            float[] sample = Config.DATA_SOURCE == Config.IMU ? transUnits(input.get(i)) : input.get(i);
            int this_time = input_time.get(i).intValue();//这一个数据的时间戳


            Omega omega = null;
            if (i == Start) {
                omega = new Omega(39.9840352054716, 60, 0, 0);
            } else {
                omega = new Omega(39.9840352054716, 60, SampleVy[i - Start - 1], SampleVx[i - Start - 1]);
            }//第二次开始用上一次的速度进行计算加速度

//			System.out.println("before "+sample[0]+" "+sample[1]+" "+sample[2]);

            // 用根据四元数得出的转换矩阵，将测得的加速度从载体坐标系转换到导航坐标系
            // 先用加速度和陀螺仪的数据更新R矩阵
            updateRMatrix(sample[0], sample[1], sample[2], sample[3], sample[4], sample[5]);


            // 用R矩阵分别对加速度和陀螺仪的数据进行转换
            Matrix acc;
            Matrix gyo;
            if (Config.AHRS_MODE == Config.AHRS_MODE_MADGWICK) {
                acc = transWithRMatrix(madgwickR, sample[0], sample[1], sample[2]);
//				System.out.println(sample[0]+" "+sample[1]+" "+sample[2]);
//				System.out.println("This is algorithms of Madgwick!");
                gyo = transWithRMatrix(madgwickR, sample[3], sample[4], sample[5]);
            } else {
                acc = transWithRMatrix(mahonyR, sample[0], sample[1], sample[2]);
//				System.out.println(" after "+acc.get(0, 0)+" "+acc.get(1, 0)+" "+acc.get(2, 0));
//				System.out.println("This is algorithms of Mahony!");
                gyo = transWithRMatrix(mahonyR, sample[3], sample[4], sample[5]);

            }
            //By 陈一曲，到这儿应该就是根据姿态角，将加速度转成了p系的加速度，也就是说导航方程那儿的f转换完成了。
//			
            // 将陀螺仪数据传入进行处理，若陀螺仪数据较小则忽略
            double gx = gyo.get(0, 0);
            double gy = gyo.get(1, 0);
            double gz = gyo.get(2, 0);
            if (Math.abs(gx) < GYRO_THRE) gx = 0;
            if (Math.abs(gy) < GYRO_THRE) gy = 0;
            if (Math.abs(gz) < GYRO_THRE) gz = 0;
            AngularVelocity sampleAV = new AngularVelocity(gx, gy, gz);
            EngineINS.addNewAngularVelocity(sampleAV);

            w = new Matrix(3, 3);
            double[][] wget = omega.getWadd();
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                    w.set(a, b, wget[a][b]);//这儿应该就根据经纬度和高度获得的角速度矩阵求出来了，但是这里计算用到的速度全是GPS给的，这个还得改


            // 将加速度数据传入进行处理
            //在坐标系转换后给加速度进行单位转换
            double ax, ay, az;
            if (Config.DATA_SOURCE == Config.IMU) {
//                acc = accCal(acc, w, LastSampleVx, g);
//                ax = acc.get(0, 0) * Config.G2MS2;
//                ay = acc.get(1, 0) * Config.G2MS2;
//                az = acc.get(2, 0) * Config.G2MS2;
                ax = acc.get(0, 0);
                ay = acc.get(1, 0);
                az = acc.get(2, 0);
            } else {
                //加速度转换后进行矫正
                acc = accCal(acc, w, LastSampleVx, g);
                ax = acc.get(0, 0);
                ay = acc.get(1, 0);
                az = acc.get(2, 0);
            }//by 陈一曲，如果按照之前说的要消除地球自转等误差的话，这里应该不要减去g的影响，转换单位等全部弄完再说


            templeA = new Acceleration(ax, ay, az);
            //判断加速度阈值

            templeV = EngineINS.addNewAccelerationSample(templeA);
            templeS = EngineINS.addNewVelocitySample(templeV);

            if (Timecount < this_time && !kalman_flag){//表示这是新的时间数据，且是第一个，这个对应着有新的GPS数据，找到对应的GPS数据，然后给卡尔曼处理
                float[] this_GPS = input_GPS.get(Timecount);//找到相同时间戳的GPS数据
                Timecount++;
                kalmanFilter.input_GPS_INS_Data(this_GPS[1],this_GPS[2],this_GPS[3],this_GPS[4],this_GPS[5],this_GPS[6],sample[0],sample[1],sample[2]);//初始化数值
                kalmanFilter.initial(mahonyR,0.00007292, omega.getRn(), omega.getRm());
                kalmanFilter.setX(X);
                kalmanFilter.setZ(this_GPS[4], this_GPS[5], this_GPS[6], this_GPS[7], this_GPS[8], this_GPS[9],templeV.x, templeV.y, templeV.z, templeS.x,templeS.y,templeS.z);
                kalmanFilter.itegration();
                kalman_flag = true;
            }//卡尔曼处理的过程

            X = kalmanFilter.getX();

            if(kalman_flag) {
                EngineINS.addX(X);
                Matrix Cpn = new Matrix(3,3);
                Cpn.set(0,0,1);
                Cpn.set(0,1,X.get(2,0) * -1);
                Cpn.set(0,2,X.get(1,0));
                Cpn.set(1,0,X.get(2,0));
                Cpn.set(1,1,1);
                Cpn.set(1,2,X.get(0,0) * -1);
                Cpn.set(2,0,X.get(1,0) * -1);
                Cpn.set(2,1,X.get(0,0));
                Cpn.set(2,2,1);
                Cpn.print(2,15);
                mahonyR = Cpn.times(mahonyR);
            }

            EngineINS.outputCurrentState();
            kalman_flag = false;
        }
//        //速度矫正
//        EngineINS.removeVelocityDrift(SampleVx);
//        EngineINS.removeVelocityDrift(SampleVy);
//        EngineINS.removeVelocityDrift(SampleVz);
        //计算位移
//        EngineINS.addCalibratedVelocity(SampleVx, SampleVy, SampleVz);
        IOAdapter.output();
    }

    // 通过四元数构造转换矩阵R，然后acc = R · acc完成转换。
    private static void updateRMatrix(float aX, float aY, float aZ, float gX, float gY, float gZ) {

        madgwickAHRS.update(gX, gY, gZ, aX, aY, aZ);
        mahonyAHRS.update(gX, gY, gZ, aX, aY, aZ);
//		double[] euler = madgwickAHRS.getEuler();		// 0 = Yaw; 1 = Roll; 2 = Pitch;
        float[] q1 = madgwickAHRS.Quaternion;        // 四元数

        double q0q0 = q1[0] * q1[0];
        double q1q1 = q1[1] * q1[1];
        double q2q2 = q1[2] * q1[2];
        double q3q3 = q1[3] * q1[3];
        double q0q1 = q1[0] * q1[1];
        double q0q2 = q1[0] * q1[2];
        double q0q3 = q1[0] * q1[3];
        double q1q2 = q1[1] * q1[2];
        double q1q3 = q1[1] * q1[3];
        double q2q3 = q1[2] * q1[3];

        madgwickR = new Matrix(3, 3);
        madgwickR.set(0, 0, q0q0 + q1q1 - q2q2 - q3q3);
        madgwickR.set(0, 1, 2 * (q1q2 - q0q3));
        madgwickR.set(0, 2, 2 * (q1q3 + q0q2));
        madgwickR.set(1, 0, 2 * (q1q2 + q0q3));
        madgwickR.set(1, 1, q0q0 - q1q1 + q2q2 - q3q3);
        madgwickR.set(1, 2, 2 * (q2q3 - q0q1));
        madgwickR.set(2, 0, 2 * (q1q3 - q0q2));
        madgwickR.set(2, 1, 2 * (q2q3 + q0q1));
        madgwickR.set(2, 2, q0q0 - q1q1 - q2q2 + q3q3);

        float[] q2 = mahonyAHRS.Quaternion;        // 四元数

        q0q0 = q2[0] * q2[0];
        q1q1 = q2[1] * q2[1];
        q2q2 = q2[2] * q2[2];
        q3q3 = q2[3] * q2[3];
        q0q1 = q2[0] * q2[1];
        q0q2 = q2[0] * q2[2];
        q0q3 = q2[0] * q2[3];
        q1q2 = q2[1] * q2[2];
        q1q3 = q2[1] * q2[3];
        q2q3 = q2[2] * q2[3];

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

    private static Matrix accCal(Matrix acc, Matrix w, Matrix Vep, Matrix g) {
        w = w.times(Vep);
        return acc.minus(g).minus(w);

    }

    private static ArrayList<float[]> smoothen(ArrayList<float[]> input) {

        ArrayList<float[]> smoothened = new ArrayList<float[]>();

        float[] sum10 = new float[6];
        for (int i = 0; i < WINDOW - 1; i++) {

            float[] sampleI = input.get(i);
            for (int j = 0; j < 6; j++) {
                sum10[j] += sampleI[j];
            }
        }
        for (int i = WINDOW - 1; i < input.size(); i++) {

            float[] sampleI = input.get(i);
            for (int j = 0; j < 6; j++) {
                sum10[j] = (sum10[j] + sampleI[j]) / WINDOW;
            }
            smoothened.add(sum10.clone());
            sampleI = input.get(i - WINDOW + 1);
            for (int j = 0; j < 6; j++) {
                sum10[j] = sum10[j] * WINDOW - sampleI[j];
            }
        }
        return smoothened;
    }

    private static float[] transUnits(float[] sample) {

        float[] result = new float[6];
        for (int i = 0; i < 3; i++) {
			result[i] = sample[i] * Config.G2MS2;
//            result[i] = sample[i];
        }
        for (int i = 3; i < 6; i++) {
            result[i] = sample[i] * Config.C2RADS;
//			result[i] = sample[i] ;
        }
        return result;
    }
}
