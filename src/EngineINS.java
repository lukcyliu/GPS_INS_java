/*
 * 惯导主要部分。
 */

import java.util.ArrayList;

import Jama.Matrix;
import tools.Acceleration;
import tools.Angle;
import tools.AngularVelocity;
import tools.Position;
import tools.Velocity;

public class EngineINS {

    @SuppressWarnings("unused")
    // 将配置文件中的参数导入
//    private static final double SP = Config.DATA_SOURCE == Config.IMU ? Config.SAMPLE_PERIOD_IMU : Config.SAMPLE_PERIOD_PHONE;
    private static final double SP =  (double)(1.0/100);
    private static final Acceleration ACC_THRE = new Acceleration(Config.ACC_THRESHOLD);
    private static  final Velocity VAL_THRE = new Velocity(Config.Val_THRESHOLD);
    private static final boolean CENT_ACC_ON = Config.CENTRIPETAL_ACCELERATION_ON;
    private static final boolean STA_ACC_ON = Config.STATIC_ACCELERATION_ON;

    // 静止加速度
    private static boolean hasStaticStateSet = true;
    private static Acceleration staticAcc = new Acceleration();

    // 记录状态
    private static Acceleration lastAcc = new Acceleration();
    private static AngularVelocity lastAV = new AngularVelocity();
    private static Velocity lastV = new Velocity();
    private static Acceleration centripetalAcc = new Acceleration();
    private static Position currentPos = new Position();
    private static Position tempPos = new Position();

    // 拐弯相关部分
    private static ArrayList<AngularVelocity> turningWindowAV = new ArrayList<>();
    private static Angle turningAngle = new Angle();
    private static boolean isTurning = false;

    public static Velocity addNewAccelerationSample(Acceleration sampleACC) {

        // 必须要先获得静止时测得的加速度结果，目前默认为已设置
        if (!hasStaticStateSet) return null;
//		if (!hasStaticStateSet) return;

        // 减去静止加速度
        if (STA_ACC_ON) {
            sampleACC.x -= staticAcc.x;
            sampleACC.y -= staticAcc.y;
            sampleACC.z -= staticAcc.z;
        }

        // 如果检测到拐弯，则进行向心加速度校正
        if (CENT_ACC_ON && isTurning) {
            centripetalAcc = getCentripetalAcceleration();
            sampleACC.x -= centripetalAcc.x;
            sampleACC.y -= centripetalAcc.y;
            sampleACC.z -= centripetalAcc.z;
            isTurning = false;
//            System.out.println("Turning");
//            System.out.println(centripetalAcc.x);
//            System.out.println(centripetalAcc.y);
//            System.out.println(centripetalAcc.z);
            IOAdapter.writeCentripetalAcceleration(centripetalAcc.x + "," + centripetalAcc.y + "," + centripetalAcc.z + "\n");
        }
        Velocity sampleV = new Velocity(lastV);
        // 用计算梯形面积和的方法完成积分
        //梯形积分法
//        sampleV.x += (sampleACC.x + lastAcc.x) * SP * 0.5;
//        sampleV.y += (sampleACC.y + lastAcc.y) * SP * 0.5;
//        sampleV.z += (sampleACC.z + lastAcc.z) * SP * 0.5;
        //矩形积分法
        sampleV.x += sampleACC.x * SP ;
        sampleV.y += sampleACC.y * SP ;
        sampleV.z += sampleACC.z * SP ;

        // 去掉小的波动
		if (Math.abs(sampleACC.x) < ACC_THRE.x) {
            if (Math.abs(lastV.x) < VAL_THRE.x)
                sampleV.x = 0;
            else
                sampleV.x = lastV.x;
        }
		if (Math.abs(sampleACC.y) < ACC_THRE.y){
            if(Math.abs(lastV.y) < VAL_THRE.y)
                sampleV.y = 0;
            else
                sampleV.y = lastV.y;
        }
		if (Math.abs(sampleACC.z) < ACC_THRE.z){
            if(Math.abs(lastV.z) < VAL_THRE.z)
                sampleV.z = 0;
            else
                sampleV.z = lastV.z;
        }
        sampleV.t = sampleACC.t;
        lastAcc = new Acceleration(sampleACC);
        IOAdapter.writeAcceleration(lastAcc.x + "," + lastAcc.y + "," + lastAcc.z + "\n");
//        System.out.println(sampleV.x + " " + sampleV.y + " " + sampleV.z);
        lastV = new Velocity(sampleV);
        double[] V = new double[3];
        V[0] = sampleV.x;
        V[1] = sampleV.y;
        V[2] = sampleV.z;
        // 用求得的新的即时速度来更新位置
//		addNewVelocitySample(sampleV);
        return sampleV;
    }

    public static void addNewAngularVelocity(AngularVelocity sampleAV) {

        // 必须要先获得静止时测得的加速度结果，目前默认为已设置
        if (!hasStaticStateSet) return;

        // 如果当前转弯列表为空，则添加一个大小为0的角速度，方便计算
        if (turningWindowAV.isEmpty()) turningWindowAV.add(new AngularVelocity());

        // 角速度积分得到角度
        AngularVelocity listLastAV = turningWindowAV.get(turningWindowAV.size() - 1);
        turningAngle.x += (sampleAV.x + listLastAV.x) * SP * 0.5;
        turningAngle.y += (sampleAV.y + listLastAV.y) * SP * 0.5;
        turningAngle.z += (sampleAV.z + listLastAV.z) * SP * 0.5;

        turningWindowAV.add(sampleAV);

        // 如果列表中数量已经超过规定的阈值，则将队首元素移除
        if (turningWindowAV.size() > Config.TURNING_SAMPLE_NUMBER_THRESHOLD) {
            AngularVelocity removedAV = turningWindowAV.remove(0);
            AngularVelocity firstAV = turningWindowAV.get(0);
            turningAngle.x -= (removedAV.x + firstAV.x) * SP * 0.5;
            turningAngle.y -= (removedAV.y + firstAV.y) * SP * 0.5;
            turningAngle.z -= (removedAV.y + firstAV.y) * SP * 0.5;
        }

        // 如果大于45度则认为拐弯
        if (Math.abs(turningAngle.x) + Math.abs(turningAngle.y) + Math.abs(turningAngle.z) > Math.PI / 4) {
            isTurning = true;
            turningWindowAV.clear();
            turningAngle = new Angle();
        } else {
            isTurning = false;
        }
//		System.out.println(turningAngle.x);
//		System.out.println(turningAngle.y);
//		System.out.println(turningAngle.z);
        lastAV = new AngularVelocity(sampleAV);
        IOAdapter.writeAngularVelocity(lastAV.x + "," + lastAV.y + "," + lastAV.z + "\n");
    }

    //计算并减去速度漂移
    public static void removeVelocityDrift(float[] a) {
        int nlength = a.length;
        int[] b = new int[nlength];
        int countB = 0;
        int[] e = new int[nlength];
        int countE = 0;
//        System.out.println("The length of Array is : " + nlength);
        int i;
        //找begin下标
        for (i = 0; i < nlength; i++) {
            if (a[i] != 0 && i == 0) {
                b[countB++] = i;
            } else if (a[i] != 0 && a[i - 1] == 0) {
                b[countB++] = i;
            }
        }
        //找End下标
        for (i = 0; i < nlength; i++) {
            if (a[i] != 0 && i == nlength - 1) {
                e[countE++] = i;
            } else if (a[i] != 0 && a[i + 1] == 0) {
                e[countE++] = i;
            }

        }
        //减去drift
        for (i = 0; i < countE; i++) {
            int k;
            double drift;
            int count = 1;
            for (k = b[i]; k <= e[i]; k++) {

                drift = (a[e[i]] / (e[i] - b[i] + 1)) * count;
                a[k] = (float) (a[k] - drift);
                count++;
            }
        }
    }

    //
    public static void addCalibratedVelocity(float[] sampleVx, float[] sampleVy, float[] sampleVz) {
        for (int i = 0; i < sampleVx.length; i++) {
            Velocity sampleV = new Velocity(sampleVx[i], sampleVy[i], sampleVz[i]);
            // 用求得的新的速度来更新位置
            addNewVelocitySample(sampleV);
        }
    }

    // 求当前状态的向心加速度，用来对加速度进行校正
    private static Acceleration getCentripetalAcceleration() {
        Acceleration ca = new Acceleration();        // 向心加速度centripetal acceleration
        // 为了方便代码阅读
        double vx = lastV.x;
        double vy = lastV.y;
        double vz = lastV.z;
        double ax = (Math.abs(lastAV.x) > 2 ? lastAV.x : 0);
        double ay = (Math.abs(lastAV.y) > 2 ? lastAV.y : 0);
        double az = (Math.abs(lastAV.z) > 2 ? lastAV.z : 0);//这部分没明白
        if (vx * vx + vy * vy + vz * vz < 1) return ca;
        if (ax * ax + ay * ay + az * az < 1) return ca;

        // 向心加速度的大小用a = w · v求得，而方向为w和v向量积的方向。
        // w和v的向量积：
        ca.x = ay * vz - az * vy;
        ca.y = az * vx - ax * vz;
        ca.z = ax * vy - ay * vx;
        // 向量积的模：
        double _1_mold = 1 / Math.sqrt(Math.pow(ca.x, 2) + Math.pow(ca.y, 2) + Math.pow(ca.z, 2));
        // 向心加速度的模：
        double ca_mold = Math.sqrt((vx * vx + vy * vy) * az * az
                + (vx * vx + vz * vz) * ay * ay
                + (vy * vy + vz * vz) * ax * ax);
        // 向量积除以向量积的模得到向量积方向的单位向量，再乘以向心加速度的模得到向心加速度。
        // 手机得到的加速度数值为与其自身加速度相反的向量
        ca.x = ca.x * ca_mold * _1_mold * -1 * 0.01;
        ca.y = ca.y * ca_mold * _1_mold * -1 * 0.01;
        ca.z = ca.z * ca_mold * _1_mold * -1 * 0.01;

        return ca;
    }

    // 调用这个方法设置静止加速度
    public static void setStaticAcceleration(Acceleration acc) {
        staticAcc = new Acceleration(acc);
        hasStaticStateSet = true;
    }

    public static Position addNewVelocitySample(Velocity sampleV) {
        // 同样用计算梯形面积和的方法完成速度的积分，更新当前位置
        //梯形积分法
//        tempPos.x = (sampleV.x + lastV.x) * SP * 0.5;
//        tempPos.y = (sampleV.y + lastV.y) * SP * 0.5;
//        tempPos.z = (sampleV.z + lastV.z) * SP * 0.5;、
        //矩形积分法
        tempPos.x = sampleV.x * SP ;
        tempPos.y = sampleV.y * SP ;
        tempPos.z = sampleV.z * SP ;
        tempPos.t = sampleV.t;
        currentPos.x += tempPos.x;
        currentPos.y += tempPos.y;
        currentPos.z += tempPos.z;
        currentPos.t = sampleV.t;
//        if(flag){
//            currentPos.x += X.get(7,0);
//            currentPos.y += X.get(6,0);
//            currentPos.z += X.get(8,0);
//        }
        lastV = new Velocity(sampleV);
        // 每次更新完之后都输出
//        outputCurrentState();
        return tempPos;
    }

    public static void addX(Matrix X){
        lastV.x += X.get(3,0);
        lastV.y += X.get(4,0);
        lastV.z += X.get(5,0);
        currentPos.x += X.get(7,0);
        currentPos.y += X.get(6,0);
        currentPos.z += X.get(8,0);
    }

    public static void outputCurrentState() {
        // 分别在四个文件中输出每时刻的位置，加速度，速度和向心加速度
        IOAdapter.writePosition(currentPos.x + "," + currentPos.y + "," + currentPos.z + "\n");
//        IOAdapter.writeAcceleration(lastAcc.x + "," + lastAcc.y + "," + lastAcc.z + "\n");
        IOAdapter.writeVelocity(lastV.x + "," + lastV.y + "," + lastV.z + "\n");
//        if (CENT_ACC_ON)
//            IOAdapter.writeCentripetalAcceleration(centripetalAcc.x + "," + centripetalAcc.y + "," + centripetalAcc.z + "\n");//这儿，把角速度和lastAcc换数组存一下
//        IOAdapter.writeAngularVelocity(lastAV.x + "," + lastAV.y + "," + lastAV.z + "\n");
    }
}
