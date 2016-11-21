/*
 * 程序运行相关参数的配置。
 */

import tools.Acceleration;
import tools.Velocity;

public class Config {

    //test
    // 设置文件路径和文件名（无后缀）
    public static final String DIR = "C:\\Users\\Administrator\\Desktop\\TestSensorData\\sensordata1121\\";
    public static final String FILE_NAME = "IMU1479464814533_2";
    public static final String GPS_FILE_NAME = "GPS1479464814533_1";

    // IMU和手机采集的数据格式不同
    public static final byte DATA_SOURCE = 2;
    public static final byte IMU = 1;
    public static final byte PHONE = 2;

    //数据测量的有效区间
    public static final int DATA_START = 10;
    public static final int DATA_END = 11000;

    // 两种AHRS算法
    public static final byte AHRS_MODE = 2;
    public static final byte AHRS_MODE_MADGWICK = 1;
    public static final byte AHRS_MODE_MAHONY = 2;

    // SMOOTHEN_ON：平滑开关，可以设置平滑窗口
    // CENTRIPETAL_ACCELERATION_ON：向心加速度校正，不准确有待调试
    // STATIC_ACCELERATION_ON：静止加速度校正，目前采用取前10个加速度数据取平均作为静止加速度
    public static final boolean SMOOTHEN_ON = true;
    public static final boolean CENTRIPETAL_ACCELERATION_ON = true;
    public static final boolean STATIC_ACCELERATION_ON = false;

    // 平滑窗口长度
    public static final int SMOOTHEN_WINDOW = 10;

    // 陀螺仪和加速度数据过零阈值
    public static final double GYRO_THRESHOLD = 0.05;
    public static final Acceleration ACC_THRESHOLD = new Acceleration(0.05, 0.05, 1);
    public static final Velocity Val_THRESHOLD = new Velocity(0.1, 0.1, 0.05);

    // 单位转换。IMU采集的数据和我们计算时用的数据单位不同
    public static final float G2MS2 = 9.8f;
    public static final float C2RADS = (float) Math.PI / 180;

    // 手机和IMU的采样间隔不同
    public static final float SAMPLE_PERIOD_PHONE = 0.02f;
    public static final float SAMPLE_PERIOD_IMU = (float) (1.0 / 256);

    // 拐弯检测记录数据个数
    public static final float TURNING_SAMPLE_NUMBER_THRESHOLD = 500;
}
