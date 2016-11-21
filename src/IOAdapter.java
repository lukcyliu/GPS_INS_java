/*
 * 用于读入文件和生成相关数据文件。
 */

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class IOAdapter {

    @SuppressWarnings("unused")
    private static final String splitEx = Config.DATA_SOURCE == Config.IMU ? "\t" : ",";

    private static final String DIR = Config.DIR;
    private static final String FILE_NAME = Config.FILE_NAME;
    private static final String GPS_FILE_NAME = Config.GPS_FILE_NAME;
    private static final String INPUT_FILE_NAME = FILE_NAME + ".csv";
    private static final String INPUT_GPS_FILE_NAME = GPS_FILE_NAME + ".csv";
    //	private static final String INPUT_FILE_NAME = FILE_NAME + " new.csv";
    private static final String POSITION_OUTPUT_FILE_NAME = FILE_NAME + " position.txt";
    private static final String ACCELERATION_OUTPUT_FILE_NAME = FILE_NAME + " acceleration.txt";
    private static final String VELOCITY_OUTPUT_FILE_NAME = FILE_NAME + " velocity.txt";
    private static final String CENTRIPETAL_ACCELERATION_OUTPUT_FILE_NAME = FILE_NAME + " centripetal acceleration.txt";
    private static final String ANGULAR_VELOCITY_OUTPUT_FILE_NAME = FILE_NAME + " angular velocity.txt";
    private static BufferedWriter positionWriter;
    private static BufferedWriter accelerationWriter;
    private static BufferedWriter velocityWriter;
    private static BufferedWriter centripetalAccelerationWriter;
    private static BufferedWriter angularVelocityWriter;
    private static BufferedReader br;
    private static int Timer = 0;
    //11.04 添加输入参数数量
    public static ArrayList<float[]> input(int n) {
        ArrayList<float[]> data = new ArrayList<>();        // 文件中格式为"ax,ay,az,gx,gy,gz" by 陈一曲，如果要进行p系误差修正的话，加入四个新的数据
        // 当前的纬度L，当前的高度，N向速度VN，E向速度VE

        try {
            br = new BufferedReader(new FileReader(new File(DIR + INPUT_FILE_NAME)));
            String line;
            String[] lineSplit = new String[7];
            while ((line = br.readLine()) != null) {
                lineSplit = line.split(",");
                float[] lineSplitToFloat = new float[n];
                for (int i = 0; i < n; i++) {
                    lineSplitToFloat[i] = Float.parseFloat(lineSplit[i + 1]);
                }
                data.add(lineSplitToFloat);
            }

            br.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
//		System.out.print(data);
        return data;
    }

    public static ArrayList<Integer> getTime(){
        ArrayList<Integer> data = new ArrayList<>();
        try {
            br = new BufferedReader(new FileReader(new File(DIR + INPUT_FILE_NAME)));
            String line;
            String[] lineSplit = new String[7];
            while((line = br.readLine()) != null){
                lineSplit = line.split(",");
                Integer i = new Integer(Integer.parseInt(lineSplit[0]));
                data.add(i);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return data;
    }

    public static ArrayList<float[]> getGPSdata(int n){
        ArrayList<float[]> data = new ArrayList<>();
        try {
            br = new BufferedReader(new FileReader(new File(DIR + INPUT_GPS_FILE_NAME)));
            String line;
            String[] lineSplit = new String[6];
            while((line = br.readLine()) != null){
                lineSplit = line.split(",");
                float[] lineSplitToFloat = new float[n];
                for(int i = 0;i < n;i++)//第一个是时间戳，后面是经纬度和高度，三向速度
                    lineSplitToFloat[i] = Float.parseFloat(lineSplit[i]);
                data.add(lineSplitToFloat);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return data;
    }
    // 以下没什么好看的

    public static void createOutputFile() {
        try {
            positionWriter = new BufferedWriter(new FileWriter(new File(DIR + POSITION_OUTPUT_FILE_NAME)));
            accelerationWriter = new BufferedWriter(new FileWriter(new File(DIR + ACCELERATION_OUTPUT_FILE_NAME)));
            velocityWriter = new BufferedWriter(new FileWriter(new File(DIR + VELOCITY_OUTPUT_FILE_NAME)));
            centripetalAccelerationWriter = new BufferedWriter(new FileWriter(new File(DIR + CENTRIPETAL_ACCELERATION_OUTPUT_FILE_NAME)));
            angularVelocityWriter = new BufferedWriter(new FileWriter(new File(DIR + ANGULAR_VELOCITY_OUTPUT_FILE_NAME)));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writePosition(String output) {
        if (null == positionWriter) return;
        try {
            positionWriter.write(output);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writeAcceleration(String output) {
        if (null == accelerationWriter) return;
        try {
            accelerationWriter.write(output);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writeVelocity(String output) {
        if (null == velocityWriter) return;
        try {
            velocityWriter.write(output);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writeCentripetalAcceleration(String output) {
        if (null == centripetalAccelerationWriter) return;
        try {
            centripetalAccelerationWriter.write(output);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writeAngularVelocity(String output) {
        if (null == angularVelocityWriter) return;
        try {
            angularVelocityWriter.write(output);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void output() {
        try {
            positionWriter.flush();
            positionWriter.close();
            accelerationWriter.flush();
            accelerationWriter.close();
            velocityWriter.flush();
            velocityWriter.close();
            centripetalAccelerationWriter.flush();
            centripetalAccelerationWriter.close();
            angularVelocityWriter.flush();
            angularVelocityWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
