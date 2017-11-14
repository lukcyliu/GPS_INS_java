package smooth;

import java.io.*;
import java.util.ArrayList;

/**
 * Created by lukcy on 2017/7/14.
 */
public class CsvHelper {
    BufferedReader br;
    BufferedWriter smoothWriter;
    OutputStreamWriter outputStreamWriter;

//    static final String DIR = "C:\\Users\\lukcy\\Desktop\\惯导测试数据\\0928跑车\\";
//    static final String inputFilename= "9.28跑车输入.csv";
//    static final String outputFilename="9.28跑车输出.csv";
    static final String DIR = "C:\\Users\\lukcy\\Desktop\\惯导测试数据\\0828跑车\\";
    static final String inputFilename= "8.28滑动滤波输入.csv";
    static final String outputFilename="8.28滑动滤波输出.csv";
    public ArrayList<double[]> read(int n){
        ArrayList<double[]> data = new ArrayList<double[]>();
        try{
            br = new BufferedReader(new FileReader(new File(DIR+inputFilename)));
            String line ;
            while((line = br.readLine())!= null){
                String[] lineSplit = line.split(",") ;
                double[] lineSplittoDouble = new double[n];
                for(int i = 0; i < n; i++)
                    lineSplittoDouble[i] = Double.parseDouble(lineSplit[i]);
                data.add(lineSplittoDouble);
            }

            br.close();
        }catch (FileNotFoundException e){
            e.printStackTrace();
        }catch (IOException e){
            e.printStackTrace();
        }
        return data;
    }

    public void write(String output){
        if(smoothWriter == null)
            return;
        try{
            smoothWriter.write(output);
            smoothWriter.flush();

        }catch (IOException e){
            e.printStackTrace();
        }
    }

    public void creatOutputFile(){
        try {
            smoothWriter = new BufferedWriter(new FileWriter(new File(DIR + outputFilename)));
        }catch (IOException e){
            e.printStackTrace();
        }
    }
}
