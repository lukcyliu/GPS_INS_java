package smooth;

import java.util.ArrayList;


/**
 * Created by lukcy on 2017/7/14.
 */
public class SmoothTest {
    int firstSmooth = 0;
     int width = 20;
    double[][] acc = new double[3][width];
    double[][] gyo = new double[3][width];
    double[][] mag = new double[3][width];
    double[][] angle = new double[3][width];
    public  ArrayList<double[][]>  slideWindows(double ax, double ay,double az,double gx, double gy,double gz,double mx, double my,double mz){

        ArrayList<double[][]> queue = new ArrayList<double[][]>();
        if (firstSmooth == 0) {
            for (int i=0; i < width; i++){
                acc[0][i] = ax;
                acc[1][i] = ay;
                acc[2][i] = az;
                gyo[0][i] = gx;
                gyo[1][i] = gy;
                gyo[2][i] = gz;
                mag[0][i] = mx;
                mag[1][i] = my;
                mag[2][i] = mz;
            }
            firstSmooth = 1;
        }
        else{
            for (int i=1; i < width; i++){
                acc[0][i-1] = acc[0][i];
                acc[1][i-1] = acc[1][i];
                acc[2][i-1] = acc[2][i];
                gyo[0][i-1] = gyo[0][i];
                gyo[1][i-1] = gyo[1][i];
                gyo[2][i-1] = gyo[2][i];
                mag[0][i-1] = mag[0][i];
                mag[1][i-1] = mag[1][i];
                mag[2][i-1] = mag[2][i];
            }
            acc[0][width-1] = ax;
            acc[1][width-1] = ay;
            acc[2][width-1] = az;
            gyo[0][width-1] = gx;
            gyo[1][width-1] = gy;
            gyo[2][width-1] = gz;
            mag[0][width-1] = mx;
            mag[1][width-1] = my;
            mag[2][width-1] = mz;
        }
        queue.add(acc);
        queue.add(gyo);
        queue.add(mag);
        return queue;
    }

    public void display(){

    }
    public static void main(String[] args) {
        int count=0;
        SmoothTest smoothTest = new SmoothTest();
        //io read csv.
        CsvHelper csvHelper = new CsvHelper();
        csvHelper.creatOutputFile();
        ArrayList<double[]> input = csvHelper.read(9);
        System.out.println(input.size());
        double[] data = new double[9];
        ArrayList<double[][]> smoothRes;
        double ax,ay,az,gx,gy,gz,mx,my,mz;
        double smoothAx=0,smoothAy=0,smoothAz=0,smoothGx=0,smoothGy=0,smoothGz=0,smoothMx=0,smoothMy=0,smoothMz=0;

        for (int i = 0; i < input.size(); i++){
            data = input.get(i);
            ax = data[0]; ay = data[1]; az = data[2];
            gx = data[3]; gy = data[4]; gz = data[5];
            mx = data[6]; my = data[7]; mz = data[8];
            smoothRes = smoothTest.slideWindows(ax,ay,az,gx,gy,gz,mx,my,mz);
            for(int j = 0; j < smoothTest.width; ++j) {
                smoothAx += smoothRes.get(0)[0][j];
                smoothAy += smoothRes.get(0)[1][j];
                smoothAz += smoothRes.get(0)[2][j];
                smoothGx += smoothRes.get(1)[0][j];
                smoothGy += smoothRes.get(1)[1][j];
                smoothGz += smoothRes.get(1)[2][j];
                smoothMx += smoothRes.get(2)[0][j];
                smoothMy += smoothRes.get(2)[1][j];
                smoothMz += smoothRes.get(2)[2][j];
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

            csvHelper.write(smoothAx+","+smoothAy+","+smoothAz+","+
                                    smoothGx+","+smoothGy+","+smoothGz+","+
                                    smoothMx+","+smoothMy+","+smoothMz+"\n");
//            System.out.println((count++)+"smoothAcc = "+(float)smoothAx+" "+(float)smoothAy+" "+(float)smoothAz+" ; "+
//                                "smoothGyo = "+(float)smoothGx+" "+(float)smoothGy+" "+(float)smoothGz+" ; "+
//                                "smoothMag = "+(float)smoothMx+" "+(float)smoothMy+" "+(float)smoothMz);
            smoothAx=0;smoothAy=0;smoothAz=0;smoothGx=0;smoothGy=0;smoothGz=0;smoothMx=0;smoothMy=0;smoothMz=0;
        }

    }
}
