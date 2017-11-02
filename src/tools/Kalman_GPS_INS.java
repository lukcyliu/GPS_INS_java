package tools;

import Jama.Matrix;

import javax.xml.ws.soap.MTOM;
import java.util.Arrays;

/**
 * Created by lukcy on 2017/9/20.
 */
public class Kalman_GPS_INS {

    //----------------------------------------融合相关参数--------------------------------------------------------------//
    //Re长半轴 r短半轴 f椭球扁率 e椭球偏心率 wie地球自转角速率
    double earthRe=6378137,earthr=6356752.3142,earthf=1/298.257,earthe=0.0818,earthwie=7.292e-5;
    double G0 = 9.8015;
    double Q_wg = (1/(57*3600))*(1/(57*3600));//陀螺的随机漂移为0.5度每小时
    double Q_wa = Math.pow(((0.5e-4)*G0),2);//加速度计的随机偏差为0.5e-4*g
    double[] Q_diag = {Q_wg, Q_wg, Q_wg, Q_wa, Q_wa, Q_wa };

    double[][] tg = {{300},{300},{300}};//陀螺仪误差漂移相关时间
    double[][] ta = {{1000},{1000},{1000}};//加表误差漂移相关时间
    Matrix Tg = new Matrix(tg);
    Matrix Ta = new Matrix(ta);
    double Rlamt = 1e-5 * 3.1415926 /(60 * 180);//经纬度误差均方根，弧度制
    double Rl = 1e-5 * 3.1415926 /(60 * 180);
    double Rh = 1e-11;//高度误差均方根，单位米
    double Rvx = 1e-7;//速度误差均方根，单位 米/秒
    double Rvy = 1e-7;
    double Rvz = 5e-9;
    double[] Rk = {Rlamt, Rl, Rh, Rvx, Rvy, Rvz};

    double[] PP0k = {(0.1/(57))*(0.1/(57)), (0.1/(57))*(0.1/(57)), (0.1/(57))*(0.1/(57)),
            0.01*0.01, 0.01*0.01, 0.01*0.01,
            0, 0, 0,
            (0.1/(57*3600))*(0.1/(57*3600)), (0.1/(57*3600))*(0.1/(57*3600)), (0.1/(57*3600))*(0.1/(57*3600)),
            ((1e-4)*G0)*((1e-4)*G0), ((1e-4)*G0)*((1e-4)*G0), ((1e-4)*G0)*((1e-4)*G0)};
    Matrix Q = DiagMatrix(Q_diag);
    Matrix R = DiagMatrix(Rk);
    Matrix PP0 = DiagMatrix(PP0k);//初始误差协方差阵,加速度计的初始偏值均取1e-4*g 陀螺的常值漂移取0.1度每小时
    Matrix X = new Matrix(15,1);
    Matrix PP = PP0;

    public  Matrix  kalman_GPS_INS_pv(double[][] Dpv,double Ve,double Vn,double Vu,double L,double h,Matrix mahonyR,Matrix Fn,double tao,double Rm,double Rn) {
        double wie = 7.292e-5;
        Matrix XX;
        double fe = Fn.get(0, 0);
        double fn = Fn.get(1, 0);
        double fu = Fn.get(2, 0);
        //连续系统状态转换阵 F 的时间更新
        Matrix F ;
        double secL2 = (1 / (Math.sin(L) * Math.sin(L)));
        double Rnh2 = (Rn + h) * (Rn + h);
        double Rmh2 = ((Rm + h) * (Rm + h));
        double tg = Tg.get(0, 0);
        double ta = Ta.get(0, 0);

        double[][] matrixF = {
                {0, -wie * Math.sin(L) - Ve * Math.tan(L) / (Rn + h), Vn / (Rm + h), 0, 1 / (Rn + h), 0, -wie * Math.sin(L), 0, -Ve / Rnh2, mahonyR.get(0,0), mahonyR.get(1,0), mahonyR.get(2,0), 0, 0, 0},
                {wie * Math.sin(L) + Ve * Math.tan(L) / (Rn + h), 0, wie * Math.cos(L) + Ve / (Rn + h), -1 / (Rm + h), 0, 0, 0, 0, Vn / Rmh2, mahonyR.get(0,1), mahonyR.get(1,1), mahonyR.get(2,1), 0, 0, 0},
                {-Vn / (Rm + h), -wie * Math.cos(L) - Ve / (Rn + h), 0, 0, -Math.tan(L) / (Rn + h), 0, -wie * Math.cos(L) - Ve * secL2 / (Rn + h), 0, Ve * Math.tan(L) / Rnh2, mahonyR.get(0,2), mahonyR.get(1,2), mahonyR.get(2,2), 0, 0, 0},
                {0, -fu, fe, Vu / (Rm + h), -2 * wie * Math.sin(L) - Ve * Math.tan(L) / (Rn + h) - Ve * Math.tan(L) / (Rn + h), Vn / (Rm + h), -2 * wie * Math.cos(L) * Ve - Ve * Ve * secL2 / (Rn + h), 0, Ve * Ve * Math.tan(L) / Rnh2 - Vn * Vu / Rmh2, 0, 0, 0, mahonyR.get(0,0), mahonyR.get(1,0), mahonyR.get(2,0)},
                {fu, 0, -fn, 2 * wie * Math.sin(L) + Ve * Math.tan(L) / (Rn + h), (Vn * Math.tan(L) + Vu) / (Rn + h), 2 * wie * Math.cos(L) + Ve / (Rn + h), (2 * wie * Math.cos(L) + Ve * (secL2) / (Rn + h)) * Vn - 2 * wie * Vu * Math.sin(L), 0, (Ve * Vn * Math.tan(L) - Ve * Vu) / Rnh2, 0, 0, 0,mahonyR.get(0,1), mahonyR.get(1,1), mahonyR.get(2,1)},
                {-fe, fn, 0, -Vn / (Rm + h), -2 * wie * Math.cos(L) - 2 * Ve / (Rn + h), 0, 2 * Ve * wie * Math.sin(L), 0, Ve * Ve / Rnh2 + Vn * Vn / Rmh2, 0, 0, 0, mahonyR.get(0,2), mahonyR.get(1,2), mahonyR.get(2,2)},
                {0, 0, 0, 1 / (Rm + h), 0, 0, 0, 0, -Vn / Rmh2, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 1 / ((Rn + h) * Math.cos(L)), 0, Ve * Math.tan(L) / ((Rn + h) * Math.cos(L)), 0, -Ve / (Math.cos(L) * Rnh2), 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / tg, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / tg, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / tg, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / ta, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / ta, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / ta}
        };

        F = new Matrix(matrixF);
//        F.print(15,15);

        Matrix G ;

        double[][] matrixG = {
                {mahonyR.get(0,0), mahonyR.get(1,0), mahonyR.get(2,0),0,0,0},
                {mahonyR.get(0,1), mahonyR.get(1,1), mahonyR.get(2,1),0,0,0},
                {mahonyR.get(0,2), mahonyR.get(1,2), mahonyR.get(2,2),0,0,0},
                {0, 0, 0, mahonyR.get(0,0), mahonyR.get(1,0), mahonyR.get(2,0)},
                {0, 0, 0, mahonyR.get(0,1), mahonyR.get(1,1), mahonyR.get(2,1)},
                {0, 0, 0, mahonyR.get(0,2), mahonyR.get(1,2), mahonyR.get(2,2)},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0}
        };
        G = new Matrix(matrixG);
//        G.print(15,6);

        Matrix H ;
        double[][] matrixH = {
                {0, 0 , 0 , 0 , 0 , 0 , Rm+h , 0,0,0,0,0,0,0,0},
                {0 , 0 , 0 , 0 , 0 , 0 , 0 , (Rn+h)*Math.cos(L) , 0 , 0 , 0 , 0 , 0 , 0 , 0},
                {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0},
                {0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0},
                {0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0},
                {0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0},
        };
        H = new Matrix(matrixH);

//        H.print(6,15);

        //连续系统离散化
        Matrix eye15 = EyeMatrix(15);
        Matrix A = eye15.plus(F.times(tao));
        Matrix B = G.times(tao);
        //卡尔曼滤波开始
        X = A.times(X);
        Matrix P = A.times(PP).times(A.transpose()).plus(B.times(Q).times(B.transpose()));
//        P.print(3,7);
        Matrix Y = H.times(P).times(H.transpose()).plus(R);
//        System.out.println("Y = ");
//        Y.print(3,7);
//        System.out.println("Y.Inv = ");
//        Y.inverse().print(3,7);
        Matrix K = P.times(H.transpose()).times(Y.inverse());
//        K.print(3,7);
        PP = (eye15.minus(K.times(H))).times(P);
//        PP.print(3,7);
        Matrix Z = new Matrix(Dpv);
        System.out.println("Z = ");
        Z.transpose().print(3,7);
        Matrix W = Z.minus(H.times(X));
        System.out.println("W = ");
        W.transpose().print(3,7);
        X = X.plus(K.times(W));
        XX = X;
        System.out.println("XX = ");
        XX.transpose().print(3,7);
        return XX;
    }

    //设置对角矩阵
    private static Matrix DiagMatrix(double eye[]){
        int n = eye.length;
        Matrix aMatrix = new Matrix(n,n);
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                if(i==j)
                    aMatrix.set(i, j, eye[i]);
            }
        }
        return aMatrix;
    }
    //设置单位阵
    private static Matrix EyeMatrix(int n){
        Matrix bMatrix;
        double[] eye = new double[n];
        Arrays.fill(eye, 1);
        bMatrix = DiagMatrix(eye);
        return bMatrix;
    }

}
