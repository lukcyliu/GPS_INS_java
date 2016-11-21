/*步骤：
 * computer A(t-1) and Q(t-1)
 * 1、X_neg(t)=A(t-1)X_posi(t-1)
 * 2、P_neg(t)=A(t-1)P_posi(t-1)A(t-1)'+Q(t-1)
 * computer H(t) and R(t) with measurement model
 * 3、K(t)=P_neg(t)H(t)'(H(t)P_neg(t)H(t)'+R(t))-1
 * formulate Z(t)
 * 4、X_posi(t)=X_neg(t)+K(t)(Z(t)-H(t)X_neg(t))
 * 5、P_posi(t)=(I-K(t)H(t))P_neg(t)(I-K(t)H(t))'+K(t)R(t)K(t)'
 * */

package tools;

import java.util.Arrays;

import Jama.Matrix;

public class NewKalmanFilter {

//	private static final double Wie = 7.292115147e-5;
//	private static final double Rn = 6378137;
//	private static final double Rm = 6378137;
	private static final double Q_error_acc_x = 0.01566;
	private static final double Q_error_acc_y = 0.0088888;
	private static final double Q_error_acc_z = 0.039031;
	private static final double Q_error_gyo_x = 0.000063813;
	private static final double Q_error_gyo_y = 0.00029791;
	private static final double Q_error_gyo_z = 0.000012515;
//	private static final double Q_error_gyo = 1;
//	private static final double Q_error_acc = 1;

	private double L;
	private double Lamda;
	private double h;
	private double Ve;
	private double Vn;
	private double Vu;
	private double fe;
	private double fn;
	private double fu;

//	private double Faie = 0;
//	private double Fain = 0;
//	private double Faiu = 0;
//	private double delVe = 0;
//	private double delVn = 0;
//	private double delVu = 0;
//	private double delL = 0;
//	private double delLamda = 0;
//	private double delh = 0;
//	private double egyo_x = 0;
//	private double egyo_y = 0;
//	private double egyo_z = 0;
//	private double eacc_x = 0;
//	private double eacc_y = 0;
//	private double eacc_z = 0;


    private Matrix Cbn = new Matrix(3,3);

	private Matrix X = new Matrix(15, 1);
    private Matrix T = new Matrix(15, 15);
	//这里先注释掉系统W(t)和矩阵B(t) B(t)W(t)目的为把加速计和陀螺仪噪声从b系转到n系再与我们的状态变量相加
//	private Matrix W = new Matrix(6, 1);
//	private Matrix B = new Matrix(15,6);
	private Matrix P = new Matrix(15, 15);
	private Matrix Q = new Matrix(15, 15);

    private Matrix Z = new Matrix(6, 1);
	private Matrix H = new Matrix(6, 15);
	private Matrix I = new Matrix(15, 15);
	private Matrix R = new Matrix(6, 6);
	private Matrix K = new Matrix(15, 6);
	private Matrix Y = new Matrix(6,1);
	private Matrix S = new Matrix(6,6);
	//构造函数
	public NewKalmanFilter(){

	}
    public Matrix getCbn() {
        return Cbn;
    }

    private void setCbn(Matrix cbn) {
        this.Cbn = cbn;
    }




    public Matrix getT() {
        return T;
    }

    private void setT(double Wie,double Rn,double Rm) {
        double sinL = Math.sin(L);
        double cosL = Math.cos(L);
        double tanL = Math.tan(L);
        double Rnh = Rn + h;
        double Rmh = Rm + h;
        double Ve2 = Ve*Ve;
        double secL = 1/Math.sin(L);
        double sec2L = Math.pow(1/Math.sin(L), 2);
//        double MatrixT[][] = {
//                {0                        ,    Wie*sinL+Ve*tanL/Rnh , -(Wie*cosL+Ve/Rnh) ,       0                   , -1/Rmh                 , 0                     , 0                                       , 0 , 0,Cbn.get(0,0),Cbn.get(0,1),Cbn.get(0,2),0           ,0           ,0           },
//                {-(Wie*sinL+Ve*tanL/Rnh)  ,            0            ,       -Vn/Rmh      ,      1/Rnh                , 0                      , 0                     , -Wie*sinL                               , 0 , 0,Cbn.get(1,0),Cbn.get(1,1),Cbn.get(1,2),0           ,0           ,0           },
//                {Wie*cosL+Ve/Rnh          ,           Vn/Rmh        ,          0         ,     tanL/Rnh              , 0                      , 0                     , Wie*cosL+Ve*sec2L/Rnh                   , 0 , 0,Cbn.get(2,0),Cbn.get(2,1),Cbn.get(2,2),0           ,0           ,0           },
//                {0                        ,           -fu           ,         fn         ,   (Vn*tanL-Vu)/Rnh        , 2*Wie*sinL+Ve*sinL/Rnh , -(2*Wie*cosL+Ve/Rnh)  , 2*Wie*(cosL*Vn+sinL*Vu)+Ve*Vn*sec2L/Rnh , 0 , 0,0           ,0           ,0           ,Cbn.get(0,0),Cbn.get(0,1),Cbn.get(0,2)},
//                {fu                       ,            0            ,         -fe        , -2*(Wie*sinL+Ve*tanL/Rnh) , -Vu/Rmh                , -Vn/Rmh               , -(2*Wie*cosL*Ve+Ve2*sec2L/Rnh)          , 0 , 0,0           ,0           ,0           ,Cbn.get(1,0),Cbn.get(1,1),Cbn.get(1,2)},
//                {-fn                      ,            fe           ,          0         , 2*(Wie*sinL+Ve*tanL/Rnh)  , 2*Vn/Rmh               , 0                     , -2*Wie*sinL*Ve                          , 0 , 0,0           ,0           ,0           ,Cbn.get(2,0),Cbn.get(2,1),Cbn.get(2,2)},
//                {0                        ,            0            ,          0         ,       0                   , 1/Rmh                  , 0                     , 0                                       , 0 , 0,0           ,0           ,0           ,0           ,0           ,0           },
//                {0                        ,            0            ,          0         ,    secL/Rnh               , 0                      , 0                     , Ve*secL*tanL/Rnh                        , 0 , 0,0           ,0           ,0           ,0           ,0           ,0           },
//                {0                        ,            0            ,          0         ,       0                   , 0                      , 1                     , 0                                       , 0 , 0,0           ,0           ,0           ,0           ,0           ,0           },
//                {0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                       , 0 , 0,-1/38       ,0           ,0           ,0           ,0           ,0           },
//                {0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                       , 0 , 0,0           ,-1/38       ,0           ,0           ,0           ,0           },
//                {0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                       , 0 , 0,0           ,0           ,-1/38       ,0           ,0           ,0           },
//                {0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                       , 0 , 0,0           ,0           ,0           ,-1/38       ,0           ,0           },
//                {0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                       , 0 , 0,0           ,0           ,0           ,0           ,-1/38       ,0           },
//                {0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                       , 0 , 0,0           ,0           ,0           ,0           ,0           ,-1/38       }
//        };
		double MatrixT[][] = {
				{0                        ,    Wie*sinL+Ve*tanL/Rnh , -(Wie*cosL+Ve/Rnh) ,       0                   , -1/Rmh                 , 0                     , 0                                                   , 0 , 0,Cbn.get(0,0)  ,Cbn.get(0,1)  ,Cbn.get(0,2)  ,0             ,0             ,0             },
				{-(Wie*sinL+Ve*tanL/Rnh)  ,            0            ,       -Vn/Rmh      ,      1/Rnh                , 0                      , 0                     , -Wie*sinL/(30.8*3600)                               , 0 , 0,Cbn.get(1,0)  ,Cbn.get(1,1)  ,Cbn.get(1,2)  ,0             ,0             ,0             },
				{Wie*cosL+Ve/Rnh          ,           Vn/Rmh        ,          0         ,     tanL/Rnh              , 0                      , 0                     , Wie*cosL+Ve*sec2L/Rnh/(30.8*3600)                   , 0 , 0,Cbn.get(2,0)  ,Cbn.get(2,1)  ,Cbn.get(2,2)  ,0             ,0             ,0             },
				{0                        ,           -fu           ,         fn         ,   (Vn*tanL-Vu)/Rnh        , 2*Wie*sinL+Ve*sinL/Rnh , -(2*Wie*cosL+Ve/Rnh)  , 2*Wie*(cosL*Vn+sinL*Vu)+Ve*Vn*sec2L/Rnh/(30.8*3600) , 0 , 0,0             ,0             ,0             ,Cbn.get(0,0)  ,Cbn.get(0,1)  ,Cbn.get(0,2)  },
				{fu                       ,            0            ,         -fe        , -2*(Wie*sinL+Ve*tanL/Rnh) , -Vu/Rmh                , -Vn/Rmh               , -(2*Wie*cosL*Ve+Ve2*sec2L/Rnh)/(30.8*3600)          , 0 , 0,0             ,0             ,0             ,Cbn.get(1,0)  ,Cbn.get(1,1)  ,Cbn.get(1,2)  },
				{-fn                      ,            fe           ,          0         , 2*(Wie*sinL+Ve*tanL/Rnh)  , 2*Vn/Rmh               , 0                     , -2*Wie*sinL*Ve/(30.8*3600)                          , 0 , 0,0             ,0             ,0             ,Cbn.get(2,0)  ,Cbn.get(2,1)  ,Cbn.get(2,2)  },
				{0                        ,            0            ,          0         ,       0                   , (double)1              , 0                     , 0                                                   , 0 , 0,0             ,0             ,0             ,0             ,0             ,0             },
				{0                        ,            0            ,          0         ,       (double)1           , 0                      , 0                     , 0                                                   , 0 , 0,0             ,0             ,0             ,0             ,0             ,0             },
				{0                        ,            0            ,          0         ,       0                   , 0                      , (double)1             , 0                                                   , 0 , 0,0             ,0             ,0             ,0             ,0             ,0             },
				{0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                                   , 0 , 0,(double)-1/100,0             ,0             ,0             ,0             ,0             },
				{0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                                   , 0 , 0,0             ,(double)-1/100,0             ,0             ,0             ,0             },
				{0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                                   , 0 , 0,0             ,0             ,(double)-1/100,0             ,0             ,0             },
				{0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                                   , 0 , 0,0             ,0             ,0             ,(double)-1/100,0             ,0             },
				{0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                                   , 0 , 0,0             ,0             ,0             ,0             ,(double)-1/100,0             },
				{0                        ,            0            ,          0         ,       0                   , 0                      , 0                     , 0                                                   , 0 , 0,0             ,0             ,0             ,0             ,0             ,(double)-1/100}
		};//有所更改，因为X的经纬度高度是作为笛卡尔距离来计算的，所以应修改原转移矩阵中关于X中这三个元素的计算的公式，改为针对距离的计算公式
        T = new Matrix(MatrixT);
//		T.print(2,10);
    }//关于T矩阵的设置，如果Z矩阵的输入是位移的话，这儿要改一下


    //参数是自己设置的各种误差数据
    public void setX(double Faie,double Fain,double Faiu,
                     double delVe,double delVn,double delVu,
                     double delL,double delLamda,double delh,
                     double egyo_x,double egyo_y,double egyo_z,
                     double eacc_x,double eacc_y,double eacc_z){
        X.set(0,0,Faie);
        X.set(1,0,Fain);
        X.set(2,0,Faiu);
        X.set(3,0,delVe);
        X.set(4,0,delVn);
        X.set(5,0,delVu);
        X.set(6,0,delL);
        X.set(7,0,delLamda);
        X.set(8,0,delh);
        X.set(9,0,egyo_x);
        X.set(10,0,egyo_y);
        X.set(11,0,egyo_z);
        X.set(12,0,eacc_x);
        X.set(13,0,eacc_y);
        X.set(14,0,eacc_z);
    }
    public void setX(Matrix X_old){
		X = X_old;
	}
    public Matrix getX() {
        return X;
    }

	//获取到的GPS和INS的数据
	public void setZ(double Vegps,double Vngps,double Vugps,double Lgps,double Lamdagps,double hgps,
					 double Veins,double Vnins,double Vuins,double Lins,double Lamdains,double hins) {
		double MatrixZ[][] = {
				{Vegps - Veins},
				{Vngps - Vnins},
				{Vugps - Vuins},
				{Lgps - Lins},
				{Lamdagps - Lamdains},
				{hgps - hins}
		};
		this.Z = new Matrix(MatrixZ);
//		System.out.println("z矩阵");
//		Z.print(2,15);
	}

    public Matrix getZ() {
        return Z;
    }

	public void input_GPS_INS_Data(double L,double Lamda,double h,double Ve,double Vn,double Vu,double fe,double fn,double fu){
		this.L = L;
		this.Lamda = Lamda;
		this.h = h;
		this.Ve = Ve;
		this.Vn = Vn;
		this.Vu = Vu;
		this.fe = fe;
		this.fn = fn;
		this.fu = fu;
	}

	public void initial(Matrix cbn,double Wie,double Rn,double Rm) {
		setT(Wie,Rn,Rm);
		setCbn(cbn);
		double MatrixQ[] = {0,0,0,0,0,0,0,0,0,Q_error_gyo_x,Q_error_gyo_y,Q_error_gyo_z,Q_error_acc_x,Q_error_acc_y,Q_error_acc_z};
		Q = DiagMatrix(MatrixQ);
		double MatrixR[] = {0.005,0.005,0.005,1,1,1};
		R = DiagMatrix(MatrixR);

		//测量噪声方差矩阵R

		//测量转移矩阵H
		double MatrixH[][] = {
				{0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0}};
		H = new Matrix(MatrixH);

		I = EyeMatrix(15);
//
//		R = EyeMatrix(6);

        //缺少K的初始化
	}

	public void itegration() {
		X = T.times(X);//k-1|k-1 -> k|k-1
		P = T.times(P).times(T.transpose()).plus(Q);//k-1|k-1 -> k|k-1
		Y = Z.minus(H.times(X));
		S = H.times(P).times(H.transpose()).plus(R);
		K = P.times(H.transpose()).times(S.inverse());
		X = X.plus(K.times(Y));//k|k-1 -> k|k
//		System.out.println("X矩阵");
//		X.print(2,15);
		P = I.minus(K.times(H)).times(P);//k|k-1 -> k|k
//		System.out.println("k增益");
//		K.print(2,15);
	}

	public Matrix getP() {
		return P;
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
		double[] eye = new double[n];
		Arrays.fill(eye, 1);
		Matrix bMatrix = DiagMatrix(eye);
		return bMatrix;
	}
}
