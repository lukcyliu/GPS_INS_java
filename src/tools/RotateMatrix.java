package tools;
//坐标系变换
//pitch 俯仰角   θ
//roll横滚角   φ
//yaw 航向角   ψ 

import Jama.Matrix;

public class RotateMatrix {
    public static double[] deRotaingBToN(double Xb, double Yb, double Zb,
                                         double yaw, double pitch, double roll) {
        // 将度数变为数字
        yaw = (yaw * Math.PI / 180);
        pitch = (pitch * Math.PI / 180);
        roll = (roll * Math.PI / 180);
        //用来装变换后的XYZ的值的矩阵
        double[] BToN = new double[3];

        Matrix rollMatrix = new Matrix(3, 3);
        rollMatrix.set(0, 0, 1);
        rollMatrix.set(0, 1, 0);
        rollMatrix.set(0, 2, 0);
        rollMatrix.set(1, 0, 0);
        rollMatrix.set(1, 1, Math.cos(roll));
        rollMatrix.set(1, 2, Math.sin(roll));
        rollMatrix.set(2, 0, 0);
        rollMatrix.set(2, 1, -Math.sin(roll));
        rollMatrix.set(2, 2, Math.cos(roll));

        Matrix pithMatrix = new Matrix(3, 3);
        pithMatrix.set(0, 0, Math.cos(pitch));
        pithMatrix.set(0, 1, 0);
        pithMatrix.set(0, 2, -Math.sin(pitch));
        pithMatrix.set(1, 0, 0);
        pithMatrix.set(1, 1, 1);
        pithMatrix.set(1, 2, 0);
        pithMatrix.set(2, 0, Math.sin(pitch));
        pithMatrix.set(2, 1, 0);
        pithMatrix.set(2, 2, Math.cos(pitch));

        Matrix yawMatrix = new Matrix(3, 3);
        yawMatrix.set(0, 0, Math.cos(yaw));
        yawMatrix.set(0, 1, Math.sin(yaw));
        yawMatrix.set(0, 2, 0);
        yawMatrix.set(1, 0, -Math.sin(yaw));
        yawMatrix.set(1, 1, Math.cos(yaw));
        yawMatrix.set(1, 2, 0);
        yawMatrix.set(2, 0, 0);
        yawMatrix.set(2, 1, 0);
        yawMatrix.set(2, 2, 1);

        Matrix CMatrix = new Matrix(3, 3);
        CMatrix = rollMatrix.times(pithMatrix).times(yawMatrix);

        //原始载体坐标系中的XYZ值
        Matrix originMatrix = new Matrix(3, 1);
        originMatrix.set(0, 0, Xb);
        originMatrix.set(1, 0, Yb);
        originMatrix.set(2, 0, Zb);

        //转换以后的矩阵
        Matrix NewMatrix = new Matrix(3, 1);
        NewMatrix = CMatrix.times(originMatrix);

        for (int i = 0; i < BToN.length; i++) {
            BToN[i] = (float) NewMatrix.get(i, 0);
        }
        return BToN;
    }

}
