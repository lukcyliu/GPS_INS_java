package tools;

/// MahonyAHRS class. Madgwick's implementation of Mayhony's AHRS algorithm.
/// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
public class MahonyAHRS {
    public float SamplePeriod;
    // the algorithm proportional gain.
    public float Kp;
    // the algorithm integral gain.
    public float Ki;
    // the Quaternion output.
    public float[] Quaternion = {0,0,0,0};

    // the integral error.
    private float[] eInt;
    // Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
    // <param name="kp">
    // Algorithm proportional gain.
    // </param>
    // <param name="ki">
    // Algorithm integral gain.
    // </param>
    public MahonyAHRS(float samplePeriod, float kp, float ki,float[] eint) {
        SamplePeriod = samplePeriod;
        Kp = kp;
        Ki = ki;
        eInt = eint;
    }

    // Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
    // <param name="gx">
    // Gyroscope x axis measurement in radians/s.
    // </param>
    // <param name="gy">
    // Gyroscope y axis measurement in radians/s.
    // </param>
    // <param name="gz">
    // Gyroscope z axis measurement in radians/s.
    // </param>
    // <param name="ax">
    // Accelerometer x axis measurement in any calibrated units.
    // </param>
    // <param name="ay">
    // Accelerometer y axis measurement in any calibrated units.
    // </param>
    // <param name="az">
    // Accelerometer z axis measurement in any calibrated units.
    // </param>
    // <param name="mx">
    // Magnetometer x axis measurement in any calibrated units.
    // </param>
    // <param name="my">
    // Magnetometer y axis measurement in any calibrated units.
    // </param>
    // <param name="mz">
    // Magnetometer z axis measurement in any calibrated units.
    // </param>
    // <remarks>
    // Optimised for minimal arithmetic.
    // </remarks>
    public void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
        float norm;
        float hx, hy, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;
        float pa, pb, pc;

        // Auxiliary variables to avoid repeated arithmetic
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = (float) Math.sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0f) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = (float) Math.sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0f) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        hx = 2f * mx * (0.5f - q3q3 - q4q4) + 2f * my * (q2q3 - q1q4) + 2f * mz * (q2q4 + q1q3);
        hy = 2f * mx * (q2q3 + q1q4) + 2f * my * (0.5f - q2q2 - q4q4) + 2f * mz * (q3q4 - q1q2);
        bx = (float) Math.sqrt((hx * hx) + (hy * hy));
        bx=0;
        bz = 2f * mx * (q2q4 - q1q3) + 2f * my * (q3q4 + q1q2) + 2f * mz * (0.5f - q2q2 - q3q3);

        // Estimated direction of gravity and magnetic field
        vx = -2f * (q2q4 - q1q3);
        vy = -2f * (q1q2 + q3q4);
        vz = -(q1q1 - q2q2 - q3q3 + q4q4);
        wx = 2f * bx * (0.5f - q3q3 - q4q4) + 2f * bz * (q2q4 - q1q3);
        wy = 2f * bx * (q2q3 - q1q4) + 2f * bz * (q1q2 + q3q4);
        wz = 2f * bx * (q1q3 + q2q4) + 2f * bz * (0.5f - q2q2 - q3q3);

        // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
        if (Ki > 0f) {
            eInt[0] += ex*SamplePeriod;      // accumulate integral error
            eInt[1] += ey*SamplePeriod;
            eInt[2] += ez*SamplePeriod;
        } else {
            eInt[0] = 0.0f;     // prevent integral wind up
            eInt[1] = 0.0f;
            eInt[2] = 0.0f;
        }

        // Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0];
        gy = gy + Kp * ey + Ki * eInt[1];
        gz = gz + Kp * ez + Ki * eInt[2];

        // Integrate rate of change of quaternion
        pa = q2;
        pb = q3;
        pc = q4;
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * SamplePeriod);
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * SamplePeriod);
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * SamplePeriod);
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * SamplePeriod);

        // Normalise quaternion
        norm = (float) Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        norm = 1.0f / norm;
        Quaternion[0] = q1 * norm;
        Quaternion[1] = q2 * norm;
        Quaternion[2] = q3 * norm;
        Quaternion[3] = q4 * norm;
    }

    // Algorithm IMU update method. Requires only gyroscope and accelerometer data.
    // <param name="gx">
    // Gyroscope x axis measurement in radians/s.
    // </param>
    // <param name="gy">
    // Gyroscope y axis measurement in radians/s.
    // </param>
    // <param name="gz">
    // Gyroscope z axis measurement in radians/s.
    // </param>
    // <param name="ax">
    // Accelerometer x axis measurement in any calibrated units.
    // </param>
    // <param name="ay">
    // Accelerometer y axis measurement in any calibrated units.
    // </param>
    // <param name="az">
    // Accelerometer z axis measurement in any calibrated units.
    // </param>
    public void update(float gx, float gy, float gz, float ax, float ay, float az) {
        float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;
        float pa, pb, pc;

        // Normalise accelerometer measurement
        norm = (float) Math.sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0f) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Estimated direction of gravity
        vx = 2.0f * (q2 * q4 - q1 * q3);
        vy = 2.0f * (q1 * q2 + q3 * q4);
        vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

        // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
        if (Ki > 0f) {
            eInt[0] += ex;      // accumulate integral error
            eInt[1] += ey;
            eInt[2] += ez;
        } else {
            eInt[0] = 0.0f;     // prevent integral wind up
            eInt[1] = 0.0f;
            eInt[2] = 0.0f;
        }

        // Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0];
        gy = gy + Kp * ey + Ki * eInt[1];
        gz = gz + Kp * ez + Ki * eInt[2];

        // Integrate rate of change of quaternion
        pa = q2;
        pb = q3;
        pc = q4;
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * SamplePeriod);
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * SamplePeriod);
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * SamplePeriod);
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * SamplePeriod);

        // Normalise quaternion
        norm = (float) Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        norm = 1.0f / norm;
        Quaternion[0] = q1 * norm;
        Quaternion[1] = q2 * norm;
        Quaternion[2] = q3 * norm;
        Quaternion[3] = q4 * norm;
    }

    public void setQuaternion(float[] quaternion) {
        Quaternion = quaternion;
    }

    public float[] getQuaternion() {
        return Quaternion;
    }

    public float[] quaternProd(float[] a,float[] b){
        float[] ab = new float[4];
        ab[0] = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
        ab[1] = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
        ab[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
        ab[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
        return ab;
    }
    public float[] quaternConj(float[] q){
        float[] qConj = {q[0],-q[1],-q[2],-q[3]};
        return qConj;
    }
}
