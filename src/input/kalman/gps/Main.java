package input.kalman.gps;

import java.io.IOException;

public class Main {
    public static void main(String[] args) throws IOException {
        new KalmanFilter();
        new GPSDataFactory();
    }
}
