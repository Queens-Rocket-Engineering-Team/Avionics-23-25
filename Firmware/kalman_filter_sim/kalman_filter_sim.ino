#include <Arduino.h>
#include <math.h>

// Define matrix dimensions
const int n = 2; // State vector size (altitude, velocity)
const int m = 1; // Measurement vector size (altitude)

// EKF state and covariance matrices
float x[n] = {0, 0}; // State vector [altitude, velocity]
float P[n][n] = {{1, 0}, {0, 1}}; // Estimate error covariance
float F[n][n] = {{1, 0.01}, {0, 1}}; // State transition matrix
float Q[n][n] = {{0.1, 0}, {0, 0.1}}; // Process noise covariance
float R[m][m] = {{1}}; // Measurement noise covariance (pressure sensor)
float H[m][n] = {{1, 0}}; // Measurement matrix (pressure sensor)
float G[n] = {0.5 * 0.01 * 0.01, 0.01}; // Control input model (for acceleration)

// Matrix multiplication helper function
void matMul(float a[][n], float b[][n], float result[][n], int aRows, int aCols, int bCols) {
    for (int i = 0; i < aRows; i++) {
        for (int j = 0; j < bCols; j++) {
            result[i][j] = 0;
            for (int k = 0; k < aCols; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// Matrix addition helper function
void matAdd(float a[][n], float b[][n], float result[][n], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

// Predict step
void predict(float accel) {
    // Update state estimate
    x[0] = F[0][0] * x[0] + F[0][1] * x[1] + G[0] * accel;
    x[1] = F[1][0] * x[0] + F[1][1] * x[1] + G[1] * accel;

    // Update error covariance
    float FP[n][n];
    matMul(F, P, FP, n, n, n);
    float FTP[n][n];
    matMul(FP, F, FTP, n, n, n);
    matAdd(FTP, Q, P, n, n);
}

// Update step
void update(float altitude_measurement) {
    // Compute innovation
    float z[m] = {altitude_measurement};
    float y[m];
    y[0] = z[0] - (H[0][0] * x[0] + H[0][1] * x[1]);

    // Compute innovation covariance
    float HP[m][n];
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            HP[i][j] = H[i][j] * P[j][j];
        }
    }
    float S[m][m];
    S[0][0] = HP[0][0] * H[0][0] + R[0][0];

    // Compute Kalman gain
    float K[n][m];
    K[0][0] = P[0][0] * H[0][0] / S[0][0];
    K[1][0] = P[1][1] * H[0][0] / S[0][0];

    // Update state estimate
    x[0] = x[0] + K[0][0] * y[0];
    x[1] = x[1] + K[1][0] * y[0];

    // Update error covariance
    float I_KH[n][n] = {{1 - K[0][0] * H[0][0], 0}, {0, 1 - K[1][0] * H[0][0]}};
    float I_KHP[n][n];
    matMul(I_KH, P, I_KHP, n, n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P[i][j] = I_KHP[i][j];
        }
    }
}

// Simulate rocket flight profile and test EKF accuracy
void simulateFlight() {
    // Parameters for simulation
    float actual_altitude = 0.0;
    float actual_velocity = 0.0;
    float actual_acceleration = 0.0;
    float time = 0.0;
    float dt = 0.01; // Time step
    float total_time = 10.0; // Total simulation time
    float gravity = -9.81; // Gravity acceleration

    Serial.println("Time\tActual Altitude\tMeasured Altitude\tEstimated Altitude\tActual Velocity\tEstimated Velocity");

    while (time < total_time) {
        // Simulate rocket flight: a simple parabolic motion
        if (time < 2.0) {
            actual_acceleration = 30.0; // Rocket thrust
        } else {
            actual_acceleration = gravity; // Free fall
        }

        // Update actual state
        actual_velocity += actual_acceleration * dt;
        actual_altitude += actual_velocity * dt + 0.5 * actual_acceleration * dt * dt;

        // Simulate sensor measurements with noise
        float measured_altitude = actual_altitude + random(-5, 5) / 10.0;
        float measured_acceleration = actual_acceleration + random(-1, 1) / 10.0;

        // Apply EKF
        predict(measured_acceleration);
        update(measured_altitude);

        // Print results in a format suitable for the Serial Plotter
        Serial.print(time);
        Serial.print("\t");
        Serial.print(actual_altitude);
        Serial.print("\t");
        Serial.print(measured_altitude);
        Serial.print("\t");
        Serial.print(x[0]);
        Serial.print("\t");
        Serial.print(actual_velocity);
        Serial.print("\t");
        Serial.println(x[1]);

        // Increment time
        time += dt;
        delay(10); // Adjust delay for real-time plotting
    }
}

// Setup function
void setup() {
    Serial.begin(9600);
    simulateFlight();
}

// Loop function
void loop() {
    // No code needed here for this simulation
}
