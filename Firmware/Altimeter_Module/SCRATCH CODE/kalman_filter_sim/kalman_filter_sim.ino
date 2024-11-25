#include <Arduino.h>
#include <math.h>

// Define matrix dimensions
const int n = 2; // State vector size (altitude, velocity)
const int m = 2; // Measurement vector size (pressure altitude, altimeter altitude)

// EKF state and covariance matrices
float x[n] = {0, 0}; // State vector [altitude, velocity]
float P[n][n] = {{1, 0}, {0, 1}}; // Estimate error covariance
float F[n][n] = {{1, 0.01}, {0, 1}}; // State transition matrix
float Q[n][n] = {{0.1, 0}, {0, 0.1}}; // Process noise covariance
float R[m][m] = {{1, 0}, {0, 1}}; // Measurement noise covariance (pressure sensor, altimeter)
float H[m][n] = {{1, 0}, {1, 0}}; // Measurement matrix (pressure sensor, altimeter)

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
void predict() {
    // Update state estimate
    float x_new[n];
    x_new[0] = F[0][0] * x[0] + F[0][1] * x[1];
    x_new[1] = F[1][0] * x[0] + F[1][1] * x[1];
    x[0] = x_new[0];
    x[1] = x_new[1];

    // Update error covariance
    float FP[n][n];
    matMul(F, P, FP, n, n, n);
    float FTP[n][n];
    matMul(FP, F, FTP, n, n, n);
    matAdd(FTP, Q, P, n, n);
}

// Update step
void update(float pressure_altitude, float altimeter_altitude) {
    // Compute innovation
    float z[m] = {pressure_altitude, altimeter_altitude};
    float y[m];
    y[0] = z[0] - (H[0][0] * x[0] + H[0][1] * x[1]);
    y[1] = z[1] - (H[1][0] * x[0] + H[1][1] * x[1]);

    // Compute innovation covariance
    float HP[m][n];
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            HP[i][j] = H[i][j] * P[j][j];
        }
    }
    float S[m][m];
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            S[i][j] = HP[i][0] * H[0][j] + HP[i][1] * H[1][j] + R[i][j];
        }
    }

    // Compute Kalman gain
    float K[n][m];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            K[i][j] = (P[i][0] * H[0][j] + P[i][1] * H[1][j]) / S[j][j];
        }
    }

    // Update state estimate
    float x_new[n];
    for (int i = 0; i < n; i++) {
        x_new[i] = x[i];
        for (int j = 0; j < m; j++) {
            x_new[i] += K[i][j] * y[j];
        }
    }
    x[0] = x_new[0];
    x[1] = x_new[1];

    // Update error covariance
    float I_KH[n][n];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            I_KH[i][j] = i == j ? 1 : 0;
            for (int k = 0; k < m; k++) {
                I_KH[i][j] -= K[i][k] * H[k][j];
            }
        }
    }
    float I_KHP[n][n];
    matMul(I_KH, P, I_KHP, n, n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P[i][j] = I_KHP[i][j];
        }
    }
}

// Function to calculate altitude from pressure
float calculateAltitudeFromPressure(float pressure) {

    return 0;
}

// Function to get altitude from altimeter
float getAltimeterAltitude() {

    return 0;
}

void setup() {
    Serial.begin(9600);
}

void loop() {
    float pressure = 1000.0f; // Example pressure value
    float pressure_altitude = calculateAltitudeFromPressure(pressure);
    float altimeter_altitude = getAltimeterAltitude();

    // Run the EKF predict and update steps
    predict();
    update(pressure_altitude, altimeter_altitude);

    // Print the filtered altitude
    Serial.print("Filtered Altitude: ");
    Serial.println(x[0]);

    delay(1000); // Delay for 1 second
}