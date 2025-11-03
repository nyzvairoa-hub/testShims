#include "pxt.h" // Ignore the red squiggle! The compiler finds this.

// ------------------------------------------
// --- GLOBAL STATE VARIABLES (Persistent Memory) ---
// ------------------------------------------

// State Vector X: [position, velocity]. These hold the filter's estimate.
// The filter's power comes from these values persisting between loop cycles.
static float X_k[2]; // X_k[0] = Position, X_k[1] = Velocity

// Covariance Matrix P: [ [P11, P12], [P21, P22] ]. This holds the uncertainty.
static float P_k[2][2];

// Fixed (Tuned) Constants
// Q (Process Noise): How much uncertainty to add to the model.
const float Q_k_pos = 0.001; // Noise for position
const float Q_k_vel = 0.01;  // Noise for velocity
// R (Measurement Noise): Sensor uncertainty (e.g., HuskyLens noise).
const float R_k = 0.5;

// ------------------------------------------
// --- FUNCTION 1: RESET STATE (shim=KalmanController::resetState) ---
// ------------------------------------------

/**
 * Resets the Kalman Filter's state and covariance matrix.
 */
// The 'void' return is mapped to the 'void' in the TypeScript shim.
extern "C" void resetState() {
    // Initialize State (start at position 0, velocity 0)
    X_k[0] = 0.0;
    X_k[1] = 0.0;

    // Initialize Covariance (high initial uncertainty)
    P_k[0][0] = 1.0;
    P_k[0][1] = 0.0;
    P_k[1][0] = 0.0;
    P_k[1][1] = 1.0;
}

// ------------------------------------------
// --- FUNCTION 2: UPDATE FILTER (shim=KalmanController::updateFilter) ---
// ------------------------------------------

/**
 * Executes one full Prediction/Correction cycle of the Kalman Filter.
 * (This is a simplified 1D Constant Velocity KF for demonstration)
 *
 * @param z_k_measured The raw X-coordinate from the HuskyLens.
 * @param dt The time elapsed since the last cycle (Delta Time).
 * @return The filtered and predicted position (X_k[0]).
 */
extern "C" float updateFilter(float z_k_measured, float dt) {
    // --- PHASE 1: PREDICTION (F * X_k-1) ---
    // F = [ [1, dt], [0, 1] ]
    
    // 1. Predict New State (Position and Velocity)
    float X_predicted_pos = X_k[0] + (X_k[1] * dt);
    float X_predicted_vel = X_k[1]; // Velocity assumed constant

    // 2. Predict New Covariance (P_predicted = F * P_k * F_T + Q)
    // This is the most complex matrix math step.
    float P_predicted[2][2];
    P_predicted[0][0] = P_k[0][0] + dt*P_k[1][0] + dt*(P_k[0][1] + dt*P_k[1][1]) + Q_k_pos;
    P_predicted[0][1] = P_k[0][1] + dt*P_k[1][1];
    P_predicted[1][0] = P_k[1][0] + dt*P_k[1][1];
    P_predicted[1][1] = P_k[1][1] + Q_k_vel;


    // --- PHASE 2: CORRECTION (Fuse prediction with actual measurement) ---

    // 3. Calculate Kalman Gain (K_k)
    // S = P_predicted[0][0] + R_k (Since H=[1 0])
    float S = P_predicted[0][0] + R_k;
    
    float K_gain[2];
    K_gain[0] = P_predicted[0][0] / S; // Gain for Position
    K_gain[1] = P_predicted[1][0] / S; // Gain for Velocity

    // 4. Update State Estimate (X_k = X_predicted + K_k * residual)
    // residual = (z_k - H * X_predicted)
    float residual = z_k_measured - X_predicted_pos;
    
    X_k[0] = X_predicted_pos + (K_gain[0] * residual); // Final Position Estimate
    X_k[1] = X_predicted_vel + (K_gain[1] * residual); // Final Velocity Estimate

    // 5. Update Covariance (P_k = (I - K_k * H) * P_predicted)
    P_k[0][0] = P_predicted[0][0] * (1.0 - K_gain[0]);
    P_k[0][1] = P_predicted[0][1] * (1.0 - K_gain[0]);
    P_k[1][0] = P_predicted[1][0] - (K_gain[1] * P_predicted[0][0]);
    P_k[1][1] = P_predicted[1][1] - (K_gain[1] * P_predicted[0][1]);

    // Return the stable, lag-free estimate of the target's position
    return X_k[0];
}
