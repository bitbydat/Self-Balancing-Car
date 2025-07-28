#include "KF.h"


float Kalman_Filter_y(float Accel, float Gyro)		
{
	float Q_angle = 0.001;
	float Q_gyro = 0.003;	
	float R_angle = 0.5;	
	char  C_0 = 1;
    	const float dt = 0.005;
	static float angle_KF = 0;
	static float Q_bias = 0, Angle_err = 0;
	static float PCt_0 = 0, PCt_1 = 0, E = 0;
	static float K_0 = 0, K_1 = 0, t_0 = 0, t_1 = 0;
	static float Pdot[4] = {0,0,0,0};
	static float PP[2][2];
    
	angle_KF += (Gyro - Q_bias) * dt; 
	
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; 
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;  
	PP[0][1] += Pdot[1] * dt;  
	PP[1][0] += Pdot[2] * dt; 
	PP[1][1] += Pdot[3] * dt;
	
	Angle_err = Accel - angle_KF;	
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle_KF	+= K_0 * Angle_err;	   
	Q_bias	+= K_1 * Angle_err;	 
//	angle_dot   = Gyro - Q_bias;	
	return angle_KF;
}


float KF_Y(float acce_X, float acce_Z, float gyro_Y) // Input quantities: X-axis acceleration, Z-axis acceleration, Y-axis angular velocity.
{
    static float x_hat[2][1] = {0}; // Posterior Estimation
    static float x_hat_minus[2][1] = {0}; // Prior estimates
    static float p_hat[2][2] = {{1, 0}, {0, 1}}; // Posterior error covariance matrix
    static float p_hat_minus[2][2] = {0}; // Prior error covariance matrix
    static float K[2][1] = {0}; // Kalman Gain
    const float Ts = 0.005; // Sampling interval (5ms)
    const float I[2][2] = {{1, 0}, {0, 1}};
    float u[1][1] = {{gyro_Y}};
    float A[2][2] = {{1, -Ts}, {0, 1}}; // A Matrix
    float B[2][1] = {{Ts}, {0}}; // B Matrix
    float C[1][2] = {{1, 0}};//C Matrix
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}}; // Process noise
    float R[1][1] = {{1e-4}}; // Measurement noise
    float A_T[2][2] = {{1, 0}, {-Ts, 1}}; // Transpose of matrix A
    float C_T[2][1] = {{1}, {0}}; // Transpose of the C matrix
    float temp_1[2][1] = {0}; // Used to store intermediate calculation results
    float temp_2[2][1] = {0}; // Used to store intermediate calculation results
    float temp_3[2][2] = {0}; // Used to store intermediate calculation results
    float temp_4[2][2] = {0}; // Used to store intermediate calculation results
    float temp_5[1][2] = {0}; // Used to store intermediate calculation results
    float temp_6[1][1] = {0}; // Used to store intermediate calculation results
    float y = atan2f(-acce_X, acce_Z); // Calculating Angle Using Acceleration
    
		/* Prediction section */
    // A priori estimation formula
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    // A priori Error Covariance Formula
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    
		/* Correction part */
    // Kalman Gain Formula
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    //  Posterior Estimation Formula
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    // Update error covariance formula
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    // Return Value
    return x_hat[0][0];
}

/**************************************************************************
Function: Matrix multiplication
Input: The two matrices to be multiplied and their sizes
Output: The matrix after multiplication
**************************************************************************/
void mul(int A_row, int A_col, int B_row, int B_col, float A[][A_col], float B[][B_col], float C[][B_col])
{
    if (A_col == B_row)
    {
        for (int i = 0; i < A_row; i++)
        {
            for (int j = 0; j < B_col; j++)
            {
                C[i][j] = 0; 
                for (int k = 0; k < A_col; k++)
                {
                    C[i][j] += A[i][k]*B[k][j];
                }
            }
        }
    }
}

