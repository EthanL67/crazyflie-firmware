/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "CONTROLLERLQR"
#include "debug.h"


// Call the PID controller in this example to make it possible to fly. When you implement you own controller, there is
// no need to include the pid controller.
#include "controller.h"
#include "controller_pid.h"
#include "arm_math.h"


// Alternative 1D array definition for better memory alignment
static const float32_t K_matrix[4*12] = {
  // Row 0
  4.48986671755197e-14, -1.88653828703231e-14, -5.65212566808244e-15, 3.44997191249614e-15, 
  8.95292141616676e-15, 1.00000000000000, 7.85950795904996e-15, -4.20560340185763e-15, 
  -5.78167295060859e-15, 5.03625895440539e-15, 1.27225933188135e-14, 1.03082491238813,
  
  // Row 1
  5.42752947251153, -1.42849526593552e-13, 5.60079737169229e-14, 2.13092106503780e-14, 
  0.999999999999804, -8.59700433982214e-15, 1.00012998630995, -2.66271009440117e-14, 
  4.22544541534156e-14, 2.38534120034320e-14, 1.45177790176061, -3.28981819551547e-15,
  
  // Row 2
  1.49807249938405e-13, 5.42752947251274, -3.94030240293691e-15, -1.00000000000005, 
  4.49891865995052e-14, 7.10817428074312e-15, 1.53351545674190e-14, 1.00012998631022, 
  1.51086266099298e-15, -1.45177790176103, 4.91718676463854e-14, 3.77821844516395e-16,
  
  // Row 3
  -4.99486082383900e-14, -6.12020672096454e-15, 1.00000000000000, 6.21461068199196e-16, 
  -1.82670539599029e-14, 1.49699514122910e-15, -4.06533304225868e-15, 2.31755067219498e-15, 
  1.00003234647685, 1.38093876035688e-15, -1.02094231582995e-14, 5.86373244478807e-16
};


void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
    // DEBUG_PRINT("Hello World!\n");
  }
}

// The new controller goes here --------------------------------------------

void controllerOutOfTreeInit() {
  // Initialize your controller data here...

  // Call the PID controller instead in this example to make it possible to fly
  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Implement your controller here...
  if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    float32_t state_vector[12]; // Fill this with your state values
    float32_t result[4];

    control->controlMode = controlModeForceTorque;

    // set state variables
    state_vector[0] = state->attitude.roll;   // phi
    state_vector[1] = -state->attitude.pitch; // theta (pitch is inverted on cf)
    state_vector[2] = state->attitude.yaw;    // psi
    state_vector[3] = state->position.x;      // x
    state_vector[4] = state->position.y;      // y
    state_vector[5] = state->position.z;      // z
    state_vector[6] = 0;  // p
    state_vector[7] = 0;  // q
    state_vector[8] = 0;  // r
    state_vector[9] = state->velocity.x;      // u
    state_vector[10] = state->velocity.y;     // v
    state_vector[11] = state->velocity.z;     // w
    
    // You can use your fmmul function to multiply K_matrix with state_vector
    fmmul((float32_t *)K_matrix, state_vector, result, 4, 12, 1);

    // copy control signal
    control->thrust = result[0];
    control->torqueX = result[1];
    control->torqueY = result[2];
    control->torqueZ = result[3];
  }
}

void fmmul(float32_t *A, float32_t *B, float32_t *C, uint16_t m, uint16_t n, uint16_t p) {
  arm_matrix_instance_f32 A_instance, B_instance, C_instance;
  
  arm_mat_init_f32(&A_instance, m, n, A);
  arm_mat_init_f32(&B_instance, n, p, B);
  arm_mat_init_f32(&C_instance, m, p, C);
  
  arm_mat_mult_f32(&A_instance, &B_instance, &C_instance);
}


/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)
