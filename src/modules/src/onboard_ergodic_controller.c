#include "commander.h"
#include "stabilizer_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "onboard_ergodic_controller.h"
#include "param.h"
#include "log.h"

// Trying to setup the triggering mechanism
static bool run_controller = false;

PARAM_GROUP_START(ergodicController)
PARAM_ADD(PARAM_UINT8, run, &run_controller)
PARAM_GROUP_STOP(ergodicController)

LOG_GROUP_START(ergodicController)
LOG_ADD(LOG_UINT8, run, &run_controller)
LOG_GROUP_STOP(ergodicController)

// Base parameters of ergodic controller:
static int N = 1; // number of agents
static int n = 2; // number of dimensions
static int L1 = 1; // width of region (in meters)
static int L2 = 1; // length of region (in meters)
static int maxK = 50; // maximum fourier series frequency to go up to

static void onboard_ergodic_controller_task(void* param) {
    setpoint_t setpoint;

    while (1) {
        if (run_controller) {
            // Generate a velocity setpoint
            setpoint.mode.x = modeVelocity;
            setpoint.mode.y = modeVelocity;
            setpoint.mode.z = modeVelocity;
            setpoint.mode.yaw = modeVelocity;

            setpoint.velocity.x = 0.5f;  // Forward velocity (m/s)
            setpoint.velocity.y = 0.0f;  // No sideways motion
            setpoint.velocity.z = 0.0f;  // Maintain altitude
            setpoint.attitudeRate.yaw = 0.0f;  // No yaw rotation

            // Send the setpoint to the control loop
            commanderSetSetpoint(&setpoint, 10);  // 10 ms delay in control loop

            vTaskDelay(M2T(100));  // 100 ms between setpoints
        }
    }
}

void onboard_ergodic_controller_init() {
    xTaskCreate(onboard_ergodic_controller_task, "ErgodicCtrl", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

static velocity_t obtain_ergodic_control() {

}

static velocity_t get_B_j(float t) {
    velocity_t output;
    output.x = 0.0f;
    output.y = 0.0f;
    output.z = 0.0f;
}

static float get_s() {
    return (n+1) / 2.0f;
}

static float get_Lambda(float k1, float k2) {
    float output = 1.0f;
    float s = get_s();
    for (int i = 1; i <= s; i++) {
        output = output / (1+norm_sqr(k1,k2));
    }
    if (n % 2 == 0) output = output / sqrt(1+norm_sqr(k1,k2));
    return output;
}

static float norm_sqr(float v1, float v2) {
    return v1*v1 + v2*v2;
}

static float sqrt(float num) {
    if (num <= 0) return 0;
    float root=num/3;
    int i;
    for (i=0; i<32; i++)
        root = (root + num / root) / 2;
    return root;
}

static float norm(float v1, float v2) {
    return sqrt(hypot(v1, v2));
}