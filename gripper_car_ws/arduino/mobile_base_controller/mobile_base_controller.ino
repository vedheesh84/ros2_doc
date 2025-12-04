/*
 * MOBILE BASE CONTROLLER - Arduino Mega 2560
 * ============================================
 *
 * Four-wheel differential drive controller with:
 * - PID velocity control per wheel
 * - Quadrature encoder feedback
 * - Serial communication with ROS2
 * - TEST mode for simulated encoder feedback (no hardware required)
 *
 * Hardware:
 * - Arduino Mega 2560
 * - 2x L298N motor drivers
 * - 4x DC motors with quadrature encoders
 *
 * Serial Protocol:
 *   ROS -> Arduino:
 *     VEL,<linear_x>,<angular_z>  - Velocity command (m/s, rad/s)
 *     STOP                        - Emergency stop
 *     PID,<kp>,<ki>,<kd>         - Update PID gains
 *     RST                         - Reset encoder counts
 *     TEST,ON                     - Enable test mode (simulate encoders)
 *     TEST,OFF                    - Disable test mode (use real encoders)
 *
 *   Arduino -> ROS:
 *     ENC,<fl>,<fr>,<bl>,<br>    - Cumulative encoder ticks
 *     STATUS,<message>           - Status messages
 *     ERROR,<message>            - Error messages
 */

// =====================================================
// CONFIGURATION
// =====================================================

// Serial
#define SERIAL_BAUD 115200

// Robot parameters
#define WHEEL_RADIUS 0.075      // meters
#define WHEEL_BASE 0.35         // meters (left-right wheel distance = 2 * 0.175)
#define ENCODER_CPR 360         // Counts per revolution (after decoding)

// Timing
#define PID_SAMPLE_TIME 20      // milliseconds
#define ENCODER_PUB_RATE 50     // Hz
#define CMD_TIMEOUT 500         // milliseconds - stop if no command received

// PID gains (can be tuned via serial)
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// NOTE: On this hardware the motor driver ENABLE (PWM) pins are bridged to HIGH
// so we do not use PWM. Motor drivers are driven using IN1/IN2 (full on/off).
// The code still computes PID values, but without PWM the controller maps the
// computed output to full-forward / full-reverse / stop (simple bang-bang).
// You can tune the motor_on_threshold below to require a larger PID output
// before the motor is driven.
#define MOTOR_ON_THRESHOLD 30    // Minimum scaled PID output before enabling motor (0-255)

// =====================================================
// PIN DEFINITIONS - Arduino Mega 2560 + 2x L298N
// =====================================================

// Motor Driver 1 (L298N #1) - Front motors
#define M_FL_IN1  22    // Front Left Direction A
#define M_FL_IN2  23    // Front Left Direction B
#define M_FR_IN1  24    // Front Right Direction A
#define M_FR_IN2  25    // Front Right Direction B

// Motor Driver 2 (L298N #2) - Back motors
#define M_BL_IN1  26    // Back Left Direction A
#define M_BL_IN2  27    // Back Left Direction B
#define M_BR_IN1  28    // Back Right Direction A
#define M_BR_IN2  29    // Back Right Direction B

// Quadrature Encoders (using interrupt-capable pins)
#define ENC_FL_A  18    // Front Left Channel A (INT.3)
#define ENC_FL_B  31    // Front Left Channel B
#define ENC_FR_A  19    // Front Right Channel A (INT.2)
#define ENC_FR_B  33    // Front Right Channel B
#define ENC_BL_A  20    // Back Left Channel A (INT.1)
#define ENC_BL_B  35    // Back Left Channel B
#define ENC_BR_A  21    // Back Right Channel A (INT.0)
#define ENC_BR_B  37    // Back Right Channel B

// =====================================================
// GLOBAL VARIABLES
// =====================================================

// Encoder counts (volatile for ISR access)
volatile long enc_counts[4] = {0, 0, 0, 0};  // FL, FR, BL, BR
long prev_enc_counts[4] = {0, 0, 0, 0};

// Target velocities (rad/s for each wheel)
float target_vel[4] = {0, 0, 0, 0};  // FL, FR, BL, BR
float current_vel[4] = {0, 0, 0, 0};

// PID state for each wheel
float pid_integral[4] = {0, 0, 0, 0};
float pid_prev_error[4] = {0, 0, 0, 0};
// motor_state: -1 = reverse, 0 = stopped, 1 = forward
int motor_state[4] = {0, 0, 0, 0};

// Timing
unsigned long last_pid_time = 0;
unsigned long last_enc_pub_time = 0;
unsigned long last_cmd_time = 0;
unsigned long last_test_update_time = 0;

// Command buffer
String cmd_buffer = "";

// TEST MODE - Simulates encoder feedback without real hardware
bool test_mode = false;
float sim_enc_accumulator[4] = {0.0, 0.0, 0.0, 0.0};  // Fractional tick accumulator

// Last published encoder counts (for change detection)
long last_published_counts[4] = {0, 0, 0, 0};

// =====================================================
// MOTOR CONTROL
// =====================================================

// Motor IN pin arrays for easy access (no PWM used)
const int motor_in1_pins[4] = {M_FL_IN1, M_FR_IN1, M_BL_IN1, M_BR_IN1};
const int motor_in2_pins[4] = {M_FL_IN2, M_FR_IN2, M_BL_IN2, M_BR_IN2};

// Motor direction multipliers (adjust if motors spin wrong way)
// 1 = normal, -1 = reversed
const int motor_direction[4] = {1, -1, 1, -1};  // FL, FR, BL, BR

void setupMotors() {
    for (int i = 0; i < 4; i++) {
        pinMode(motor_in1_pins[i], OUTPUT);
        pinMode(motor_in2_pins[i], OUTPUT);

        // Initialize to stopped (both IN pins LOW)
        digitalWrite(motor_in1_pins[i], LOW);
        digitalWrite(motor_in2_pins[i], LOW);
    }
}

// Map a signed control value to motor on/off/direction using IN pins only.
// pwm_like: assumed -255..255 (signed). With no PWM, we map to full forward
// or full reverse when abs(value) >= MOTOR_ON_THRESHOLD, otherwise stop.
void setMotorOutput(int motor_idx, int pwm_like) {
    // Account for wiring direction
    int signed_val = pwm_like * motor_direction[motor_idx];

    if (signed_val > MOTOR_ON_THRESHOLD) {
        // Full forward
        digitalWrite(motor_in1_pins[motor_idx], HIGH);
        digitalWrite(motor_in2_pins[motor_idx], LOW);
        motor_state[motor_idx] = 1;
    } else if (signed_val < -MOTOR_ON_THRESHOLD) {
        // Full reverse
        digitalWrite(motor_in1_pins[motor_idx], LOW);
        digitalWrite(motor_in2_pins[motor_idx], HIGH);
        motor_state[motor_idx] = -1;
    } else {
        // Stop
        digitalWrite(motor_in1_pins[motor_idx], LOW);
        digitalWrite(motor_in2_pins[motor_idx], LOW);
        motor_state[motor_idx] = 0;
    }
}

void stopAllMotors() {
    for (int i = 0; i < 4; i++) {
        setMotorOutput(i, 0);
        target_vel[i] = 0;
        pid_integral[i] = 0;
        pid_prev_error[i] = 0;
    }
}

// =====================================================
// ENCODER HANDLING
// =====================================================

void setupEncoders() {
    // Set encoder pins as inputs with pullups
    pinMode(ENC_FL_A, INPUT_PULLUP);
    pinMode(ENC_FL_B, INPUT_PULLUP);
    pinMode(ENC_FR_A, INPUT_PULLUP);
    pinMode(ENC_FR_B, INPUT_PULLUP);
    pinMode(ENC_BL_A, INPUT_PULLUP);
    pinMode(ENC_BL_B, INPUT_PULLUP);
    pinMode(ENC_BR_A, INPUT_PULLUP);
    pinMode(ENC_BR_B, INPUT_PULLUP);

    // Attach interrupts (2x decoding - rising edge on channel A)
    attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isr_enc_fl, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isr_enc_fr, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_enc_bl, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_enc_br, RISING);
}

// Encoder ISRs - 2x decoding (count on rising edge of A)
void isr_enc_fl() {
    if (digitalRead(ENC_FL_B)) {
        enc_counts[0]--;
    } else {
        enc_counts[0]++;
    }
}

void isr_enc_fr() {
    if (digitalRead(ENC_FR_B)) {
        enc_counts[1]++;
    } else {
        enc_counts[1]--;
    }
}

void isr_enc_bl() {
    if (digitalRead(ENC_BL_B)) {
        enc_counts[2]--;
    } else {
        enc_counts[2]++;
    }
}

void isr_enc_br() {
    if (digitalRead(ENC_BR_B)) {
        enc_counts[3]++;
    } else {
        enc_counts[3]--;
    }
}

void getEncoderCounts(long* counts) {
    // Disable interrupts for atomic read
    noInterrupts();
    for (int i = 0; i < 4; i++) {
        counts[i] = enc_counts[i];
    }
    interrupts();
}

void resetEncoders() {
    noInterrupts();
    for (int i = 0; i < 4; i++) {
        enc_counts[i] = 0;
        prev_enc_counts[i] = 0;
    }
    interrupts();

    // Also reset simulated encoder accumulators
    for (int i = 0; i < 4; i++) {
        sim_enc_accumulator[i] = 0.0;
    }
}

// =====================================================
// TEST MODE - Simulated Encoder Update
// =====================================================

void updateSimulatedEncoders() {
    // Only run in test mode
    if (!test_mode) return;

    unsigned long now = millis();
    float dt = (now - last_test_update_time) / 1000.0;  // Convert to seconds

    // Limit dt to reasonable values
    if (dt <= 0 || dt > 0.1) {
        last_test_update_time = now;
        return;
    }

    // Simulate encoder ticks based on target wheel velocities
    // ticks = (omega * dt) / (2 * PI) * CPR
    // where omega is wheel angular velocity in rad/s

    for (int i = 0; i < 4; i++) {
        // Calculate expected ticks from target velocity
        float delta_ticks = (target_vel[i] * dt * ENCODER_CPR) / (2.0 * PI);

        // Accumulate fractional ticks
        sim_enc_accumulator[i] += delta_ticks;

        // Extract whole ticks and add to encoder count
        long whole_ticks = (long)sim_enc_accumulator[i];
        if (whole_ticks != 0) {
            noInterrupts();
            enc_counts[i] += whole_ticks;
            interrupts();
            sim_enc_accumulator[i] -= whole_ticks;
        }
    }

    last_test_update_time = now;
}

// =====================================================
// PID CONTROLLER
// =====================================================

float computePID(int motor_idx, float target, float current, float dt) {
    float error = target - current;

    // Proportional term
    float p_term = Kp * error;

    // Integral term with anti-windup
    pid_integral[motor_idx] += error * dt;
    pid_integral[motor_idx] = constrain(pid_integral[motor_idx], -100, 100);
    float i_term = Ki * pid_integral[motor_idx];

    // Derivative term
    float d_term = 0;
    if (dt > 0) {
        d_term = Kd * (error - pid_prev_error[motor_idx]) / dt;
    }
    pid_prev_error[motor_idx] = error;

    // Compute output
    float output = p_term + i_term + d_term;

    // Scale to PWM range (assuming max velocity ~10 rad/s maps to 255 PWM)
    float pwm_scale = 25.5;  // 255 / 10
    output = output * pwm_scale;

    return constrain(output, -PWM_MAX, PWM_MAX);
}

void updatePID() {
    unsigned long now = millis();
    float dt = (now - last_pid_time) / 1000.0;  // Convert to seconds

    if (dt < (PID_SAMPLE_TIME / 1000.0)) {
        return;  // Not time yet
    }

    // Get current encoder counts
    long counts[4];
    getEncoderCounts(counts);

    // Calculate wheel velocities (rad/s)
    for (int i = 0; i < 4; i++) {
        long delta = counts[i] - prev_enc_counts[i];
        prev_enc_counts[i] = counts[i];

        // Convert encoder ticks to radians
        // velocity = (delta_ticks / CPR) * 2*PI / dt
        current_vel[i] = (delta * 2.0 * PI) / (ENCODER_CPR * dt);
    }

    // Update PID for each wheel
    for (int i = 0; i < 4; i++) {
        int pwm = (int)computePID(i, target_vel[i], current_vel[i], dt);

        // If target is zero, just stop
        if (abs(target_vel[i]) < 0.01) {
            pwm = 0;
            pid_integral[i] = 0;
        }

        setMotorOutput(i, pwm);
    }

    last_pid_time = now;
}

// =====================================================
// DIFFERENTIAL DRIVE KINEMATICS
// =====================================================

void setVelocityCommand(float linear_x, float angular_z) {
    // Differential drive inverse kinematics
    // v_left = linear_x - angular_z * (wheel_base / 2)
    // v_right = linear_x + angular_z * (wheel_base / 2)

    float v_left = linear_x - (angular_z * WHEEL_BASE / 2.0);
    float v_right = linear_x + (angular_z * WHEEL_BASE / 2.0);

    // Convert linear velocity to wheel angular velocity (rad/s)
    float omega_left = v_left / WHEEL_RADIUS;
    float omega_right = v_right / WHEEL_RADIUS;

    // Set target velocities for all 4 wheels
    // FL and BL get left velocity, FR and BR get right velocity
    target_vel[0] = omega_left;   // Front Left
    target_vel[1] = omega_right;  // Front Right
    target_vel[2] = omega_left;   // Back Left
    target_vel[3] = omega_right;  // Back Right

    // Update last command time
    last_cmd_time = millis();
}

// =====================================================
// SERIAL COMMUNICATION
// =====================================================

void processCommand(String cmd) {
    cmd.trim();

    if (cmd.startsWith("VEL,")) {
        // Parse velocity command: VEL,linear_x,angular_z
        int idx1 = cmd.indexOf(',');
        int idx2 = cmd.indexOf(',', idx1 + 1);

        if (idx2 > idx1) {
            float linear_x = cmd.substring(idx1 + 1, idx2).toFloat();
            float angular_z = cmd.substring(idx2 + 1).toFloat();
            setVelocityCommand(linear_x, angular_z);
        }
    }
    else if (cmd == "STOP") {
        stopAllMotors();
        Serial.println("STATUS,Stopped");
    }
    else if (cmd.startsWith("PID,")) {
        // Parse PID gains: PID,kp,ki,kd
        int idx1 = cmd.indexOf(',');
        int idx2 = cmd.indexOf(',', idx1 + 1);
        int idx3 = cmd.indexOf(',', idx2 + 1);

        if (idx3 > idx2 && idx2 > idx1) {
            Kp = cmd.substring(idx1 + 1, idx2).toFloat();
            Ki = cmd.substring(idx2 + 1, idx3).toFloat();
            Kd = cmd.substring(idx3 + 1).toFloat();
            Serial.print("STATUS,PID updated: Kp=");
            Serial.print(Kp);
            Serial.print(" Ki=");
            Serial.print(Ki);
            Serial.print(" Kd=");
            Serial.println(Kd);
        }
    }
    else if (cmd == "RST") {
        resetEncoders();
        Serial.println("STATUS,Encoders reset");
    }
    else if (cmd == "TEST,ON") {
        test_mode = true;
        last_test_update_time = millis();
        resetEncoders();
        Serial.println("STATUS,Test mode ENABLED - simulating encoders");
    }
    else if (cmd == "TEST,OFF") {
        test_mode = false;
        resetEncoders();
        Serial.println("STATUS,Test mode DISABLED - using real encoders");
    }
    else if (cmd == "STATUS") {
        // Return current status
        Serial.print("STATUS,Mode:");
        Serial.print(test_mode ? "TEST" : "NORMAL");
        Serial.print(",Vel:[");
        Serial.print(target_vel[0], 2);
        Serial.print(",");
        Serial.print(target_vel[1], 2);
        Serial.print(",");
        Serial.print(target_vel[2], 2);
        Serial.print(",");
        Serial.print(target_vel[3], 2);
        Serial.println("]");
    }
}

void readSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            if (cmd_buffer.length() > 0) {
                processCommand(cmd_buffer);
                cmd_buffer = "";
            }
        } else if (c != '\r') {
            cmd_buffer += c;
        }
    }
}

void publishEncoderCounts() {
    unsigned long now = millis();
    unsigned long pub_interval = 1000 / ENCODER_PUB_RATE;

    if (now - last_enc_pub_time >= pub_interval) {
        long counts[4];
        getEncoderCounts(counts);

        // Only publish if any encoder value has changed
        bool changed = false;
        for (int i = 0; i < 4; i++) {
            if (counts[i] != last_published_counts[i]) {
                changed = true;
                break;
            }
        }

        if (changed) {
            Serial.print("ENC,");
            Serial.print(counts[0]);
            Serial.print(",");
            Serial.print(counts[1]);
            Serial.print(",");
            Serial.print(counts[2]);
            Serial.print(",");
            Serial.println(counts[3]);

            // Update last published values
            for (int i = 0; i < 4; i++) {
                last_published_counts[i] = counts[i];
            }
        }

        last_enc_pub_time = now;
    }
}

void checkCommandTimeout() {
    if (millis() - last_cmd_time > CMD_TIMEOUT) {
        // No command received for too long, stop motors
        if (target_vel[0] != 0 || target_vel[1] != 0 ||
            target_vel[2] != 0 || target_vel[3] != 0) {
            stopAllMotors();
            // Only print once
            Serial.println("STATUS,Command timeout - stopped");
        }
    }
}

// =====================================================
// MAIN
// =====================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ; // Wait for serial port to connect
    }

    setupMotors();
    setupEncoders();

    last_pid_time = millis();
    last_enc_pub_time = millis();
    last_cmd_time = millis();
    last_test_update_time = millis();

    Serial.println("STATUS,Mobile Base Controller Ready");
    Serial.print("STATUS,Wheel Radius: ");
    Serial.print(WHEEL_RADIUS);
    Serial.print("m, Wheel Base: ");
    Serial.print(WHEEL_BASE);
    Serial.println("m");
    Serial.println("STATUS,Send TEST,ON to enable encoder simulation");
}

void loop() {
    // Read and process serial commands
    readSerial();

    // Check for command timeout
    checkCommandTimeout();

    // Update simulated encoders (only active in TEST mode)
    updateSimulatedEncoders();

    // Update PID controllers (skipped in TEST mode since no real motors)
    if (!test_mode) {
        updatePID();
    }

    // Publish encoder counts
    publishEncoderCounts();
}
