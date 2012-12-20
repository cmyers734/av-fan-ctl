/*-----------------------------------------------------------------------------
 * A/V Cabinet Fan Controller
 * Christopher Myers 2012
 */

#include "pitches.h"

// Defines ---------------------------------------------------------------------
#define DIO_BUZZER           12
#define PWM_FAN              11
#define AN_IN_CABINET_TEMP   0
#define AN_INTAKE_AIR_TEMP   2

#define DEGF_TO_DEGC(x) ((x - 32) * 5/9)

#define BUZZER_COMMAND_OFF    0
#define BUZZER_COMMAND_ALARM  1

#define BUZZER_STATE_OFF 0
#define BUZZER_STATE_ON  1

// Globals ---------------------------------------------------------------------
float g_fan_command_pct, g_fan_command_pct_prev;
float g_fan_command_pct_raw;
float g_fan_command_pct_i_state;
float g_in_cabinet_temperature_degc;
float g_intake_air_temperature_degc;
int g_fan_pwm_dc = 0;
int g_diag_inversion_fault = 0;
int g_diag_gradient_fault = 0;
int g_diag_overheat_fault = 0;

int g_buzzer_state = BUZZER_STATE_OFF;
int g_buzzer_command = BUZZER_COMMAND_OFF;

// Cals ------------------------------------------------------------------------
// This cal determines the desired in-cabinet temperature
const float k_in_cabinet_temperature_set_point_degc = DEGF_TO_DEGC(68);
const float k_p = 0.05;
const float k_i = 0.01;
const float k_fan_command_pct_alpha = 0.6; // Rate limit on the fan duty cycle
const float k_max_fan_command_pct_i_state = 1.0;
const int k_min_fan_cutoff_dc = 10;
// If the temperature gradient is only 2 degrees, we have a gradient condition
const float k_min_temp_difference_gradient_fault_detect_degc = 2.0;
// Above 90 degrees and we will have an overheating problem
const float k_min_temp_thresh_gradient_fault_detect_degc = DEGF_TO_DEGC(90);
// Above 100 degrees and we have an overheating problem
const float k_min_temp_thresh_overheat_fault_detect_degc = DEGF_TO_DEGC(100);

// Desired loop time
const unsigned long loop_time_ms = 1000L;

//------------------------------------------------------------------------------
void setup() { 
 //Initialize serial and wait for port to open:
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  g_fan_command_pct = 0;
  g_fan_command_pct_prev = 0;
  g_fan_command_pct_i_state = 0;
  
  // prints title with ending line break 
  Serial.println("A/V Cabinet Fan Controller v0.1"); 
  Serial.println("Christopher Myers - 2012"); 
  
  pinMode(DIO_BUZZER, OUTPUT);
  
  // Play startup sound
  tone(DIO_BUZZER, NOTE_C4, 250);
  delay(250);
  tone(DIO_BUZZER, NOTE_E4, 250);
  delay(250);
} 

//------------------------------------------------------------------------------
// Take a 10-bit analog to digital reading (assumes 5V reference) and convert it
// to degrees celsius.
//
float convert_tmp36_adc_to_degc(int analog_reading) { 
  // converting that reading to voltage, for 3.3v arduino use 3.3
  float v = (float)analog_reading * (5.0 / 1023.0);
  
  // The TMP36 has linear characteristics where 1.0V = 50 degC
  // Offset Voltage (V): 0.5
  // Output Voltage Scaling (mV/°C): 10
  // Output Voltage @ 25°C (mV): 750
  float temperature = (v - 0.5) * 100.0;
  
  return temperature;
}

//------------------------------------------------------------------------------
float degc_to_degf(float deg_c) {
  return (deg_c * (9.0/5.0)) + 32.0;
}

//------------------------------------------------------------------------------
float filter_low_pass(float x_prev, float x, float a) {
  return (x_prev * a) + (x * (1.0 - a));
}

//------------------------------------------------------------------------------
void task_input() {
  int reading = analogRead(AN_IN_CABINET_TEMP);  
  g_in_cabinet_temperature_degc = convert_tmp36_adc_to_degc(reading);
  
  reading = analogRead(AN_INTAKE_AIR_TEMP);  
  g_intake_air_temperature_degc = convert_tmp36_adc_to_degc(reading);  
}

//------------------------------------------------------------------------------
void task_fan_control() {
  // Fan control
  float temperature_error = g_in_cabinet_temperature_degc - k_in_cabinet_temperature_set_point_degc;
  
  // PI Control
  float p_term = temperature_error * k_p;
  float i_term = g_fan_command_pct_i_state + (temperature_error * k_i);
  
  // Store away previous so we can run a low-pass filter
  g_fan_command_pct_prev = g_fan_command_pct;
  
  // Saturation on the integrator
  i_term = min(i_term, k_max_fan_command_pct_i_state);
  g_fan_command_pct_i_state = i_term;
  
  // Keep from 0 to 1.0 (100%)
  g_fan_command_pct_raw = constrain(p_term + i_term, 0, 1);
   
  // Filter the fan command
  g_fan_command_pct = filter_low_pass(g_fan_command_pct_prev, g_fan_command_pct_raw, k_fan_command_pct_alpha);
  
  // Apply fan commanded duty cycle
  g_fan_pwm_dc = (g_fan_command_pct) * 255.0;  
}

//------------------------------------------------------------------------------
// Look for anomalies and set fault conditions
// TODO: Read fan feedback to check for stuck fan
void task_diagnostics() {
  g_diag_inversion_fault = 0;
  g_diag_gradient_fault = 0;
  g_diag_overheat_fault = 0;

  // IF the cabinet air is colder than the intake air and it's above the setpoint, 
  // then we cannot possibly cool the cabinet.
  if (g_intake_air_temperature_degc > g_in_cabinet_temperature_degc) {
    if (g_in_cabinet_temperature_degc > k_in_cabinet_temperature_set_point_degc) {
      g_diag_inversion_fault = 1;
    } else {
      // Is not a problem because we don't need to cool the cabinet
    }
  }
  
  // IF the difference between the intake air and the cabinet air is "low"
  // AND the cabinet temperature is "high" then we're in danger of not being able
  // to cool down the cabinet
  if ((g_in_cabinet_temperature_degc > g_intake_air_temperature_degc)
      && (abs(g_in_cabinet_temperature_degc - g_intake_air_temperature_degc) < k_min_temp_difference_gradient_fault_detect_degc)
      && (g_in_cabinet_temperature_degc > k_min_temp_thresh_gradient_fault_detect_degc)) {
    g_diag_gradient_fault = 1;    
  }
  
  // IF we're over the maximum temperature, we have an overheat fault
  if (g_in_cabinet_temperature_degc > k_min_temp_thresh_overheat_fault_detect_degc) {
    g_diag_overheat_fault = 1;
  }
}

//------------------------------------------------------------------------------
void task_output() {
  task_output_fan();
  task_output_buzzer();
}

//------------------------------------------------------------------------------
void task_output_fan() {
  if (g_fan_pwm_dc < k_min_fan_cutoff_dc) {
    g_fan_pwm_dc = 0;
  }
  
  analogWrite(PWM_FAN, g_fan_pwm_dc);
}

//------------------------------------------------------------------------------
void task_output_buzzer() {
  static unsigned long then = 0;
  static unsigned long buzzer_on_millis = 0;
  static int buzzer_command_prev;
  unsigned long now = millis();
  unsigned long elapsed = now - then;
  
  if (now < then) {
    elapsed = then - now;
  }
  
  if (g_diag_inversion_fault) {
    g_buzzer_command = BUZZER_COMMAND_ALARM;
  } else {
    g_buzzer_command = BUZZER_COMMAND_OFF;
  }

  if (g_buzzer_command == BUZZER_COMMAND_ALARM)  {
    if (buzzer_command_prev != g_buzzer_command) {
      // New to this state, initialize
      buzzer_on_millis = 0;
      g_buzzer_state = BUZZER_STATE_OFF;
    }
    
    if (g_buzzer_state == BUZZER_STATE_ON) {
      // TODO: Make a cal
      if (buzzer_on_millis > 1000) {
        g_buzzer_state = BUZZER_STATE_OFF;
      }
    } else {
      if (buzzer_on_millis > 1000) {
        g_buzzer_state = BUZZER_STATE_ON;
      }    
    }
    
    buzzer_on_millis += elapsed;
    
    if (g_buzzer_state == BUZZER_STATE_ON) {
      tone(DIO_BUZZER, NOTE_C4, 500);
    }
  } else if (g_buzzer_command == BUZZER_COMMAND_OFF)  {
    noTone(DIO_BUZZER);
    buzzer_on_millis = 0;
    g_buzzer_state = BUZZER_STATE_OFF;
  } else {
    noTone(DIO_BUZZER);  
    buzzer_on_millis = 0;
    g_buzzer_state = BUZZER_STATE_OFF;
  }
  
  buzzer_command_prev = g_buzzer_command;
  then = now;
}

//------------------------------------------------------------------------------
void print_state() {
  Serial.println("");
  
  Serial.print("Set Point (degC):       ");
  Serial.println(k_in_cabinet_temperature_set_point_degc);
  
  Serial.print("Cabinet Temp (degC):    ");
  Serial.println(g_in_cabinet_temperature_degc); 
   
  Serial.print("Intake Temp (degC):     ");
  Serial.println(g_intake_air_temperature_degc); 
   
  Serial.print("Commanded Pct Raw:      ");
  Serial.println(g_fan_command_pct_raw);
    
  Serial.print("Commanded Pct Filtered: ");
  Serial.println(g_fan_command_pct);
  
  Serial.print("Commanded DC:           ");
  Serial.println(g_fan_pwm_dc);
  
  Serial.print("FAULT Overheat:         ");
  Serial.println(g_diag_overheat_fault);
  
  Serial.print("FAULT Gradient:         ");
  Serial.println(g_diag_gradient_fault);
  
  Serial.print("FAULT Inversion:        ");
  Serial.println(g_diag_inversion_fault);
}

//------------------------------------------------------------------------------
void loop() { 
  unsigned long start_time = millis();
  
  task_input();
  task_fan_control();
  task_diagnostics();
  task_output();
  
  print_state();
  
  // Manage loop frequency
  unsigned long end_time = millis();
  unsigned long delay_ms = loop_time_ms;
  unsigned long elapsed_ms = end_time - start_time;
      
  if (end_time < start_time) {
    // Uptime counter overflow (will happen every ~50 days)
    // Just delay nominal value
    delay_ms = loop_time_ms;
  } else {
    delay_ms = max(loop_time_ms - elapsed_ms, 0);
  }
  
  delay(delay_ms);   
} 
