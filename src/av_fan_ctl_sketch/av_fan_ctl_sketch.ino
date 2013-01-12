/*-----------------------------------------------------------------------------
 * A/V Cabinet Fan Controller
 * Christopher Myers 2012
 */

#include <EEPROM.h>
#include "pitches.h"

// Defines ---------------------------------------------------------------------
// Pin definitions
#define DIO_RELAY_CONTROL    3
#define DIO_BUZZER           12
#define DIO_RPM_FEEDBACK     11
#define PWM_FAN              6
#define AN_CABINET_TEMP      2
#define AN_INTAKE_TEMP       0

// Relay Control
#define RELAY_OFF  0
#define RELAY_ON   1

#define RELAY_OVERRIDE_OFF       RELAY_OFF
#define RELAY_OVERRIDE_ON        RELAY_ON
#define RELAY_OVERRIDE_DISABLED  2

// Utility
#define DEGF_TO_DEGC(x) ((x - 32) * 5/9)

// EEPROM
#define EE_COOKIE        "v1"
#define EE_DATA_ADDR    0

// Buzzer Driver
#define BUZZER_COMMAND_OFF    0
#define BUZZER_COMMAND_ALARM  1

#define BUZZER_STATE_OFF 0
#define BUZZER_STATE_ON  1

// Globals ---------------------------------------------------------------------
// Fan Control
float g_fan_command_pct, g_fan_command_pct_prev;
float g_fan_command_pct_raw;
float g_fan_command_pct_i_state;
unsigned short int g_fan_pwm_dc = 0;

// Relay Control
unsigned char g_relay_command = RELAY_OFF;
unsigned char g_relay_override = RELAY_OVERRIDE_DISABLED;

// Inputs
float g_cabinet_temperature_degc;
float g_intake_temperature_degc;
unsigned short int g_cabinet_temperature_counts;
unsigned short int g_intake_temperature_counts;

// Feedback
unsigned long int g_rpm_feedback_pulse_width_us = 0L;

// Diagnostics
boolean g_diag_inversion_fault = 0;
boolean g_diag_gradient_fault = 0;
boolean g_diag_overheat_fault = 0;

// Buzzer
int g_buzzer_state = BUZZER_STATE_OFF;
int g_buzzer_command = BUZZER_COMMAND_OFF;

// EEPROM
boolean g_eeprom_was_valid = 0;

// Serial
boolean g_serial_print_state_mode = 1;


// Cals ------------------------------------------------------------------------
const float k_p = 0.10; // 10% of DC per degree C
const float k_i = 0.01;

// We're going to sample the cabinet temperature every 1000ms
#define K_CABINET_TEMPERATURE_DT  1000.0
// But want it to have a time constant of 2s
#define K_CABINET_TEMPERATURE_RC  2000.0
// alpha = dt / (RC + dt)
const float k_cabinet_temperature_alpha = (K_CABINET_TEMPERATURE_DT / (K_CABINET_TEMPERATURE_RC + K_CABINET_TEMPERATURE_DT)); 

// We're going to sample the intake temperature every 1000ms
#define K_INTAKE_TEMPERATURE_DT  1000.0
// But want it to have a time constant of 2s
#define K_INTAKE_TEMPERATURE_RC  2000.0
// alpha = dt / (RC + dt)
const float k_intake_temperature_alpha = (K_INTAKE_TEMPERATURE_DT / (K_INTAKE_TEMPERATURE_RC + K_INTAKE_TEMPERATURE_DT)); 

// We're going to update the fan duty cycle every 500ms
#define K_FAN_PCT_DT  500.0
// But want it to have a time constant of 2s
#define K_FAN_PCT_RC  2000.0
// Rate limit on the fan duty cycle
// alpha = dt / (RC + dt)
const float k_fan_command_pct_alpha = (K_FAN_PCT_DT / (K_FAN_PCT_RC + K_FAN_PCT_DT)); 
const float k_max_fan_command_pct_i_state = 0.8;
const float k_min_fan_command_pct_i_state = 0.0;
const int k_min_fan_cutoff_dc = 10;
// If the temperature gradient is only 2 degrees, we have a gradient condition
const float k_min_temp_difference_gradient_fault_detect_degc = 2.0;
// Above 90 degrees and we will have an overheating problem
const float k_min_temp_thresh_gradient_fault_detect_degc = DEGF_TO_DEGC(90);
// Above 100 degrees and we have an overheating problem
const float k_min_temp_thresh_overheat_fault_detect_degc = DEGF_TO_DEGC(100);

const unsigned int k_rpm_feedback_measure_timeout_us = 1000000L;

const float k_relay_on_threshold_dc_pct = 0.05;

// Desired loop time
const unsigned long k_loop_time_ms = 500L;

// Persistent -----------------------------------------------------------------
struct EEPROM_Data_tag {
  // This is for mere detection if they are your settings
  char cookie[4];
  // This cal determines the desired in-cabinet temperature
  float k_cabinet_temperature_set_point_degc;
} g_eeprom_data = {
  EE_COOKIE,
  DEGF_TO_DEGC(80)
};


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
  g_relay_command = RELAY_OFF;
  
  Serial.setTimeout(1000);
  
  // prints title with ending line break 
  Serial.println("A/V Cabinet Fan Controller v0.1"); 
  Serial.println("Christopher Myers - 2012"); 
  
  pinMode(DIO_RELAY_CONTROL, OUTPUT);
  pinMode(DIO_RPM_FEEDBACK, INPUT);
  pinMode(DIO_BUZZER, OUTPUT);
  pinMode(PWM_FAN, OUTPUT);
  
  loadConfigFromEEPROM();
  Serial.print("EEPROM contents valid: ");
  Serial.println(g_eeprom_was_valid 
    ? "yes"
    : "no"); 
  
  // Play startup sound
  tone(DIO_BUZZER, NOTE_C4, 250);
  delay(250);
  tone(DIO_BUZZER, NOTE_E4, 250);
  delay(250);
} 

//------------------------------------------------------------------------------
void loadConfigFromEEPROM() {
  g_eeprom_was_valid = 0;
  
  // Check the cookie to make sure settings are valid
  // If not, we'll just not overwrite the defaults in g_eeprom_data
  if (EEPROM.read(EE_DATA_ADDR + 0) == EE_COOKIE[0] &&
      EEPROM.read(EE_DATA_ADDR + 1) == EE_COOKIE[1] &&
      EEPROM.read(EE_DATA_ADDR + 2) == EE_COOKIE[2]) {
      for (unsigned int t = 0; t < sizeof(EEPROM_Data_tag); t++) {
        *((char*)&g_eeprom_data + t) = EEPROM.read(EE_DATA_ADDR + t);
      }
      
      g_eeprom_was_valid = 1;
    }
}

//------------------------------------------------------------------------------
void saveConfigToEEPROM() {
  for (unsigned int t = 0; t < sizeof(EEPROM_Data_tag); t++) {
    EEPROM.write(EE_DATA_ADDR + t, *((char*)&g_eeprom_data + t));
  }
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
float degf_to_degc(float deg_f) {
  return (deg_f - 32.0) * (5.0/9.0);
}

//------------------------------------------------------------------------------
float degc_to_degf(float deg_c) {
  return (deg_c * (9.0/5.0)) + 32.0;
}

//------------------------------------------------------------------------------
float filter_low_pass(float x_prev, float x, float a) {
  return (x * a) + (x_prev * (1.0 - a));
}

//------------------------------------------------------------------------------
void task_input() {
  static int cycle = 0;
  unsigned short temp_prev;
  
  // Alternate sample cycles
  if (cycle == 0) {
    g_cabinet_temperature_counts = analogRead(AN_CABINET_TEMP);  
    
    temp_prev = g_cabinet_temperature_degc;
    g_cabinet_temperature_degc = convert_tmp36_adc_to_degc(g_cabinet_temperature_counts);    
    g_cabinet_temperature_degc = filter_low_pass(temp_prev, g_cabinet_temperature_degc, k_cabinet_temperature_alpha);
    
    cycle = 1;
  } else {
    temp_prev = g_intake_temperature_degc;
    g_intake_temperature_counts = analogRead(AN_INTAKE_TEMP);  
    g_intake_temperature_degc = convert_tmp36_adc_to_degc(g_intake_temperature_counts);  
    g_intake_temperature_degc = filter_low_pass(temp_prev, g_intake_temperature_degc, k_intake_temperature_alpha);
    
    // Read the RPM feedback on the second cycle
    g_rpm_feedback_pulse_width_us = pulseIn(DIO_RPM_FEEDBACK, HIGH, k_rpm_feedback_measure_timeout_us);
    
    cycle = 0;
  }  
}

//------------------------------------------------------------------------------
void task_fan_control() {
  float temperature_error = g_cabinet_temperature_degc - g_eeprom_data.k_cabinet_temperature_set_point_degc;
  
  // PI Control
  float p_term = temperature_error * k_p;
  p_term = max(p_term, 0);
  float i_term = g_fan_command_pct_i_state + (temperature_error * k_i);
  
  // Store away previous so we can run a low-pass filter
  g_fan_command_pct_prev = g_fan_command_pct;
  
  // Saturation on the integrator
  i_term = min(i_term, k_max_fan_command_pct_i_state);
  i_term = max(i_term, k_min_fan_command_pct_i_state);
  g_fan_command_pct_i_state = i_term;
  
  Serial.println(temperature_error);
  Serial.println(p_term);
  Serial.println(i_term);
  
  // Keep from 0 to 1.0 (100%)
  g_fan_command_pct_raw = constrain(p_term + i_term, 0, 1);
  Serial.println(g_fan_command_pct_raw);
   
  // Filter the fan command
  g_fan_command_pct = filter_low_pass(g_fan_command_pct_prev, g_fan_command_pct_raw, k_fan_command_pct_alpha);
  
  // Apply fan commanded duty cycle
  g_fan_pwm_dc = (g_fan_command_pct) * 255.0;  
}

//------------------------------------------------------------------------------
// This feature determines whether the relay needs to be on.  
void task_relay_control() {
  // If the fan duty cycle is > 0, then we obviously need the fan on.
  if (g_fan_command_pct > k_relay_on_threshold_dc_pct) {
    g_relay_command = RELAY_ON;
  } else {
    g_relay_command = RELAY_OFF;
  }
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
  if (g_intake_temperature_degc > g_cabinet_temperature_degc) {
    if (g_cabinet_temperature_degc > g_eeprom_data.k_cabinet_temperature_set_point_degc) {
      g_diag_inversion_fault = 1;
    } else {
      // Is not a problem because we don't need to cool the cabinet
    }
  }
  
  // IF the difference between the intake air and the cabinet air is "low"
  // AND the cabinet temperature is "high" then we're in danger of not being able
  // to cool down the cabinet
  if ((g_cabinet_temperature_degc > g_intake_temperature_degc)
      && (abs(g_cabinet_temperature_degc - g_intake_temperature_degc) < k_min_temp_difference_gradient_fault_detect_degc)
      && (g_cabinet_temperature_degc > k_min_temp_thresh_gradient_fault_detect_degc)) {
    g_diag_gradient_fault = 1;    
  }
  
  // IF we're over the maximum temperature, we have an overheat fault
  if (g_cabinet_temperature_degc > k_min_temp_thresh_overheat_fault_detect_degc) {
    g_diag_overheat_fault = 1;
  }
}

//------------------------------------------------------------------------------
void task_serial_interaction() {
  char buffer[64] = {0};
  
  if (Serial.available()) {
    Serial.readBytesUntil('\n', buffer, sizeof(buffer));
    Serial.print("### Command Received: ");
    Serial.println(buffer);
    
    if (strncmp(":q=on", buffer, 5) == 0) {
      g_serial_print_state_mode = 1;
    } else if (strncmp(":q=off", buffer, 6) == 0) {
      g_serial_print_state_mode = 0;
    }
    
    // Change Set Point
    else if (strncmp(":sp=", buffer, 4) == 0) {
      // Turn off spewing
      g_serial_print_state_mode = 0;
      
      float new_set_point = atof(buffer + 4);
      if (new_set_point > 0 && new_set_point < 200) {
        g_eeprom_data.k_cabinet_temperature_set_point_degc = degf_to_degc(new_set_point);
        Serial.print("### New set point accepted: ");
        Serial.println(g_eeprom_data.k_cabinet_temperature_set_point_degc);
      } else {
        Serial.println("### New set point rejected!");
      }
    }
    
    // Save data to EEPROM
    else if (strncmp(":w", buffer, 2) == 0) {
      // Turn off spewing
      g_serial_print_state_mode = 0;
      
      saveConfigToEEPROM();
      Serial.println("### Data written to EEPROM");      
    }
    
    // Command output
    else if (strncmp(":o.relay", buffer, 7) == 0) {
      // Turn off spewing
      g_serial_print_state_mode = 0;
      
      if (strncmp("=on", buffer + 8, 3) == 0) {
        g_relay_override = RELAY_OVERRIDE_ON;
      } else if (strncmp("=off", buffer + 8, 4) == 0) {
        g_relay_override = RELAY_OVERRIDE_OFF;
      } else {
        g_relay_override = RELAY_OVERRIDE_DISABLED;
      }
      
      Serial.print("### New relay override state: "); 
      Serial.println(g_relay_override);     
    }
  }  
  
  if (0 != g_serial_print_state_mode) {
    print_state();
  }
}

//------------------------------------------------------------------------------
void task_output() {
  task_output_fan();
  task_output_buzzer();
  task_output_relay();
}

//------------------------------------------------------------------------------
void task_output_fan() {
  if (g_fan_pwm_dc < k_min_fan_cutoff_dc) {
    g_fan_pwm_dc = 0;
  }
  
  analogWrite(PWM_FAN, g_fan_pwm_dc);
}

//------------------------------------------------------------------------------
void task_output_relay() {
  unsigned char relay_state;
  
  if (g_relay_override != RELAY_OVERRIDE_DISABLED) {
    relay_state = g_relay_override;
  } else {
    relay_state = g_relay_command;
  }
  
  digitalWrite(DIO_RELAY_CONTROL, relay_state == RELAY_ON ? HIGH : LOW);
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
  
  // TODO:
  if (g_diag_gradient_fault || g_diag_overheat_fault) {
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
  Serial.println(g_eeprom_data.k_cabinet_temperature_set_point_degc);
  
  Serial.print("Cabinet Temp (degC):    ");
  Serial.println(g_cabinet_temperature_degc); 
   
  Serial.print("Cabinet Temp (counts):  ");
  Serial.println(g_cabinet_temperature_counts); 
   
  Serial.print("Intake Temp (degC):     ");
  Serial.println(g_intake_temperature_degc); 
   
  Serial.print("Intake Temp (counts):   ");
  Serial.println(g_intake_temperature_counts); 
   
  Serial.print("Commanded Pct:          ");
  Serial.println(g_fan_command_pct);
    
  Serial.print("Commanded Relay:        ");
  Serial.println(g_relay_command);
  
  Serial.print("Commanded DC:           ");
  Serial.println(g_fan_pwm_dc);
  
  Serial.print("Pulse Width:            ");
  Serial.println(g_rpm_feedback_pulse_width_us);  
  
  Serial.print("FAULT Overheat:         ");
  Serial.println(g_diag_overheat_fault);
  
  Serial.print("FAULT Gradient:         ");
  Serial.println(g_diag_gradient_fault);
  
  Serial.print("FAULT Inversion:        ");
  Serial.println(g_diag_inversion_fault);
}

//------------------------------------------------------------------------------
void loop() { 
  static unsigned long last_delay_ms = 1000L;
  unsigned long start_time = millis();

  task_input();
  task_fan_control();
  task_relay_control();
  task_diagnostics();
  task_output();
  
  task_serial_interaction();

  // Manage loop frequency
  unsigned long end_time = millis();
  unsigned long delay_ms = k_loop_time_ms;
  unsigned long elapsed_ms = end_time - start_time;
      
  if (end_time < start_time) {
    // Uptime counter overflow (will happen every ~50 days)
    // Just delay nominal value
    delay_ms = last_delay_ms;
  } else {
    if (k_loop_time_ms < elapsed_ms) {
      delay_ms = 0L;
    } else {
      delay_ms = k_loop_time_ms - elapsed_ms;
    }
  }
  
  last_delay_ms = delay_ms;
  
  delay(delay_ms);   
} 
