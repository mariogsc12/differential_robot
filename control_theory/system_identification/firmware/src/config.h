// -------- PINOUTS CONFIGURATION -----------

#define ENCODER_PPR 35*56

// L298N H-Bridge Connection PINs
// ---- right motor -----
#define L298N_enA 4  // PWM
#define L298N_in1 5  // Dir Motor A
#define L298N_in2 6 // Dir Motor A

// ---- left motor -----
#define L298N_enB 19  // PWM
#define L298N_in4 20  // Dir Motor B
#define L298N_in3 21  // Dir Motor B

// Wheel Encoders Connection PINs
// ---- right motor -----
#define right_encoder_phaseA 7  // Interrupt 
#define right_encoder_phaseB 18  

// ---- left motor -----
#define left_encoder_phaseA 1   // Interrupt
#define left_encoder_phaseB 2