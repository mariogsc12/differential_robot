// -------- PINOUTS CONFIGURATION -----------

#define ENCODER_PPR 35*56

// L298N H-Bridge Connection PINs
// ---- right motor -----
#define L298N_enA 7  // PWM
#define L298N_in2 5  // Dir Motor A
#define L298N_in1 6  // Dir Motor A

// ---- left motor -----
#define L298N_enB 11  // PWM
#define L298N_in4 8  // Dir Motor B
#define L298N_in3 7  // Dir Motor B

// Wheel Encoders Connection PINs
// ---- right motor -----
#define right_encoder_phaseA 10  // Interrupt 
#define right_encoder_phaseB 9  

// ---- left motor -----
#define left_encoder_phaseA 2   // Interrupt
#define left_encoder_phaseB 4