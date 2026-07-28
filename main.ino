// We have to load a library --> Arduino IDE → Sketch → Include Library → Manage Libraries (or Tools → Manage Libraries, depending on your IDE version) → search "IRremote" → install the one by shirriff / z3t0 / Armin Joachimsmeyer

#include <AccelStepper.h>
#include <IRremote.h> //for remote control

//-------------------------------
//you can change these
const int ir_receive_pin = 13;
const int force_pin = A0;
const int led_pin = 2;
const int force_threshold = 500;
const int tray_degs = 360;
const int door_degs = 360;

const unsigned long DOOR_REMOTE_CODE = 0xFF906F; //establishes which button to use (up button)
//---------------------------------
class SpoolAssembly {
  private:
    static const int STEPS_PER_REV = 4096;
    AccelStepper stepper;
    int displacement = 0;

    int button_pin;
    unsigned long last_change = 0;
    bool last_state = HIGH;      // raw, possibly-bouncy reading from the last loop
    bool stable_state = HIGH;    // debounced state we've actually committed to
    const unsigned long debounce_delay = 50;

  public:
    SpoolAssembly(int in1, int in2, int in3, int in4, int btn_pin)
      : stepper(AccelStepper::HALF4WIRE, in1, in3, in2, in4), button_pin(btn_pin) {
      stepper.setMaxSpeed(500);       // steps/sec — tune to taste
      stepper.setAcceleration(200);   // steps/sec^2
      pinMode(button_pin, INPUT_PULLUP);
    }

    // positive = CCW, negative = CW — non-blocking, call run() in loop() to actually move
    int rotate(int degs) {
      int steps = (int)((long)degs * STEPS_PER_REV / 360);
      stepper.move(steps);   // sets a relative target; run() will step toward it over time
      displacement += degs;
      return steps;
    }

    int return_to_zero() {
      return rotate(-displacement);
    }

    int get_displacement() {
      return displacement;
    }

    int toggle(int degs) {
      if (abs(displacement) > 0) {
        return return_to_zero();
      } else {
        return rotate(degs);
      }
    }

    // must be called every loop() iteration — takes one step toward target if it's time to
    void run() {
      stepper.run();
    }

    // edge-triggered: returns true exactly once per debounced press (HIGH -> LOW transition)
    bool button_pressed() {
      bool reading = digitalRead(button_pin);

      if (reading != last_state) {
        last_change = millis();   // reading changed (possibly bounce) — restart the timer
      }
      last_state = reading;

      bool pressed_edge = false;
      if ((millis() - last_change) > debounce_delay && reading != stable_state) {
        stable_state = reading;          // commit the new debounced state
        if (reading == LOW) pressed_edge = true;
      }
      return pressed_edge;
    }
};
//Declaring tray and door objects
SpoolAssembly tray(8, 9, 10, 11, 3);
SpoolAssembly door(5, 6, 7, 12, 4);

void setup() {
  Serial.begin(9600);
  pinMode(led_pin, OUTPUT);
  IrReceiver.begin(ir_receive_pin, ENABLE_LED_FEEDBACK);
}

void loop() {
  //turns on LED if there is mail
  if (detect_mail()) digitalWrite(led_pin, HIGH);
  else digitalWrite(led_pin, LOW);

  //uses button to toggle door
  if (tray.button_pressed()) tray.toggle(tray_degs);
  if (door.button_pressed()) door.toggle(door_degs);

  handleDoorRemote(); // constantly checks if the remote button has been pressed or not
  
  // must run every iteration, unconditionally, so each motor keeps stepping toward its target
  tray.run();
  door.run();
}

bool detect_mail() {
  if (analogRead(force_pin) > force_threshold) return true;
  return false;
}

void handleDoorRemote() {
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.decodedRawData == DOOR_REMOTE_CODE) {
      door.toggle(door_degs);
    }
    IrReceiver.resume();
  }
}
