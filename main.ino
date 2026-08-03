#include <AccelStepper.h>
#include <IRremote.h>


//-------------------------------
// Pins and settings
//-------------------------------

const int ir_receive_pin = 10;
const int force_pin = A0;
const int led_pin = 2;
const int buzzer_pin = 3;

const int force_threshold = 5;
const int door_degs = 900;
unsigned long last_remote_press = 0;
const unsigned long remote_delay = 500;

// Remote button command
const byte DOOR_REMOTE_COMMAND = 0x18;


//---------------------------------


class SpoolAssembly {
  private:
    static const int STEPS_PER_REV = 4096;

    AccelStepper stepper;
    int displacement = 0;

    int button_pin;

    unsigned long last_change = 0;
    bool last_state = HIGH;
    bool stable_state = HIGH;

    const unsigned long debounce_delay = 50;


  public:

    SpoolAssembly(int in1, int in2, int in3, int in4, int btn_pin)
      : stepper(
          AccelStepper::HALF4WIRE,
          in1,
          in3,
          in2,
          in4
        ),
        button_pin(btn_pin) {

      stepper.setMaxSpeed(3000);
      stepper.setAcceleration(200);

      pinMode(button_pin, INPUT_PULLUP);
    }


    int rotate(int degs) {

      int steps =
        (int)((long)degs * STEPS_PER_REV / 360);

      stepper.move(steps);

      displacement += degs;

      return steps;
    }


    int return_to_zero() {

      return rotate(-displacement);
    }


    int toggle(int degs) {

      if (abs(displacement) > 0) {

        return return_to_zero();

      } else {

        return rotate(degs);
      }
    }


    void run() {

      stepper.run();
    }


    bool button_pressed() {

      bool reading = digitalRead(button_pin);

      if (reading != last_state) {

        last_change = millis();
      }

      last_state = reading;

      bool pressed_edge = false;

      if (
        (millis() - last_change) > debounce_delay &&
        reading != stable_state
      ) {

        stable_state = reading;

        if (reading == LOW) {

          pressed_edge = true;
        }
      }

      return pressed_edge;
    }
};


// Door motor pins: 5, 6, 7, 12
// Physical button: pin 4

SpoolAssembly door(5, 6, 7, 8, 4);


void setup() {

  Serial.begin(9600);

  pinMode(led_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);

  digitalWrite(led_pin, LOW);
  noTone(buzzer_pin);

  IrReceiver.begin(
    ir_receive_pin,
    ENABLE_LED_FEEDBACK
  );

  Serial.println("Mail Buddy ready");
}


void loop() {

  bool mail_detected = detect_mail();


  // -------------------------------
  // MAIL SENSOR
  // -------------------------------

  if (mail_detected) {

    digitalWrite(led_pin, HIGH);

    tone(buzzer_pin, 2000);

  } else {

    digitalWrite(led_pin, LOW);

    noTone(buzzer_pin);
  }


  // -------------------------------
  // PHYSICAL BUTTON
  // -------------------------------

  if (door.button_pressed()) {

    Serial.println("Physical button pressed");

    door.toggle(door_degs);
  }


  // -------------------------------
  // REMOTE BUTTON
  // -------------------------------

  handleDoorRemote();


  // Motor must run every loop
  door.run();
}


//---------------------------------
// Mail detection
//---------------------------------

bool detect_mail() {

  return analogRead(force_pin) > force_threshold;
}


//---------------------------------
// Remote control
//---------------------------------

void handleDoorRemote() {

  if (IrReceiver.decode()) {

    byte command =
      IrReceiver.decodedIRData.command;


    Serial.print("Remote command: 0x");
    Serial.println(command, HEX);


    if (command == DOOR_REMOTE_COMMAND) {

      if (millis() - last_remote_press > remote_delay) {

        Serial.println("Remote button detected!");
        Serial.println("Toggling door motor...");

        door.toggle(door_degs);

        last_remote_press = millis();

      } else {

        Serial.println("Ignoring repeated IR signal");
      }


    } else {

      Serial.println("Unknown remote button");
    }


    IrReceiver.resume();
  }
}
