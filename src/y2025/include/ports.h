#pragma once

struct ports {
  struct driver_ {
    static constexpr int kXbox_DSPort = 0;
  };

  struct operator_ {
    static constexpr int kXbox_DSPort = 1;
  };

  struct drivetrain_ {  // TODO: change all ports (preferably, when setting
                        // them, make them ascending in this order)
    static constexpr int kFRDrive_CANID = 2;
    static constexpr int kFLDrive_CANID = 5;
    static constexpr int kBLDrive_CANID = 8;
    static constexpr int kBRDrive_CANID = 11;

    static constexpr int kFLSteer_CANID = 7;
    static constexpr int kFRSteer_CANID = 4;
    static constexpr int kBLSteer_CANID = 10;
    static constexpr int kBRSteer_CANID = 13;

    static constexpr int kFRCANCoder_CANID = 3;
    static constexpr int kFLCANCoder_CANID = 6;
    static constexpr int kBLCANCoder_CANID = 9;
    static constexpr int kBRCANCoder_CANID = 12;
  };

  struct leds_ {
    static constexpr int kLEDStrip1 = 6;
  };

  struct coral_ss_ {
    struct telescope_ {
      static constexpr int kTelescope_CANID = 15;
    };
    struct wrist_ {
      static constexpr int kWristMotor_CANID = 16;
      static constexpr int kWristCANCoder_CANID = 17;
    };
    struct end_effector_ {
      static constexpr int kEE_CANID = 18;
    };
  };

  struct algal_ss_ {
    struct elevator_ {
      static constexpr int kElevator_CANID = 20;
    };
    struct wrist_ {
      static constexpr int kWristMotor_CANID = 21;
      static constexpr int kWristCANCoder_CANID = 22;
    };
    struct end_effector_ {
      static constexpr int kEE1_CANID = 23;
      static constexpr int kEE2_CANID = 24;
    };
  };

  struct climber_ {
    static constexpr int kClimber_CANID = 26;
  };
};
