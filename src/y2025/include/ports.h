#pragma once

struct ports {
  struct driver_ {
    static constexpr int kXbox_DSPort = 0;
  };

  struct operator_ {
    static constexpr int kXbox_DSPort = 1;
  };

  struct drivetrain_ {
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

  struct ictest_ {
    static constexpr int kMotor_CANID = 17;
  };
};
