package frc.robot.constants;

public class RobotMap {

  public static class BrakeSwitchIDs {
    public static final int brakeSwitchChannel = 2;
  }

  public static class CoralScorerArmIDs {
    public static final int coralScorerRotationID = 10; // Is Arm Rotation in Phoenix Tuner.
    public static final int coralScorerRotationCANcoderID = 19;
    public static final int coralScorerFlywheelID = 11;
    public static final int coralScorerCANrangeID = 15;
  }

  public static class ElevatorIDs {
    public static final int leftElevatorID = 8;
    public static final int rightElevatorID = 9;
    public static final int elevatorCANrangeID = 18;
  }

  public static class WinchIDs {
    public static final int leftWinchID = 12;
    public static final int rightWinchID = 13;
  }

  public static class ClimbIDs {
    public static final int deployClimbID = 14;
    public static final int deployClimbCANcoderID = 16;
  }

  public static class ledIDs {
    public static final int CANdleID = 0;
  }

  // Add examples of vars
  public static class exampleIDs {
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 0;
  }
}
