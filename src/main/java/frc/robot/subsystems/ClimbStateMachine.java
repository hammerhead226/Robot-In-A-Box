package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

public class ClimbStateMachine {
  public enum CLIMB_STATES {
    DEPLOY,
    WINCH,
  }

  private CLIMB_STATES targetState = CLIMB_STATES.DEPLOY;

  public CLIMB_STATES getTargetState() {
    return targetState;
  }

  public void setClimbState(CLIMB_STATES state) {
    targetState = state;
  }

  public void advanceTargetState() {
    switch (targetState) {
      case DEPLOY:
        targetState = CLIMB_STATES.WINCH;
        break;
      case WINCH:
        break;
    }
    Logger.recordOutput("Climb Target State", targetState);
  }
}
