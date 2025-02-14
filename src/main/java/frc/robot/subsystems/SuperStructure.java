package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerArm.ScoralArmState;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel.ScoralFlywheelState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.led.LED;

public class SuperStructure extends SubsystemBase {

  private final Elevator elevator;
  private final CoralScorerArm csArm;
  private final CoralScorerFlywheel csFlywheel;
  private final Drive drive;
  private final LED led;
  // private SuperStructureState currentState;
  private SuperStructureState currentState;
  private SuperStructureState lastState;
  public boolean override = false;
  private SuperStructureState wantedState;
  public boolean successfullScore = false;

  public SuperStructure(
      Elevator elevator,
      CoralScorerArm csArm,
      CoralScorerFlywheel csFlywheel,
      Drive drive,
      LED led) {
    this.elevator = elevator;
    this.csArm = csArm;
    this.csFlywheel = csFlywheel;
    this.drive = drive;
    this.led = led;
    lastState = SuperStructureState.STOW;
    currentState = SuperStructureState.STOW;
    wantedState = SuperStructureState.STOW;
  }

  public void setWantedState(SuperStructureState wantedState) {
    this.wantedState = wantedState;
    // requestedState = wantedState;
    // this.currentState = wantedState;
  }

  public SuperStructureState getState() {
    return currentState;
  }

  public void Stow() {
    elevator.setWantedState(ElevatorState.STOW);
    csArm.setWantedState(ScoralArmState.STOW);
    csFlywheel.setWantedState(ScoralFlywheelState.ZERO);
  }

  public void goToSource() {
    elevator.setWantedState(ElevatorState.SOURCE);
    csArm.setWantedState(ScoralArmState.SOURCE);
    csFlywheel.setWantedState(ScoralFlywheelState.INTAKING_CORAL);
  }

  public void gotoFirstLevel() {
    elevator.setWantedState(ElevatorState.L1);
    csArm.setWantedState(ScoralArmState.L1);
    csFlywheel.setWantedState(ScoralFlywheelState.ZERO);
    led.setState(LED_STATE.FLASHING_YELLOW);
  }

  public void gotoSecondLevel() {
    elevator.setWantedState(ElevatorState.L2);
    csArm.setWantedState(ScoralArmState.L2);
    csFlywheel.setWantedState(ScoralFlywheelState.ZERO);
    led.setState(LED_STATE.FLASHING_YELLOW);
  }

  public void gotoThirdLevel() {
    elevator.setWantedState(ElevatorState.L3);
    csArm.setWantedState(ScoralArmState.L3);
    csFlywheel.setWantedState(ScoralFlywheelState.ZERO);
    led.setState(LED_STATE.FLASHING_YELLOW);
  }

  public void gotoFourthLevel() {
    elevator.setWantedState(ElevatorState.L4);
    csArm.setWantedState(ScoralArmState.L4);
    csFlywheel.setWantedState(ScoralFlywheelState.ZERO);
    led.setState(LED_STATE.FLASHING_YELLOW);
  }

  public void gotoProcessorLevel() {
    elevator.setWantedState(ElevatorState.PROCESSOR);
    csArm.setWantedState(ScoralArmState.PROCESSOR);
    csFlywheel.setWantedState(ScoralFlywheelState.ZERO);
    led.setState(LED_STATE.FLASHING_YELLOW);
  }

  public void score() {
    csFlywheel.setWantedState(ScoralFlywheelState.SCORING_CORAL);
  }

  public void scored() {
    successfullScore = true;
  }

  public boolean hasScored() {
    return false;
  }

  public boolean subsystemsAtGoal() {

    return elevator.atGoal() || csArm.atGoal(2);
  }

  public boolean hasStructureReachedGoal() {

    switch (currentState) {
      case STOW:
        return elevator.hasReachedGoal(0) && csArm.hasReachedGoal(0);
      case L1:
        return elevator.hasReachedGoal(FieldConstants.ReefHeight.L1.height)
            && csArm.hasReachedGoal(FieldConstants.ReefHeight.L1.pitch);
      case L2:
        return elevator.hasReachedGoal(FieldConstants.ReefHeight.L2.height)
            && csArm.hasReachedGoal(FieldConstants.ReefHeight.L2.pitch);
      case L3:
        return elevator.hasReachedGoal(FieldConstants.ReefHeight.L3.height)
            && csArm.hasReachedGoal(FieldConstants.ReefHeight.L3.pitch);
      case L4:
        return elevator.hasReachedGoal(FieldConstants.ReefHeight.L4.height)
            && csArm.hasReachedGoal(FieldConstants.ReefHeight.L4.pitch);
      case SOURCE:
       return elevator.hasReachedGoal(0)
                        && csArm.hasReachedGoal(40);

      case SCORING_CORAL:
      return csFlywheel.seesCoral() == CoralState.CURRENT
      || csFlywheel.seesCoral() == CoralState.SENSOR;
      default:
        return false;
    }
  }

  public void applyState() {
    switch (currentState) {
      case STOW:
        Stow();

        break;
      case SOURCE:
         if ( csFlywheel.seesCoral() == CoralState.SENSOR) {
           Stow();
           led.setState(LED_STATE.GREEN);
         } else {
          goToSource();
          led.setState(LED_STATE.FLASHING_YELLOW);
        }
        break;
      case L1:
        gotoFirstLevel();

        break;
      case L2:
        gotoSecondLevel();

        break;
      case L3:
        gotoThirdLevel();

        break;
      case L4:
        gotoFourthLevel();

        break;
      case PROCESSOR:
        gotoProcessorLevel();
        break;
      case SCORING_CORAL:
        // score();
        if (csFlywheel.seesCoral() == CoralState.CURRENT
            || csFlywheel.seesCoral() == CoralState.SENSOR) {
          scored();

        } else {
          score();
          scored();
        }
        break;

      default:
        Stow();
        led.setState(LED_STATE.BLUE);
    }
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("current State", currentState)
    if (Drive.speedX > 1 || Drive.speedY > 1) {
      currentState = SuperStructureState.STOW;
    } else if (wantedState != currentState) {
      currentState = wantedState;
    }

    applyState();

    if (successfullScore) {
      setWantedState(SuperStructureState.STOW);
      successfullScore = false;
    }

    if (elevator.atGoal() && csArm.atGoal(2)) {
      led.setState(LED_STATE.GREEN);
    }
  }
}
