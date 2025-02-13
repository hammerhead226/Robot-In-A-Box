package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerArm.ScoralArmState;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
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
  }

  public void goToSource() {
    elevator.setWantedState(ElevatorState.SOURCE);
    csArm.setWantedState(ScoralArmState.SOURCE);
  }

  public void gotoFirstLevel() {
    elevator.setWantedState(ElevatorState.L1);
    csArm.setWantedState(ScoralArmState.L1);
  }

  public void gotoSecondLevel() {
    elevator.setWantedState(ElevatorState.L2);
    csArm.setWantedState(ScoralArmState.L2);
  }

  public void gotoThirdLevel() {
    elevator.setWantedState(ElevatorState.L3);
    csArm.setWantedState(ScoralArmState.L3);
  }

  public void gotoFourthLevel() {
    elevator.setWantedState(ElevatorState.L4);
    csArm.setWantedState(ScoralArmState.L4);
  }

  public void gotoProcessorLevel() {
    elevator.setWantedState(ElevatorState.PROCESSOR);
    csArm.setWantedState(ScoralArmState.PROCESSOR);
  }

  public void applyState() {
    switch (currentState) {
      case STOW:
        Stow();
        break;
      case SOURCE:
        goToSource();
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
      default:
        Stow();
    }
  }

  /*  public SequentialCommandGroup getSuperStructureCommand() {
      lastWantedState = wantedState;
      switch (wantedState) {
        case STOW:
          return new SequentialCommandGroup(
              new ParallelCommandGroup(
                  elevator.setElevatorTarget(0, 5),
                  csArm.setArmTarget(40, 0),
                  csFlywheel.stopCommand(),
                  led.setStateCommand(LED_STATE.BLUE)));

        case L1:
          if (elevator.atGoal() && csArm.atGoal(2)) {
            setWantedState(SuperStructureState.L1ATGOAL);
          }
          return new SequentialCommandGroup(
              new ParallelCommandGroup(
                  elevator.setElevatorTarget(FieldConstants.ReefHeight.L1.height, 0.1),
                  csArm.setArmTarget(FieldConstants.ReefHeight.L1.pitch, 2),
                  led.setStateCommand(LED_STATE.FLASHING_GREEN)));
          //  csFlywheel.runVoltsCommmand(12),
          // new WaitCommand(1),
          // led.setStateCommand(LED_STATE.GREEN)

        case L1ATGOAL:
          return new SequentialCommandGroup(led.setStateCommand(LED_STATE.GREEN));

        case L2:
          if (elevator.atGoal() && csArm.atGoal(2)) {
            setWantedState(SuperStructureState.L1ATGOAL);
          }
          return new SequentialCommandGroup(
              new ParallelCommandGroup(
                  elevator.setElevatorTarget(FieldConstants.ReefHeight.L2.height, 0.1),
                  csArm.setArmTarget(FieldConstants.ReefHeight.L2.pitch, 2)));
        case L3:
          if (elevator.atGoal() && csArm.atGoal(2)) {
            setWantedState(SuperStructureState.L1ATGOAL);
          }
          return new SequentialCommandGroup(
              new ParallelCommandGroup(
                  elevator.setElevatorTarget(FieldConstants.ReefHeight.L3.height, 0.1),
                  csArm.setArmTarget(FieldConstants.ReefHeight.L3.pitch, 2)));
        case L4:
          if (elevator.atGoal() && csArm.atGoal(0.2)) {
            setWantedState(SuperStructureState.L1ATGOAL);
          }
          return new SequentialCommandGroup(
              new ParallelCommandGroup(
                  elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 7),
                  csArm.setArmTarget(FieldConstants.ReefHeight.L4.pitch, 2)));

        case SOURCE:
          if (csFlywheel.seesCoral() == CoralState.CURRENT
              || csFlywheel.seesCoral() == CoralState.SENSOR) {
            return new SequentialCommandGroup(
                new WaitCommand(0.5),
                new InstantCommand(() -> setWantedState(SuperStructureState.STOW)));

          } else {
            return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.setElevatorTarget(1, 0.1),
                    csArm.setArmTarget(FieldConstants.ReefHeight.L2.pitch, 2)),
                csFlywheel.runVelocityCommand(200));
          }

        case SCORING_CORAL:
          //if (csFlywheel.seesCoral() == CoralState.SENSOR
             // || csFlywheel.seesCoral() == CoralState.CURRENT) {
            return new SequentialCommandGroup(csFlywheel.runVoltsCommmand(1));
         // } else {
         //   return new SequentialCommandGroup(
        //        new WaitCommand(0.5),
         //       new InstantCommand(() -> setWantedState(SuperStructureState.STOW)));
         // }

        default:
          return new SequentialCommandGroup(
              new ParallelCommandGroup(elevator.setElevatorTarget(0, 0), csArm.setArmTarget(40, 0)));
      }
    }
  */
  @Override
  public void periodic() {
    // Logger.recordOutput("current State", currentState)
    if (Drive.speedX > 1 || Drive.speedY > 1 || Drive.rotationDegs > 30) {
      currentState = SuperStructureState.STOW;
    } else if (wantedState != currentState) {
      currentState = wantedState;
    }
    
    applyState();
  }
}
