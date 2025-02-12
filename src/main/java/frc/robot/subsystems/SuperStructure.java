package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import org.littletonrobotics.junction.Logger;

public class SuperStructure  {

  private final Elevator elevator;
  private final CoralScorerArm csArm;
  private final CoralScorerFlywheel csFlywheel;
  private final Drive drive;
  private final LED led;
  // private SuperStructureState currentState;
  private SuperStructureState wantedState;
  private SuperStructureState lastWantedState;
  public boolean override = false;
  private SuperStructureState requestedState;
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
    lastWantedState = SuperStructureState.STOW;
    wantedState = SuperStructureState.STOW;
    requestedState = SuperStructureState.STOW;
  }

  public void setWantedState(SuperStructureState wantedState) {
    this.wantedState = wantedState;
    requestedState = wantedState;
    // this.currentState = wantedState;
  }

  public void checkSpeed() {
    if (Drive.speedX > 2 || Drive.speedY > 2 || Drive.rotationDegs > 50) {
      this.wantedState = SuperStructureState.STOW;
    }
    else  {

     this.wantedState = requestedState;
     }
  }

  public boolean shouldTrigger() {
    return wantedState != lastWantedState;
  }

  public SuperStructureState getState() {
    return wantedState;
  }

  public SequentialCommandGroup getSuperStructureCommand() {
    lastWantedState = wantedState;
    switch (wantedState) {
      case STOW:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(0, 0),
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
        if (elevator.atGoal() && csArm.atGoal(2)) {
          setWantedState(SuperStructureState.L1ATGOAL);
        }
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 0.1),
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
        if (csFlywheel.seesCoral() == CoralState.SENSOR
            || csFlywheel.seesCoral() == CoralState.CURRENT) {
          return new SequentialCommandGroup(csFlywheel.runVelocityCommand(300));
        } else {
          return new SequentialCommandGroup(
              new WaitCommand(0.5),
              new InstantCommand(() -> setWantedState(SuperStructureState.STOW)));
        }

      default:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(elevator.setElevatorTarget(0, 0), csArm.setArmTarget(40, 0)));
    }
  }

  
}
