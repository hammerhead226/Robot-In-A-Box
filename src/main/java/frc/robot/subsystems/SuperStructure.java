package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;

public class SuperStructure {

  private final Elevator elevator;
  private final CoralScorerArm csArm;
  private final CoralScorerFlywheel csFlywheel;
  private final Drive drive;
  private final LED led;
  private SuperStructureState currentState;
  private SuperStructureState wantedState;
  public boolean override = false;

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

    currentState = SuperStructureState.STOW;
    wantedState = SuperStructureState.STOW;
  }

  public void setWantedState(SuperStructureState desiredState) {

    this.wantedState = desiredState;
    this.currentState = wantedState;
  }

  public void checkSpeed() {
    if (Drive.speedX > 2 || Drive.speedY > 2 || Drive.rotationDegs > 50) {
      this.currentState = SuperStructureState.STOW;
    } else {

      currentState = wantedState;
    }
  }

  public SequentialCommandGroup getSuperStructureCommand() {

    switch (currentState) {
      case STOW:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(0, 0),
                csArm.setArmTarget(40, 0),
                csFlywheel.stopCommand()));

      case L1:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L1.height, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L1.pitch, 2)),
            csFlywheel.runVoltsCommmand(12));
      case L2:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L2.height, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L2.pitch, 2)),
            csFlywheel.runVoltsCommmand(12));
      case L3:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L3.height, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L3.pitch, 2)),
            csFlywheel.runVoltsCommmand(12));
      case L4:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L4.pitch, 2)),
            csFlywheel.runVoltsCommmand(12));

      case SOURCE:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(1, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L2.pitch, 2)),
            csFlywheel.runVoltsCommmand(12));
      default:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(elevator.setElevatorTarget(0, 0), csArm.setArmTarget(40, 0)));
    }
  }
}
