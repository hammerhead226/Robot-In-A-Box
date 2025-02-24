package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ToReefHeight;
import frc.robot.commands.GoToStow;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakingCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoringProccessorSequential;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.Winch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;
import org.littletonrobotics.junction.Logger;

public class SuperStructure {
  private final Drive drive;
  private final Elevator elevator;
  private final ScoralArm scoralArm;
  private final ScoralRollers scoralRollers;
  private final LED led;
  private final ClimberArm climberArm;
  private final Winch winch;
  private SuperStructureState currentState;
  private SuperStructureState wantedState;
  private SuperStructureState lastReefState;
  public boolean override = false;
  int counter = 0;

  public SuperStructure(
      Drive drive,
      Elevator elevator,
      ScoralArm scoralArm,
      ScoralRollers scoralRollers,
      LED led,
      ClimberArm climberArm, Winch winch) {
    this.drive = drive;
    this.elevator = elevator;
    this.scoralArm = scoralArm;
    this.scoralRollers = scoralRollers;
    this.led = led;
    this.climberArm = climberArm;
    this.winch = winch;
    wantedState = SuperStructureState.STOW;
    currentState = SuperStructureState.STOW;
    lastReefState = SuperStructureState.L4;
  }

  public void setWantedState(SuperStructureState wantedState) {
    if (wantedState == SuperStructureState.L1
        || wantedState == SuperStructureState.L2
        || wantedState == SuperStructureState.L3
        || wantedState == SuperStructureState.L4) {
      led.setState(LED_STATE.RED);
    } else if (wantedState == SuperStructureState.SOURCE) {
      led.setState(LED_STATE.GREY);
    } else if (wantedState == SuperStructureState.PROCESSOR) {
      led.setState(LED_STATE.YELLOW);
    } else if (wantedState == SuperStructureState.CLIMB_STAGE_ONE) {
      led.setState(LED_STATE.PAPAYA_ORANGE);
    }
    this.wantedState = wantedState;
  }

  public void setCurrentState(SuperStructureState currentState) {
    // if (currentState == SuperStructureState.STOW) {
    //   led.setState(LED_STATE.BLUE);
    // }
    this.currentState = currentState;
  }

  public SuperStructureState getWantedState() {
    return wantedState;
  }

  public SuperStructureState getCurrentState() {
    return currentState;
  }

  public boolean elevatorExtended() {
    return elevator.isExtended();
  }

  public boolean atGoals() {
    // switch (currentState) {
    //   case STOW:
    //     return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH)
    //         && scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);
    //   case L1:
    //     return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L1_SETPOINT_INCHES)
    //         && scoralArm.hasReachedGoal(FieldConstants.ReefHeight.L1.pitch);
    //   case L2:
    //     return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L2_SETPOINT_INCHES)
    //         && scoralArm.hasReachedGoal(
    //             SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);
    //   case L3:
    //     return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L3_SETPOINT_INCHES)
    //         && scoralArm.hasReachedGoal(
    //             SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);
    //   case L4:
    //     return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L4_SETPOINT_INCHES)
    //         && scoralArm.hasReachedGoal(
    //             SubsystemConstants.ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG);
    //   case SOURCE:
    //     return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH)
    //         && scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);
    //   case SCORING_CORAL:
    //     // TODO:: UNCOMMENT
    //     // return csFlywheel.seesCoral() == CoralState.CURRENT
    //     // || csFlywheel.seesCoral() == CoralState.SENSOR;
    //     return true;
    //   default:
    //     return false;
    // }

    // TODO:: COMMENT
    return true;
  }

  public SequentialCommandGroup getSuperStructureCommand() {
    counter++;
    Logger.recordOutput("bruhufe", counter);
    switch (wantedState) {
      case STOW:
        led.setState(LED_STATE.BLUE);
        currentState = SuperStructureState.STOW;
        return new GoToStow(elevator, scoralArm, scoralRollers)
            .andThen(
                climberArm.setArmTarget(SubsystemConstants.ClimberConstants.STOW_SETPOINT_DEG, 2));

      case INTAKE_ALGAE:
        double height =
            drive.getNearestParition(6) % 2 == 0
                ? FieldConstants.ReefHeight.L4.height
                : FieldConstants.ReefHeight.L4.height;

        currentState = SuperStructureState.INTAKE_ALGAE;
        return new IntakeAlgae(elevator, scoralArm, scoralRollers, height);

      case L1:
        currentState = SuperStructureState.L1;
        lastReefState = SuperStructureState.L1;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L1_SETPOINT_INCHES,
            FieldConstants.ReefHeight.L1.pitch);

      case L2:
        currentState = SuperStructureState.L2;
        lastReefState = SuperStructureState.L2;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L2_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);

      case L3:
        currentState = SuperStructureState.L3;
        lastReefState = SuperStructureState.L3;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L3_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);

      case L4:
        currentState = SuperStructureState.L4;
        lastReefState = SuperStructureState.L4;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L4_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG);

      case SOURCE:
        currentState = SuperStructureState.SOURCE;
        return new IntakingCoral(scoralRollers)
            .andThen(
                new InstantCommand(() -> led.setState(LED_STATE.BLUE))
                    .andThen(
                        new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW))
                            .andThen(
                                new InstantCommand(() -> this.setWantedState(lastReefState)))));

      case PROCESSOR:
        led.setState(LED_STATE.FLASHING_GREEN);
        currentState = SuperStructureState.PROCESSOR;
        return new ScoringProccessorSequential(scoralRollers, scoralArm, elevator);

      case SCORING_CORAL:
        currentState = SuperStructureState.SCORING_CORAL;
        led.setState(LED_STATE.FLASHING_GREEN);
        return new SequentialCommandGroup(
            new ScoreCoral(elevator, scoralArm, scoralRollers),
            new InstantCommand(() -> led.setState(LED_STATE.BLUE)),
            new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW)),
            new InstantCommand(() -> this.setWantedState(SuperStructureState.STOW)));

        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> led.setState(LED_STATE.FLASHING_GREEN)),
        //     new ScoringCoral(scoralRollers),
        //     new WaitCommand(1),
        //     new GoToStowL4(elevator, scoralArm, scoralRollers),
        //     new InstantCommand(() -> led.setState(LED_STATE.BLUE)),
        //     new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW)),
        //     new InstantCommand(() -> this.setWantedState(SuperStructureState.STOW)));

      case CLIMB_STAGE_ONE:
        currentState = SuperStructureState.CLIMB_STAGE_ONE;
        return new SequentialCommandGroup(
            climberArm.setArmTarget(SubsystemConstants.ClimberConstants.DEPLOY_SETPOINT_DEG, 2));
      case CLIMB_STAGE_TWO:
        currentState = SuperStructureState.CLIMB_STAGE_TWO;
        return new SequentialCommandGroup(new InstantCommand(()-> climberArm.setBrakeMode(true), climberArm), new ParallelCommandGroup(climberArm.runVoltsCommand(5), winch.runVoltsCommmand(12)));
      case HANG:
        currentState = SuperStructureState.HANG;
        led.setState(LED_STATE.BLUE);
        return new SequentialCommandGroup(new ParallelCommandGroup(new InstantCommand(()-> climberArm.armStop(), climberArm), winch.stopWinch()),new InstantCommand(()-> climberArm.setBrakeMode(true), climberArm));
      default:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(0, 0), scoralArm.setArmTarget(40, 0)));
    }
  }

  public void advanceWantedState() {
    switch (currentState) {
        // case NONE:
        //   break;
      case STOW:
        break;
      case L1, L2, L3, L4:
        setWantedState(SuperStructureState.SCORING_CORAL);
        break;
      case SCORING_CORAL, SOURCE, PROCESSOR, INTAKE_ALGAE:
        setWantedState(SuperStructureState.STOW);
        break;
      case CLIMB_STAGE_ONE:
        setWantedState(SuperStructureState.CLIMB_STAGE_TWO);
        break;
      case CLIMB_STAGE_TWO:
        setWantedState(SuperStructureState.HANG);
        break;
      case HANG:
        setWantedState(SuperStructureState.CLIMB_STAGE_ONE);
        break;

      default:
        break;
    }
  }

  public boolean isTargetAReefState() {
    return wantedState == SuperStructureState.L1
        || wantedState == SuperStructureState.L2
        || wantedState == SuperStructureState.L3
        || wantedState == SuperStructureState.L4;
  }
}
