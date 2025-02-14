package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.ElevatorState;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevator;

  private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI");

  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

  private static final LoggedTunableNumber barkG = new LoggedTunableNumber("Bar/kG");

  // CHANGE THESE VALUES TO MATCH THE ELEVATOR
  private static final int maxVelocityExtender = 1;
  private static final int maxAccelerationExtender = 1;

  private TrapezoidProfile extenderProfile;
  private TrapezoidProfile extenderProfile2;
  private TrapezoidProfile.Constraints extenderConstraints =
      new TrapezoidProfile.Constraints(maxVelocityExtender, maxAccelerationExtender);
  private TrapezoidProfile.Constraints extenderConstraints2 =
      new TrapezoidProfile.Constraints(maxVelocityExtender - 0.5, maxAccelerationExtender - 0.2);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent2 = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderGoal2 = new TrapezoidProfile.State();

  private double goal;
  private ElevatorFeedforward elevatorFFModel;
  private final ElevatorVis measured;

  private ElevatorVis measuredVisualizer;
  private ElevatorVis setpointVisualizer;
  private ElevatorState currentState;
  private ElevatorState wantedState;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;

    switch (SimConstants.currentMode) {
      case REAL:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        kI.initDefault(0);

        barkG.initDefault(0);
        break;
      case REPLAY:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        kI.initDefault(0);

        barkG.initDefault(0);
        break;
      case SIM:
        kS.initDefault(0.0);
        kG.initDefault(0.01);
        kV.initDefault(0.55);
        kA.initDefault(0);

        kP.initDefault(13.4);
        kI.initDefault(1);

        barkG.initDefault(1.7);
        break;
      default:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        kI.initDefault(0);

        barkG.initDefault(0);
        break;
    }
    measured = new ElevatorVis("measured", Color.kRed);

    // CHANGE THIS VALUE TO MATCH THE ELEVATOR
    // setExtenderGoal(1.3);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
    extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);

    extenderProfile2 = new TrapezoidProfile(extenderConstraints2);
    extenderCurrent2 = extenderProfile2.calculate(0, extenderCurrent2, extenderGoal2);

    measuredVisualizer = new ElevatorVis("measured", Color.kRed);
    setpointVisualizer = new ElevatorVis("setpoint", Color.kGreen);

    currentState = ElevatorState.STOW;
    wantedState = ElevatorState.STOW;

    updateTunableNumbers();
  }

  public boolean atGoal() {
    return (Math.abs(extenderCurrent.position - goal)
        <= SubsystemConstants.ElevatorConstants.DEFAULT_THRESHOLD);
  }

  public double getElevatorPosition() {
    return eInputs.elevatorPositionInch;
  }

  private double getElevatorError() {
    return eInputs.positionSetpointInch - eInputs.elevatorPositionInch;
  }

  public boolean elevatorAtSetpoint(double thersholdInches) {
    return (Math.abs(getElevatorError()) <= thersholdInches);
  }

  public void setExtenderGoal(double setpoint) {
    goal = setpoint;
    extenderGoal = new TrapezoidProfile.State(setpoint, 0);
    extenderGoal2 = new TrapezoidProfile.State(setpoint, 0);
  }

  public void setPositionExtend(double position, double velocity) {
    elevator.setPositionSetpoint(position, elevatorFFModel.calculate(velocity));
  }

  public void elevatorStop() {
    elevator.stop();
  }

  public double calculateAngle() {
    double angle = 0.0;
    return angle;
  }

  public void setWantedState(ElevatorState wantedState) {

    this.wantedState = wantedState;
  }

  public ElevatorState getCurrentState() {
    return currentState;
  }

  public ElevatorState getWantedState() {
    return wantedState;
  }

  public void setConstraints(
      double maxVelocityMetersPerSec, double maxAccelerationMetersPerSecSquared) {
    extenderConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSec, maxAccelerationMetersPerSecSquared);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
  }

  public boolean isExtended() {
    return extenderGoal.position == SubsystemConstants.ElevatorConstants.EXTEND_SETPOINT_INCH;
  }

  public Command setElevatorTarget(double goalInches, double thersholdInches) {

    return new InstantCommand(() -> setExtenderGoal(goalInches), this)
        .until(() -> elevatorAtSetpoint(thersholdInches));
  }

  public void handleStates() {

    switch (currentState) {
      case ZERO:
        setExtenderGoal(0);
        break;
      case STOW:
        setExtenderGoal(0.4);
        break;
      case L1:
        setExtenderGoal(FieldConstants.ReefHeight.L1.height);
        break;
      case L2:
        setExtenderGoal(FieldConstants.ReefHeight.L2.height);
        break;
      case L3:
        setExtenderGoal(FieldConstants.ReefHeight.L3.height);
        break;
      case L4:
        setExtenderGoal(FieldConstants.ReefHeight.L4.height);
        break;
      case SOURCE:
        setExtenderGoal(SubsystemConstants.ElevatorConstants.INTAKE_SETPOINT_INCHES);
        break;
      case PROCESSOR:
        setExtenderGoal(SubsystemConstants.ElevatorConstants.PROCESSOR_SETPOINT_INCHES);
        break;
    }
  }

  @AutoLogOutput(key = "elevator")
  public Pose3d getElevatorPose() {
    if (getElevatorstage2Pose().getZ() < extenderCurrent.position) {
      return new Pose3d(0, 0, extenderCurrent2.position + 0.5, new Rotation3d());
    } else {
      return new Pose3d(0, 0, extenderCurrent.position + 0.9, new Rotation3d());
    }
  }

  @AutoLogOutput(key = "elevatorstage2")
  public Pose3d getElevatorstage2Pose() {

    return new Pose3d(0, 0, extenderCurrent2.position + 0.5, new Rotation3d());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());
    /*if (Drive.speedX > 2 || Drive.speedY > 2 || Drive.rotationDegs > 50) {
       this.currentState = ElevatorState.STOW;
     } else {
       this.currentState = getWantedState();
     }
    */
    // handleStates();
    elevator.updateInputs(eInputs);
    measured.update(extenderCurrent.position);
    CoralScorerArm.measuredVisualizer.updateVertical(extenderCurrent.position);

    extenderCurrent =
        extenderProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, extenderCurrent, extenderGoal);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    extenderCurrent2 =
        extenderProfile2.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, extenderCurrent2, extenderGoal2);
    // handleStates();

    Logger.processInputs("Elevator", eInputs);

    measuredVisualizer.update(0.55 + extenderCurrent.position);
    setpointVisualizer.update(0.55 + extenderGoal.position);

    CoralScorerArm.measuredVisualizer.updateVertical(extenderCurrent.position + 0.1);
    CoralScorerArm.setpointVisualizer.updateVertical(extenderGoal.position + 0.1);

    updateTunableNumbers();
  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      elevator.configurePID(kP.get(), kI.get(), 0);
    }
    if (kS.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      elevatorFFModel = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }
  }
}
