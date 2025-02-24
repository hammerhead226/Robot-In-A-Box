package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ApproachReefPerpendicular;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReinitializingCommand;
import frc.robot.constants.RobotMap;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.ClimberArmIOSim;
import frc.robot.subsystems.climber.ClimberArmIOTalonFX;
import frc.robot.subsystems.climber.Winch;
import frc.robot.subsystems.climber.WinchIO;
import frc.robot.subsystems.climber.WinchIOSim;
import frc.robot.subsystems.climber.WinchIOTalonFX;
import frc.robot.subsystems.commoniolayers.ArmIO;
import frc.robot.subsystems.commoniolayers.FlywheelIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.led.LED_IOSim;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralArmIOSim;
import frc.robot.subsystems.scoral.ScoralArmIOTalonFX;
import frc.robot.subsystems.scoral.ScoralRollers;
import frc.robot.subsystems.scoral.ScoralRollersIOSim;
import frc.robot.subsystems.scoral.ScoralRollersIOTalonFX;
import frc.robot.subsystems.scoral.ScoralSensorCANRangeIO;
import frc.robot.subsystems.scoral.ScoralSensorIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final LED led;

    // Controller
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipController = new CommandXboxController(1);
    // private final Joystick joystikc = new Joystick(0);
    // private final JoystickButton btn = new JoystickButton(joystikc, 4);
    // private final KeyboardInputs keyboard = new KeyboardInputs(0);

    private ScoralArm scoralArm;
    // private final CoralScorerFlywheel coralIntake;

    public static Elevator elevator;
    public ClimberArm climberArm;
    private Vision vision;
    SuperStructure superStructure;
    private final Winch winch;

    // public final Trigger elevatorBrakeTrigger;
    // private final Trigger stateTrigger;
    // private final Trigger slowModeTrigger;

    private ScoralRollers scoralRollers;

    public final Trigger elevatorBrakeTrigger;
    // private final Trigger stateTrigger;
    private Trigger slowModeTrigger;
    private Trigger reefAlignTrigger;
    private Trigger approachPerpendicularTrigger;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final SendableChooser<Command> autos;
    private DigitalInput brakeSwitch;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        brakeSwitch = new DigitalInput(RobotMap.BrakeSwitchIDs.brakeSwitchChannel);
        elevatorBrakeTrigger = new Trigger(() -> brakeSwitch.get());
        // // Real robot, instantiate hardware IO implementations

        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    RobotMap.ElevatorIDs.leftElevatorID,
                    RobotMap.ElevatorIDs.rightElevatorID,
                    RobotMap.ElevatorIDs.elevatorCANrangeID));
        // winch = new Winch(new WinchIOTalonFX(12, 13));
        winch =
            new Winch(
                new WinchIOTalonFX(RobotMap.WinchIDs.leftWinchID, RobotMap.WinchIDs.rightWinchID));
        drive =
            new Drive(
                new GyroIOPigeon2() {},
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        scoralArm =
            new ScoralArm(
                new ScoralArmIOTalonFX(
                    RobotMap.CoralScorerArmIDs.coralScorerRotationID,
                    RobotMap.CoralScorerArmIDs.coralScorerRotationCANcoderID));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-reef", () -> drive.getPose().getRotation())
                // new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                // new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                // new VisionIOPhotonVision("photon", new Transform3d())
                );

        climberArm =
            new ClimberArm(
                new ClimberArmIOTalonFX(
                    RobotMap.ClimbIDs.deployClimbID, RobotMap.ClimbIDs.deployClimbCANcoderID));
        // climberArm = new ClimberArm(new ClimberArmIO() {});

        // scoralRollers =
        // new ScoralRollers(
        // new ScoralRollersIOSim(),
        // new ScoralSensorIO() {},
        // CoralState.DEFAULT,
        // AlgaeState.DEFAULT);
        scoralRollers =
            new ScoralRollers(
                new ScoralRollersIOTalonFX(RobotMap.CoralScorerArmIDs.coralScorerFlywheelID),
                new ScoralSensorCANRangeIO(RobotMap.CoralScorerArmIDs.coralScorerCANrangeID),
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOCANdle(RobotMap.ledIDs.CANdleID, "CAN Bus 2"));
        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm, winch);

        break;
      case SIM:
        elevatorBrakeTrigger = new Trigger(() -> true);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        scoralArm = new ScoralArm(new ScoralArmIOSim());
        winch = new Winch(new WinchIOSim());
        // Transform3d bruh = new Transform3d(new Transform2d());
        Rotation3d bruh = new Rotation3d();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera 1 sim", new Transform3d(0, 0, 0, bruh), drive::getPose)
                // new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                // new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                // new VisionIOPhotonVision("photon", new Transform3d())
                );
        elevator = new Elevator(new ElevatorIOSim());
        scoralRollers =
            new ScoralRollers(
                new ScoralRollersIOSim(),
                new ScoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOSim());

        climberArm = new ClimberArm(new ClimberArmIOSim());

        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm, winch);
        break;

      default:
        elevatorBrakeTrigger = new Trigger(() -> true);
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        scoralArm = new ScoralArm(new ArmIO() {});
        // vision =
        // new Vision(
        // drive.getToPoseEstimatorConsumer(),
        // new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
        // new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
        // new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
        // new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIO() {});
        scoralRollers =
            new ScoralRollers(
                new FlywheelIO() {},
                new ScoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IO() {});

        winch = new Winch(new WinchIO() {});

        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm, winch);
        break;
    }

    // Set up auto routines
    // NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive,
    // led));

    // NamedCommands.registerCommand("L1", new SequentialCommandGroup(
    // new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)), new
    // WaitUntilCommand(() -> superStructure.atGoals())));

    // NamedCommands.registerCommand("L2", new SequentialCommandGroup(
    // new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)), new
    // WaitUntilCommand(() -> superStructure.atGoals())));

    // NamedCommands.registerCommand("L3", new SequentialCommandGroup( new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.L3)), new WaitUntilCommand(() ->
    // superStructure.atGoals())));

    // NamedCommands.registerCommand(
    // "L4",
    // new SequentialCommandGroup(
    // new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.L4)),
    // new WaitUntilCommand(() -> superStructure.atGoals())));

    // NamedCommands.registerCommand(
    // "INTAKE",
    // new SequentialCommandGroup(
    // new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.SOURCE)),
    // new WaitUntilCommand(() -> superStructure.atGoals())));

    // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
    // AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Top R3a",
    // AutoBuilder.buildAuto("R3a(L3)-S1c-R2a(L3)-S2c-R1b(L3)-S3c-R6a(L3)"));
    // autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));
    // autoChooser.addDefaultOption("1.1 auto", AutoBuilder.buildAuto("1.1.auto"));

    // Set up auto routines

    // Set up SysId routines

    NamedCommands.registerCommand(
        "L1",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "L2",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "L3",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "L4",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));

    NamedCommands.registerCommand(
        "SOURCE_INTAKE",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SOURCE)).andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "ALGAE_INTAKE",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.INTAKE_ALGAE)).andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand("STOW", new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW)).andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
    .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand("SCORE_CORAL", new SequentialCommandGroup(
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL)),
        new WaitUntilCommand(() -> superStructure.atGoals()),
        superStructure.getSuperStructureCommand()));
    // NamedCommands.registerCommand("Stow", new Stow(elevator, csArm));

    autos = new SendableChooser<>();

    // autos.addOption("AutoTest",
    AutoBuilder.buildAuto("Bottom-R5a(L4)-S3c-R6a(L4)-F2-R6b(L4)-S2c");
    autos.addOption("AutoTestTwo", AutoBuilder.buildAuto("Bottom-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption(
        "AutoSourceBottom", AutoBuilder.buildAuto("Bottom-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption(
        "AutoSourceMiddle", AutoBuilder.buildAuto("Middle-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption("AutoSourceTop", AutoBuilder.buildAuto("Top-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption("AutoTestTop", AutoBuilder.buildAuto("Top-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));
    autos.addOption(
        "AutoTestMiddle", AutoBuilder.buildAuto("Middle-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));
    autos.addOption(
        "AutoTestBottom", AutoBuilder.buildAuto("Bottom-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));

    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));
    // autoChooser.addOption("toReefTest", AutoBuilder.buildAuto("toReefTest"));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autos);
    // Configure the button bindings
    // configureButtonBindings();
    // stateTrigger = new Trigger(() -> superStructure.changedStated());

    slowModeTrigger = new Trigger(() -> superStructure.elevatorExtended());

    reefAlignTrigger =
        new Trigger(
            () ->
                driveController.leftTrigger().getAsBoolean()
                    || driveController.rightTrigger().getAsBoolean());
    approachPerpendicularTrigger =
        new Trigger(
            () -> reefAlignTrigger.getAsBoolean() && (!drive.isNearReef() && drive.isAtReefSide()));
    // speedModeTrigger = new Trigger(() -> superStructure.elevatorExtended());
    configureButtonBindings();
    // test();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void test() {
    // driveController.b().onTrue(elevator.setElevatorTarget(20, 1));
    // driveController.b().onFalse(elevator.setElevatorTarget(0, 1));
    // driveController.a().onTrue(led.setStateCommand(LED_STATE.BLUE));
    // driveController.y().onTrue(led.setStateCommand(LED_STATE.GREEN));
    // driveController.x().onTrue(led.setStateCommand(LED_STATE.FIRE));
    // driveController.a().onTrue(winch.runVoltsCommmand(1));
    // driveController.a().onFalse(new InstantCommand(() -> winch.stop(), winch));
    // driveController.b().onTrue(climberArm.setArmTarget(20, 1));
    // driveController.b().onTrue(climberArm.setArmTarget(0, 1));

    // driveController.y().onFalse(new InstantCommand(() -> scoralArm.armStop()));
    // driveController.x().onTrue(scoralArm.setArmTarget(20, 1));
    // driveController.x().onFalse(new InstantCommand(() -> scoralArm.armStop()));
    // driveController.b().onTrue(scoralRollers.runVoltsCommmand(2));
    // driveController.b().onFalse(scoralRollers.stopCommand());
   

       
          .onTrue(new InstantCommand(() -> superStructure.setWantedState(Super
     * tructureSt
      driveController
          .x()
     * 
          .onTrue(new InstantCommand(() -> superStructure.set
      d
          .y()
            .onTrue(new InstantCommand(() -> superStructure.setWantedStat
        driveController
            .a()
            .onTrue(
                new InstantCommand(() -> superStructure.setWantedState(Supe
        driveController
            .rightBumper()
            .onTrue(
                new WaitUntilCommand(() -> superStructure.atGoals())

                        new ReinitializingCommand(
                            () -> superStructure.getSuperStructureCom
                            elevator,
                            scoralArm,
                            scoralRollers,
                            drive,
                       
                    
                
        // driveControl
                povU
                onTrue(
        // new InstantC
                ) ->
                eController.povUp().onTrue(climberArm.setArmTarget(0, 2));
        driveController
                
                riveCont
                        
        // .onTrue(
                coralArm.setAr
                ubsystem
                        
                                roller.x(
                                        er.x().onFalse(scoralRolle
                                                ().onTrue(new GoToReefHeight(elevator, scoralArm
                                                
                                                ().onTrue(
                                                ().onFalse(sco
                                                ().onT
                                                
                                roller

        // .onTrue(
        // new Para
        // winch.ru
        // 130),
        // climberArm.setArmTarget(130, 1)));
        // driveController
        // .b()

        // new ParallelCom
        // new 
        // new Inst
        
        
        ivate v
        slowModeTrigger.onTrue(new InstantCommand(() -> drive.enableSlowM
        slowModeTrigger.onFalse(new InstantCommand(() -> drive.enabl
        
        driverCo
        manipControls();
        
        
        ivate void drive
        driveController
            .st
            .onTrue
                Commands.runOnce(
                        () ->
                
                                new Pose2d(dr
                        dr
               
        drive.setDef
            DriveCommands.joystickDr
                drive,
                superStructure,
     

              () -> -driveController.getLeft
                () -> -driveController.getRightX(),
                () -> driveController.leftBumper().getAsBoolean(),

                () -> dri
        
     

              drive, superStructure
        // reefAlignTri
                   new C
                       n
                           new InstantCom
                                 (!dr 
                                        
                                ()
                                
                new WaitUntilCom
                        .andThen(
                              
                                    () 
                            
                                    scoralArm,
                                    scoralRollers,
                                    drive,
                                    led))
                            .andThen(new InstantCommand(() -> superStructur
                        

            .a()
                .onTrue(new InstantCommand(() 
                        
        driveController
           )
           (new InstantCommand(() -> superStructure.setWantedState(SuperStructu
        dr ller
           

        // driveControl
                   .povUp()
                   .onTr
                        troller
                                
                                        InstantCommand(() -> super
                                                
                                                
                                                
                                                
                                                ommand
                                                
                                ler

            .onTrue(
                    
                

            .povDown()
                .onT
                    new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
                    .an
                    
                                () -> superStructure.getSuperStructureCommand(),
                          
                   
                        scoralRollers,
        // 
                       
                    
                

        // new InstantC
                levatorB
                ew Insta
                        

        ivate void mani
                pControlle
                .x()
                        rue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));

            .y()
                .onTrue(ne
                pControl
                        
                                new Insta
                                        
                                                
                                                antComman
                                                
                                                
                                                
                                                

                    .andThen(
                        new ReinitializingCommand(
                            () -> superS
                            elevator,
     

                          drive,
                       
                
                
         private void t
                wMod
                wModeTrigger.onFalse(new InstantCommand(() ->
         drive.enableSl
                
                ve.setDefaultCommand(
         DriveCommands.
                ve,
                erStructure,

         () -> -driveCo
                -> -driveC
                -> drive
                        riveController.leftTrigger().getAsBoolean(),
                                Controlle
                                        
                                                 - Align Commands go in Drive
                                                .onTrue((
                                                .onFalse(n
                                                (0)));
                                                
                                                
    /

    // manipController
    // .leftBumper()
    // .onTrue(
    // new IntakeFromSourceParallel(c

    // () ->
    // csFlywheel.seesCoral() == Co
    // || csF
    // .withTimeout(5)
    // manipController
    // .leftBumper()
    // .onFalse(
    // new ParallelCommandGroup(
    // new Stow(elevator, csArm), new InstantCommand(() ->
    // csFlywheel.runVolts(0))));

    // manipController
    // .rightBumper()
    // .onTrue(
    // new IntakingAlgaeParallel

    // .withTimeout(5));
    // manipController

    // .onFalse(
    // new ParallelC
    // new Stow
    // csFlywheel.runVolts(0))));
    
    // manip
    // .a()
    // .onTrue(new ReleaseClawParallel(FieldConstants.R
    // csArm,
    // csFlywheel));
    // manipControll
    // .a()
    // .onFalse(
    // new ParallelCommandGroup(
    // new Stow(elevator, csArm),

    
    // manipControlle
    // .b()
    // .onTrue(new ReleaseClawParallel(FieldConstants.ReefHei
    // csArm,
    // csFlywheel));
    // manipController
    // .b()
    // .onFalse(
    // new ParallelCommandGroup(
    // new Stow(elevator, csArm), new InstantCommand(() ->
    // csFlywheel.runVolts(0))));

    // manipController
    // .x()
    // .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator,
    // csArm,
    // csFlywheel));
    // manipController
    // .x()
    // .onFalse(
    // new ParallelCommandGroup(
    // new Stow(elevator, csArm), new InstantCommand(() ->
    // csFlywheel.runVolts(0))));

    // manipController
    // .y()
    // .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator,
    // csArm,
    // csFlywheel));
    // manipController
    // .y()
    // .onFalse(
    // new ParallelCommandGroup(
    // new Stow(elevator, csArm), new InstantCommand(() ->
    // csFlywheel.runVolts(0))));

    // manipController
    // .lef
    // .onTrue(
    // new Re
    // () -> superSt
    // led));
    
    // manipCont
    // }
    /**
     * Use this to pass the auton

     * @return the com
     */
    public Command getAutonomousCommand() {
      return 
    }
    
    public 
      return sco
    }
    
    public Drive getDrive() {

    }
    
    public Elev
      return elevator;
    }
    

      return superStructure;
    }
    
     ublic LED getLED() {
      
     
     
    public ScoralRollers getScoralRollers()
        return scoralRollers;
    }

    public ClimberArm getClimber() {
        return climberArm
    }

    
        
    

    
        
    

    
        
    

    
        
    

    
        
    

    
        
    