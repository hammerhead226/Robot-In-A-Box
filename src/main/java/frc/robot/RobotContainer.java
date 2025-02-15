// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;import du.wimport edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxCimport du.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlignToReefAuto;
import frc.robot.commands.ApproachReefPerpendicular;
import frc.robot.commands.DriveCommands;import rc.rimport frc.robot.commands.Stow;
// import frc.robot.commands.IntakeFromSource;
import frc.robot.constants.SimConstants;import rc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.ClimberArmIOSim;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerArmIOSim;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheelIOSim;
import frc.robot.subsystems.coralscorer.CoralSensorIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.KeyboardInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
 * 
  public static Drive drive;
 * 
  private LED led;
 * 

  // Controller
  private final CommandXboxCo
    private final
    private final Joystick joy
    private final Jo

    
    private CoralScorerArm csArm;
    // private final CoralScorerFlywheel coralIntake;
    
    public static Elevator elevator;
    private ClimberArm climberArm;

    SuperStructure superStructure
    // public final Trigger elevatorBrakeTrigger;

    private final Trigger speedTrigg
    
    private CoralScorerFly
    
    // Dashboard inputs
    private final LoggedDashboardChooser<C
    

    public RobotContainer() {

        case REAL:
          // Real robot, instantiate hardware IO implementatio

       
     *   //     new Drive(
     
          //         new Gyro
            //         new ModuleIOTalonFX(
              //      
                //         new ModuleIOTalonFX(TunerConstants.BackLeft
                //        
                 
                //  new CoralScorerArm(
                 
                // 
                // Vision(
                // drive.getToPoseEstimatorConsumer(),

                //        new VisionIOLimelight("limelight 2", drive.getRawGyr

                //         
                //  change lea
                el new Elevator(new ElevatorIOTalonFX(
                // rm = new ClimberArm(new ClimberArmIOTalonFX(0, 0, 0));
                // el =
                // CoralScorerFlywheel(
                // new CoralScorerFlywheelIOSim(),
                //         new CoralSensorIO() {},
                //         CoralState.DEFAULT,
                //         AlgaeState.DEFAULT);
                // led = new LE
                // rStructure = new SuperSt
                dr 
                   ve(
                    GyroIO() {},
                    ModuleIOSim(TunerCo
                        new ModuleIOSim(TunerConstants.Fro
                        new ModuleIOSim(TunerConstants.BackLeft),
                        
                        new CoralScore
                        rA
                        
                        
                        Vision(
                        drive.getToPoseEstimatorConsumer(),

                        new VisionIOLimelight("limelight 2", drive.get

                         berArm = ne
                        el =
                        CoralScorerFlywheel(
                        new CoralScorerFlywheelIOSim(),
                        new CoralSensorIO() {},
                        CoralState.DEFAULT,
                        AlgaeState.DEFAULT);
                led = new LE rStructure = new SuperSt
                        
                        
                        
                        
                        obot, instantiate ph
                drive =
                    new Drive(
                      

                     
                        new ModuleIOSim(TunerConstants.BackLeft),
                        
                        new CoralScore
                        rA
                        
                        
                        Vision(
                        drive.getToPoseEstimatorConsumer(),

                        new VisionIOLimelight("limelight 2", drive.get

                         ator = new 
                        rm = new ClimberArm(new ClimberArmI
                        el =
                        CoralScorerFlywheel(
                        new CoralScorerFlywheelIOSim(),
                        new CoralSensorIO() {},
                        CoralState.DEFAULT,
                        AlgaeState.DEFAULT);
                led = new LE rStructure = new SuperSt
                        
                        
                        
                        
                        yed robot, disable I
                drive =
                    new Drive(
                      

                    
                        new ModuleIO() {},
                        
                        new CoralScore
                        rA
                        
                        
                        Vision(
                        
                        drive.getToPoseE
                        st
                        new VisionIOLime
                        lig

                        new VisionIOLimelight("limelight 3", drive.get
                         ator = new 
                        rm = new ClimberArm(new ClimberArmI
                        el =
                        CoralScorerFlywheel(
                        new CoralScorerFlywheelIOSim(),
                        new CoralSensorIO() {},
                        CoralState.DEFAULT,
                
                        AlgaeState.DEFAULT);
                led = new LE rStructure = new SuperSt
                        
                        
                        
                        
                        to routines
                amedCommands.registerCommand
                ("A
                
                amedCo
        /

        //         new InstantC
        //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal()
        // ));

        //     "L2",
        // new S
        //     new InstantCommand(() -
        // new WaitUntilCommand(() 
        // > superStructure.hasStructureReachedGoal())));
        // mands.registerCommand(
        //     "L3",
        // new S
        //     new InstantCommand(() -
        // new WaitUntilCommand(() 
        // > superStructure.hasStructureReachedGoal())));
        // mands.registerCommand(
        //     "L4",
        // new S
        //     new InstantCommand(() -
        // new WaitUntilCommand(() 
        // > superStructure.hasStructureReachedGoal())));
        // mands.registerCommand(
        //     "STOW ", new InstantComman
        // rStru
        // dCommands.registerCommand(
        // RE",
        // 
        // SequentialCommandGroup(
        //         new InstantCommand(
        //         () -> superStructure.setW
        //         new WaitUntilCommand(() -> superStructure.hasStruc
        // NamedCommands.registerCommand(
        // "INTAKE"
        // new SequentialCommandGroup(
        // new InstantCommand(
        // WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
         
        autoChooser = new LoggedDashboard
         
        // up SysId routines
        au .addOption(
        // 
           Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterizati

            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

            "Drive SysId (Quasis
            drive.sysIdQuasist
                Chooser.addOption(
            "Drive SysId (Quas
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", dri
                Chooser.addOption(
            "Drive SysId (Dyna
                Chooser.addOption(
                "Top R3a", AutoBuilder.buildAuto("R3a(L3)-S1c-R2a(L3)-S2c
        autoChooser.addDefault
                Chooser.addDefaultOption("1.1 auto", AutoBuilder.buildAuto("1.1.auto"));
        /*
                dCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));
        NamedCommands.register
                "L1", new InstantCommand(() -> Super.setWantedState(SuperStructureState.L1)));
        NamedCommands.registerCommand(
            "L2", new InstantCommand(() -> Super.setWantedState(SuperStructureState.
        Na
         *     "L3", new InstantCommand(() -> Super.setWantedState(SuperStructureState
         * L3)));
         * NamedCommands.registerCommand(
         * "L4", new InstantCommand(() ->
         * Super.setWantedState(SuperStructureState.L4)));
         * NamedCommands.registerCommand(
         * "STOW ", new InstantCommand(()
         * -> Super.setWantedState(SuperStructureState.STO
         * NamedCommands.registerCommand(
         * "SCORE ",
         * 
         *     new InstantCommand(() -> S
         * dCommands.registerCommand(
         * 
         *     "INTAKE ", new InstantComm
         * */
         * 
         * // autoChooser.addOption("toRe
         * onfigure 
         * onfigureButtonBindings()
         * 
         * // stateTrigger = new Trigger(
         * levatorBrakeTrigger = new Trigger((
         *  -> RobotController.getUserButton());
         dT
        // configureButtonBindings();
        test();
        
        
        *
         Use this method to define your button->command mappings. Buttons 
         instantiating a {@link Gener
         edu.wp
     

    pri
      driveController
     * 
          .a()
          .onTrue(new InstantCommand(() -> elevator.setElevatorTarget(0.1, 0), elev
     * tor));
      driveController
       
          .onFalse(new In
        
                
                e void configureButtonBindings() {
        speedTrigger.on
                eybo
                eyboard
      //     .getXButton()
    //     .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L2));
    // keyboard
    //     .getZButton()
    //     .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L1));
    // stateTrigger.onTrue(superStructure.getSuperStructureCommand());

    // keyboard.getCButton().whileTrue(new AlignToCage(drive));
    keyboard
        .getXButton()
        .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L2));
    keyboard
        .getZButton()
        .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L1));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.a().getAsBoolean(),
            () -> driveController.leftTrigger().getAsBoolean(),
            () -> driveController.rightTrigger().getAsBoolean(),
            () -> driveController.rightBumper().getAsBoolean(),
            () -> driveController.b().getAsBoolean(),
            () -> driveController.x().getAsBoolean()));

    // driveController.x().onTrue(new Stow(elevator, csArm));
    //   elevatorBrakeTrigger.onTrue(new InstantCommand(() -> elevator.breakMode(true), elevator));
    //  elevatorBrakeTrigger.onFalse(new InstantCommand(() -> elevator.breakMode(false)));

    driveController
        .y()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)));
    // driveController.y().onFalse(new InstantCommand(() -> csFlywheel.stop(), csFlywheel));
    driveController
        .a()
        .onTrue(
            new InstantCommand(
                () -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL)));
        .a()
        .onFalse(
            new ConditionalCommand(
                new ApproachReefPerpendicular(drive).withTimeout(2),
                new InstantCommand(),
                () -> (!drive.isNearReef() && drive.isAtReefSide())));

    driveController.a().onTrue(new SetClawLevel(ReefHeight.L1, elevator, csArm));
    driveController.a().onFalse(/*csFlywheel
                    .runVelocityCommand(2000)
                    .until(() -> csFlywheel.getLastCoralState() != CoralState.NO_CORAL).andThen*/ (new Stow(elevator, csArm)));

    // driveController.a().onFalse(new SetClawLevel(ElevatorState.STOW, elevator, csArm));
    // driveController
    //     .rightBumper()
    //     .onTrue(new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.SOURCE)));

    // driveController
    //     .rightBumper()
    //     .onFalse(new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.STOW)));

    // // driveController
    // //   .a()
    // // .onTrue(new InstantCommand(() -> Super.setWantedState(SuperStructureState.L1)));

    // driveController.a().onTrue(new AlignToReefAuto(drive, led));
    // driveController
    //     .x()
    //     .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));
    // //  driveController
    // //    .a()
    // //   .onFalse(new InstantCommand(() -> Super.setWantedState(SuperStructureState.STOW)));

    // driveController.y().onTrue(new AlgaeIntoProcesser(elevator, csArm, csFlywheel));
    // driveController.y().onFalse(new Stow(elevator, csArm));

    // why is this like this?
    // driveController.leftBumper().onTrue(new InstantCommand(() -> drive.setNearestReefSide()));
    // driveController.rightBumper().onTrue(new SetClawLevel(ElevatorState.SOURCE, elevator,
    // csArm));
    // driveController.rightBumper().onFalse(new SetClawLevel(ElevatorState.STOW, elevator, csArm));
    // driveController.leftBumper().whileTrue(new AutoAlignToSource(drive, led));
    // driveController.rightBumper().whileTrue(new AlignToReefAuto(drive, led));
    // driveController.rightTrigger().whileTrue(new AlignToProcessor(drive, led));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // controller.y().onTrue(csArm.setArmTarget(30, 0));

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // controller.y().whileTrue(elevator.setElevatorTarget(1.83, 1));
    // controller.y().whileFalse(elevator.setElevatorTarget(1, 1));

    // controller.x().whileTrue(csArm.setArmTarget(90, 1));
    // controller.x().whileFalse(csArm.setArmTarget(-90, 1));

    /*controller.b().whileTrue(algaeArm.setArmTarget(70, 2));
    controller.b().whileFalse(algaeArm.setArmTarget(20, 2));*/

    // manipController.rightTrigger().onTrue(new Stow(elevator, csArm));
    // driveController.a().whileTrue(new ReleaseClawParallel(scoringLevel, elevator, csArm,
    // csFlywheel));

    // manipController.a().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L1.height, 1)));
    // manipController.b().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L2.height, 1)));
    // manipController.x().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L3.height, 1)));
    // manipController.y().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 1)));

    // keyboard
    //    .getCButton()
    //    .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)));
    // keyboard
    //    .getVButton()
    //    .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));
    manipController
        .x()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));
    manipController
        .y()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)));
    manipController
        .a()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));
    manipController
        .b()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)));

    // manipController.leftBumper().whileTrue(new AutoAlignToSource(drive, led));
    // manipController.leftBumper().onTrue(new IntakeFromSourceParallel(csFlywheel, csArm,
    // elevator));
    // manipController.leftBumper().onFalse(new Stow(elevator, csArm));
    manipController.rightBumper().onTrue(new IntakingAlgae(elevator, csFlywheel, csArm));
    manipController.rightBumper().onFalse(new Stow(elevator, csArm));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}


    
     
     
     
     
    
        
    