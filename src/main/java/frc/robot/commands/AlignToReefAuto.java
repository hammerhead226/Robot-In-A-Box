
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.constants.SubsystemConstants;

public class AlignToReefAuto extends Command {
    Drive drive;
    LED led;
    public AlignToReefAuto(Drive drive, LED led) {
        this.drive = drive;
        this.led = led;

        addRequirements(drive, led);
    }

    @Override
    public void initialize() {
        led.setState(SubsystemConstants.LED_STATE.ALIGNING);
        reefSidePosition = getReefSidePosition(getNearestReefSide());

        generatedPathCommand =
            AutoBuilder.followPath(reefSidePosition,...);
    }

    private ReefSide getNearestReefSide() {
        start = getAllianceReefPosition();
        end = drive.getPosition().minus();
        revolutions = new Rotation2D(start, end).getRevolutions();
        
        return ...map revolution to ReefSide...;

        // might look something like
        // return new ReefSide(Math.floor(getRevolutions * 6));
    }

    @Override
    public void execute() {
        //we could switch to PID as we get closer to the wall? for instance if both CANRanges measure a short distance
        generatedPathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        led.setState(LED_STATE.ALIGNED)
    }

    @Override
    public boolean isFinished() {
        // or any other way we can measure "close enough" to desired position
        return Math.hypo(getRobotPosition, reefSidePosition) <= distanceThershold;
    }
}

