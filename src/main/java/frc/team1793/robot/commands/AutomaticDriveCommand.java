package frc.team1793.robot.commands;

import org.strongback.command.Command;
import org.strongback.components.AngleSensor;
import org.strongback.drive.TankDrive;

public class AutomaticDriveCommand extends Command {

    private AngleSensor leftEncoder;
    private AngleSensor rightEncoder;
    private TankDrive drive;
    private double distance;
    private final double TOLERANCE = 0.05;

    public AutomaticDriveCommand(AngleSensor leftEncoder, AngleSensor rightEncoder, TankDrive drive, double distance) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.drive = drive;
        this.distance = distance;
        this.leftEncoder.zero();
        this.rightEncoder.zero();
    }

    @Override
    public boolean execute() {
        while ((leftEncoder.getAngle() + rightEncoder.getAngle() * -1) / 2 < distance - TOLERANCE) {
            drive.arcade(.7, 0);
        }
        return true;
    }
}
