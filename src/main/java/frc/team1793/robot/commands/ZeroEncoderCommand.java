package frc.team1793.robot.commands;

import org.strongback.command.Command;
import org.strongback.components.AngleSensor;

public class ZeroEncoderCommand extends Command {

    AngleSensor leftEncoder;
    AngleSensor rightEncoder;

    public ZeroEncoderCommand(AngleSensor leftEncoder, AngleSensor rightEncoder) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    @Override
    public boolean execute() {
        leftEncoder.zero();
        rightEncoder.zero();
        return true;
    }
}
