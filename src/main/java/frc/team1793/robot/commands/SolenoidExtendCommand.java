package frc.team1793.robot.commands;

import org.strongback.command.Command;
import org.strongback.components.Solenoid;


public class SolenoidExtendCommand extends Command {

    private final Solenoid solenoid;

    public SolenoidExtendCommand(Solenoid solenoid) {
        this.solenoid = solenoid;
    }

    @Override
    public boolean execute() {
        solenoid.extend();

        return true;
    }
}
