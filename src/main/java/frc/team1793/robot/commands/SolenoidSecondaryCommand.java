package frc.team1793.robot.commands;

import org.strongback.command.Command;
import org.strongback.components.Solenoid;


public class SolenoidSecondaryCommand extends Command {

    private final Solenoid solenoid;

    public SolenoidSecondaryCommand(Solenoid solenoid) {
        this.solenoid = solenoid;
    }

    @Override
    public boolean execute() {
        solenoid.extend();

        return true;
    }
}
