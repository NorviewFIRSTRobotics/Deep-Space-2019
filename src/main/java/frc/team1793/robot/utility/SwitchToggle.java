package frc.team1793.robot.utility;

import org.strongback.Strongback;
import org.strongback.command.Command;

/**
 * Created by melvin on 2/15/2017.
 * Toggle between two commands in press of a button
 */
public class SwitchToggle {
    private boolean running;
    private final Command start;
    private final Command stop;

    public SwitchToggle(Command start, Command stop) {
        this.start = start;
        this.stop = stop;
    }

    public void execute() {
        if (running) {
            Strongback.submit(stop);
        } else {
            Strongback.submit(start);
        }
        running = !running;
    }
}
