package frc.team1793.robot.commands;

import org.strongback.command.Command;
import org.strongback.components.Motor;
import org.strongback.components.Switch;
import org.strongback.components.ui.ContinuousRange;

public class BallRollerCommand extends Command {
    private Motor ballRoller;
    private Switch ballSensor;
    private ContinuousRange ballRollerSpeed;
    private boolean engageSensor;

    /**
     * @param ballRoller      motor controller
     * @param ballSensor      false:  no obstructions
     *                        true:   obstructions (e.g. ball)
     * @param ballRollerSpeed Looks for value of -1 or 1 to show which direction is forward and which is backward;
 *                        -1:   backward
     */
    public BallRollerCommand(Motor ballRoller, Switch ballSensor, ContinuousRange ballRollerSpeed) {
        this.ballRoller = ballRoller;
        this.ballSensor = ballSensor;
        this.ballRollerSpeed = ballRollerSpeed;

    }

    @Override
    public boolean execute() {
        System.out.println("execute");
        ballRoller.setSpeed(ballRollerSpeed.read());
        return true;
    }

    @Override
    public void end() {
//        ballRoller.stop();
        System.out.println("stop");
    }

    @Override
    public void interrupted() {
        ballRoller.stop();
        System.out.println("interr");
    }
}
