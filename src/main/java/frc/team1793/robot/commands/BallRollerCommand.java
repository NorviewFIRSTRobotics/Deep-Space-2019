package frc.team1793.robot.commands;

import frc.team1793.robot.Robot;
import org.strongback.command.Command;
import org.strongback.components.AngleSensor;
import org.strongback.components.Motor;
import org.strongback.components.Switch;
import org.strongback.components.ui.Gamepad;

public class BallRollerCommand extends Command {
    private Motor ballRoller;
    private Switch ballSensor;
    private Gamepad hatchController;
    private double ballRollerSpeed;
    private boolean engageSensor;

    /**
     * @param ballRoller
     *                  motor controller
     * @param ballSensor
     *                  false:  no obstructions
     *                  true:   obstructions (e.g. ball)
     * @param engageSensor
     *                  false:  ignore sensor
     *                  true:   listen for sensor
     * @param hatchController
     *                  Game pad
     * @param ballRollerSpeed Looks for value of -1 or 1 to show which direction is forward and which is backward;
     *                        -1:   backward
     *                        1:    forward.
     */
    public BallRollerCommand(Motor ballRoller, Switch ballSensor, Gamepad hatchController, double ballRollerSpeed, boolean engageSensor) {
        this.ballRoller = ballRoller;
        this.ballSensor = ballSensor;
        this.hatchController = hatchController;
        this.ballRollerSpeed = ballRollerSpeed;
        this.engageSensor = engageSensor;

    }

    @Override
    public boolean execute() {
        while (!Robot.BALL_ROLLER_OVERRIDE.isTriggered()) {
            ballRoller.setSpeed(ballRollerSpeed);
            if (engageSensor && ballSensor.isTriggered()) {
                return true;
            }
        }
        return true;
    }
}
