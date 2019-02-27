package frc.team1793.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1793.robot.commands.BallRollerCommand;
import frc.team1793.robot.commands.SolenoidExtendCommand;
import frc.team1793.robot.commands.SolenoidRetractCommand;
import frc.team1793.robot.utility.SwitchToggle;
import org.strongback.Strongback;
import org.strongback.SwitchReactor;
import org.strongback.command.Command;
import org.strongback.command.CommandGroup;
import org.strongback.components.AngleSensor;
import org.strongback.components.Motor;
import org.strongback.components.Solenoid;
import org.strongback.components.Switch;
import org.strongback.components.ui.ContinuousRange;
import org.strongback.components.ui.FlightStick;
import org.strongback.components.ui.Gamepad;
import org.strongback.drive.TankDrive;
import org.strongback.hardware.Hardware;

import java.util.concurrent.TimeUnit;

public class Robot extends TimedRobot {

    public static TankDrive drive;
    private FlightStick driveController;
    private Gamepad hatchController;
    private ContinuousRange driveSpeed;
    private ContinuousRange turnSpeed;
    private AngleSensor leftEncoder;
    private AngleSensor rightEncoder;
    private Solenoid grabber;
    private Solenoid grabberExtension;
    private Solenoid ballExtension;
    private Switch ballSensor;
    private Motor ballRoller;
    private int direction = 1; //1 = hatch, -1 = ball

    public static Switch BALL_ROLLER_OVERRIDE;
    @Override
    public void robotInit() {

        //Drive
        Motor left = Hardware.Motors.spark(0).invert();
        Motor right = Hardware.Motors.spark(1);
        ballRoller = Hardware.Motors.talonSRX(2);
//        Motor left = Motor.compose(Hardware.Motors.spark(0), Hardware.Motors.spark(1)).invert();
//        Motor right = Motor.compose(Hardware.Motors.spark(2), Hardware.Motors.spark(3));
        drive = new TankDrive(left, right);

        //Sensors
        double dpp = 6 * Math.PI / 1024;
        leftEncoder = Hardware.AngleSensors.encoder(0, 1, dpp);
        rightEncoder = Hardware.AngleSensors.encoder(2, 3, dpp);
        leftEncoder.zero();
        rightEncoder.zero();
        ballSensor = Hardware.Switches.normallyOpen(4);


        //Pneumatics
        grabber = Hardware.Solenoids.solenoid(2, Solenoid.Direction.STOPPED);
        grabberExtension = Hardware.Solenoids.solenoid(0, Solenoid.Direction.STOPPED);
        ballExtension = Hardware.Solenoids.solenoid(1, Solenoid.Direction.STOPPED);

        CameraServer.getInstance().startAutomaticCapture();


        timedControls();
    }

    @Override
    public void robotPeriodic() {
        pushToDashboard();
        if (hatchController.getB().isTriggered()) {
            if (CameraServer.getInstance().getServer().isValid()) {
                CameraServer.getInstance().getServer().close();
            } else {
                CameraServer.getInstance().getServer();
            }
        }
    }


    @Override
    public void autonomousInit() {
        Strongback.disable();
        Strongback.start();
        //Strongback.submit (new SolenoidRetractCommand((org.strongback.components.Solenoid)grabber));
        //Strongback.submit (new SolenoidRetractCommand((org.strongback.components.Solenoid) grabberExtension));
    }


    @Override
    public void teleopInit() {
        Strongback.disable();
        Strongback.start();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        drive.arcade(driveSpeed.read() * direction, turnSpeed.read() * direction);
    }

    @Override
    public void disabledInit() {
        Strongback.disable();
    }


    private void timedControls() {
        driveController = Hardware.HumanInterfaceDevices.logitechAttack3D(0);
        hatchController = Hardware.HumanInterfaceDevices.xbox360(1);

        driveSpeed = driveController.getPitch().scale(0.7);
        turnSpeed = driveController.getRoll().scale(0.5);

        BALL_ROLLER_OVERRIDE = hatchController.getRightStick();

        SwitchReactor switchReactor = Strongback.switchReactor();
//        switchReactor.onTriggered(driveController.getButton(3), () -> Strongback.submit(new ZeroEncoderCommand(leftEncoder, rightEncoder)));
        switchReactor.onTriggered(hatchController.getRightBumper(), new SwitchToggle(new SolenoidRetractCommand(grabber), new SolenoidExtendCommand(grabber))::execute);
        switchReactor.onTriggered(hatchController.getLeftBumper(), new SwitchToggle(new SolenoidRetractCommand(grabberExtension), new SolenoidExtendCommand(grabberExtension))::execute);
        switchReactor.onTriggered(hatchController.getA(), new SwitchToggle(new SolenoidRetractCommand(ballExtension), new SolenoidExtendCommand(ballExtension))::execute);
        //Reverse direction
        switchReactor.onTriggered(driveController.getButton(3), () -> Strongback.submit(() -> direction *= -1));
        //Ball Retriever    Press down Left Stick   Xbox Controller
        switchReactor.onTriggered(hatchController.getLeftStick(), () -> Strongback.submit(new BallRollerCommand(ballRoller, ballSensor, hatchController,1, true)));
       //Low Goal   Button X    Xbox Controller
        switchReactor.onTriggered(hatchController.getX(), () ->
                Strongback.submit(CommandGroup.runSequentially(
                        new SolenoidExtendCommand(ballExtension),
                        Command.pause(1),
                        new BallRollerCommand (ballRoller, ballSensor, hatchController, -0.5, false),
                        new SolenoidRetractCommand(ballExtension)
                ))
        );
        //High Goal     Button Y    Xbox Controller
        switchReactor.onTriggered(hatchController.getY(), () ->
                Strongback.submit(CommandGroup.runSequentially(
                        new SolenoidExtendCommand(ballExtension),
                        Command.pause(1),
                        new BallRollerCommand (ballRoller, ballSensor, hatchController, 0.5, false),
                        new SolenoidRetractCommand(ballExtension)
                ))
        );
    }

    private void pushToDashboard() {
        SmartDashboard.putNumber("driveSpeed", driveSpeed.read());
        SmartDashboard.putNumber("turnSpeed", turnSpeed.read());
        SmartDashboard.putNumber("leftEncoder", leftEncoder.getAngle());
        SmartDashboard.putNumber("rightEncoder", rightEncoder.getAngle());
        SmartDashboard.putNumber("gameTime", Timer.getMatchTime());
        SmartDashboard.putBoolean("ballSensor", ballSensor.isTriggered());
        SmartDashboard.putNumber("direction", direction);
    }
}