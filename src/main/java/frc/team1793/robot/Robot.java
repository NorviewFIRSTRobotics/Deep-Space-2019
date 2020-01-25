package frc.team1793.robot;

/**
 * These allow us to access code from other sources such as FRC, Tyler, and other vendors.
 */

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1793.robot.commands.BallRollerCommand;
import frc.team1793.robot.commands.SolenoidPrimaryCommand;
import frc.team1793.robot.commands.SolenoidSecondaryCommand;
import frc.team1793.robot.utility.SwitchToggle;
import org.strongback.Strongback;
import org.strongback.SwitchReactor;
import org.strongback.command.Command;
import org.strongback.components.AngleSensor;
import org.strongback.components.Motor;
import org.strongback.components.Solenoid;
import org.strongback.components.Switch;
import org.strongback.components.ui.ContinuousRange;
import org.strongback.components.ui.FlightStick;
import org.strongback.components.ui.Gamepad;
import org.strongback.drive.TankDrive;
import org.strongback.hardware.Hardware;
import org.strongback.hardware.Hardware2019;

/**
 * This class states all of the fields to be called later on within the code.
 */
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

    /**
     * This function runs when the robot is started and creates the base line code for you to use later on in other functions.
     */
    @Override
    public void robotInit() {

        //Drive
        Motor left = Hardware.Motors.spark(0).invert();
        Motor right = Hardware.Motors.spark(1);
        ballRoller = Hardware.Motors.talonSRX(2);
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

        //Cameras
//        UsbCamera = CameraServer.getInstance().startAutomaticCapture();
//        camera.setResolution(360, 360);
//
//        visionThread = new VisionThread(camera, new GRIPPipeline(), pipeline -> {
//            if (!pipeline.hslThresholdOutput().isEmpty()){
//                Rect r = Imgproc.boundingRect(pipeline.hslThresholdOutput().get(0));
//                synchronized (imgLock){
//                    centerX = r.x + (r.width / 2);
//                }
//            }
//        });
//        visionThread.start();
        CameraServer.getInstance().startAutomaticCapture();
        CameraServer.getInstance().startAutomaticCapture();

        timedControls();
    }

    /**
     * This function is called periodically when the robot is turned on.
     */
    @Override
    public void robotPeriodic() {
        pushToDashboard();
    }

    /**
     * This function allows for the control of the drive system of the robot.
     */
    public void drive() {
        drive.arcade(driveSpeed.scale(direction).read(), turnSpeed.read());
    }

    /**
     * This function runs at the start of the Autonomous phase in order to clear out code in the command line and start the robot code.
     * The function below opens the hatch claw initially and adds the open hatch claw command to the queue.
     */
    @Override
    public void autonomousInit() {
        Strongback.disable();
        Strongback.start();
        Command openHatchClaw = new SolenoidSecondaryCommand(grabber);
        Strongback.submit(openHatchClaw);
//        double centerX;
//        synchronized (imgLock){
//            centerX = this.centerX;
//        }
//        double turn = centerX - (360/2);

    }

    /**
     * This function runs at the start of the TeleOperated phase to clear out code in the command line and start the robot code.
     */
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
        drive();
    }

    /**
     * This function is called periodically during the autonomous period.
     */
    @Override
    public void autonomousPeriodic() {
        drive();
    }

    /**
     * This function kills and flushes out code in the command list.
     */
    @Override
    public void disabledInit() {
        Strongback.disable();
    }

    /**
     * This function starts and handles the controls for the robot.
     * Put code here for drive functions.
     */
    private void timedControls() {
        driveController = Hardware.HumanInterfaceDevices.logitechAttack3D(0);
        hatchController = Hardware.HumanInterfaceDevices.xbox360(1);

        driveSpeed = driveController.getPitch().scale(driveController.getTrigger().isTriggered() ? 0.33 : 1);
        turnSpeed = driveController.getRoll().scale(1);

        BALL_ROLLER_OVERRIDE = hatchController.getRightStick();

        SwitchReactor switchReactor = Strongback.switchReactor();

        /**
         *Reverse direction.
         */
        switchReactor.onTriggered(driveController.getButton(3), () -> Strongback.submit(() -> direction *= -1));

        /**
         * Press Right Bumper to extend and retract grabber piston.
         */
        switchReactor.onTriggered(hatchController.getRightBumper(), new SwitchToggle(new SolenoidPrimaryCommand(grabber), new SolenoidSecondaryCommand(grabber))::execute);

        /**
         * Press Left Bumper to extend and retract the extension piston for grabber.
         */
        switchReactor.onTriggered(hatchController.getLeftBumper(), new SwitchToggle(new SolenoidPrimaryCommand(grabberExtension), new SolenoidSecondaryCommand(grabberExtension))::execute);

        /**
         * Press B to extend ball piston.
         */
        switchReactor.onTriggered(hatchController.getB(), new SwitchToggle(new SolenoidPrimaryCommand(ballExtension), new SolenoidSecondaryCommand(ballExtension))::execute);

        /**
         * Hold X to roll ball down the ramp.
         */
        switchReactor.onTriggered(hatchController.getX(), new SwitchToggle(new BallRollerCommand(ballRoller, ballSensor, () -> 1), Command.create(() -> ballRoller.stop()))::execute);
        /*switchReactor.onTriggered(hatchController.getX(), () -> Strongback.submit(
                new SwitchToggle(new BallRollerCommand(ballRoller, ballSensor, () -> -1), Command.create(() -> ballRoller.stop())) {
                    @Override
                    public void execute() {
                        System.out.println(ballSensor.isTriggered());
                        if (ballSensor.isTriggered()) {
                            this.running = false;
                        }

                        super.execute();
                    }
                }::execute));*/

        /**
         * Hold Y to pick up ball and move it up ramp.
         */
        switchReactor.onTriggered(hatchController.getY(), new SwitchToggle(new BallRollerCommand(ballRoller, ballSensor, () -> -1), Command.create(() -> ballRoller.stop()))::execute);

        /**
         * Hold A to release ball into Low Goal.
         */
        switchReactor.onTriggered(hatchController.getA(), new SwitchToggle(new BallRollerCommand(ballRoller, ballSensor, () -> 1), Command.create(() -> ballRoller.stop()))::execute);
    }

    /**
     * This function displays specified values to SmartDashboard.
     */
    private void pushToDashboard() {
        SmartDashboard.putNumber("driveSpeed", driveSpeed.read());
        SmartDashboard.putNumber("turnSpeed", turnSpeed.read());
        SmartDashboard.putNumber("leftEncoder", leftEncoder.getAngle());
        SmartDashboard.putNumber("rightEncoder", rightEncoder.getAngle());
        SmartDashboard.putNumber("gameTime", Timer.getMatchTime());
        SmartDashboard.putBoolean("ballSensor", ballSensor.isTriggered());
        SmartDashboard.putString("direction", direction == 1 ? "Hatch Side" : "Cargo Side" + "");
    }
}