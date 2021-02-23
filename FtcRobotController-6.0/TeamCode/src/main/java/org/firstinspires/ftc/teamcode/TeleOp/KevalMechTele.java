/*
    Ideas to Implement:

    Automate some parts of teleop
        Automate tower shooting (power and POSSIBLY location)
        Automate power shooting (either turn the robot just enough or move the robot just enough)

    Use slowmode with other areas of the robot, like slides

 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.RobotControlMethods;

import java.util.concurrent.TimeUnit;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

@Config
@TeleOp(name = "KevalMechTele", group = "a")
public class KevalMechTele extends OpMode {

    public enum ShooterState {
        START_SHOOTER,
        SHOOT_RINGS,
        STOP_SHOOTER
    }

    // Configuration parameters
    public static double slowModePower = 0.25;
    public static double normalModePower = 0.8;
    public static double buttonIsPressedThreshold = 0.10;
    public static double shooterSetPower = 0.8;
    public static double powerShotPower = 0.6;
    public static int buttonPressCooldown = 250;
    public static Double[] ringPusherPositions = {0.75, 0.5, 0.3};
    public static Integer[] cooldowns = {300, 300, 400};
    public static double pushServoPosition = ringPusherPositions[0];
    public static double setLiftPower = 0;
    public static double wobbleLiftPower = 0.5;
    public static double wobbleGrabPosition = 0;
    public static double wobbleReleasePosition = 0.7;
    public static double wobbleGrabberPosition = wobbleReleasePosition;

    // State variables
    DcMotorEx fl, fr, bl, br, shooter, topIntake, bottomIntake, wobbleLifter;
    Servo ringPusher, wobbleGrabber;
    double flPower, frPower, blPower, brPower, topIntakePower, bottomIntakePower, shooterPower;

    // ElapsedTime variables
    ElapsedTime ringPusherTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // FTC Dashboard helps edit variables on the fly and graph telemetry values
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    // Shooter/intake variables
    int shooterWindup = 2000;
    int shooterCooldown = cooldowns[0];
    int ringPusherIteration = 1;
    int currentArrayIndex = 0;
    boolean isShooterOnForward = false;
    boolean isShooterOnBackward = false;
    boolean isIntakeOnForward = false;
    boolean isIntakeOnBackward = false;
    boolean powerShot = false;
    long intakeLastPressed = 0;
    static long shooterLastPressed = 0;

    long wobbleLastPressed = 0;

    ShooterState shooterState = ShooterState.START_SHOOTER;

    // Allows me to import the RPM and ticks per rotation variables
    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null, null, null, null, null,
            null, null);

    int currentTicks, previousTicks, deltaTicks;
    double currentTime, previousTime, deltaTime, velocity;
    ElapsedTime shooterPIDFTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {
        // Set up the motors and servos
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        topIntake = hardwareMap.get(DcMotorEx.class, "topRoller");
        bottomIntake = hardwareMap.get(DcMotorEx.class, "bottomRoller");
        wobbleLifter = hardwareMap.get(DcMotorEx.class, "lift");
        ringPusher = hardwareMap.get(Servo.class, "push");
        wobbleGrabber = hardwareMap.get(Servo.class, "grab");

        // TODO: Find which motors to reverse
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        topIntake.setDirection(DcMotorEx.Direction.REVERSE);
        wobbleLifter.setDirection(DcMotorEx.Direction.REVERSE);

        // Set the motors to stay in place when a power of 0 is passed
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wobbleLifter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(1.13, 0.11, 0, 11.25);

        ringPusher.setPosition(ringPusherPositions[0]);
    }

    @Override
    public void loop() {
        /*
        The left joystick to move forward/backward/left/right, right joystick to turn

        gamepad 1 controls movement and wobble goal
        gamepad 2 controls the shooter and intake
         */

        // region Gamepad 1

        double y = gamepad1.left_stick_y * -1; // Reversed
        double x = gamepad1.left_stick_x * (sqrt(2)); // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        flPower = (normalModePower) * (y + x - rx);
        frPower = (normalModePower) * (y - x + rx);
        blPower = (normalModePower) * (y - x - rx);
        brPower = (normalModePower) * (y + x + rx);

        // Normalizes all values back to 1
        if (abs(flPower) > 1 || abs(blPower) > 1 || abs(frPower) > 1 || abs(brPower) > 1) {
            // Find the largest power
            double max;
            max = Math.max(abs(flPower), abs(blPower));
            max = Math.max(abs(frPower), max);
            max = Math.max(abs(brPower), max);

            max = abs(max);

            // Divide everything by max (it's positive so we don't need to worry about signs)
            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
        }

        // Slow mode
        if (gamepad1.left_trigger > buttonIsPressedThreshold) {
            flPower *= slowModePower;
            frPower *= slowModePower;
            blPower *= slowModePower;
            brPower *= slowModePower;
        }

        // Wobble goal up/down
        if (gamepad1.right_bumper) {
            setLiftPower = wobbleLiftPower;
        } else if (gamepad1.left_bumper) {
            setLiftPower = -wobbleLiftPower;
        } else {
            setLiftPower = 0;
        }

        // Wobble goal grab/release
        if (gamepad1.dpad_left) {
            // Grab wobble goal
            wobbleGrabberPosition = wobbleGrabPosition;
        }

        if (gamepad1.dpad_right) {
            // Release wobble goal
            wobbleGrabberPosition = wobbleReleasePosition;
        }

        // endregion


        // region Gamepad 2

        // Button to toggle between power shots and regular shooting
        if (gamepad2.a && System.currentTimeMillis() - shooterLastPressed > buttonPressCooldown) {
            powerShot = !powerShot;
        }

        // Toggle shooter and intake
        if (gamepad2.dpad_up && System.currentTimeMillis() - shooterLastPressed > buttonPressCooldown)  {
            shooterLastPressed = System.currentTimeMillis();
            isShooterOnForward = !isShooterOnForward;
            isShooterOnBackward = false;
        } else if (gamepad2.dpad_down && System.currentTimeMillis() - shooterLastPressed > buttonPressCooldown) {
            shooterLastPressed = System.currentTimeMillis();
            isShooterOnBackward = !isShooterOnBackward;
            isShooterOnForward = false;
        }

        if (gamepad2.dpad_right && System.currentTimeMillis() - intakeLastPressed > buttonPressCooldown) {
            intakeLastPressed = System.currentTimeMillis();
            isIntakeOnForward = !isIntakeOnForward;
            isIntakeOnBackward = false;
        } else if (gamepad2.dpad_left && System.currentTimeMillis() - intakeLastPressed > buttonPressCooldown) {
            intakeLastPressed = System.currentTimeMillis();
            isIntakeOnBackward = !isIntakeOnBackward;
            isIntakeOnForward = false;
        }

        // Sets shooter power
        if (isShooterOnForward && !powerShot) {
            shooterPower = shooterSetPower;
        } else if (isShooterOnForward && powerShot){
            shooterPower = powerShotPower;
        } else if (isShooterOnBackward) {
            shooterPower = -shooterSetPower;
        } else {
            shooterPower = 0;
        }

        // Sets intake power
        if (isIntakeOnForward) {
            topIntakePower = 1;
            bottomIntakePower = 1;
        } else if (isIntakeOnBackward) {
            topIntakePower = -1;
            bottomIntakePower = -1;
        } else {
            topIntakePower = 0;
            bottomIntakePower = 0;
        }

        // Iterates through array for the ring pusher servo
        if (gamepad2.right_bumper && System.currentTimeMillis() - wobbleLastPressed > buttonPressCooldown) {
            wobbleLastPressed = System.currentTimeMillis();
            currentArrayIndex++;
            if(currentArrayIndex >= ringPusherPositions.length) {
                currentArrayIndex = 0;
            }
            pushServoPosition = ringPusherPositions[currentArrayIndex];
        }
        else if (gamepad2.left_bumper && System.currentTimeMillis() - wobbleLastPressed > buttonPressCooldown) {
            wobbleLastPressed = System.currentTimeMillis();
            currentArrayIndex--;
            if(currentArrayIndex < 0) {
                currentArrayIndex = ringPusherPositions.length - 1;
            }
            pushServoPosition = ringPusherPositions[currentArrayIndex];
        }

        // Automatically shoots all three rings
        switch (shooterState) {
            case START_SHOOTER:
                if (gamepad2.x) {
                    shooterPower = shooterSetPower;
                    isShooterOnForward = true;
                    pushServoPosition = ringPusherPositions[0];
                    shooterTimer.reset();
                    shooterState = ShooterState.SHOOT_RINGS;
                }

                break;

            case SHOOT_RINGS:
                // Different shooter cooldowns for each iteration
                switch (ringPusherIteration % 3) {
                    case 0:
                        shooterCooldown = cooldowns[2];
                        break;

                    case 1:
                        shooterCooldown = cooldowns[0];
                        break;

                    case 2:
                        shooterCooldown = cooldowns[1];
                        break;
                }

                if (shooterTimer.time(TimeUnit.MILLISECONDS) > shooterWindup &&
                        ringPusherTimer.time(TimeUnit.MILLISECONDS) >= shooterCooldown &&
                        ringPusherIteration <= 9) {
                    currentArrayIndex++;
                    if (currentArrayIndex >= ringPusherPositions.length) {
                        currentArrayIndex = 0;
                    }
                    pushServoPosition = ringPusherPositions[currentArrayIndex];
                    ringPusherTimer.reset();
                    ringPusherIteration++;
                } else if (ringPusherIteration > 9) {
                    shooterState = ShooterState.STOP_SHOOTER;
                }
                break;

            case STOP_SHOOTER:
                shooterPower = 0;
                isShooterOnForward = false;
                isShooterOnBackward = false;
                ringPusherIteration = 1;
                pushServoPosition = ringPusherPositions[0];
                currentArrayIndex = 0;
                shooterState = ShooterState.START_SHOOTER;
                break;
        }

        // Cancels state machine
        if (gamepad2.b && shooterState != ShooterState.STOP_SHOOTER) {
            shooterState = ShooterState.STOP_SHOOTER;
        }

        //endregion

        // Sets all powers and servo positions
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        topIntake.setPower(topIntakePower);
        bottomIntake.setPower(bottomIntakePower);
        wobbleLifter.setPower(setLiftPower);

        shooter.setVelocity(shooterPower * (robot.SHOOTER_TICKS_PER_ROTATION * (robot.SHOOTER_MAX_RPM / 60)));

        ringPusher.setPosition(pushServoPosition);
        wobbleGrabber.setPosition(wobbleGrabberPosition);

        // Telemetry data to assist drivers
        telemetry.addData("Push Position", ringPusher.getPosition());
        telemetry.addData("Grab Position", wobbleGrabber.getPosition());
        telemetry.addData("State Machine", shooterState);
        telemetry.addData("Power Shot?", powerShot);
        telemetry.addData("Wobble Grabber Position", wobbleGrabberPosition);
        telemetry.update();

        packet.put("Current Shooter Velocity", calculateShooterVelocity());
        packet.put("Set Shooter Velocity", shooter.getVelocity());
        dashboard.sendTelemetryPacket(packet);
    }

    public double calculateShooterVelocity() {
        currentTicks = shooter.getCurrentPosition();
        deltaTicks = currentTicks - previousTicks;
        previousTicks = currentTicks;

        currentTime = shooterPIDFTimer.time(TimeUnit.MILLISECONDS);
        deltaTime = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        velocity = deltaTicks / deltaTime;
        return velocity;
    }
}