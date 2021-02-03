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
    };

    // Configuration parameters
    double slowModePower = 0.35;
    double normalModePower = 0.8;
    double buttonIsPressedThreshold = 0.10;
    public static double shooterSetPower = 0.7;
    double pushServoPosition = 0.3;

    double grabPosition = 0.25;
    double setLiftPower = 0;
    double normalLiftPower = 0.25;
    double wobbleLiftPower = 0.4;
    double wobbleGrabberPosition = 0;

    public static int ringPusherIteration = 1;

    int cooldown = 250;
    Integer[] cooldowns = {600, 100, 250};
    int shooterCooldown = cooldowns[0];

    // State variables
    DcMotorEx fl, fr, bl, br, shooter, topIntake, bottomIntake, wobbleLifter;
    Servo ringPusher, wobbleGrabber;
    double flPower, frPower, blPower, brPower, topIntakePower, bottomIntakePower, shooterPower;

    // ElapsedTime variables
    ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime ringPusherTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    FtcDashboard dashboard;

    Double[] ringPusherPositions = {0.3, 0.1, 0.4};
    int currentArrayIndex = 0;

    long wobbleLastPressed = 0;
    long shooterLastPressed = 0;
    long intakeLastPressed = 0;

    boolean isShooterOnForward = false;
    boolean isShooterOnBackward = false;
    boolean isIntakeOnForward = false;
    boolean isIntakeOnBackward = false;

    ShooterState shooterState = ShooterState.START_SHOOTER;

    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null, null, null, null, null,
            null, null);

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

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

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wobbleLifter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        wobbleGrabber.setPosition(grabPosition);
    }

    @Override
    public void loop() {
        /*
        The left joystick to move forward/backward/left/right, right joystick to turn

        gamepad 1 controls movement
        gamepad 2 controls the shooter, intake, and wobble goal
         */

        //region Gamepad 1

        double y = gamepad1.left_stick_y * -1; // Reversed
        double x = gamepad1.left_stick_x * (sqrt(2)); // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        flPower = (normalModePower) * (y + x - rx);
        frPower = (normalModePower) * (y - x + rx);
        blPower = (normalModePower) * (y - x - rx);
        brPower = (normalModePower) * (y + x + rx);

        if (abs(flPower) > 1 || abs(blPower) > 1 ||
                abs(frPower) > 1 || abs(brPower) > 1 ) {
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

        if (gamepad1.left_trigger > buttonIsPressedThreshold){
            flPower *= slowModePower;
            frPower *= slowModePower;
            blPower *= slowModePower;
            brPower *= slowModePower;
        }

        if (gamepad1.b && gamepad1.dpad_up) {
            setLiftPower = wobbleLiftPower;
        } else if (gamepad1.dpad_up) {
            setLiftPower = normalLiftPower;
        } else if (gamepad1.dpad_down) {
            setLiftPower = -wobbleLiftPower;
        } else {
            setLiftPower = 0;
        }

        if (gamepad1.dpad_left && System.currentTimeMillis() - wobbleLastPressed > cooldown) {
            wobbleLastPressed = System.currentTimeMillis();
            // Grab wobble goal
            wobbleGrabberPosition = 0;
        }

        if (gamepad1.dpad_right && System.currentTimeMillis() - wobbleLastPressed > cooldown) {
            wobbleLastPressed = System.currentTimeMillis();
            // Release wobble goal
            wobbleGrabberPosition = 0.5;
        }

        //endregion


        //region Gamepad 2

        if (gamepad2.dpad_up && System.currentTimeMillis() - shooterLastPressed > cooldown)  {
            shooterLastPressed = System.currentTimeMillis();
            isShooterOnForward = !isShooterOnForward;
            isShooterOnBackward = false;
        } else if (gamepad2.dpad_down && System.currentTimeMillis() - shooterLastPressed > cooldown) {
            shooterLastPressed = System.currentTimeMillis();
            isShooterOnBackward = !isShooterOnBackward;
            isShooterOnForward = false;
        }

        if (gamepad2.dpad_right && System.currentTimeMillis() - intakeLastPressed > cooldown) {
            intakeLastPressed = System.currentTimeMillis();
            isIntakeOnForward = !isIntakeOnForward;
            isIntakeOnBackward = false;
        } else if (gamepad2.dpad_left && System.currentTimeMillis() - intakeLastPressed > cooldown) {
            intakeLastPressed = System.currentTimeMillis();
            isIntakeOnBackward = !isIntakeOnBackward;
            isIntakeOnForward = false;
        }

        if (isShooterOnForward) {
            shooterPower = shooterSetPower;
        } else if (isShooterOnBackward) {
            shooterPower = -shooterSetPower;
        } else {
            shooterPower = 0;
        }

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

        if (gamepad2.right_bumper && System.currentTimeMillis() - wobbleLastPressed > cooldown) {
            wobbleLastPressed = System.currentTimeMillis();
            currentArrayIndex++;
            if(currentArrayIndex >= ringPusherPositions.length) {
                currentArrayIndex = 0;
            }
            pushServoPosition = ringPusherPositions[currentArrayIndex];
        }
        else if (gamepad2.left_bumper && System.currentTimeMillis() - wobbleLastPressed > cooldown) {
            wobbleLastPressed = System.currentTimeMillis();
            currentArrayIndex--;
            if(currentArrayIndex < 0) {
                currentArrayIndex = ringPusherPositions.length - 1;
            }
            pushServoPosition = ringPusherPositions[currentArrayIndex];
        }

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

                if (ringPusherTimer.time(TimeUnit.MILLISECONDS) >= shooterCooldown && ringPusherIteration <= 9) {
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
                ringPusherIteration = 1;
                pushServoPosition = ringPusherPositions[0];
                currentArrayIndex = 0;
                shooterState = ShooterState.START_SHOOTER;
                break;
        }

        if (gamepad2.b && shooterState != ShooterState.STOP_SHOOTER) {
            shooterState = ShooterState.STOP_SHOOTER;
        }

        //endregion

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        topIntake.setPower(topIntakePower);
        bottomIntake.setPower(bottomIntakePower);
        wobbleLifter.setPower(setLiftPower);

        shooter.setPower(shooterPower);

        ringPusher.setPosition(pushServoPosition);
        wobbleGrabber.setPosition(wobbleGrabberPosition);

        telemetry.addData("Push Position", ringPusher.getPosition());
        telemetry.addData("Grab Position", wobbleGrabber.getPosition());
        telemetry.addData("State Machine", shooterState);
        telemetry.update();

        topIntakePower = 0;
    }
}