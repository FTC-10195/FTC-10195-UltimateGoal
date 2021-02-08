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

import static android.os.SystemClock.sleep;

@Config
@TeleOp(name = "ShooterPIDFTest", group = "a")
public class ShooterPIDFTest extends OpMode {

    public enum ShooterState {
        START_SHOOTER,
        SHOOT_RINGS,
        STOP_SHOOTER
    }

    // Configuration parameters
    public static double shooterSetPower = 0.45;
    public static double powerShotPower = 0.37;
    public static double pushServoPosition = 0.3;
    public static double setLiftPower = 0;
    public static int cooldown = 250;
    public static Double[] ringPusherPositions = {0.1, 0.4};
    public static Integer[] cooldowns = {750, 750};

    // State variables
    DcMotorEx fl, fr, bl, br, shooter, topIntake, bottomIntake, wobbleLifter;
    Servo ringPusher, wobbleGrabber;
    double flPower, frPower, blPower, brPower, topIntakePower, bottomIntakePower, shooterPower;

    // ElapsedTime variables
    ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime ringPusherTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    // FTC Dashboard helps edit variables on the fly and graph telemetry values
    FtcDashboard dashboard;
    TelemetryPacket packet;

    // Shooter/intake variables
    int shooterCooldown = cooldowns[0];
    int ringPusherIteration = 1;
    int currentArrayIndex = 0;
    boolean isShooterOnForward = false;
    boolean isShooterOnBackward = false;
    boolean isIntakeOnForward = false;
    boolean isIntakeOnBackward = false;
    boolean powerShot = false;
    long intakeLastPressed = 0;
    long shooterLastPressed = 0;

    long wobbleLastPressed = 0;

    ShooterState shooterState = ShooterState.START_SHOOTER;

    // Allows me to import the RPM and ticks per rotation variables
    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null, null, null, null, null,
            null, null);


    ElapsedTime shooterPIDFTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double currentTime = 0;
    double previousTime = 0;
    double deltaTime = 0;
    int currentTicks = 0;
    int previousTicks = 0;
    int deltaTicks = 0;
    double velocity = 0;

    static ElapsedTime shooterRotation = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static double setShooterPower;
    public static double currentShooterVelocity;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        // Set up the motors and servos
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        topIntake = hardwareMap.get(DcMotorEx.class, "topRoller");
        bottomIntake = hardwareMap.get(DcMotorEx.class, "bottomRoller");
        ringPusher = hardwareMap.get(Servo.class, "push");

        // TODO: Find which motors to reverse
        topIntake.setDirection(DcMotorEx.Direction.REVERSE);

        // Set the motors to stay in place when a power of 0 is passed
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(32767 / ((shooterSetPower) * 29120),
                32767 / ((shooterSetPower) * 291200), 0, 32767 / ((shooterSetPower) * 2912));

        TelemetryPacket packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        // region Gamepad 2

        // Button to toggle between power shots and regular shooting
        if (gamepad2.a && System.currentTimeMillis() - shooterLastPressed > cooldown) {
            powerShot = !powerShot;
        }

        // Toggle shooter and intake
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
                switch (ringPusherIteration % 2) {
                    case 0:
                        shooterCooldown = cooldowns[1];
                        break;

                    case 1:
                        shooterCooldown = cooldowns[0];
                        break;
                }

                if (ringPusherTimer.time(TimeUnit.MILLISECONDS) >= shooterCooldown && ringPusherIteration <= 6) {
                    currentArrayIndex++;
                    if (currentArrayIndex >= ringPusherPositions.length) {
                        currentArrayIndex = 0;
                    }
                    pushServoPosition = ringPusherPositions[currentArrayIndex];
                    ringPusherTimer.reset();
                    ringPusherIteration++;
                } else if (ringPusherIteration > 6) {
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
        topIntake.setPower(topIntakePower);
        bottomIntake.setPower(bottomIntakePower);

        telemetry.addData("shooter velocity", shooterPower * (robot.SHOOTER_TICKS_PER_ROTATION * (robot.SHOOTER_MAX_RPM / 60)));

        ringPusher.setPosition(pushServoPosition);

        // Telemetry data to assist drivers
        telemetry.addData("State Machine", shooterState);
        telemetry.addData("Power Shot?", powerShot);
        telemetry.update();

        setShooterVelocity();
    }

    public double calculateShooterVelocity() {
        currentTime = shooterPIDFTimer.time(TimeUnit.SECONDS);
        deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        currentTicks = shooter.getCurrentPosition();
        deltaTicks = currentTicks - previousTicks;
        previousTicks = currentTicks;

        velocity = deltaTicks / deltaTime;
        return velocity;
    }

    public void setShooterVelocity() {
        while (isShooterOnForward) {
            setShooterPower = Math.sin(shooterRotation.time(TimeUnit.SECONDS)) / 3 + 0.5;
            currentShooterVelocity = calculateShooterVelocity();
            telemetry.addData("setShooterPower", setShooterPower);
            telemetry.addData("currentShooterVelocity", currentShooterVelocity);
            shooter.setVelocity(setShooterPower * (robot.SHOOTER_TICKS_PER_ROTATION * (robot.SHOOTER_MAX_RPM / 60)));
            sleep(2500);
            packet.put("Set Shooter Velocity", setShooterPower);
            packet.put("Current Shooter Velocity", currentShooterVelocity);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}