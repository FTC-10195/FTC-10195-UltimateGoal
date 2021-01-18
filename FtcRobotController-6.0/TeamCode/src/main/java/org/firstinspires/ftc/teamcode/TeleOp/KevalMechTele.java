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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

@Config
@TeleOp(name = "KevalMechTele", group = "a")
public class KevalMechTele extends OpMode {

    // Configuration parameters
    double slowModePower = 0.35;
    double normalModePower = 0.8;
    double buttonIsPressedThreshold = 0.10;
    public static double shooterSetPower = 0.45;
    double pushServoPosition = 0.3;

    double grabPosition = 0.25;
    double setLiftPower = 0;
    double normalLiftPower = 0.25;
    double wobbleLiftPower = 0.4;

    int cooldown = 500;

    /*
    boolean fieldOriented = false;

     */

    // State variables
    DcMotor fl, fr, bl, br, shooter, intake, wobbleLifter;
    Servo ringPusher, wobbleGrabber;
    BNO055IMU imu;
    private double currentAngle = 0;
    Orientation lastAngles = new Orientation();
    double flPower, frPower, blPower, brPower, intakePower, shooterPower;

    // ElapsedTime variables
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(1,0,0);
    double integral = 0, lastError = 0, error, deltaError, derivative, P, I, D, correction;
    FtcDashboard dashboard;
    double currentShooterPower;

    Double[] ringPusherPositions = {0.3, 0.1, 0.4};
    int currentArrayIndex = 0;

    long lastPressed = 0;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");
        wobbleLifter = hardwareMap.dcMotor.get("lift");

        ringPusher = hardwareMap.servo.get("push");
        wobbleGrabber = hardwareMap.servo.get("grab");

        // TODO: Find which motors to reverse
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMUSetup();
        PIDSetup();

        wobbleGrabber.setPosition(grabPosition);

    }

    public void IMUSetup() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void PIDSetup () {
        
    }

    @Override
    public void loop() {
        /*
        The left joystick to move forward/backward/left/right, right joystick to turn

        gamepad 1 controls movement
        gamepad 2 controls the shooter, intake, and wobble goal
         */

        double y = gamepad1.left_stick_y * -1; // Reversed
        double x = gamepad1.left_stick_x * (sqrt(2)); // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        /*
        if (gamepad1.y) {
            fieldOriented = !fieldOriented;
            currentAngle = 0;
            telemetry.addData("fieldOriented", fieldOriented);
            telemetry.update();
            sleep(50);
        }

        if (fieldOriented) {
            double robotAngleInDegrees = getCurrentAngle();
            double robotAngleInRadians = robotAngleInDegrees * PI/180;
            y = x * sin(robotAngleInRadians) + y * cos(robotAngleInRadians);
            x = y * sin(robotAngleInRadians) + x * cos(robotAngleInRadians);
        }

         */

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

        if(gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up) {
            if (gamepad1.dpad_up) {
                flPower = 0.25;
                frPower = 0.25;
                blPower = 0.25;
                brPower = 0.25;
            }

            if (gamepad1.dpad_down) {
                flPower = -0.25;
                frPower = -0.25;
                blPower = -0.25;
                brPower = -0.25;
            }

            if (gamepad1.dpad_left) {
                flPower = -0.25;
                frPower = 0.25;
                blPower = 0.25;
                brPower = -0.25;
            }

            if (gamepad1.dpad_right) {
                flPower = 0.25;
                frPower = -0.25;
                blPower = -0.25;
                brPower = 0.25;
            }
        }

        if (gamepad2.right_trigger > buttonIsPressedThreshold) {
            //shootRing(shooterSetPower);
        } else {
            integral = 0;
            lastError = 0;
            error = 0;
            deltaError = 0;
            derivative = 0;
            P = 0;
            I = 0;
            D = 0;
            correction = 0;
        }

        if (gamepad2.a) {
            shooterPower = shooterSetPower;
        }

        if (gamepad2.left_trigger > buttonIsPressedThreshold) {
            intakePower = 1;
        }

        if (gamepad2.y) {
            intakePower = -1;
        }

        if (gamepad2.right_bumper && System.currentTimeMillis() - lastPressed > cooldown) {
            lastPressed = System.currentTimeMillis();
            currentArrayIndex++;
            if(currentArrayIndex >= ringPusherPositions.length) {
                currentArrayIndex = 0;
            }
            pushServoPosition = ringPusherPositions[currentArrayIndex];
        }
        else if (gamepad2.left_bumper && System.currentTimeMillis() - lastPressed > cooldown) {
            lastPressed = System.currentTimeMillis();
            currentArrayIndex--;
            if(currentArrayIndex < 0) {
                currentArrayIndex = ringPusherPositions.length - 1;
            }
            pushServoPosition = ringPusherPositions[currentArrayIndex];
        }

        if (gamepad2.dpad_left && System.currentTimeMillis() - lastPressed > cooldown) {
            lastPressed = System.currentTimeMillis();
            // Grab wobble goal
            wobbleGrabber.setPosition(0.1);
        }

        if (gamepad2.dpad_right && System.currentTimeMillis() - lastPressed > cooldown) {
            lastPressed = System.currentTimeMillis();
            // Release wobble goal
            wobbleGrabber.setPosition(0.5);
        }

        if (gamepad2.b) {
            setLiftPower = wobbleLiftPower;
        } else {
            setLiftPower = normalLiftPower;
        }

        if (gamepad2.dpad_up) {
            wobbleLifter.setPower(setLiftPower);
        } else if (gamepad2.dpad_down) {
            wobbleLifter.setPower(-setLiftPower);
        } else {
            wobbleLifter.setPower(0);
        }

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        intake.setPower(intakePower);
        shooter.setPower(shooterPower);

        ringPusher.setPosition(pushServoPosition);

        telemetry.addData("Push Position", ringPusher.getPosition());
        telemetry.addData("Grab Position", wobbleGrabber.getPosition());
        telemetry.update();

        intakePower = 0;
        shooterPower = 0;
    }

/*
    private void shootRing(double targetPower) {
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        currentShooterPower = shooter.getPower();
        dashboardTelemetry.addData("Shooter Power", currentShooterPower);
        dashboardTelemetry.update();

        error = targetPower - currentShooterPower;
        deltaError = error - lastError;
        integral += error * PIDTimer.time();

        derivative = deltaError / PIDTimer.time();
        P = pidCoefficients.p * error;
        I = pidCoefficients.i * integral;
        D = pidCoefficients.d * derivative;
        correction = P + I + D;

        shooter.setPower(targetPower + correction);
        lastError = error;
        PIDTimer.reset();
    }

 */

    /*
    private double getCurrentAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double changeInAngle = angles.firstAngle - lastAngles.firstAngle;
        if (currentAngle > 179){
            currentAngle -= 360;
        } else if(currentAngle < -180){
            currentAngle += 360;
        } else if(currentAngle > 360){
            currentAngle -= 360;
        }
        currentAngle += changeInAngle;
        lastAngles = angles;

        return currentAngle;
    }

     */
}