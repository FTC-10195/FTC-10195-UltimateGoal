package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TESTING", group = "b")
public class TestAutonomous extends LinearOpMode {
    ElapsedTime timeElapsed;
    String zone = "A";

    // State variables
    private DcMotorEx fl, fr, bl, br, shooter, topIntake, bottomIntake, wobbleLifter;
    Servo ringPusher, wobbleGrabber;
    BNO055IMU imu;

    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null, null, null, null, null, null, null);

    public void setup() {
        IMUSetup();
        motorSetup();
        gyroSetup();
        RobotControlMethodsSetup();
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

    public void motorSetup() {
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

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        topIntake.setDirection(DcMotorEx.Direction.REVERSE);
        bottomIntake.setDirection(DcMotorEx.Direction.REVERSE);
        wobbleLifter.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void gyroSetup() {
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }

    private void RobotControlMethodsSetup() {
        robot.resetRobotControlMethods(fl, fr, bl, br, shooter, topIntake, bottomIntake, wobbleLifter, ringPusher, wobbleGrabber, imu);
    }

    public void runOpMode() throws InterruptedException {
        timeElapsed = new ElapsedTime();
        setup();
        runRobot();
    }

    public void runRobot() throws InterruptedException {
        waitForStart();

        //TODO: Test Program

        if (opModeIsActive() && !isStopRequested()) {
            //Robot instructions go here
            timeElapsed.startTime();
            robot.move("forward", 24, 1);
            sleep(500);
            robot.move("backward", 24, 1);
            sleep(500);
            robot.move("left", 24, 1);
            sleep(500);
            robot.move("right", 24, 1);

            sleep(2000);


            robot.turn("left", 90, 1);
            sleep(500);
            robot.turn("right", 180, 1);
            sleep(500);
            robot.turn("left", 90, 1);

            sleep(2000);



            robot.moveInAnyDirection(45, 36, 1);
            sleep(500);
            robot.moveInAnyDirection(315, 36, 1);
            sleep(500);
            robot.moveInAnyDirection(225, 36, 1);
            sleep(500);
            robot.moveInAnyDirection(135, 36, 1);
            sleep(500);

            sleep(5000);

            robot.moveInAnyDirection(30, 18, 1);
            sleep(500);
            robot.moveInAnyDirection(300, 18, 1);
            sleep(500);
            robot.moveInAnyDirection(210, 18, 1);
            sleep(500);
            robot.moveInAnyDirection(120, 18, 1);

            sleep(2000);

            robot.moveInAnyDirectionWithTurning(90, 180, 36, 1);
            sleep(500);
            robot.moveInAnyDirectionWithTurning(-90, 180, 36, 1);

            telemetry.addData("Time", timeElapsed.time());
            timeElapsed.reset();
        }
    }
}