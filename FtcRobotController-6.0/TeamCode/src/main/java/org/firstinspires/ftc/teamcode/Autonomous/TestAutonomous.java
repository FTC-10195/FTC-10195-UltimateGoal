package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TESTING", group = "b")
public class TestAutonomous extends LinearOpMode {
    ElapsedTime timeElapsed;
    String zone = "A";

    // State variables
    private DcMotor fl, fr, bl, br;
    BNO055IMU imu;

    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null);

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
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void gyroSetup() {
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }

    private void RobotControlMethodsSetup() {
        robot.resetRobotControlMethods(fl, fr, bl, br, imu);
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


            robot.turn("left", 90);
            sleep(500);
            robot.turn("right", 180);
            sleep(500);
            robot.turn("left", 90);

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