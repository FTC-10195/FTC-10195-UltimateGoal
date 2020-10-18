package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BasicMecanumAutonomous", group = "a")
public class MecanumAuto extends LinearOpMode {
    ElapsedTime timeElapsed;

    // State variables
    private DcMotor fl, fr, bl, br;
    BNO055IMU imu;

    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null);

    public void setup() {
        IMUSetup();
        motorSetup();
        gyroSetup();
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

        robot.resetRobotControlMethods(fl, fr, bl, br, imu);

        fr.setDirection(DcMotor.Direction.REVERSE);
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

    public void runOpMode() {
        timeElapsed = new ElapsedTime();
        setup();
        runRobot();
    }

    public void runRobot() {
        waitForStart();

        //TODO: Test Program
        /*
        if (opModeIsActive() && !isStopRequested()) {
            //Robot instructions go here
            timeElapsed.startTime();
            robot.move("forward", 24, 1);
            sleep(2000);
            robot.move("backward", 24, 1);
            sleep(2000);
            robot.move("left", 24, 1);
            sleep(2000);
            robot.move("right", 24, 1);

            sleep(5000);

            robot.turn("left", 90);
            sleep(2000);
            robot.turn("right", 180);
            sleep(2000);
            robot.turn("left", 90);

            sleep(5000);

            robot.moveInAnyDirection(45, 18, 1);
            sleep(2000);
            robot.moveInAnyDirection(315, 18, 1);
            sleep(2000);
            robot.moveInAnyDirection(225, 18, 1);
            sleep(2000);
            robot.moveInAnyDirection(135, 18, 1);
            sleep(2000);
            robot.moveInAnyDirection(30, 18, 1);
            sleep(2000);
            robot.moveInAnyDirection(300, 18, 1);
            sleep(2000);
            robot.moveInAnyDirection(210, 18, 1);
            sleep(2000);
            robot.moveInAnyDirection(120, 18, 1);

            sleep(5000);

            robot.moveInAnyDirectionWithTurning(90, 180, 36, 1);
            sleep(2000);
            robot.moveInAnyDirectionWithTurning(-90, 180, 36, 1);

            telemetry.addData("Time", timeElapsed.time());
            timeElapsed.reset();
         */

        //TODO: Target Zone A
        /*
        if (opModeIsActive() && !isStopRequested()) {
            robot.move("forward", 64, 1);
            sleep(100);
            robot.move("right", 36,  1);
        }
         */

        //TODO: Target Zone B
        /*
        if (opModeIsActive() && !isStopRequested()) {
            robot.move("forward", 88, 1);
            sleep(100);
            robot.move("right", 15, 1);
        }
         */


        //TODO: Target Zone C
        /*
        if (opModeIsActive() && !isStopRequested()) {
            robot.move("forward", 64, 1);
            sleep(100);
            robot.move("right", 35, 1);
            sleep(100);
            robot.move("forward", 48, 1);
        }
         */
    }
}