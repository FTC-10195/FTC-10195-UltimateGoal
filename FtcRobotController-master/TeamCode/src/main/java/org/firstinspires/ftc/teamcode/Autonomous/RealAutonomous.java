package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.VuforiaTest;

@Autonomous(name = "RealAutonomous", group = "a")
public class RealAutonomous extends LinearOpMode {
    ElapsedTime timeElapsed;
    String zone = "A";

    // State variables
    private DcMotor fl, fr, bl, br;
    BNO055IMU imu;

    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null);
    VuforiaTest vuforia = new VuforiaTest();

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

    private void RobotControlMethodsSetup() {
        robot.resetRobotControlMethods(fl, fr, bl, br, imu);
    }

    public void runOpMode() {
        timeElapsed = new ElapsedTime();
        setup();
        runRobot();
    }

    public void runRobot() {
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Robot instructions go here
            timeElapsed.startTime();
            robot.move("forward", 36, 1);

            // Detect # of rings


            switch (zone) {
                case "A":
                    robot.move("forward", 28, 1);
                    robot.move("right", 37, 0.5);
                    robot.move("backward", 72, 1);
                    robot.move("left", 15, 1);
                    robot.move("forward", 64, 1);
                    robot.move("right", 9, 0.5);
                    robot.move("backward", 9, 1);
                    robot.move("right", 12, 0.35);
                    robot.move("left", 55, 1);
            }

            // shoot ring
            robot.move("right", 7.5, 1);
            // shoot ring
            robot.move("right", 7.5, 1);
            // shoot ring

            robot.move("forward", 9, 1);


            telemetry.addData("Time Elapsed", timeElapsed.time());
            timeElapsed.reset();
            sleep(5000);
        }
    }
}