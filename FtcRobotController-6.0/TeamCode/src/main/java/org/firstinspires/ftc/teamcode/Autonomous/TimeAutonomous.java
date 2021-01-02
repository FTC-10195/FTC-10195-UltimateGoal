package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RealAutonomous", group = "a")
public class TimeAutonomous extends LinearOpMode {
    String zone = "A";

    // State variables
    DcMotor fl, fr, bl, br;
    BNO055IMU imu;

    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null);

    public void setup() {
        motorSetup();
    }

    public void motorSetup() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runOpMode() throws InterruptedException {
        setup();
        runRobot();
    }

    public void runRobot() throws InterruptedException {
        

    }
}