package org.firstinspires.ftc.teamcode.summerprojects;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "BasicTankAutonomous", group = "e")
public class TankAuto extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;
    private int globalAngle;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //Calibrate gyro if not done already
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        right.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()){
           //Robot instructions go here
            DriveInstructions();
        }
    }

    public void DriveInstructions(){

    }

    public void motorReset() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runProgram() {
        left.setPower(0.5);
        right.setPower(0.5);
        while (right.isBusy() && left.isBusy()){}
        left.setPower(0);
        right.setPower(0);
    }
    public void goForward(int degrees){
        motorReset();
        right.setTargetPosition(degrees);
        left.setTargetPosition(degrees);
        runProgram();
    }
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double changeInAngle = angles.firstAngle - lastAngles.firstAngle;
        if (changeInAngle < -180)
            changeInAngle += 360;
        else if (changeInAngle > 180)
            changeInAngle -= 360;
        globalAngle += changeInAngle;
        lastAngles = angles;
        return globalAngle;
    }
    private void rotate(int degrees) {
        double leftPower, rightPower;
        resetAngle();

        //Turn Right
        if (degrees < 0) {
            leftPower = 0.5;
            rightPower = -0.5;
        }

        //Turn Left
        else if (degrees > 0) {
            leftPower = -0.5;
            rightPower = 0.5;
        }
        else return;
        left.setPower(leftPower);
        right.setPower(rightPower);

        //Right
        if (degrees < 0) {
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {}
        }

        //Left
        else {
            while (opModeIsActive() && getAngle() < degrees) {}
        }
        right.setPower(0);
        left.setPower(0);
        sleep(200);
        resetAngle();
    }
}