package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;
import static java.lang.Math.*;

@Disabled
public class RobotControlMethods
{
    // Configuration parameters
    final double TICKS_PER_WHEEL_ROTATION = 537.6; // TODO: Change when wheel is finalized
    final double WHEEL_SIZE_IN_INCHES = 3.94; // TODO: Change when wheel is finalized

    double decelerationPercentage = 85; // TODO: Tune
    double motorPowerMultiplier = 0.7; // TODO: Tune

    // State variables
    private DcMotor fl, fr, bl, br;
    private double flRawPower, frRawPower, blRawPower, brRawPower;
    private int currentAngle;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    public RobotControlMethods(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, BNO055IMU imu) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.imu = imu;
    }

    public void resetRobotControlMethods(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, BNO055IMU imu) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.imu = imu;
    }

    public void move(final String moveDirection, final int distanceInInches, final double motorPower)
    {
        int ticks = calculateTicks(distanceInInches);
        resetMotors();
        determineMotorTicksInAnyDirection(moveDirection, ticks, motorPower);
        runProgram(ticks);
    }

    public int calculateTicks(final double distanceInInches)
    {
        return (int) ((TICKS_PER_WHEEL_ROTATION * distanceInInches) / (WHEEL_SIZE_IN_INCHES * Math.PI));
    }

    public void resetMotors()
    {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void determineMotorTicksInAnyDirection(final String direction, int ticks, double motorPower)
    {
        switch (direction)
        {
            case "forward":
                fl.setTargetPosition((int)Math.round(1.0 * ticks));
                fr.setTargetPosition((int)Math.round(1.0 * ticks));
                bl.setTargetPosition((int)Math.round(1.0 * ticks));
                br.setTargetPosition((int)Math.round(1.0 * ticks));

                flRawPower = frRawPower = blRawPower = brRawPower = motorPower;
                break;

            case "backward":
                fl.setTargetPosition((int)Math.round(-1.0 * ticks));
                fr.setTargetPosition((int)Math.round(-1.0 * ticks));
                bl.setTargetPosition((int)Math.round(-1.0 * ticks));
                br.setTargetPosition((int)Math.round(-1.0 * ticks));
                flRawPower = frRawPower = blRawPower = brRawPower = -motorPower;
                break;

            case "left":
                fl.setTargetPosition((int)Math.round(-1.0 * ticks));
                fr.setTargetPosition((int)Math.round(1.0 * ticks));
                bl.setTargetPosition((int)Math.round(1.0 * ticks));
                br.setTargetPosition((int)Math.round(-1.0 * ticks));

                flRawPower = brRawPower = -motorPower;
                frRawPower = blRawPower = motorPower;
                break;

            case "right":
                fl.setTargetPosition((int)Math.round(1.0 * ticks));
                fr.setTargetPosition((int)Math.round(-1.0 * ticks));
                bl.setTargetPosition((int)Math.round(-1.0 * ticks));
                br.setTargetPosition((int)Math.round(1.0 * ticks));

                flRawPower = brRawPower = motorPower;
                frRawPower = blRawPower = -motorPower;
                break;

            default:
                try {
                    throw new IllegalStateException("Invalid move direction: " + direction);
                } catch (IllegalStateException e) {
                    e.printStackTrace();
                }
        }
    }

    public void runProgram(final double ticks)
    {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(motorPowerMultiplier * flRawPower);
        fr.setPower(motorPowerMultiplier * frRawPower);
        bl.setPower(motorPowerMultiplier * blRawPower);
        br.setPower(motorPowerMultiplier * brRawPower);

        while ( fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy() )
        {
            if (fl.getCurrentPosition() >= (decelerationPercentage * ticks)) {
                // Decelerates at a rate of 1/(1-deceleration_percentage) percent of power per percent of position
                try {
                    double decelerationRate = -1 / (1 - decelerationPercentage);
                    motorPowerMultiplier *= Math.max(decelerationRate * ((fl.getCurrentPosition() / ticks) - 1), 0.2);
                } catch (Exception e){
                    break;
                }
            }
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        sleep(100);
    }

    public void turn(final String direction, final int degrees)
    {
        double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
        resetAngle();

        switch (direction){
            case "left":
                flPower = -motorPowerMultiplier * flRawPower;
                frPower = motorPowerMultiplier * frRawPower;
                blPower = -motorPowerMultiplier * blRawPower;
                brPower = motorPowerMultiplier * brRawPower;
                break;

            case "right":
                flPower = motorPowerMultiplier * flRawPower;
                frPower = -motorPowerMultiplier * frRawPower;
                blPower = motorPowerMultiplier * blRawPower;
                brPower = -motorPowerMultiplier * brRawPower;
                break;

            default:
                try {
                    throw new IllegalStateException("Invalid turn direction: " + direction);
                } catch (IllegalStateException e) {
                    e.printStackTrace();
                }
        }

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        TurnUntilAngleReached(degrees);
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    private void TurnUntilAngleReached(final int degrees)
    {
        if (degrees < 0)
        {
            while (true)
            {
                if ((currentAngle() <= degrees)) break;
            }
        }

        else // degrees > 0
        {
            while (true)
            {
                if ((currentAngle() >= degrees)) break;
            }
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        sleep(100);
        resetAngle();
    }

    private double currentAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double changeInAngle = angles.firstAngle - lastAngles.firstAngle;

        if (currentAngle > 0)
        {
            currentAngle = ((currentAngle + 180) % (360)) - 180;
        }

        else // currentAngle < 0
        {
            currentAngle = ((currentAngle - 180) % (360)) + 180;
        }

        currentAngle += changeInAngle;
        lastAngles = angles;
        return currentAngle;
    }

    public void stopRobot()
    {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void moveInAnyDirection(double angleInDegrees, double distanceInInches, double motorPower){
        int ticks = calculateTicks(distanceInInches);
        motorPowerMultiplier *= motorPower;
        resetMotors();
        determineMotorTicksInAnyDirection(angleInDegrees, ticks);
        determineMotorPowersInAnyDirection(angleInDegrees);
        runProgram(ticks);
    }

    private void determineMotorTicksInAnyDirection(double angleInDegrees, double ticks) {
        double flAndbrPositions = ticks * sin(toRadians(angleInDegrees) + (PI / 4));
        double frAndblPositions = ticks * sin(toRadians(angleInDegrees) - (PI / 4));

        fl.setTargetPosition((int) flAndbrPositions);
        fr.setTargetPosition((int) frAndblPositions);
        bl.setTargetPosition((int) frAndblPositions);
        br.setTargetPosition((int) flAndbrPositions);
    }

    private void determineMotorPowersInAnyDirection(double moveAngleInDegrees){
        flRawPower = brRawPower = sin(toRadians(moveAngleInDegrees) + (PI / 4));
        frRawPower = blRawPower = sin(toRadians(moveAngleInDegrees) - (PI / 4));

        double greatestMotorPower = Math.max(Math.abs(flRawPower), Math.abs(frRawPower));
        greatestMotorPower = Math.max(greatestMotorPower, Math.abs(blRawPower));
        greatestMotorPower = Math.max(greatestMotorPower, Math.abs(brRawPower));

        flRawPower /= greatestMotorPower;
        frRawPower /= greatestMotorPower;
        blRawPower /= greatestMotorPower;
        brRawPower /= greatestMotorPower;
    }

    public void moveInAnyDirectionWithTurning(double moveAngleInDegrees, double turnAngleInDegrees, double distanceInInches, double motorPower){
        int ticks = calculateTicks(distanceInInches);
        motorPowerMultiplier *= motorPower;
        resetMotors();
        determineMotorTicksWithTurning(moveAngleInDegrees, turnAngleInDegrees, ticks);
        determineMotorPowersWithTurning(moveAngleInDegrees, turnAngleInDegrees);
        runProgram(ticks);
    }

    private void determineMotorTicksWithTurning(double moveAngleInDegrees, double turnAngleInDegrees, double ticks) {
        double flAndbrPositions = ticks * (sin(toRadians(moveAngleInDegrees) + (PI / 4)) + toRadians(turnAngleInDegrees));
        double frAndblPositions = ticks * (sin(toRadians(moveAngleInDegrees) - (PI / 4)) + toRadians(turnAngleInDegrees));

        fl.setTargetPosition((int) flAndbrPositions);
        fr.setTargetPosition((int) frAndblPositions);
        bl.setTargetPosition((int) frAndblPositions);
        br.setTargetPosition((int) flAndbrPositions);
    }

    private void determineMotorPowersWithTurning(double moveAngleInDegrees, double turnAngleInDegrees){
        flRawPower = brRawPower = sin(toRadians(moveAngleInDegrees) + (PI / 4)) + toRadians(turnAngleInDegrees);
        frRawPower = blRawPower = sin(toRadians(moveAngleInDegrees) - (PI / 4)) + toRadians(turnAngleInDegrees);

        double greatestMotorPower = Math.max(Math.abs(flRawPower), Math.abs(frRawPower));
        greatestMotorPower = Math.max(greatestMotorPower, Math.abs(blRawPower));
        greatestMotorPower = Math.max(greatestMotorPower, Math.abs(brRawPower));

        flRawPower /= greatestMotorPower;
        frRawPower /= greatestMotorPower;
        blRawPower /= greatestMotorPower;
        brRawPower /= greatestMotorPower;
    }

    public double toRadians(double angleInDegrees){
        return angleInDegrees * PI / 180;
    }

    // This method is useless but required since this is a LinearOpMode
    public void runOpMode(){}
}