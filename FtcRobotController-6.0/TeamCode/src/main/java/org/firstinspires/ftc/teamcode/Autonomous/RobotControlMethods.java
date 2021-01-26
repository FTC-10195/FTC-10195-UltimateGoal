package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

import static android.os.SystemClock.sleep;
import static java.lang.Math.PI;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@Disabled
public class RobotControlMethods
{
    // Configuration parameters
    final double TICKS_PER_WHEEL_ROTATION = 537.6; //Amount of ticks logged in one wheel rotation
    final double WHEEL_SIZE_IN_INCHES = 3.94; // Diameter of the wheel (in inches)

    double decelerationThreshold = 70; // When to start decelerating;
    double motorPowerMultiplier = 0.7; // Controls the speed of the robot;

    // State variables
    DcMotorEx fl, fr, bl, br, shooter, topIntake, bottomIntake, wobbleLifter;
    Servo ringPusher, wobbleGrabber;
    double flRawPower, frRawPower, blRawPower, brRawPower;
    double currentAngle;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    ElapsedTime robotTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    // Shooter variables
    public static double shooterDelay = 2.5;
    public static double ringPusherIteration = 1;
    int automaticCooldown = 600;
    Double[] ringPusherPositions = {0.3, 0.1, 0.4};
    int currentArrayIndex = 0;
    ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime ringPusherTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public RobotControlMethods(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br, DcMotorEx shooter,
                               DcMotorEx topIntake, DcMotorEx bottomIntake, DcMotorEx wobbleLifter, Servo ringPusher,
                               Servo wobbleGrabber, BNO055IMU imu) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.shooter = shooter;
        this.topIntake = topIntake;
        this.bottomIntake = bottomIntake;
        this.wobbleLifter = wobbleLifter;
        this.ringPusher = ringPusher;
        this.wobbleGrabber = wobbleGrabber;
        this.imu = imu;
    }

    public void resetRobotControlMethods(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br, DcMotorEx shooter,
                                         DcMotorEx topIntake, DcMotorEx bottomIntake, DcMotorEx wobbleLifter,
                                         Servo ringPusher, Servo wobbleGrabber, BNO055IMU imu) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.shooter = shooter;
        this.topIntake = topIntake;
        this.bottomIntake = bottomIntake;
        this.wobbleLifter = wobbleLifter;
        this.ringPusher = ringPusher;
        this.wobbleGrabber = wobbleGrabber;
        this.imu = imu;
    }

    /**
     *
     * @param moveDirection the direction that the robot moves in (forward, backward, left, right)
     * @param distanceInInches the amount of inches that the robot will travel
     * @param motorPower the speed at which the robot will travel
     */
    public void move(final String moveDirection, final double distanceInInches, final double motorPower) throws InterruptedException {
        double ticks = calculateTicks(distanceInInches);
        resetMotors();
        determineMotorTicks(moveDirection, ticks, motorPower);
        runProgram(ticks);
    }

    /**
     * calculates ticks from inches
     * @param distanceInInches the distance in inches the robot will travel
     * @return the amount of ticks corresponding to the given amount of degrees
     */
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

    /**
     * determines how many ticks to set the motors to based on direction
     * @param direction the direction that the robot will travel (forward, backward, left, right)
     * @param ticks the distance in ticks that the robot will travel, converted from inches
     * @param motorPower the speed at which the robot will travel
     */
    public void determineMotorTicks(final String direction, double ticks, double motorPower)
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

        while ( fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy() ) {
            if (fl.getCurrentPosition() >= (decelerationThreshold * ticks)) {
                // Decelerates at a certain percent of power per percent of position
                try {
                    double decelerationRate = -1 / (1 - decelerationThreshold);
                    motorPowerMultiplier *= Math.max(decelerationRate * ((fl.getCurrentPosition() / ticks) - 1), 0.2);
                } catch (Exception e) {
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

    public void turn(final String direction, final int degrees, int motorPower)
    {
        double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
        resetAngle();

        switch (direction){
            case "left":
                flPower = -motorPowerMultiplier * motorPower;
                frPower = motorPowerMultiplier * motorPower;
                blPower = -motorPowerMultiplier * motorPower;
                brPower = motorPowerMultiplier * motorPower;
                break;

            case "right":
                flPower = motorPowerMultiplier * motorPower;
                frPower = -motorPowerMultiplier * motorPower;
                blPower = motorPowerMultiplier * motorPower;
                brPower = -motorPowerMultiplier * motorPower;
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
                if ((angleConversion() <= degrees)) break;
            }
        }

        else // degrees >= 0
        {
            while (true)
            {
                if ((angleConversion() >= degrees)) break;
            }
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        sleep(100);
        resetAngle();
    }

    private double angleConversion()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double changeInAngle = angles.firstAngle - lastAngles.firstAngle;

        /*
        if (currentAngle > 0)
        {
            currentAngle = ((currentAngle + 180) % (360)) - 180;
        }

        else // currentAngle < 0
        {
            currentAngle = ((currentAngle - 180) % (360)) + 180;
        }

         */

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

    public void moveInAnyDirection(double angleInDegrees, double distanceInInches, double motorPower){
        int ticks = calculateTicks(distanceInInches);
        motorPowerMultiplier *= motorPower;
        resetMotors();
        determineMotorTicks(angleInDegrees, ticks);
        determineMotorPowersInAnyDirection(angleInDegrees);
        runProgram(ticks);
    }

    private void determineMotorTicks(double angleInDegrees, double ticks) {
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

    public void overcomeEncoderProblem(double distanceInInches, double timeInSeconds, double startPower) {
        double ticks = calculateTicks(distanceInInches);

        fl.setTargetPosition((int) ticks);
        fr.setTargetPosition((int) ticks);
        bl.setTargetPosition((int) ticks);
        br.setTargetPosition((int) ticks);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(startPower);
        fr.setPower(startPower);
        bl.setPower(startPower);
        br.setPower(startPower);

        robotTimer.reset();

        while ( fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy() ) {
            double intendedPosition = ticks * (robotTimer.time(TimeUnit.SECONDS) / timeInSeconds);
            double flScaledError = (intendedPosition - fl.getCurrentPosition()) / intendedPosition;
            double frScaledError = (intendedPosition - fr.getCurrentPosition()) / intendedPosition;
            double blScaledError = (intendedPosition - bl.getCurrentPosition()) / intendedPosition;
            double brScaledError = (intendedPosition - br.getCurrentPosition()) / intendedPosition;

            if (flScaledError > startPower || frScaledError > startPower || blScaledError > startPower
                    || brScaledError > startPower) {
                double maxPower = Math.max(Math.max(flScaledError, frScaledError), Math.max(blScaledError, brScaledError));
                flScaledError /= (maxPower / startPower);
                frScaledError /= (maxPower / startPower);
                blScaledError /= (maxPower / startPower);
                brScaledError /= (maxPower / startPower);
            }

            fl.setPower((distanceInInches > 0) ? max(0, flScaledError) : min(0, flScaledError));
            fr.setPower((distanceInInches > 0) ? max(0, frScaledError) : min(0, frScaledError));
            bl.setPower((distanceInInches > 0) ? max(0, blScaledError) : min(0, blScaledError));
            br.setPower((distanceInInches > 0) ? max(0, brScaledError) : min(0, brScaledError));
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        sleep(100);
    }

    public void shootRings(int numberOfRings) {
        shooter.setPower(1);
        shooterTimer.reset();
        while (shooterTimer.time(TimeUnit.SECONDS) < shooterDelay){}
        while (ringPusherIteration <= (3 * numberOfRings)) {
            telemetry.addData("ringPusherIteration", ringPusherIteration);
            telemetry.update();
            currentArrayIndex++;
            if (currentArrayIndex >= ringPusherPositions.length) {
                currentArrayIndex = 0;
            }
            telemetry.addLine("Running!");
            telemetry.update();
            ringPusher.setPosition(ringPusherPositions[currentArrayIndex]);
            telemetry.addLine("Still Running!");
            telemetry.update();
            ringPusherTimer.reset();
            ringPusherIteration++;

            sleep(automaticCooldown);
        }
        shooter.setPower(0);
        ringPusherIteration = 1;
    }

    public void grabWobble() {
        wobbleGrabber.setPosition(0);
        sleep(300);
    }

    public void releaseWobble() {
        wobbleGrabber.setPosition(0.5);
        sleep(300);
    }

    public void liftWobble() {
        wobbleLifter.setPower(0.4);
    }

    public void lowerWobble(double time) {
        wobbleLifter.setPower(-0.4);
        robotTimer.reset();
        while (robotTimer.time() < time) {}
        wobbleLifter.setPower(0);
    }

    public void wobble(String action) {
        lowerWobble(0.5);
        switch (action.toLowerCase()) {
            case "grab": default:
                grabWobble();
                break;

            case "release":
                releaseWobble();
                break;
        }
        liftWobble();
    }

    public double toRadians(double angleInDegrees){
        return angleInDegrees * PI / 180;
    }
}
