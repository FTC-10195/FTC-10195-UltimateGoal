/*
    Ideas to Implement:

    Automate some parts of teleop
        Example from skystone: robot automatically places block with the input of the current height
    Use slowmode with other areas of the robot, like slides
        Example from skystone: slide goes up slower to get the right height

 */

package org.firstinspires.ftc.teamcode.summerprojects;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

@TeleOp(name = "OttoTeleop", group = "a")
public class OttoTeleop extends OpMode {

    //Configuration parameters
    double slowModePower = 0.25;
    double boostModePower = 1;
    double normalModePower = 0.7;
    double buttonIsPressedThreshold = 0.25;

    //State variables
    DcMotor fl, fr, bl, br;
    double flPower, frPower, blPower, brPower;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        /*
        CONTROL:

        Gamepad 1:
        Left Stick - Movement (move joystick in direction you want to move)
        Right Stick - Rotation (move left to turn left, move right to turn right)

        Gamepad 2:
        TBD
         */

        gamepad1.left_stick_y *= -1; // This is flipped by default, so I flip it back

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double robotSpeed = sqrt(x*x + y*y);

        /*
        The movement vector (robotSpeed * sin(atan2(y,x) + (PI / 4))) controls the movement of the
        robot.
        The rotation modification (rx) controls the rotation of the robot.
         */

        flPower = (robotSpeed * sin(atan2(y,x) + (PI / 4)) + rx);
        frPower = (robotSpeed * cos(atan2(y,x) + (PI / 4)) - rx);
        blPower = (robotSpeed * cos(atan2(y,x) + (PI / 4)) + rx);
        brPower = (robotSpeed * sin(atan2(y,x) + (PI / 4)) - rx);

        double greatestMotorPower = Math.max(Math.max(flPower, frPower), Math.max(blPower, brPower));

        if (gamepad1.right_trigger > buttonIsPressedThreshold){
            flPower *= boostModePower;
            frPower *= boostModePower;
            blPower *= boostModePower;
            brPower *= boostModePower;
        } else if (gamepad1.left_trigger > buttonIsPressedThreshold){
            flPower *= slowModePower;
            frPower *= slowModePower;
            blPower *= slowModePower;
            brPower *= slowModePower;
        }

        if (greatestMotorPower != 0 && ((flPower > 1 || frPower > 1) || (blPower > 1 || brPower > 1))) {
            flPower /= greatestMotorPower;
            frPower /= greatestMotorPower;
            blPower /= greatestMotorPower;
            brPower /= greatestMotorPower;
        }

        // This scales all values down; used to cap speed

        flPower *= normalModePower;
        frPower *= normalModePower;
        blPower *= normalModePower;
        brPower *= normalModePower;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
}