/*
    Ideas to Implement:

    Automate some parts of teleop
        Example from skystone: robot automatically places block with the input of the current height
    Use slowmode with other areas of the robot, like slides
        Example from skystone: slide goes up slower to get the right height

 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

@TeleOp(name = "KevalMechTele", group = "a")
public class KevalMechTele extends OpMode {

    //Configuration parameters
    double slowModePower = 0.35;
    double boostModePower = 1;
    double normalModePower = 0.7;
    double buttonIsPressedThreshold = 0.10;

    //State variables
    DcMotor fl, fr, bl, br;
    double flPower, frPower, blPower, brPower;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        //TODO: Find which motors to reverse
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        /*
        The left joystick to move forward/backward/left/right, right joystick to turn

        gamepad 1 controls movement
        gamepad 2 currently has no function
         */

        double y = gamepad1.left_stick_y * -1; // Reversed
        double x = gamepad1.left_stick_x * (sqrt(2)); // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        flPower = (normalModePower) * (y + x + rx);
        frPower = (normalModePower) * (y - x - rx);
        blPower = (normalModePower) * (y - x + rx);
        brPower = (normalModePower) * (y + x - rx);

        if (abs(flPower) > 1 || abs(blPower) > 1 ||
                abs(frPower) > 1 || abs(brPower) > 1 ) {
            // Find the largest power
            double max = 0;
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

        if (gamepad1.right_trigger > buttonIsPressedThreshold){
            flPower *= boostModePower;
            frPower *= boostModePower;
            blPower *= boostModePower;
            brPower *= boostModePower;
        }

        else if (gamepad1.left_trigger > buttonIsPressedThreshold){
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

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
}