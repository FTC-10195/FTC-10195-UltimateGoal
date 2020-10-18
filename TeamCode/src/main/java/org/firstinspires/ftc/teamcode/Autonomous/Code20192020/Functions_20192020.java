package org.firstinspires.ftc.teamcode.Autonomous.Code20192020;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class Functions_20192020 {

    //Declare motors
    DcMotor fl; //Front left wheel
    DcMotor fr; //Front right wheel
    DcMotor bl; //Back left wheel
    DcMotor br; //Back right wheel

    //Methods for moving

    public Functions_20192020(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    public void resetFunctions(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    public void DriveForward(double power, int distance) {
        //Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Convert distance to ticks
        int ticks = (int) ((1120) / (3 * 3.14159)) * (distance);

        int doublePower = (int) (3 * power);

        //Set target position
        fl.setTargetPosition(ticks);
        fr.setTargetPosition(ticks + 400);
        bl.setTargetPosition(ticks);
        br.setTargetPosition(ticks + 400);

        //Get current position
        int flPos = fl.getCurrentPosition();
        int frPos = fr.getCurrentPosition();
        int blPos = bl.getCurrentPosition();
        int brPos = br.getCurrentPosition();

        //Set mode to RUN_TO_POSITION
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (flPos < ticks && frPos < ticks && blPos < ticks && brPos < ticks) {

            //While all encoder counts are less than the amount given
            fl.setPower(power);
            fr.setPower(doublePower);
            bl.setPower(power);
            br.setPower(doublePower);

            //Get current position to update the position values
            flPos = fl.getCurrentPosition();
            frPos = fr.getCurrentPosition();
            blPos = bl.getCurrentPosition();
            brPos = br.getCurrentPosition();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DriveBackward(double power, int distance) {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Convert distance to ticks
        int ticks = (int) ((1120) / (3 * 3.14159)) * (distance);

        int doublePower = (int) (3 * power);

        //Set target position
        fl.setTargetPosition(-ticks);
        fr.setTargetPosition(-ticks - 400);
        bl.setTargetPosition(-ticks);
        br.setTargetPosition(-ticks - 400);

        //Set mode to RUN_TO_POSITION
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(-power);
        fr.setPower(-doublePower);
        bl.setPower(-power);
        br.setPower(-doublePower);

        while ((fl.isBusy() && fr.isBusy()) && (bl.isBusy() && br.isBusy())) {
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void DriveLeft(double power, int distance) {
        //Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Convert distance to ticks
        int ticks = (int) ((1120) / (3 * 3.14159)) * (distance);

        int doublePower = (int) (3 * power);

        //Set target position
        fl.setTargetPosition(-ticks);
        fr.setTargetPosition(ticks + 400);
        bl.setTargetPosition(ticks);
        br.setTargetPosition(-ticks - 400);

        //Get current position
        int flPos = fl.getCurrentPosition();
        int frPos = fr.getCurrentPosition();
        int blPos = bl.getCurrentPosition();
        int brPos = br.getCurrentPosition();

        //Set mode to RUN_TO_POSITION
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (flPos > -ticks && frPos < ticks && blPos < ticks && brPos > -ticks) {
            //Telemetry to show where the wheels are
            //While all encoder counts are less than the amount given
            fl.setPower(-power);
            fr.setPower(doublePower);
            bl.setPower(power);
            br.setPower(-doublePower);

            //Get current position to update the position values
            flPos = fl.getCurrentPosition();
            frPos = fr.getCurrentPosition();
            blPos = bl.getCurrentPosition();
            brPos = br.getCurrentPosition();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DriveRight(double power, int distance) {
        //Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Convert distance to ticks
        int ticks = (int) ((1120) / (3 * 3.14159)) * (distance);

        int doublePower = (int) (3 * power);

        //Set target position
        fl.setTargetPosition(ticks);
        fr.setTargetPosition(-ticks - 400);
        bl.setTargetPosition(-ticks);
        br.setTargetPosition(ticks + 400);

        //Get current position
        int flPos = fl.getCurrentPosition();
        int frPos = fr.getCurrentPosition();
        int blPos = bl.getCurrentPosition();
        int brPos = br.getCurrentPosition();

        //Set mode to RUN_TO_POSITION
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (flPos < ticks && frPos > -ticks && blPos > -ticks && brPos < ticks) {
            //Telemetry to show where the wheels are
            //While all encoder counts are less than the amount given
            fl.setPower(power);
            fr.setPower(-doublePower);
            bl.setPower(-power);
            br.setPower(doublePower);

            //Get current position
            flPos = fl.getCurrentPosition();
            frPos = fr.getCurrentPosition();
            blPos = bl.getCurrentPosition();
            brPos = br.getCurrentPosition();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnLeft(double power, double degrees) {
        //Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Convert degrees to distance
        double distance = (degrees / 360) * 18 * Math.PI;

        //Convert distance to ticks
        double ticks = (int) ((1120) / (3 * 3.14159)) * (distance);

        int doublePower = (int) (3 * power);

        //Set target position
        fl.setTargetPosition((int) (-ticks));
        fr.setTargetPosition((int) (ticks + 400));
        bl.setTargetPosition((int) (-ticks));
        br.setTargetPosition((int) (ticks + 400));

        //Get current position
        int flPos = fl.getCurrentPosition();
        int frPos = fr.getCurrentPosition();
        int blPos = bl.getCurrentPosition();
        int brPos = br.getCurrentPosition();

        //Set mode to RUN_TO_POSITION
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (flPos > -ticks && frPos < ticks && blPos > -ticks && brPos < ticks) {
            //Telemetry to show where the wheels are
            //While all encoder counts are less than the amount given
            fl.setPower(-power);
            fr.setPower(doublePower);
            bl.setPower(-power);
            br.setPower(doublePower);

            //Get current position
            flPos = fl.getCurrentPosition();
            frPos = fr.getCurrentPosition();
            blPos = bl.getCurrentPosition();
            brPos = br.getCurrentPosition();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnRight(double power, double degrees) {
        //Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Convert degrees to distance
        double distance = (degrees / 360) * 18 * Math.PI;

        //Convert distance to ticks
        double ticks = (int) ((1120) / (3 * 3.14159)) * (distance);

        int doublePower = (int) (3 * power);

        //Set target position
        fl.setTargetPosition((int) ticks);
        fr.setTargetPosition((int) (-ticks - 400));
        bl.setTargetPosition((int) ticks);
        br.setTargetPosition((int) (-ticks - 400));

        //Get current position
        int flPos = fl.getCurrentPosition();
        int frPos = fr.getCurrentPosition();
        int blPos = bl.getCurrentPosition();
        int brPos = br.getCurrentPosition();

        //Set mode to RUN_TO_POSITION
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (flPos < ticks && frPos > -ticks && blPos < ticks && brPos > -ticks) {
            //Telemetry to show where the wheels are
            //While all encoder counts are less than the amount given
            fl.setPower(power);
            fr.setPower(-doublePower);
            bl.setPower(power);
            br.setPower(-doublePower);

            //Get current position
            flPos = fl.getCurrentPosition();
            frPos = fr.getCurrentPosition();
            blPos = bl.getCurrentPosition();
            brPos = br.getCurrentPosition();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
    public void FoundationGrab() {
        //Grab foundation
        FoundationServo1.setPosition(0.1);
        FoundationServo2.setPosition(0.75);
    }

    public void FoundationRelease() {
        //Release foundation
        FoundationServo1.setPosition(0.2); //Works
        FoundationServo2.setPosition(0.6); //Works
    }

    public void ArmOut(double power, int distance) {
        //Moves the grabbing mechanism outward
        ExtendSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ExtendSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Convert distance to ticks
        int ticks = (int) ((1120) / (3 * 3.14159)) * (distance);

        //Set target position
        ExtendSlide.setTargetPosition(ticks);

        //Set mode to RUN_TO_POSITION
        ExtendSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtendSlide.setPower(power);

        while ((fl.isBusy() && fr.isBusy()) && (bl.isBusy() && br.isBusy())) {
        }

        ExtendSlide.setPower(0);
    }

    public void ArmIn(double power, int distance) {
        //Moves the grabbing mechanism inward
        ExtendSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ExtendSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Convert distance to ticks
        int ticks = (int) ((1120) / (3 * 3.14159)) * (distance);

        //Set target position
        ExtendSlide.setTargetPosition(-ticks);

        //Set mode to RUN_TO_POSITION
        ExtendSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtendSlide.setPower(-power);

        while ((fl.isBusy() && fr.isBusy()) && (bl.isBusy() && br.isBusy())) {
        }

        ExtendSlide.setPower(0);
    }

    public void GrabBlock(){

    }

    public void ReleaseBlock(){

    }

    public void Telemetry(){

    }
    */
}