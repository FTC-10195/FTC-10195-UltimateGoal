package org.firstinspires.ftc.teamcode.summerprojects.Code20192020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="RunCode_20192020", group="Autonomous")
//@Disabled
public class RunCode_20192020 extends LinearOpMode {

    //Declare motors
    DcMotor fl; //Front left wheel
    DcMotor fr; //Front right wheel
    DcMotor bl; //Back left wheel
    DcMotor br; //Back right wheel

    Functions_20192020 movement = new Functions_20192020(null, null, null, null);

    public void runOpMode() {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        movement.resetFunctions(fl, fr, bl, br);

        //Reverse motors
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        //Run motors using encoders
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset servos

        //Miscellaneous

        //Wait for driver to press start
        waitForStart();

        //Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Telemetry

        //Steps go here
        if (opModeIsActive() && !isStopRequested()) {
            movement.DriveForward(0.7, 16);
            sleep(500);
            movement.TurnLeft(0.7, 10);
            sleep(500);
            movement.DriveLeft(0.7, 14);
            sleep(500);
            movement.DriveForward(0.7, 13);
            sleep(500);
            movement.DriveBackward(0.7, 40);
            sleep(500);
            movement.TurnRight(0.7, 30);
            sleep(500);
            movement.DriveRight(0.7, 58);
            sleep(500);
            movement.DriveBackward(0.7, 20);
        }
    }
}