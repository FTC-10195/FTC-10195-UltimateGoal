package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

@Config
@TeleOp(name = "ShooterTest", group = "z")
public class ShooterTest extends OpMode {
    DcMotor shooter;
    Servo ringPusher;

    public static double shooterPower = 1;

    Double[] ringPusherPositions = {0.3, 0.1, 0.4};
    int currentArrayIndex = 0;

    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ringPusher = hardwareMap.get(Servo.class, "push");
        ringPusher.setPosition(ringPusherPositions[0]);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            shooterPower -= 0.05;
            sleep(250);
        }

        if (gamepad1.y) {
            shooterPower += 0.05;
            sleep(250);
        }

        if (gamepad1.dpad_up) {
            shooter.setPower(shooterPower);
        } else if (gamepad1.dpad_down) {
            shooter.setPower(-shooterPower);
        } else {
            shooter.setPower(0);
        }

        if (gamepad1.right_bumper) {
            currentArrayIndex++;
            if(currentArrayIndex >= ringPusherPositions.length) {
                currentArrayIndex = 0;
            }
            ringPusher.setPosition(ringPusherPositions[currentArrayIndex]);
            sleep(250);
        }
        else if (gamepad1.left_bumper) {
            currentArrayIndex--;
            if(currentArrayIndex < 0) {
                currentArrayIndex = ringPusherPositions.length - 1;
            }
            ringPusher.setPosition(ringPusherPositions[currentArrayIndex]);
            sleep(250);
        }

        telemetry.addData("Shooter Power", shooterPower);
        telemetry.update();
    }
}