package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "z")
public class ServoTest extends OpMode {
    Servo ringPusher, wobbleGrabber;

    Servo[] servos = {ringPusher, wobbleGrabber};
    Servo selectedServo = servos[0];
    int selectedServoIndex = 0;

    Double[] servoPositions = {0.0, 0.0};

    @Override
    public void init() {
        ringPusher = hardwareMap.get(Servo.class, "push");
        wobbleGrabber = hardwareMap.get(Servo.class, "grab");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            selectedServoIndex++;
            if(selectedServoIndex >= servos.length) {
                selectedServoIndex = 0;
            }
            selectedServo = servos[selectedServoIndex];
        }

        if (gamepad1.left_bumper) {
            selectedServoIndex--;
            if(selectedServoIndex <= 0) {
                selectedServoIndex = servos.length - 1;
            }
            selectedServo = servos[selectedServoIndex];
        }

        if (gamepad1.dpad_up) {
            servoPositions[selectedServoIndex] += 0.1;
            selectedServo.setPosition(servoPositions[selectedServoIndex]);
        }

        if (gamepad1.dpad_down) {
            servoPositions[selectedServoIndex] -= 0.1;
            selectedServo.setPosition(servoPositions[selectedServoIndex]);
        }

        telemetry.addData("Servo 0", servoPositions[0]);
        telemetry.addData("Servo 1", servoPositions[1]);
        telemetry.addData("Selected Servo", selectedServo);
        telemetry.update();
    }
}