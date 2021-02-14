package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.RobotControlMethods;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TeleOp.KevalMechTele.cooldown;
import static org.firstinspires.ftc.teamcode.TeleOp.KevalMechTele.shooterLastPressed;

@TeleOp(name = "ShooterPIDFTuner", group = "a")
public class ShooterPIDFTuner extends OpMode {
    DcMotorEx shooter;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    ElapsedTime shooterPIDFTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double currentTime = 0;
    double previousTime = 0;
    double deltaTime = 0;
    int currentTicks = 0;
    int previousTicks = 0;
    int deltaTicks = 0;
    double velocity = 0;

    boolean isShooterOnForward = false;

    static ElapsedTime shooterRotation = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null, null, null, null, null,
            null, null);

    public static double setShooterPower;
    public static double currentShooterVelocity;

    public static double P = 1;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up && System.currentTimeMillis() - shooterLastPressed > cooldown) {
            shooterLastPressed = System.currentTimeMillis();
            isShooterOnForward = !isShooterOnForward;
        }

        shooter.setVelocityPIDFCoefficients(P, I, D, F);

        setShooterVelocity();
    }

    public double calculateShooterVelocity() {
        currentTime = shooterPIDFTimer.time(TimeUnit.SECONDS);
        deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        currentTicks = shooter.getCurrentPosition();
        deltaTicks = currentTicks - previousTicks;
        previousTicks = currentTicks;

        velocity = deltaTicks / deltaTime;
        return velocity;
    }

    public void setShooterVelocity() {
        while (isShooterOnForward) {
            double adjustedShooterTime = shooterRotation.time(TimeUnit.SECONDS) % 8;
            if (adjustedShooterTime > 6) {
                setShooterPower = 0.25;
            } else if (adjustedShooterTime > 4) {
                setShooterPower = -0.25 * (adjustedShooterTime - 4) + 0.75;
            } else if (adjustedShooterTime > 2) {
                setShooterPower = 0.75;
            } else {
                setShooterPower = 0.25 * (adjustedShooterTime) + 0.25;
            }

            currentShooterVelocity = calculateShooterVelocity();
            shooter.setVelocity(setShooterPower * (robot.SHOOTER_TICKS_PER_ROTATION * (robot.SHOOTER_MAX_RPM / 60)));
            packet.put("Set Shooter Velocity", setShooterPower);
            packet.put("Current Shooter Velocity", currentShooterVelocity);

            dashboard.sendTelemetryPacket(packet);
        }

        shooterRotation.reset();
    }
}
