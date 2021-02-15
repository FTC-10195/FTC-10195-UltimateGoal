package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    public static double setShooterVelocity;
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

    public void setShooterVelocity() {

        while (isShooterOnForward) {
            double adjustedShooterTime = shooterRotation.time(TimeUnit.SECONDS) % 8;
            if (adjustedShooterTime > 6) {
                setShooterVelocity = powerToVelocity(
                        0.25,
                        robot.SHOOTER_TICKS_PER_ROTATION,
                        robot.SHOOTER_MAX_RPM
                );
            } else if (adjustedShooterTime > 4) {
                setShooterVelocity = powerToVelocity(
                        -0.25 * (adjustedShooterTime - 4) + 0.75,
                        robot.SHOOTER_TICKS_PER_ROTATION,
                        robot.SHOOTER_MAX_RPM
                );
            } else if (adjustedShooterTime > 2) {
                setShooterVelocity = powerToVelocity(
                        0.75,
                        robot.SHOOTER_TICKS_PER_ROTATION,
                        robot.SHOOTER_MAX_RPM
                );
            } else {
                setShooterVelocity = powerToVelocity(
                        0.25 * (adjustedShooterTime) + 0.25,
                        robot.SHOOTER_TICKS_PER_ROTATION,
                        robot.SHOOTER_MAX_RPM
                );
            }

            currentShooterVelocity = calculateShooterVelocity();
            shooter.setVelocity(setShooterVelocity);

            packet.put("Set Shooter Velocity", setShooterVelocity);
            packet.put("Current Shooter Velocity", currentShooterVelocity);
            packet.put("Error", currentShooterVelocity - setShooterVelocity);

            dashboard.sendTelemetryPacket(packet);
        }

        shooterRotation.reset();
    }

    public double powerToVelocity(double power, double ticksPerRotation, double maxRPM) {
        return (power * (ticksPerRotation * (maxRPM / 60)));
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
}
