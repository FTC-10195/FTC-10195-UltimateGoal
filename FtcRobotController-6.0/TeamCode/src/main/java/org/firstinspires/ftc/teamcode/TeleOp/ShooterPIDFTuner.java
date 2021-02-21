package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.RobotControlMethods;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TeleOp.KevalMechTele.cooldown;
import static org.firstinspires.ftc.teamcode.TeleOp.KevalMechTele.shooterLastPressed;

@Config
@TeleOp(name = "ShooterPIDFTuner", group = "a")
public class ShooterPIDFTuner extends OpMode {
    DcMotorEx shooter;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    ElapsedTime shooterPIDFTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double currentTime = 0;
    double previousTime = 0;
    double deltaTime = 0;
    int currentTicks = 0;
    int previousTicks = 0;
    int deltaTicks = 0;
    double velocity = 0;

    boolean isShooterOnForward = false;

    ElapsedTime shooterRotation = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime waitBetweenLoops = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null, null, null, null, null,
            null, null);

    public static double setShooterVelocity;
    public static double currentShooterVelocity;

    // The PIDF values to tune via FTC Dashboard
    public static double P = 1;
    public static double I = 0;
    public static double D = 0;
    public static double F = 10;

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

        // Sends the telemetry data to FTC Dashboard for graphing
        packet.put("Set Shooter Velocity", setShooterVelocity);
        packet.put("Current Shooter Velocity", currentShooterVelocity);
        packet.put("Error", currentShooterVelocity - setShooterVelocity);
        packet.put("isShooterOnForward", isShooterOnForward);
        dashboard.sendTelemetryPacket(packet);

        waitBetweenLoops.reset();
        while (waitBetweenLoops.time(TimeUnit.MILLISECONDS) <  250) {}
    }

    /**
     * Sets the shooter velocity and sends telemetry data for graphing to FTC Dashboard
     */
    public void setShooterVelocity() {

        if (isShooterOnForward) {
            /*
            The trapezoidal motion profile; graph in Programming Notebook or at below link
            https://www.desmos.com/calculator/owtmixvn4t
             */
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
        }
    }

    /**
     * A simple conversion between motor power and velocity
     * @param power The power to convert to velocity
     * @param ticksPerRotation The ticks per rotation of the motor
     * @param maxRPM The max RPM of the motor
     * @return The converted velocity
     */
    public double powerToVelocity(double power, double ticksPerRotation, double maxRPM) {
        return (power * (ticksPerRotation * (maxRPM / 60)));
    }

    /**
     * Calculates the current shooter velocity
     * @return Current shooter velocity
     */
    public double calculateShooterVelocity() {
        currentTicks = shooter.getCurrentPosition();
        deltaTicks = currentTicks - previousTicks;
        previousTicks = currentTicks;

        currentTime = shooterPIDFTimer.time(TimeUnit.SECONDS);
        deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        velocity = deltaTicks / deltaTime;
        return velocity;
    }
}
