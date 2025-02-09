package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RobotDataPublisher;
import frc.robot.util.RobotDataPublisher.RobotDataSubscription;
import frc.robot.util.ShooterStateData;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX leftKraken;
    private final TalonFX rightKraken;
    private final TalonFX neo;
    private final DigitalInput beamBreak;

    private final StatusSignal<AngularVelocity> leftVel;
    private final StatusSignal<AngularVelocity> rightVel;

    MotionMagicVelocityVoltage motionMagicVelocityVoltage;

    public static class ShooterConstants {
        public static ShooterModes shooterModes;
        public static ShooterStatus shooterStatus;
        public static double velocity = 0;

        public static final int SHOOTER_LEFT_MOTOR = 5; //id
        public static final int SHOOTER_RIGHT_MOTOR = 15; //id
        public static final CurrentLimitsConfigs SHOOTER_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentLimit(65).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(65);
        public static final InvertedValue SHOOTER_INVERSION = InvertedValue.CounterClockwise_Positive;

        public static final double SHOOTER_P = 0.11;//0.043315
        public static final double SHOOTER_S = 0.25;//0.043315
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;
        public static final double SHOOTER_V = 0.12;
        public static final double MOTION_MAGIC_ACCELERATION = 0.1;
        public static final double SHOOTER_STATUS_FRAME_SECONDS = 0.01;

        public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().
                withKS(SHOOTER_S).
                withKP(SHOOTER_P).
                withKI(SHOOTER_I).
                withKD(SHOOTER_D).
                withKA(MOTION_MAGIC_ACCELERATION).
                withKV(SHOOTER_V);


        public static final int SHOOTER_NEO = 16;
        public static final int BEAM_BREAK = 5;

        public static InterpolatingTreeMap<Double, ShooterStateData> SHOOTER_MAP() {
            InterpolatingTreeMap<Double, ShooterStateData> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);
            // TODO: decrease angles by aroun 0.05 to tune
            map.put(1.375, new ShooterStateData(0.5, 125));
            map.put(1.7, new ShooterStateData(.485, 125));
            map.put(2.0, new ShooterStateData(0.478, 125));
            map.put(2.3, new ShooterStateData(0.47, 125));
            map.put(2.65, new ShooterStateData(.465, 125));
            map.put(2.8, new ShooterStateData(.4625, 125));
            map.put(3.0, new ShooterStateData(0.46, 125));
            map.put(3.8, new ShooterStateData(0.443, 125));
            return map;
        }

        public static InterpolatingTreeMap<Double, ShooterStateData> SHUTTLE_MAP() {
            InterpolatingTreeMap<Double, ShooterStateData> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);
            map.put(8.0, new ShooterStateData(0.5, 125));
            return map;
        }
    }

    public ShooterSubsystem() {
        leftKraken = new TalonFX(ShooterConstants.SHOOTER_LEFT_MOTOR, Constants.TalonFXConstants.CANIVORE_NAME);
        rightKraken = new TalonFX(ShooterConstants.SHOOTER_RIGHT_MOTOR, Constants.TalonFXConstants.CANIVORE_NAME);
        var rightMotorConfigurator = rightKraken.getConfigurator();
        var leftMotorConfigurator = leftKraken.getConfigurator();
        var rightMotorConfiguration = new TalonFXConfiguration();

        rightMotorConfiguration.MotorOutput.Inverted = ShooterConstants.SHOOTER_INVERSION;
        rightMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfiguration.CurrentLimits = ShooterConstants.SHOOTER_CURRENT_LIMIT;
        rightMotorConfiguration.Slot0 = ShooterConstants.SLOT_0_CONFIGS;
        leftKraken.setControl(new Follower(rightKraken.getDeviceID(), false));

        ShooterConstants.shooterStatus = ShooterStatus.OFF;
        ShooterConstants.shooterModes = ShooterModes.TRAP;

        leftVel = leftKraken.getVelocity();
        rightVel = rightKraken.getVelocity();

        neo = new TalonFX(ShooterConstants.SHOOTER_NEO, "canivoreBus");

        beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK);

        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

        var motionMagicConfigs = rightMotorConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;
        rightMotorConfigurator.apply(rightMotorConfiguration);
        leftMotorConfigurator.apply(rightMotorConfiguration);
    }

    @Override
    public void periodic() {
        leftVel.refresh();
        rightVel.refresh();

        SmartDashboard.putBoolean("shooter beam break", getBeamBreak());
        SmartDashboard.putNumber("left rps:", leftVel.getValue().magnitude());
        SmartDashboard.putNumber("right rps:", rightVel.getValue().magnitude());
    }

    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    private void stopFeeder() {
        neo.stopMotor();
    }

    private void spinFeeder(double speed) {
        neo.set(speed);
    }

    public void stopShooter() {
        rightKraken.stopMotor();
        leftKraken.stopMotor();
        neo.stopMotor();
        ShooterConstants.shooterStatus = ShooterStatus.OFF;
    }

    public void setDualVelocity(double leftVel, double rightVel) {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(leftVel));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(rightVel));
    }

    private void setVelocity(double velocity) {
        setDualVelocity(velocity, velocity);
    }

    public double getVelocity() {
        if (!leftVel.hasUpdated() || !rightVel.hasUpdated()) {
            leftVel.waitForUpdate(ShooterConstants.SHOOTER_STATUS_FRAME_SECONDS);
            rightVel.waitForUpdate(ShooterConstants.SHOOTER_STATUS_FRAME_SECONDS);
        }

        return (leftVel.getValue().magnitude() + rightVel.getValue().magnitude()) / 2;
    }

    public Command updateVelocityWith(RobotDataPublisher<ShooterStateData> shooterStateDataPublisher) {
        RobotDataSubscription<ShooterStateData> shooterStateDataSubscription = shooterStateDataPublisher.subscribeWith(data -> setVelocity(data.rps));

        return startEnd(shooterStateDataSubscription::start, () -> {
            shooterStateDataSubscription.cancel();
            stopShooter();
        });
    }

    public Command shooterBumpFire() {
        return runOnce(() -> setDualVelocity(60, 100));
    }

    public Command shooterTrap() {
        return startEnd(() -> setDualVelocity(35, 80), this::stopShooter);
    }

    public Command setVelocityAndStop(double vel) {
        return startEnd(() -> setVelocity(vel), this::stopShooter);
    }

    public Command setVelocityContinuous(double vel) {
        return runOnce(() -> setVelocity(vel));
    }

    public Command spinFeederAndStop(double vel) {
        return startEnd(() -> spinFeeder(vel), this::stopFeeder);
    }

    public Command spinFeederMaxAndStop() {
        return spinFeederAndStop(-1);
    }

    public enum ShooterModes {
        DEFAULT,
        SPEAKER,
        AMP,
        TRAP,
        BUMP
    }

    public enum ShooterStatus {
        FORWARD,
        BACKWARD,
        OFF
    }
}