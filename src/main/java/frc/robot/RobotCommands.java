package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotDataPublisher;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;
import frc.robot.util.controllerUtils.MultiButton;

public class RobotCommands {

    private final IntakeSubsystem intake;
    private final PivotSubsystem pivot;
    private final ShooterSubsystem shooter;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;

    public RobotCommands(IntakeSubsystem intake, PivotSubsystem pivot, ShooterSubsystem shooter, CommandSwerveDrivetrain commandSwerveDrivetrain, ArmSubsystem arm, ElevatorSubsystem elevator1) {
        this.intake = intake;
        this.pivot = pivot;
        this.shooter = shooter;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.arm = arm;
        this.elevator = elevator1;
    }

    /**
     * Requires
     * Sets shooter state in preparation for shooting
     *
     * @return {@code Command} instance
     */
    public Command setShooterState() {
        RobotDataPublisher<ShooterStateData> shooterStatePublisher = commandSwerveDrivetrain.observablePose().map(robotPose -> {
            final Pose2d speakerPose = AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0)));
            Pose2d relativeSpeaker = robotPose.relativeTo(speakerPose);
            double distance = relativeSpeaker.getTranslation().getNorm();
            return ShooterSubsystem.ShooterConstants.SHOOTER_MAP().get(distance);
        });

        return pivot.updatePivotPositionWith(shooterStatePublisher)
                .alongWith(shooter.updateVelocityWith(shooterStatePublisher));
    }

    public Command setShuttleState() {
        RobotDataPublisher<ShooterStateData> shuttleStatePublisher = commandSwerveDrivetrain.observablePose().map(robotPose -> {
            final Pose2d shuttleTarget = AllianceFlipUtil.apply(new Pose2d(2.5, 7.0, Rotation2d.fromDegrees(0)));
            Pose2d relativeShuttleTarget = robotPose.relativeTo(shuttleTarget);
            double distance = relativeShuttleTarget.getTranslation().getNorm();
            return ShooterSubsystem.ShooterConstants.SHUTTLE_MAP().get(distance);
        });
        return pivot.updatePivotPositionWith(shuttleStatePublisher).alongWith(shooter.updateVelocityWith(shuttleStatePublisher));
    }

    public Command handoff() {
        return pivot.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF).andThen(
                arm.moveUpAndStop(.5).until(() ->
                        arm.getEnc() >= ArmSubsystem.ArmConstants.ARM_HANDOFF_POSITION).andThen(
                        shooter.spinFeederAndStop(-0.3).alongWith(intake.rollOut(-0.15))
                )
        );
    }

    public Command setAmp() {
        return elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.AMP).andThen(pivot.moveDown(-0.25).unless(
                        () -> pivot.getEncoderAngle() < 0.4).withTimeout(0.6).andThen(pivot.adjustPivotPositionTo(0.03).unless(() -> !elevator.getMagSwitch())));
    }

    public Command stowAmp() {
        return pivot.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF).andThen(elevator.goDownAndStop(0.2).withTimeout(0.3).andThen(elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.DOWN)));
    }


    public Command bumpFire(){
        return pivot.adjustPivotPositionTo(.53).andThen(
                new StartEndCommand(() -> arm.moveUp(.7), arm::stop).until(() ->
                        arm.getEnc() <= ArmSubsystem.ArmConstants.ARM_HANDOFF_POSITION).andThen(
                        shooter.spinFeederAndStop(-0.3).alongWith(intake.rollOut(-.2))
                ).until(shooter::getBeamBreak)
        ).andThen(
                shooter.setVelocityAndStop(100)).andThen(
                intake.rollOut(-.1).withTimeout(0.2)).andThen(
                Commands.waitSeconds(.45)).andThen(
                shooter.spinFeederMaxAndStop().alongWith(intake.rollOut(-1).withTimeout(1)).andThen(
                        this.handoff().withTimeout(2)));
    }
}
