package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotCommands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoNamedCommands {
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final ArmSubsystem armSubsystem;
    private final RobotCommands robotCommands;

    public AutoNamedCommands(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, ArmSubsystem armSubsystem, RobotCommands robotCommands) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.armSubsystem = armSubsystem;
        this.robotCommands = robotCommands;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand("rollIn", intakeSubsystem.rollIn(0.9));
        NamedCommands.registerCommand("rollOut", intakeSubsystem.rollOut(-0.7));
        NamedCommands.registerCommand("armUp", armSubsystem.moveUpAndStop(.5).until(() -> armSubsystem.getEnc() >= 0.75));
        NamedCommands.registerCommand("armDown", armSubsystem.moveDownAndStop(.5).until(
                () -> armSubsystem.getEnc() <= .5 && armSubsystem.getEnc() >= .44).alongWith(pivotSubsystem.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF)));
        NamedCommands.registerCommand("shootBump", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(0.53),
                shooterSubsystem.shooterBumpFire(),
                new WaitCommand(.75),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1)),
                new InstantCommand(shooterSubsystem::stopShooter)
        ));

        NamedCommands.registerCommand("shootLine", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.47),
                shooterSubsystem.setVelocityContinuous(100),
                intakeSubsystem.rollOut(-0.1).withTimeout(0.25),
                new WaitCommand(.45),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1.7))
        ));

        NamedCommands.registerCommand("shootLine2", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.39),
                shooterSubsystem.setVelocityContinuous(100),
                intakeSubsystem.rollOut(-0.1).withTimeout(0.2),
                new WaitCommand(.45),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));

        NamedCommands.registerCommand("shootNearSource", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.48),
                shooterSubsystem.setVelocityContinuous(90),
                intakeSubsystem.rollOut(-0.1).withTimeout(0.2),
                new WaitCommand(.45),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));

        NamedCommands.registerCommand("shuttleMid", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.535),
                shooterSubsystem.setVelocityContinuous(100),
                intakeSubsystem.rollOut(-0.1).withTimeout(0.2),
                new WaitCommand(.45),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));


        NamedCommands.registerCommand("shootBumpCorner", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.55),
                shooterSubsystem.shooterBumpFire(),
                new WaitCommand(.75),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1)),
                new InstantCommand(shooterSubsystem::stopShooter)
        ));

        NamedCommands.registerCommand("shootBumpLast", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.55),
                shooterSubsystem.shooterBumpFire(),
                new WaitCommand(.25),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));


        NamedCommands.registerCommand("shootFar", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.45),
                shooterSubsystem.setVelocityContinuous(100),
                new WaitCommand(.75),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));


        NamedCommands.registerCommand("shootOut", shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1)));
        NamedCommands.registerCommand("handoff", robotCommands.handoff().withTimeout(1.5));
        NamedCommands.registerCommand("handoffMidLine", intakeSubsystem.rollIn(.275)
                .withTimeout(0.5).andThen(
                        robotCommands.handoff().withTimeout(1.5)));

        NamedCommands.registerCommand("pivotHandoff", pivotSubsystem.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF));
    }
}