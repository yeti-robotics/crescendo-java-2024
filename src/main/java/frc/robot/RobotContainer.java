// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ShuttleAimCommand;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.util.controllerUtils.ButtonHelper;
import frc.robot.util.controllerUtils.ControllerContainer;
import frc.robot.util.controllerUtils.MultiButton;


public class RobotContainer {

    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final PivotSubsystem pivot = new PivotSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final ArmSubsystem arm = new ArmSubsystem();
    public final CommandXboxController joystick = new CommandXboxController(1); // My joystick
    public final VisionSubsystem vision = new VisionSubsystem();

    final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(CommandSwerveDrivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(CommandSwerveDrivetrain.MaFxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    public ControllerContainer controllerContainer = new ControllerContainer();
    public SendableChooser<AutoConstants.AutoMode> autoChooser;
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());
    private boolean autoNeedsRebuild = true;
    private Command auto;

    private final RobotCommands robotCommands = new RobotCommands(intake, pivot, shooter, drivetrain, arm, elevator);

    public RobotContainer() {
        NamedCommands.registerCommand("shootBump", Commands.sequence(
                pivot.adjustPivotPositionTo(0.55),
                shooter.shooterBumpFire(),
                Commands.waitSeconds(.75), // is this waiting for a specific speed or something? should prob be replaced
                shooter.spinFeederMaxAndStop().alongWith(intake.rollOut(-1).withTimeout(1))
        ));

        var field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogCurrentPoseCallback(field::setRobotPose);

        PathPlannerLogging.setLogTargetPoseCallback(pose ->
                field.getObject("target pose").setPose(pose)
        );

        PathPlannerLogging.setLogActivePathCallback(poses ->
                field.getObject("path").setPoses(poses)
        );


//        if (Utils.isSimulation()) {
//            drivetrain.seedFieldCentric(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
//        }
        Telemetry logger = new Telemetry(CommandSwerveDrivetrain.MAX_VELOCITY_METERS_PER_SECOND);
//        drivetrain.registerTelemetry(logger::telemeterize);

        configureBindings();

//        buildAutoChooser();
//        rebuildAutoIfNecessary();

        vision.ledOff();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SmartDashboard.putNumber("shooterstate-position", 0.5);
        SmartDashboard.putBoolean("elev mag switch", elevator.getMagSwitch());
    }

    private void configureBindings() {

        buttonHelper.createButton(1, 0, robotCommands.setShuttleState().alongWith(new ShuttleAimCommand(drivetrain, () -> -joystick.getLeftY(), () -> -joystick.getLeftX())), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(8, 0, shooter.setVelocityAndStop(-70), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(7, 0, shooter.setVelocityAndStop(15), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(5, 0, robotCommands.bumpFire(), MultiButton.RunCondition.WHEN_PRESSED);

        buttonHelper.createButton(10, 0, robotCommands.handoff().withTimeout(2), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(2, 0, intake.rollOut(-.65), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(4, 0, elevator.goDownAndStop(0.2).withTimeout(0.3).andThen(elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.DOWN)).andThen(pivot.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF)), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(6, 0, shooter.spinFeederAndStop(-.1).alongWith(intake.rollIn(0.5)), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(9, 0, elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.AMP).andThen(pivot.moveDown(-0.25).unless(
                        () -> pivot.getEncoderAngle() < 0.4).withTimeout(0.6).andThen(pivot.adjustPivotPositionTo(0.03).unless(() -> !elevator.getMagSwitch()))), MultiButton.RunCondition.WHEN_PRESSED);
        intake.intakeOccupiedTrigger.onTrue(vision.blinkLimelight().alongWith(successfulIntakeRumble()));
        intake.intakeOccupiedTrigger.and(joystick.rightBumper().negate()).onTrue(robotCommands.handoff().withTimeout(2));
        buttonHelper.createButton(11, 0, shooter.shooterTrap(), MultiButton.RunCondition.WHILE_HELD);

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        // +X in velocity = forward, -Y in joystick = forward
                                        .withVelocityX(-joystick.getLeftY() * TunerConstants.kSpeedAt12Volts.magnitude())
                                        // +Y in velocity = left, -X in joystick = left
                                        .withVelocityY(-joystick.getLeftX() * TunerConstants.kSpeedAt12Volts.magnitude())
                                        // +rotational rate = counterclockwise (left), -X in joystick = left
                                        .withRotationalRate(-joystick.getRightX() * CommandSwerveDrivetrain.MaFxAngularRate)
                ));

        // Lock on to speaker
        joystick.leftTrigger().whileTrue(new AutoAimCommand(drivetrain, () -> -joystick.getLeftY(), () -> -joystick.getLeftX()).alongWith(intake.rollOut(-0.3).withTimeout(0.2).andThen(robotCommands.setShooterState())));

        // Lock on to shuttle target
        joystick.y().whileTrue(new ShuttleAimCommand(drivetrain, () -> -joystick.getLeftY(), () -> -joystick.getLeftX()).alongWith(intake.rollOut(-0.3).withTimeout(0.2).andThen(robotCommands.setShuttleState())));

        // Swerve lock
        joystick.b().whileTrue(robotCommands.bumpFire());
        joystick.x().onTrue(robotCommands.stowAmp());

        // Reset the field-centric heading
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Suck in note
        joystick.rightBumper().whileTrue(intake.rollIn(.7));

        // Arm down
        joystick.leftBumper().onTrue(arm.deployArm(0.5).alongWith(pivot.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF)));

        // Amp Toggle
        joystick.a().onTrue(robotCommands.setAmp());

        // Shoot
        joystick.rightTrigger().whileTrue(shooter.spinFeederMaxAndStop().alongWith(intake.rollOut(-1)));
        // Handoff
        joystick.povUp().onTrue(robotCommands.handoff().withTimeout(2));
        // Move elevator down
        joystick.povDown().onTrue(elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.DOWN));
        // (These are also unassigned on the gamepad map?)
        joystick.povLeft().whileTrue(pivot.moveUpWithBrake(0.05, -0.01));
        joystick.povRight().whileTrue(pivot.moveDownWithBrake(-0.05, 0.01));
        // Spin feeder
    }

    //Add haptics to beam break
    public Command successfulIntakeRumble() {
        return Commands.startEnd( //Starts the command and waits for ending condition
                        () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0), //Set both rumble motors to max
                        () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)) // Turn off rumble
                .raceWith(Commands.waitSeconds(0.5)); //Condition to end method
    }


    public void updateOdometryVision() {
        var visionResult = vision.getTargetingResults();

        if (DriverStation.getAlliance().isPresent()) { // this is here because it's sometimes not present during simulation
            if (visionResult != null && visionResult.valid && Math.abs(drivetrain.getPigeon2().getRate()) < 230) {
                Pose2d llPose = visionResult.getBotPose2d_wpiBlue();
                drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            }
        }
    }

    public void buildAutoChooser() {
        var namedCommands = new AutoNamedCommands(intake, shooter, pivot, arm, robotCommands);
        namedCommands.registerCommands();

        autoChooser = new SendableChooser<>();

        for (var autoMode : AutoConstants.AutoMode.values()) {
            autoChooser.addOption(autoMode.name, autoMode);
        }
        // note: setDefaultOption overwrites the name in the map, so we won't have duplicate options
        autoChooser.setDefaultOption(AutoConstants.AutoMode.BUMP_ONLY.name, AutoConstants.AutoMode.BUMP_ONLY);

        autoChooser.onChange(obj -> autoNeedsRebuild = true);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void rebuildAutoIfNecessary() {
        if (autoNeedsRebuild) {
            // why is there a timeout here? can't we use the FMS/practice mode timeout?
            auto = AutoBuilder.buildAuto(autoChooser.getSelected().name).withTimeout(15);
            System.out.println("AUTO NAME: " + auto);
            autoNeedsRebuild = false;
        }
    }

    public Command getAutonomousCommand() {
        return auto;
    }
}
