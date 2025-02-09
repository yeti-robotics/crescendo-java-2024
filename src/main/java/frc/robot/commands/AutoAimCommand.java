 package frc.robot.commands;

 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.wpilibj2.command.Command;
 import frc.robot.Constants;
 import frc.robot.subsystems.VisionSubsystem;
 import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
 import frc.robot.util.AllianceFlipUtil;
 import frc.robot.util.LimelightHelpers;

 import java.util.function.DoubleSupplier;

public class AutoAimCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xVelSupplier;
    private final TurnToPoint poseAimRequest;
    private final DoubleSupplier yVelSupplier;
    private double currentTag;

    public AutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier) {

        this.drivetrain = drivetrain;
        this.xVelSupplier = xVelSupplier;
        this.yVelSupplier = yVelSupplier;

        addRequirements(drivetrain);

        poseAimRequest = new TurnToPoint();
        poseAimRequest.HeadingController.setPID(5,0,0);
        poseAimRequest.HeadingController.enableContinuousInput(-Math.PI,Math.PI);
    }

    @Override
    public void initialize() {
        currentTag = LimelightHelpers.getFiducialID(VisionSubsystem.VisionConstants.LIMELIGHT_NAME);

        Translation2d  speakerCenter = AllianceFlipUtil.apply(
                Constants.FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()
        );

        poseAimRequest.setPointToFace(speakerCenter);
    }

    @Override
    public void execute() {
        if(LimelightHelpers.getFiducialID(VisionSubsystem.VisionConstants.LIMELIGHT_NAME) == currentTag) {
            drivetrain.setControl(
                    poseAimRequest.withVelocityX(xVelSupplier.getAsDouble() * 1.5).withVelocityY(yVelSupplier.getAsDouble() * 1.5)
            );
        } else {
            end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

