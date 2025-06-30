package frc.robot.util.sim;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

@Logged
public class Mechanisms {
    public Mechanism2d elevatorShooterMech;
    public Mechanism2d intakeMech;

    private final MechanismLigament2d liftLigament;
    private final MechanismLigament2d armLigament;
    private final MechanismLigament2d intakeLigament;

    private final StructArrayPublisher<Pose3d> realComponentPosePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("ComponentPoses/Real", Pose3d.struct)
                    .publish();
    private final StructArrayPublisher<Pose3d> targetComponentPosePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("ComponentPoses/Target", Pose3d.struct)
                    .publish();

    public Mechanisms() {
        elevatorShooterMech = new Mechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));

        liftLigament =
                elevatorShooterMech
                        .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                        .append(
                                new MechanismLigament2d(
                                        "lift",
                                        Units.feetToMeters(3),
                                        90,
                                        6,
                                        new Color8Bit(Color.kRed)));
        elevatorShooterMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(
                        new MechanismLigament2d(
                                "bottom",
                                Units.feetToMeters(3),
                                0,
                                6,
                                new Color8Bit(Color.kGreen)));
        armLigament =
                liftLigament.append(
                        new MechanismLigament2d(
                                "arm", Units.inchesToMeters(12), 0, 6, new Color8Bit(Color.kBlue)));

        intakeMech = new Mechanism2d(Units.inchesToMeters(/* TODO */60), Units.inchesToMeters(/* TODO */ 100));

        intakeLigament =
                intakeMech
                        .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                        .append(new MechanismLigament2d(
                                "Intake",
                                Units.feetToMeters(3), /* TODO */
                                0, /* TODO */
                                6, /* TODO */
                                new Color8Bit(Color.kBlue)
                        ));
    }
    public Mechanism2d getShooter() {
        return elevatorShooterMech;
    }

    public void updateShooter(Angle elevatorPos, Angle armPos) {
        liftLigament.setLength(Units.inchesToMeters((elevatorPos.magnitude() * 6) + 1));
        armLigament.setAngle(armPos.in(Degrees) - 90);

        SmartDashboard.putData("Mechanisms/Shooter", elevatorShooterMech);
    }

    public void updateIntake(Angle armPos) {
        intakeLigament.setAngle(armPos.in(Degrees) - 90);

        SmartDashboard.putData("Mechanisms/Intake", intakeMech);
    }

    public void publishComponentPoses(
            Angle elevatorPos, Angle shooterPos, Angle intakePos, boolean useRealPoses) {
        double elevatorHeight = Units.inchesToMeters(elevatorPos.times(8.6).magnitude());
        double shooterAngle = shooterPos.in(Radians);
        double intakeAngle = intakePos.in(Radians);

        Pose3d shooterPose =
                new Pose3d(0, Units.inchesToMeters(14.76), 0, new Rotation3d(0, shooterAngle, 0)); // TODO
        Pose3d intakePose =
                new Pose3d(0, Units.inchesToMeters(14.76), 0, new Rotation3d(0, intakeAngle, 0)); // TODO

        (useRealPoses ? realComponentPosePublisher : targetComponentPosePublisher)
                .set(
                        new Pose3d[] {
                            new Pose3d(
                                    Units.inchesToMeters(-8),
                                    0.0,
                                    Units.inchesToMeters(2.625) + elevatorHeight,
                                    Rotation3d.kZero),
                            shooterPose,
                            shooterPose.transformBy(
                                    new Transform3d(
                                            intakePose.getTranslation(),
                                            intakePose.getRotation()))
                        });
    }
}
