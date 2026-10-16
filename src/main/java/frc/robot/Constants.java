package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ShooterStateData;

import java.io.IOException;
import java.util.Map;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

public class Constants {

    public class FieldConstants{
        public static final double fieldLength = Units.inchesToMeters(651.223);

        public static final class Speaker {
            /**
             * Center of the speaker opening (blue alliance)
             */
            public static Translation3d topRightSpeaker =
                    new Translation3d(
                            Units.inchesToMeters(18.055),
                            Units.inchesToMeters(238.815),
                            Units.inchesToMeters(83.091));
            public static Translation3d bottomLeftSpeaker =
                    new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
            public static Translation3d centerSpeakerOpening =
                    bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
            public static final AprilTagFieldLayout aprilTags;
            static {
                try {
                    aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }

        public static final class Shuttle {
            public static Translation2d closeShuttleTargetCorner =
                    new Translation2d(0.0,7.0);

            public static Translation2d farShuttleTargetCorner =
                    new Translation2d(2.5, 7.0);

            public static Translation2d shuttleTargetZone =
                    closeShuttleTargetCorner.interpolate(farShuttleTargetCorner, 0.5);
        }
    }

    public class CANCoderConstants{
        public static final int COUNTS_PER_REV = 4096;
        public static final double COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;
    }

    public class OIConstants{
        public static final Map<Integer, ControllerType> CONTROLLERS = Map.of(
                0, ControllerType.CUSTOM, 1, ControllerType.XBOX
        );

        public static final int CONTROLLER_COUNT = CONTROLLERS.size(); //placehold


        public enum ControllerType {
            CUSTOM,
            XBOX
        }
    }

    public class TalonFXConstants{
        public static final boolean TALON_FUTURE_PROOF = true;
        public static final CANBus CANIVORE_NAME = new CANBus("canivoreBus");
    }
}
