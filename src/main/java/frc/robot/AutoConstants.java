package frc.robot;


import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;

public final class AutoConstants {

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.magnitude();
    public static final double MAX_ACCEL = TunerConstants.kSpeedAt12Volts.magnitude();
    public static final double MAX_THETA_VELOCITY =  MAX_SPEED / Math.hypot(22.25 / 2, 22.25 / 2);
    public static final double MAX_THETA_ACCEL = MAX_SPEED / Math.hypot(22.25 / 2, 22.25 / 2);
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
            MAX_SPEED,
            MAX_ACCEL,
            MAX_THETA_VELOCITY,
            MAX_THETA_ACCEL);

    public enum AutoMode {
        //AMP_4_TWO_PIECE("amp4TwoPiece"), // unused
        //AMP_4_THREE_PIECE("amp4ThreePiece"), // unused
        BUMP_ONLY("bumpOnly"),
        CLEAR_AUTO("1160 Auto"),
        //MID_3_TWO_PIECE("mid3TwoPiece"), // unused
        //MID_3_THREE_PIECE("mid3ThreePiece"), // unused
        // also unused: leaveAuto, mid3OnePiece
        MID_SUB_TWO_PIECE("midSubTwoPiece"),
        MID_SUB_THREE_PIECE("midSubThreePiece"),
        MID_SUB_FOUR_PIECE("midSubFourPiece"),
        MIDLINE_DASH_THREE_PIECE("midlineDashThreePiece"),
        SHUTTLE_3_SOURCE("shuttle3FromSource"),
        SOURCE_BYPASS_AUTO("source4MidBypass"),
        SOURCE_SIDE_2_PIECE("source4TwoPiece"),
        SOURCE_SIDE_3_PIECE("source4ThreePiece"),
        SOURCE_SIDE_SHUTTLE_AMP("source4ShuttleAmp");

        public final String name;
        public final PathConstraints initConstraint;
        public final PathConstraints[] pathConstraints;

        AutoMode(String name) {
            this(name, DEFAULT_CONSTRAINTS);
        }

        AutoMode(String name, PathConstraints initConstraint, PathConstraints... pathConstraints) {
            this.name = name;
            this.initConstraint = initConstraint;
            this.pathConstraints = pathConstraints;
        }
    }}