package frc.robot.subsystems;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.autostuff.AutoHelpers.ArcUtils.Point;

public class AutoConstants {
    public static final Pose2d testingPose = new Pose2d(5,10,new Rotation2d(0));
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    public static final Point HUB_RED = new Point(0, 0);
    public static final Point HUB_BLUE = new Point(0, 0);
}
