

import frc.robot.subsystems.Swerve
import frc.robot.Constants
import com.pathplanner.lib.path.PathPlannerTrajectory
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.commands.FollowPathHolonomic
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.math.kinematics.ChassisSpeeds
import java.util.function.Consumer

class FollowTrajectory(swerve: Swerve, trajectory: PathPlannerPath, flipPath:Boolean, pathFind:Boolean):SequentialCommandGroup() {
    init {
        var flipy = {flipPath};
        var getPose = {swerve.getPos()};
        var getSpeeds = {swerve.getSpeeds()};
        var drive: Consumer<ChassisSpeeds> = Consumer { speeds -> swerve.driveRobotRel(speeds)};
        
        var replanning = ReplanningConfig();

        var config = HolonomicPathFollowerConfig(
            Constants.Drivebase.TRANSLATION_PID,
            Constants.Drivebase.ROTATION_PID,
            Constants.Drivebase.MAX_AUTO_SPEEDS,
            Constants.Drivebase.RADIUS,
            replanning);

        var pathConstraints = PathConstraints(
            Constants.Drivebase.MAX_AUTO_SPEEDS,
            Constants.Drivebase.MAX_ACCEL,
            Constants.Drivebase.MAX_TURNING_SPEEDS,
            Constants.Drivebase.MAX_ANGULAR_ACCELERATION,
            );

        if (pathFind) {
            addCommands(PathfindThenFollowPathHolonomic(
                trajectory,
                pathConstraints,
                getPose,
                getSpeeds,
                drive,
                config,
                flipy,
                swerve
                ))
        } else {
            addCommands(FollowPathHolonomic(trajectory,
            getPose,
            getSpeeds,
            drive,
            config,
            flipy,
            swerve));
        }
    }
    
}
