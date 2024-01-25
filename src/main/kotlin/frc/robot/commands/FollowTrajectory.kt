import com.pathplanner.lib.commands.FollowPathHolonomic
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.subsystems.Swerve
import java.util.function.Consumer

class FollowTrajectory(
                swerve: Swerve,
                trajectory: PathPlannerPath,
                flipPath: Boolean,
                pathFind: Boolean
) : SequentialCommandGroup() {
        init {
                var flipy = { flipPath }
                var getPose = {
                        var pose = swerve.getPos()
                        SmartDashboard.putNumber("get pose pathplanner x", pose.getX())
                        SmartDashboard.putNumber("get pose pathplanner y", pose.getY())
                        SmartDashboard.putNumber(
                                        "get pose pathplanner rotation",
                                        pose.rotation.degrees
                        )

                        pose
                }
                var getSpeeds = { swerve.getSpeeds() }
                var drive: Consumer<ChassisSpeeds> = Consumer { speeds ->
                        SmartDashboard.putNumber(
                                        "path planner direction x",
                                        speeds.vxMetersPerSecond
                        )
                        SmartDashboard.putNumber(
                                        "path planner direction y",
                                        speeds.vyMetersPerSecond
                        )

                        swerve.driveRobotRel(speeds)
                }

                var replanning = ReplanningConfig()

                var config =
                                HolonomicPathFollowerConfig(
                                                Constants.Drivebase.TRANSLATION_PID,
                                                Constants.Drivebase.ROTATION_PID,
                                                Constants.Drivebase.MAX_AUTO_SPEEDS,
                                                Constants.Drivebase.RADIUS,
                                                replanning
                                )

                var pathConstraints =
                                PathConstraints(
                                                Constants.Drivebase.MAX_AUTO_SPEEDS,
                                                Constants.Drivebase.MAX_ACCEL,
                                                Constants.Drivebase.MAX_TURNING_SPEEDS,
                                                Constants.Drivebase.MAX_ANGULAR_ACCELERATION,
                                )

                if (pathFind) {
                        addCommands(
                                        PathfindThenFollowPathHolonomic(
                                                        trajectory,
                                                        pathConstraints,
                                                        getPose,
                                                        getSpeeds,
                                                        drive,
                                                        config,
                                                        flipy,
                                                        swerve
                                        )
                        )
                } else {
                        addCommands(
                                        FollowPathHolonomic(
                                                        trajectory,
                                                        getPose,
                                                        getSpeeds,
                                                        drive,
                                                        config,
                                                        flipy,
                                                        swerve
                                        )
                        )
                }
        }
}
