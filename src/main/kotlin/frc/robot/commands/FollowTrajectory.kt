

import com.pathplanner.lib.commands.FollowPathHolonomic
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.subsystems.Swerve
import java.util.function.Consumer

class FollowTrajectory(swerve: Swerve, trajectory: PathPlannerPath, flipPath: Boolean) : SequentialCommandGroup() {
    init {

        var flipy = { flipPath }
        var getPose = { swerve.getPos() }
        var getSpeeds = { swerve.getSpeeds() }
        var drive: Consumer<ChassisSpeeds> = Consumer { speeds -> swerve.driveRobotRel(speeds) }

        var replanning = ReplanningConfig()

        var config = HolonomicPathFollowerConfig(
            Constants.Drivebase.MAX_AUTO_SPEEDS,
            Constants.Drivebase.RADIUS,
            replanning,
        )

        addCommands(
            FollowPathHolonomic(
                trajectory,
                getPose,
                getSpeeds,
                drive,
                config,
                flipy,
                swerve,
            ),
        )
    }
}
