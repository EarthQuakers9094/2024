package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Swerve
import frc.robot.subsystems.Intake
import frc.robot.commands.GotoPose
import frc.robot.commands.CommandSequence
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import FollowTrajectory
import com.pathplanner.lib.path.PathPlannerPath

class ScoreAmp(private val shooter: Shooter,private val elevator: Elevator, private val swerve: Swerve, private val intake: Intake) : CommandSequence() {
    override val commands: List<Command> = listOf(
        FollowTrajectory(swerve, PathPlannerPath.fromPathFile("to amp"), true),
        GotoPose(shooter,elevator,Constants.Poses.amp,true),
        shooter.shootTime(intake, true));

    override fun finally(interrupted: Boolean) { 
        GotoPose(shooter,elevator,Constants.Poses.resting,false).schedule()
    }
}
