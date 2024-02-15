package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class GotoAmpPose(private val shooter: Shooter,private val elevator: Elevator) : SequentialCommandGroup() {
    init {
        // each subsystem used by the command must be passed into the addRequiresments() method
        addRequirements(shooter,elevator);

        addCommands(
            SetValue.setHeight(elevator, false, Constants.Poses.Amp.height),
            SetValue.setShootingAngle(shooter, false, Constants.Poses.Amp.angle));
    }
}
