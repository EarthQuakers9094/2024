package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import Pose

class GotoPosePar(private val shooter: Shooter,private val elevator: Elevator, private val pose: Pose) : SequentialCommandGroup() {
    init {
        // each subsystem used by the command must be passed into the addRequiresments() method
        val height = SetValue.setHeight(elevator,  pose.height);
        val angle = SetValue.setShootingAngle(shooter, pose.angle);
        

            addCommands(angle.alongWith(height));
    }
}
