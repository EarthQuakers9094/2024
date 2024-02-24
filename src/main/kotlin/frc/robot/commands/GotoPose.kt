package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import Pose

class GotoPose(private val shooter: Shooter,private val elevator: Elevator, private val pose: Pose, private val elevatorFirst: Boolean) : SequentialCommandGroup() {
    init {
        // each subsystem used by the command must be passed into the addRequiresments() method
        val height = SetValue.setHeight(elevator, true, pose.height);
        val angle = SetValue.setShootingAngle(shooter, true, pose.angle);
        
        if (elevatorFirst) {
            addCommands(height,angle);
        } else {
            addCommands(angle,height);
        }
    }
}
