import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.commands.CommandSequence
import frc.robot.commands.GotoPose
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter

class LocationShoot(
        private val elevator: Elevator, 
        private val shooter: Shooter,
        private val pose:Pose,
        private val elevatorFirst: Boolean) :
        CommandSequence() {

    override val commands: List<Command> =
            listOf(
                    GotoPose(shooter, elevator, pose, elevatorFirst),
                    Shoot(shooter).build()
            )

    override fun finally(interrupted: Boolean) {
        shooter.setAngle(0.0)
    }
}
//
//
