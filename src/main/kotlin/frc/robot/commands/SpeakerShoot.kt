import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.Constants
import frc.robot.commands.AimShooter
import frc.robot.commands.FaceDirection
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import frc.robot.commands.CommandSequence
import org.photonvision.PhotonCamera

class SpeakerShoot(
    private val elevator: Elevator,
    private val shooter: Shooter
) : CommandSequence() {

    val supplier = { shooter.atSpeed(false) }

    override val commands: List<Command> =
            listOf(
                InstantCommand(
                    object : Runnable {
                        override fun run() {
                            shooter.startShooting(false)
                        }
                    },
                    shooter
            ),
            WaitUntilCommand(supplier),
            InstantCommand(
                    object : Runnable {
                        override fun run() {
                            shooter.intake()
                        }
                    },
                    shooter
            ),
            WaitCommand(Constants.Shooter.shootTime)
            )

    override fun finally(interrupted: Boolean) {
        shooter.stopIntaking()
        shooter.stopShooting()
    }
}
// 
// 