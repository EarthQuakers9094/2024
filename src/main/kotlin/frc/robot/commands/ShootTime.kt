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

class ShootTime(
        private val shooter: Shooter,
        private val intake: Intake,
        private val elevator: Elevator,
        private val swerveDrive: Swerve,
        private val camera: PhotonCamera
) : CommandSequence() {

    val supplier = { shooter.atSpeed(false) }

    override val commands: List<Command> =
            listOf(
                    InstantCommand(
                            object : Runnable {
                                override fun run() {
                                    intake.startIntaking()
                                }
                            },
                            intake
                    ),
                    FaceDirection(swerveDrive, { swerveDrive.speakerAngle() }, false),
                    AimShooter(camera, shooter, swerveDrive, true),
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
        intake.stopIntaking()
        shooter.stopShooting()
    }
}
// 
// 