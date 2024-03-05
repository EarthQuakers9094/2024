import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.CommandSequence
import frc.robot.subsystems.Shooter

class Shoot(private val shooter: Shooter) : CommandSequence() {

    val supplier = { shooter.atSpeed(false) }

    val inShooter = { !shooter.noteIn() }

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
                    WaitUntilCommand(inShooter),
                    WaitCommand(0.5)
            )

    override fun finally(interrupted: Boolean) {
        shooter.stopIntaking()
        shooter.stopShooting()
    }
}
//
//
