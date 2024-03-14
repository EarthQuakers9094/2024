import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.CommandSequence
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import frc.robot.commands.GotoPose

class Shoot(private val shooter: Shooter, private val elevator: Elevator, private val amp: Boolean) : CommandSequence() {

    val supplier = { shooter.atSpeed(false) }

    val inShooter = { !shooter.noteIn() }

    override val commands: List<Command> =
            listOf(
                    InstantCommand(
                            object : Runnable {
                                override fun run() {
                                    shooter.startShooting(amp)
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
        GotoPose(shooter, elevator, Pose(0.0, 0.0), false).schedule();
    }
}
//
//
