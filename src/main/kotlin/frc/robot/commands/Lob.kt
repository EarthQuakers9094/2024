import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.CommandSequence
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import frc.robot.commands.GotoPose
import frc.robot.Constants

class Lob(private val shooter: Shooter, private val elevator:Elevator) : CommandSequence() {

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
                    GotoPose(shooter,elevator, Constants.Poses.lob, true),
            )

    override fun finally(interrupted: Boolean) {
        // shooter.stopShooting()
    }
}
//
//
