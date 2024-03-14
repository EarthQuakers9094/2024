import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants
import frc.robot.commands.CommandSequence
import frc.robot.commands.GotoPose
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter

class SpeakerShoot(private val elevator: Elevator, private val shooter: Shooter) :
        CommandSequence() {

    override val commands: List<Command> =
            listOf(
                InstantCommand(
                        object : Runnable {
                            override fun run() {
                                shooter.disableUpdates = true
                                shooter.startShooting(false);
                            }
                        },
                        shooter
                ),
                GotoPose(shooter, elevator, Constants.Poses.speakerShoot, true),
                Shoot(shooter,elevator,false).build()
            )

    override fun finally(interrupted: Boolean) {
        shooter.setAngle(0.0)
        shooter.disableUpdates = false
    }
}
//
//
