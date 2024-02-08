import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.Shooter
import frc.robot.Constants

class ShootTime(shooter: Shooter) : SequentialCommandGroup() {
    init {

        addRequirements(shooter);

        addCommands(
            InstantCommand(object: Runnable {
                override fun run() {
                    shooter.startShooting();
                }
            }),
            WaitCommand(Constants.Shooter.spinuptime),
            InstantCommand(object: Runnable {
                override fun run() {
                    shooter.intake();
                }
            }),
            WaitCommand(Constants.Shooter.shootTime),
            InstantCommand(object: Runnable {
                override fun run() {
                    shooter.stopIntaking();
                    shooter.stopShooting();
                }
            })
        )
    }
}
