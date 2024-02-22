
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Intake
import frc.robot.Constants
import java.util.function.BooleanSupplier
import frc.robot.subsystems.Swerve
import frc.robot.subsystems.Elevator
import frc.robot.commands.AimShooter
import org.photonvision.PhotonCamera


class ShootTime(private val shooter: Shooter,private val intake: Intake,private val elevator: Elevator, private val camera: PhotonCamera) : SequentialCommandGroup() {
    private enum class State {
        SpinningUp,
        Shooting,
    }

    private var spinUpTimer = Timer();

    private var state = State.SpinningUp;

    init {
        addRequirements(shooter);
        addRequirements(intake);

        val supplier = {shooter.atSpeed(false)}

        addCommands(
            InstantCommand(object: Runnable {
                override fun run() {
                    intake.startIntaking();
                }
            }),
            AimShooter(camera,shooter),
            InstantCommand(object: Runnable {
                override fun run() {
                    shooter.startShooting(false);
                }
            }),
            WaitUntilCommand(supplier),
            InstantCommand(object: Runnable {
                override fun run() {
                    shooter.intake();
                }
            }),
            WaitCommand(Constants.Shooter.shootTime));

            finallyDo(Runnable {
                shooter.stopIntaking();
                intake.stopIntaking();
                shooter.stopShooting();
            });
    }
}
