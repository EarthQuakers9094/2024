
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

class ShootTime(private val shooter: Shooter,private val intake: Intake) : Command() {

    private enum class State {
        SpinningUp,
        Shooting,
    }

    private var spinUpTimer = Timer();

    private var state = State.SpinningUp;

    init {
        addRequirements(shooter);
        addRequirements(intake);
    }

    override fun end(terminate: Boolean) {
        shooter.stopIntaking();
        intake.stopIntaking();
    }

    override fun initialize() {
        shooter.intake();
        intake.startIntaking();
        state = State.SpinningUp;
    }

    override fun isFinished():Boolean {
        return shooter.noteIn();
    }
}
