
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

class Pickup(private val shooter: Shooter,private val intake: Intake) : SequentialCommandGroup() {

    init {
        addRequirements(shooter);   

        val supplier:BooleanSupplier = BooleanSupplier {shooter.noteIn()};
        
        addCommands(
            InstantCommand(object: Runnable {
                    override fun run() {
                        shooter.intake();
                        intake.startIntaking();
                    }
            }),
            WaitUntilCommand(supplier),
            InstantCommand(object: Runnable {
                override fun run() {
                    intake.stopIntaking();
                    shooter.stopIntaking()
                }
            })
        );
    }
}
