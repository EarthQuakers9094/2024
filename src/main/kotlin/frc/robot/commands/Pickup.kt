
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Intake
import frc.robot.Constants
import frc.robot.commands.SetValue
import frc.robot.commands.GotoPose
import java.util.function.BooleanSupplier
import frc.robot.subsystems.Elevator

class Pickup(private val shooter: Shooter, elevator: Elevator,private val intake: Intake) : CommandSequence() {

    val supplier:BooleanSupplier = BooleanSupplier {shooter.noteIn()};

    override val commands: List<Command> = listOf(
        GotoPose(shooter, elevator, Constants.Poses.pickup, false),
        InstantCommand(object: Runnable {
            override fun run() {
                SmartDashboard.putNumber("running pickup", 1.0);
                shooter.intake();
                intake.startIntaking();
            }
        }, shooter, intake),
        WaitUntilCommand(supplier),
        InstantCommand(object: Runnable {
            override fun run() {
                intake.stopIntaking();
                shooter.stopIntaking();
                SmartDashboard.putNumber("running pickup", 2.0);
            }
        },intake,shooter)

        );

    override fun finally(interrupted : Boolean) {
        SmartDashboard.putNumber("running pickup", 3.0);
        intake.stopIntaking();
        shooter.stopIntaking();
        shooter.setAngle(0.0);
    }
}
