package frc.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.SpeakerAlign
import frc.robot.commands.swervedrive.drivebase.TeleopDrive
import frc.robot.subsystems.Swerve
import org.photonvision.PhotonCamera

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...


    private val aprilCamera = PhotonCamera("acam")
    private val swerveDrive = Swerve(/*aprilCamera*/)

    // private val elevator = Elevator(Constants.Elevator.motorID)

    // private val shooter =
    //         Shooter(
    //                 Constants.Shooter.topCanid,
    //                 Constants.Shooter.bottomCanID,
    //                 Constants.Shooter.shooterJointCanID,
    //                 Constants.Shooter.shooterJoint2CanID
    //         )
    //     private val intake =
    //             Intake(
    //                     Constants.Intake.motorid,
    //                     Constants.Intake.followMotorId,
    //             )

    val driverXbox = PS4Controller(Constants.OperatorConstants.kDriverControllerPort)

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    init {
        if (!aprilCamera.isConnected) {
            DriverStation.reportWarning("Hello there miles", arrayOf())
        }
        // Configure the trigger bindings
        configureBindings()

        val leftY = {
            MathUtil.applyDeadband(driverXbox.leftY, Constants.OperatorConstants.LEFT_Y_DEADBAND)
        }

        val leftX = {
            MathUtil.applyDeadband(driverXbox.leftX, Constants.OperatorConstants.LEFT_X_DEADBAND)
        }

        val omega = {
            MathUtil.applyDeadband(
                    driverXbox.getRawAxis(2),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
            )
        }

        val driveMode = { true }

        val simClosedFieldRel = TeleopDrive(swerveDrive, leftY, leftX, omega, driveMode)

        swerveDrive.defaultCommand = simClosedFieldRel
    }

    fun debugPeriodic() {
        SmartDashboard.putBoolean("Connected or nay", aprilCamera.isConnected)
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger#Trigger(java.util.function.BooleanSupplier)] constructor with an arbitrary
     * predicate, or via the named factories in
     * [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [CommandXboxController]/[edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule ExampleCommand when exampleCondition changes to true
        // Trigger { exampleSubsystem.exampleCondition() }.onTrue(ExampleCommand(exampleSubsystem))

        // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
        // cancelling on release.
        // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand())
        JoystickButton(driverXbox, 8).whileTrue(SpeakerAlign(swerveDrive, aprilCamera))
    }

    fun setMotorBrake(enabled: Boolean) {
        swerveDrive.setMotorBrake(enabled)
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            // An example command will be run in autonomous
            return SpeakerAlign(swerveDrive, aprilCamera)
            // return Autos.exampleAuto(exampleSubsystem)
        }
}
