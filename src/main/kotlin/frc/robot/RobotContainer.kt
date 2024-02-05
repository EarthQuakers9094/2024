package frc.robot

import RunAuto
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.swervedrive.drivebase.TeleopDrive
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Swerve
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter
import Config

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private val swerveDrive = Swerve()

    private var elevator: Elevator? = null;
    // (Constants.Elevator.motorID)

    private var shooter: Shooter? = null;
    //         Shooter(
    //                 Constants.Shooter.topCanid,
    //                 Constants.Shooter.bottomCanID,
    //                 Constants.Shooter.shooterJointCanID,
    //                 Constants.Shooter.shooterJoint2CanID
    //         )

    private var intake:Intake? = null;
            // Intake(
            //         Constants.Intake.motorid,
            //         Constants.Intake.followMotorId,
            // )

    val driverXbox = PS4Controller(Constants.OperatorConstants.kDriverControllerPort)
    val driverLeftStick = Joystick(Constants.OperatorConstants.driverLeftStickPort)
    val driverRightStick = Joystick(Constants.OperatorConstants.driverRightStickPort)

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    init {
        // Configure the trigger bindings
        configureBindings()

        val onTest = Config(true,false);

        if (!onTest.config) {
            intake = Intake(Constants.Intake.motorid,Constants.Intake.followMotorId);
            // shooter = Shooter(
            //         Constants.Shooter.topCanid,
            //         Constants.Shooter.bottomCanID,
            //         Constants.Shooter.shooterJointCanID,
            //         Constants.Shooter.shooterJoint2CanID);
            // elevator = Elevator(Constants.Elevator.motorID);
        }

        val leftY = {
            MathUtil.applyDeadband(
                    driverLeftStick.getY(),
                    Constants.OperatorConstants.LEFT_Y_DEADBAND
            )
        }

        val leftX = {
            MathUtil.applyDeadband(
                    driverLeftStick.getX(),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
            )
        }

        val omega = {
            MathUtil.applyDeadband(
                    driverRightStick.getX(),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
            )
        }

        val driveMode = { true }

        val simClosedFieldRel = TeleopDrive(swerveDrive, leftY, leftX, omega, driveMode)

        swerveDrive.setDefaultCommand(simClosedFieldRel)
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
            return RunAuto("4 piece")
            // return Autos.exampleAuto(exampleSubsystem)
        }
}
