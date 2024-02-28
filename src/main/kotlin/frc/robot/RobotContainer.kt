package frc.robot

import FollowTrajectory
import Pickup
import RunAuto
import Shoot
import ShootTime
import SpeakerShoot
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.AimShooter
import frc.robot.commands.Brake
import frc.robot.commands.Climb
import frc.robot.commands.FaceDirection
import frc.robot.commands.GotoPose
import frc.robot.commands.SetValue
import frc.robot.commands.swervedrive.drivebase.TeleopDrive
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import frc.robot.utils.Config
import org.photonvision.PhotonCamera

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private val aprilCamera = PhotonCamera("BW")
    private val noteCamera = PhotonCamera("NC")
    private val swerveDrive = Swerve(/*aprilCamera*/ )

    private var elevator: Elevator? = null
    // (Constants.Elevator.motorID)
    // (Constants.Elevator.motorID)

    private var shooter: Shooter? = null

    //         Shooter(
    //                 Constants.Shooter.topCanid,
    //                 Constants.Shooter.bottomCanID,
    //                 Constants.Shooter.shooterJointCanID,
    //                 Constants.Shooter.shooterJoint2CanID
    //         )

    private var intake: Intake? = null

    private var faceSpeaker = false

    val operatorExtra = CommandXboxController(Constants.OperatorConstants.kDriverControllerPort)
    val driverLeftStick = Joystick(Constants.OperatorConstants.driverLeftStickPort)
    val driverRightStick = Joystick(Constants.OperatorConstants.driverRightStickPort)

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    init {
        // Configure the trigger bindings

        val onTest = Config(true, false)

        if (!onTest.config) {
            intake =
                    Intake(
                            Constants.Intake.motorid,
                            Constants.Intake.followMotorId,
                            Constants.Intake.frontIntakeId
                    )
            shooter =
                    Shooter(
                            Constants.Shooter.topCanid,
                            Constants.Shooter.bottomCanID,
                            Constants.Shooter.shooterJointCanID,
                            Constants.Shooter.intakeMotorID
                    )
            elevator = Elevator(Constants.Elevator.motorID, Constants.Elevator.followMotorID)

            NamedCommands.registerCommand(
                    "pickup",
                    Pickup(shooter!!, elevator!!, intake!!, false).build()
            )
            NamedCommands.registerCommand(
                    "faceSpeaker",
                    FaceDirection(swerveDrive, { swerveDrive.speakerAngle() }, false)
            )
            NamedCommands.registerCommand(
                    "shoot",
                    ShootTime(shooter!!, intake!!, elevator!!, swerveDrive, aprilCamera).build()
            )
            NamedCommands.registerCommand(
                    "facedown",
                    FaceDirection(swerveDrive, { Rotation2d.fromRadians(-Math.PI / 2.0) }, false)
            )
        }

        configureBindings()

        fun applyPov(direction: Int, speed: Double): Double {
            if (direction == -1) {
                return speed
            } else if (direction == 0) {
                return speed
            } else if (direction == 180) {
                return speed * 0.5
            }

            return speed
        }

        val leftY = {
            MathUtil.applyDeadband(
                    applyPov(driverLeftStick.getPOV(), driverLeftStick.getY()),
                    Constants.OperatorConstants.LEFT_Y_DEADBAND
            )
        }

        val leftX = {
            MathUtil.applyDeadband(
                    applyPov(driverLeftStick.getPOV(), driverLeftStick.getX()),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
            )
        }

        val omega = {
            MathUtil.applyDeadband(
                    -driverRightStick.getX(),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
            )
        }

        val driveMode = { true }

        val faceSpeaker = { false }

        val simClosedFieldRel =
                TeleopDrive(swerveDrive, leftY, leftX, omega, driveMode, faceSpeaker)

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
        val mattSaysNoFancy = true
        if (shooter != null && intake != null && elevator != null) {
            SmartDashboard.putBoolean("shooter", true)

            if (!mattSaysNoFancy) {
                JoystickButton(driverLeftStick, 2)
                        .onTrue(
                                ShootTime(shooter!!, intake!!, elevator!!, swerveDrive, aprilCamera)
                                        .build()
                        )
                operatorExtra
                        .x()
                        .toggleOnTrue(
                                SequentialCommandGroup(
                                                InstantCommand(
                                                        object : Runnable {
                                                            override fun run() {
                                                                faceSpeaker = true
                                                            }
                                                        }
                                                ),
                                                AimShooter(
                                                        aprilCamera,
                                                        shooter!!,
                                                        swerveDrive,
                                                        false
                                                )
                                        )
                                        .finallyDo({ _ -> faceSpeaker = false })
                        )
                JoystickButton(operatorExtra.hid, 6)
                        .onTrue(GotoPose(shooter!!, elevator!!, Constants.Poses.highPickup, true))
                JoystickButton(driverRightStick, 3)
                        .onTrue(
                                FollowTrajectory(
                                        swerveDrive,
                                        PathPlannerPath.fromPathFile("to amp"),
                                        true
                                )
                        )
                JoystickButton(driverRightStick, 4)
                        .onTrue(
                                FollowTrajectory(
                                        swerveDrive,
                                        PathPlannerPath.fromPathFile("to pickup"),
                                        true
                                )
                        )
                JoystickButton(driverLeftStick, 3)
                        .onTrue(
                                FollowTrajectory(
                                        swerveDrive,
                                        PathPlannerPath.fromPathFile("to shoot position1"),
                                        true
                                )
                        )
                JoystickButton(driverRightStick, 4)
                        .onTrue(
                                FollowTrajectory(
                                        swerveDrive,
                                        PathPlannerPath.fromPathFile("to shoot position2"),
                                        true
                                )
                        )
            }

            operatorExtra.rightTrigger().whileTrue(Shoot(shooter!!).build())
            operatorExtra.rightBumper().whileTrue(shooter!!.backButton())
            operatorExtra
                    .leftTrigger()
                    .whileTrue(Pickup(shooter!!, elevator!!, intake!!, false).build())
            operatorExtra
                    .leftBumper()
                    .whileTrue(Pickup(shooter!!, elevator!!, intake!!, true).build())

            operatorExtra.leftStick().onTrue(SpeakerShoot(elevator!!, shooter!!).build())

            operatorExtra.y().onTrue(SetValue.setHeight(elevator!!, 46.0))
            operatorExtra.a().onTrue(SetValue.setHeight(elevator!!, 0.0))
            operatorExtra.b().onTrue(GotoPose(shooter!!, elevator!!, Constants.Poses.amp, true))

            JoystickButton(driverRightStick, 5).toggleOnTrue(Climb(elevator!!).build())
            JoystickButton(driverRightStick, 6).whileTrue(SetValue.setShootingAngle(shooter!!, 0.0))

            //            JoystickButton(driverLeftStick, 5)
            //                    .whileTrue(SetValue.setShootingAngle(shooter!!, Math.PI * 55 /
            // 180))

            JoystickButton(driverLeftStick, 7)
                    .onTrue(
                            InstantCommand(
                                    object : Runnable {
                                        override fun run() {
                                            swerveDrive.resetOdomentry(
                                                    Constants.Camera.resetPosition()
                                            )
                                        }
                                    },
                                    swerveDrive
                            ),
                    )
        }

        JoystickButton(driverLeftStick, 1).whileTrue(Brake(swerveDrive))
    }

    fun setMotorBrake(enabled: Boolean) {
        swerveDrive.setMotorBrake(enabled)
    }

    fun periodic() {
        operatorExtra.hid.setRumble(
                GenericHID.RumbleType.kBothRumble,
                (if (noteCamera.latestResult.hasTargets()) {
                    1.0
                } else {
                    0.0
                })
        )
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            // An example command will be run in autonomous
            return RunAuto("do nothing")
        }
}
