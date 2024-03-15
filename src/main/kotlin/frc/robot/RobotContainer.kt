package frc.robot

import FollowTrajectory
import Pickup
import RunAuto
import Shoot
import ShootTime
import SpeakerShoot
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.AimShooter
import frc.robot.commands.Brake
import frc.robot.commands.Climb
import frc.robot.commands.CollectNote
import frc.robot.commands.FaceDirection
import frc.robot.commands.GotoPose
import frc.robot.commands.GotoPoseParAmp
import frc.robot.commands.SetValue
import frc.robot.commands.swervedrive.drivebase.TeleopDrive
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import frc.robot.utils.Config
import org.photonvision.PhotonCamera
import Pose
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.simulation.JoystickSim
import java.util.Optional
import Lob
import LocationShoot
import kotlin.math.sign

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private val aprilCamera = PhotonCamera("ATBack")
        private val frontNoteCamera = PhotonCamera("NTFront")
        private val backNoteCamera = PhotonCamera("NTBack")
        private val swerveDrive = Swerve(/*aprilCamera*/ )

        var elevator: Elevator? = null
        // (Constants.Elevator.motorID)
        // (Constants.Elevator.motorID)

        public var shooter: Shooter? = null

        //         Shooter(
        //                 Constants.Shooter.topCanid,
        //                 Constants.Shooter.bottomCanID,
        //                 Constants.Shooter.shooterJointCanID,
        //                 Constants.Shooter.shooterJoint2CanID
        //         )

        private var intake: Intake? = null

    private var faceSpeaker = false

    private var readyShoot = false
    private var autoChooser: SendableChooser<Command>;
//     private var autoChooser: AutoBuilder;

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
                        elevator =
                                        Elevator(
                                                        Constants.Elevator.motorID,
                                                        Constants.Elevator.followMotorID
                                        )

            NamedCommands.registerCommand(
                    "pickup",
                    Pickup(shooter!!, elevator!!, intake!!, false, false).build()
            )            
            
            NamedCommands.registerCommand(
                    "pickupReturn",
                    Pickup(shooter!!, elevator!!, intake!!, false, true).build()
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
                    FaceDirection(swerveDrive, { Rotation2d.fromRadians(0.0) }, false)
            )
            NamedCommands.registerCommand(
                "shootSpeaker",
                SpeakerShoot(elevator!!, shooter!!).build()
           )
           NamedCommands.registerCommand("rev shooter", InstantCommand(
                object : Runnable {
                        override fun run() {
                                shooter!!.startShooting(false);
                        }
                }
           ))
           NamedCommands.registerCommand(
                "autoPickup",
                CollectNote(
                                                        PIDConstants(0.045, 0.0, 0.001000),
                                                        frontNoteCamera,
                                                       
                                                        swerveDrive,
                                                        10,
                                            {->shooter!!.noteIn()},
                                            false, intake!!
                                        ),
        )

           NamedCommands.registerCommand("move forward", InstantCommand(
                object : Runnable {
                        override fun run() {
                                swerveDrive.drive(Translation2d(1.0,0.0),0.0,false);
                        }
                }
           ).andThen(WaitCommand(0.3)).andThen(
                object : Runnable {
                        override fun run() {
                                swerveDrive.drive(Translation2d(0.0,0.0),0.0,false);
                        }
                }
                ));
        }
        

        configureBindings()

        fun applyBreak(breaky: Boolean, speed: Double): Double {
        //     if (breaky) {
        //         return speed * 0.8
        //     }

            return speed
        }

        fun applyTeam(speed: Double):Double {
                if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Blue)) {
                        return -speed;
                } else {
                        return speed;
                }
        }

        fun upMax(speed: Double):Double {
                val sign = speed.sign;
                return sign * Math.min(Math.abs(speed) - 0.1,1.0);
        }

        val leftY =
                if (!onTest.config) {
                    {
                        MathUtil.applyDeadband(
                                upMax(applyTeam(applyBreak(driverLeftStick.trigger, driverLeftStick.getY()))),
                                Constants.OperatorConstants.LEFT_Y_DEADBAND
                        )
                    }
                } else {
                    {
                        MathUtil.applyDeadband(
                                applyTeam(operatorExtra.getLeftY()),
                                Constants.OperatorConstants.LEFT_Y_DEADBAND
                        )
                    }
                }

        val leftX: () -> Double =
                if (!onTest.config) {
                    {
                        MathUtil.applyDeadband(
                                upMax(applyTeam(applyBreak(driverLeftStick.trigger, driverLeftStick.getX()))),
                                Constants.OperatorConstants.LEFT_X_DEADBAND
                        )
                    }
                } else {
                    {
                        MathUtil.applyDeadband(
                                applyTeam(operatorExtra.getLeftX()),
                                Constants.OperatorConstants.LEFT_X_DEADBAND
                        )
                    }
                };

        val omega = {
                        MathUtil.applyDeadband( 
                                applyBreak(driverLeftStick.trigger,driverRightStick.getX()),
                                Constants.OperatorConstants.LEFT_X_DEADBAND
                        )
                }

                val driveMode = { true }

        val faceSpeaker = { false }

                val simClosedFieldRel =
                                TeleopDrive(
                                                swerveDrive,
                                                leftY,
                                                leftX,
                                                omega,
                                                driveMode,
                                                faceSpeaker
                                )

        swerveDrive.defaultCommand = simClosedFieldRel

        autoChooser = AutoBuilder.buildAutoChooser();

                // Another option that allows you to specify the default auto by its name
                // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
            
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
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
                // Trigger { exampleSubsystem.exampleCondition()
                // }.onTrue(ExampleCommand(exampleSubsystem))

        // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
        // cancelling on release.
        // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand())
        val mattSaysNoFancy = true
        if (shooter != null && intake != null && elevator != null) {
            SmartDashboard.putBoolean("shooter", true)

            if (!mattSaysNoFancy) {
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
                                                        
                                                        shooter!!,
                                                        swerveDrive,
                                                        false
                                                )
                                        )
                                        .finallyDo({ _ -> faceSpeaker = false })
                        )
                JoystickButton(operatorExtra.hid, 6)
                        .onTrue(GotoPose(shooter!!, elevator!!, Constants.Poses.highPickup, true))
                
                        
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
JoystickButton(driverRightStick, 3)
                        .onTrue(
                                FollowTrajectory(
                                        swerveDrive,
                                        PathPlannerPath.fromPathFile("to amp"),
                                        true
                                ))
            operatorExtra.rightBumper().toggleOnTrue(Pickup(shooter!!, elevator!!, intake!!, true, false).build());
            operatorExtra.leftBumper().whileTrue(Shoot(shooter!!,elevator!!,false).build());

            operatorExtra.rightTrigger(0.5).whileTrue(SpeakerShoot(elevator!!, shooter!!).build());

            JoystickButton(driverRightStick, 2)
                    .whileTrue(ParallelCommandGroup(
                        Pickup(shooter!!, elevator!!, intake!!, false, false).build(),
                        SequentialCommandGroup(
                                        WaitUntilCommand { ->
                                                frontNoteCamera.latestResult.hasTargets() ||
                                                                backNoteCamera.latestResult.hasTargets()
                                        },
                                        CollectNote(
                                                        PIDConstants(0.045, 0.0, 0.001000),
                                                        frontNoteCamera,
                                                       
                                                        swerveDrive,
                                                        10,
                                            {->shooter!!.noteIn()},
                                            true, intake!!
                                        ),

                        )
                ))
                JoystickButton(driverRightStick, 1)
                    .whileTrue(
                Pickup(shooter!!, elevator!!, intake!!, false, true).build()
)
/*Pickup(shooter!!, elevator!!, intake!!, false).build()*/

            // orExtra.leftTrigger().whileTrue(Pickup(shooter!!, elevator!!, intake!!,
            // false).build())

            operatorExtra.leftStick().whileTrue(shooter!!.backButton().alongWith(intake!!.backButton()));

            operatorExtra.y().onTrue(
                InstantCommand(
                        object : Runnable {
                                override fun run() {
                                        shooter!!.startShooting(false);
                                        shooter!!.setAngle(Constants.Poses.speakerShoot.angle)
                                }
                        }
                )
                        // shooter?.stopShooting();
            );
            
            operatorExtra.y()
                .onFalse(
                        InstantCommand(
                                object : Runnable {
                                        override fun run() {
                                                shooter?.stopShooting();
                                        }
                                }
                        )
                                // shooter?.stopShooting();
                )


            operatorExtra.a().onTrue(GotoPose(shooter!!, elevator!!, Pose(0.0, 0.0), false).alongWith(
                InstantCommand(
                        object : Runnable {
                                override fun run() {
                                        shooter?.stopShooting();
                                }
                        }
                )
            ));
            operatorExtra.b().onTrue(GotoPoseParAmp(shooter!!, elevator!!).alongWith(
                Commands.startEnd(
                object : Runnable {
                        override fun run() {
                                shooter!!.startShooting(false);
                        }
                },
                object : Runnable {
                        override fun run() {
                                shooter?.stopShooting();
                        }
                }
                )
            ))

            operatorExtra.rightStick().whileTrue(
                shooter!!.forwardButton()
            )

            JoystickButton(driverLeftStick, 5).toggleOnTrue(Climb(elevator!!,shooter!!).build())
            JoystickButton(driverLeftStick, 6).onTrue(
                Commands.sequence(
                        //GotoPose(shooter!!,elevator!!,Constants.Poses.halfUp,false),
                        GotoPose(shooter!!,elevator!!,Constants.Poses.allUp,false)
                )
           );
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
            JoystickButton(driverLeftStick, 10).whileTrue(ShootTime(shooter!!,intake!!,elevator!!,swerveDrive,aprilCamera).build());
            JoystickButton(driverLeftStick, 12)
                .onTrue(
                        ShootTime(shooter!!, intake!!, elevator!!, swerveDrive, aprilCamera).build()
                )

            operatorExtra.povUp().toggleOnTrue(Lob(shooter!!,elevator!!).build())
            operatorExtra.povDown().toggleOnTrue(Shoot(shooter!!,elevator!!,true).build())

            operatorExtra.povLeft().toggleOnTrue(LocationShoot(elevator!!,shooter!!,Constants.Poses.highShot,true).build());

            JoystickButton(driverLeftStick,1)
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
                                                
                                                shooter!!,
                                                swerveDrive,
                                                false
                                        )
                                )
                                .finallyDo({ _ -> faceSpeaker = false })
                )
        }

        // JoystickButton(driverLeftStick, 1).whileTrue(Brake(swerveDrive))
    }

        fun setMotorBrake(enabled: Boolean) {
                swerveDrive.setMotorBrake(enabled)
        }

    fun teleperiodic() {
        operatorExtra.hid.setRumble(
                GenericHID.RumbleType.kBothRumble,
                (if (shooter?.noteIn() ?: false) {
                    0.5
                } else {
                    0.0
                })
        )
    }
    fun cleanUp() {
        operatorExtra.hid.setRumble(
                GenericHID.RumbleType.kBothRumble,
                0.0
        )
    }

//     fun settingAngle():Command {
//         return WaitCommand(2.0).andThen(InstantCommand(
//                 object : Runnable {
//                     override fun run() {
//                         shooter?.resetAngle()
//                     }
//                 }
//         )),
//     }

        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        val autonomousCommand: Command
                get() { return autoChooser.getSelected();
                        // An example command will be run in autonomous
                //        return SequentialCommandGroup(Shoot(shooter!!).build(),RunAuto("4 piece Inner"));
//                        return ParallelRaceGroup(SequentialCommandGroup(WaitUntilCommand { ->
//                            frontNoteCamera.latestResult.hasTargets() || backNoteCamera.latestResult.hasTargets()
//                        },
//                            CollectNote(
//                                        PIDConstants(0.045, 0.0, 0.001000),
//                                        frontNoteCamera,
//                                        backNoteCamera,
//                                        swerveDrive,
//                                        10)), Pickup(shooter!!, elevator!!, intake!!, false, false).build())
                }
}