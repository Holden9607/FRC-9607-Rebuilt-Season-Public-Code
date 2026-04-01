// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.cscore.HttpCamera;

public class RobotContainer {

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private final double kP_Aim = 0.04; 

    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
    private final GenericEntry shooterPowerEntry = tuningTab.add("Shooter Power", 1.0).getEntry();
    private final NetworkTable dashTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    private final Field2d m_field = new Field2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();    
    private final SendableChooser<Command> autoChooser;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    //Slow Down and Smooth Out acceleration from 0 to 100% Speed
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(6.8);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(6.8);

    private final SparkMax m_IntakeMover = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax m_Rollers = new SparkMax(3, MotorType.kBrushless);
    //private final SparkMax m_Climber = new SparkMax(4, MotorType.kBrushless);
    private final TalonFX m_Kicker = new TalonFX(9, "DINO");
    private final TalonFX m_Shooters = new TalonFX(10, "DINO");
    private final TalonFX m_IntakeActivate = new TalonFX(11, "DINO");

    public RobotContainer() {
        configureNamedCommands();
        configureBindings();

        SmartDashboard.putData("Field", m_field);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        new Trigger(DriverStation::isTeleopEnabled).onTrue(Commands.runOnce(() -> 
        m_IntakeActivate.setControl(new DutyCycleOut(0.0)))
        .alongWith(Commands.runOnce(() -> m_Rollers.set(0.0))
        .alongWith(Commands.runOnce(() -> m_Kicker.setControl(new DutyCycleOut(0.0)))
        .alongWith(Commands.runOnce(() -> m_Shooters.setControl(new DutyCycleOut(0.0)
        ))))));
    }

    public void updateDashboard() {
        String logMessage = "SYSTEM READY"; 
        double voltage = edu.wpi.first.wpilibj.RobotController.getBatteryVoltage();
        boolean isBrownedOut = edu.wpi.first.wpilibj.RobotController.isBrownedOut();

        double time = edu.wpi.first.wpilibj.DriverStation.getMatchTime();
        String stage = "PRE-MATCH";
        double shiftSeconds = 0;

        // AUTO (20 Seconds)
        if (edu.wpi.first.wpilibj.DriverStation.isAutonomousEnabled()) {
            stage = "AUTO";
            shiftSeconds = time; 
            logMessage = "AUTO - EXECUTING PATH";
        } 
        // TELEOP (140 Seconds Total)
        else if (edu.wpi.first.wpilibj.DriverStation.isTeleopEnabled()) {
            logMessage = "TELEOP - OPERATOR CONTROL";
            if (time > 130) {
                stage = "TRANSITION SHIFT"; // 140 to 130 (10s)
                shiftSeconds = time - 130; 
            } else if (time > 105) {
                stage = "SHIFT 1"; // 130 to 105 (25s)
                shiftSeconds = time - 105; 
            } else if (time > 80) {
                stage = "SHIFT 2"; // 105 to 80 (25s)
                shiftSeconds = time - 80;  
            } else if (time > 55) {
                stage = "SHIFT 3"; // 80 to 55 (25s)
                shiftSeconds = time - 55;  
            } else if (time > 30) {
                stage = "SHIFT 4"; // 55 to 30 (25s)
                shiftSeconds = time - 30;  
            } else {
                stage = "END GAME"; // 0:30 - 0:00 (30s)
                shiftSeconds = time;       
            }
        } else if (edu.wpi.first.wpilibj.DriverStation.isDisabled()) {
            logMessage = "IDLE - STANDBY";
            stage = "DISABLED";
        }

        if (voltage <= 6.8 || isBrownedOut) {
            logMessage = "BROWNOUT DETECTED: " + String.format("%.1f", voltage) + "V";
        }

        dashTable.getEntry("Match Stage").setString(stage);
        dashTable.getEntry("Shift Countdown").setDouble(Math.ceil(shiftSeconds));
        dashTable.getEntry("Match Time").setDouble(Math.ceil(time));

        m_field.setRobotPose(drivetrain.getState().Pose);

        dashTable.getEntry("Logs").setString(logMessage);
        dashTable.getEntry("Voltage").setDouble(voltage);
        dashTable.getEntry("Gyro").setDouble(drivetrain.getPigeon2().getYaw().getValueAsDouble());
        dashTable.getEntry("Shooter Power %").setDouble(shooterPowerEntry.getDouble(1.0)*100);
        
        double percent = shooterPowerEntry.getDouble(1.0)*100;
        dashTable.getEntry("Shooter Power Display").setString(String.format("%.0f%%", percent));

        boolean isReady = m_Shooters.getVelocity().getValueAsDouble() > 40.0;
        dashTable.getEntry("Shooter Ready").setBoolean(isReady);

        double rawYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double circleheading = (rawYaw % 360 + 360) % 360;

        dashTable.getEntry("Gyro Heading").setDouble(circleheading);

        double canUtilization = edu.wpi.first.wpilibj.RobotController.getCANStatus().percentBusUtilization;
        dashTable.getEntry("Can Utilization").setDouble(canUtilization * 100);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("Intake1", 
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    m_IntakeActivate.setControl(new DutyCycleOut(0.45));
                    m_Rollers.set(1.0);
                }).withTimeout(4.0),
                new InstantCommand(() -> {
                    m_IntakeActivate.setControl(new DutyCycleOut(0.0));
                    m_Rollers.set(0.0); 
                })
            )
        ); 

        NamedCommands.registerCommand("Intake2", 
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    m_IntakeActivate.setControl(new DutyCycleOut(0.65));
                    m_Rollers.set(1.0);
                }).withTimeout(4.0),
                new InstantCommand(() -> {
                    m_IntakeActivate.setControl(new DutyCycleOut(0.0));
                    m_Rollers.set(0.0); 
                })
            )
        );

        NamedCommands.registerCommand("ConstantIntake", 
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    m_IntakeActivate.setControl(new DutyCycleOut(0.65));
                    m_Rollers.set(1.0);
                }).withTimeout(6.0),
                new InstantCommand(() -> {
                    m_IntakeActivate.setControl(new DutyCycleOut(0.0));
                    m_Rollers.set(0.0); 
                })
            )
        );

        NamedCommands.registerCommand("ShortTimeShoot", 
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    m_Rollers.set(1.0);
                    m_Shooters.setControl(new DutyCycleOut(0.7));
                    m_Kicker.setControl(new DutyCycleOut(0.7));
                }).withTimeout(3.0),
                new InstantCommand(() -> {
                    m_Rollers.set(0.0); 
                    m_Shooters.setControl(new DutyCycleOut(0.0));
                    m_Kicker.setControl(new DutyCycleOut(0.0));
                })
            )
        );

        NamedCommands.registerCommand("Unload", 
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    m_Rollers.set(1.0);
                    m_Shooters.setControl(new DutyCycleOut(0.7));
                    m_Kicker.setControl(new DutyCycleOut(0.7));
                }).withTimeout(5.0),
                new InstantCommand(() -> {
                    m_Rollers.set(0.0); 
                    m_Shooters.setControl(new DutyCycleOut(0.0));
                    m_Kicker.setControl(new DutyCycleOut(0.0));
                })
            )
        );

        NamedCommands.registerCommand("LowerIntake", 
            new SequentialCommandGroup(
                new RunCommand(() -> m_IntakeMover.set(1.0)).withTimeout(1.0),
                new InstantCommand(() -> m_IntakeMover.set(0.0))
            )
        );

        NamedCommands.registerCommand("Shuttle", 
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    m_Rollers.set(1.0);
                    m_Shooters.setControl(new DutyCycleOut(1.0));
                    m_Kicker.setControl(new DutyCycleOut(1.0));
                }).withTimeout(5.0),
                new InstantCommand(() -> {
                    m_Rollers.set(0.0); 
                    m_Shooters.setControl(new DutyCycleOut(0.0));
                    m_Kicker.setControl(new DutyCycleOut(0.0));
                })
            )
        );

        NamedCommands.registerCommand("Shooters", 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunCommand(() -> {
                        m_Shooters.setControl(new DutyCycleOut(1.0));
                        m_Kicker.setControl(new DutyCycleOut(1.0));
                        m_Rollers.set(1.0);
                    }),
                    drivetrain.applyRequest(() -> brake)
                ).withTimeout(5.0),
                new InstantCommand(() -> {
                    m_Shooters.setControl(new DutyCycleOut(0));
                    m_Kicker.setControl(new DutyCycleOut(0));
                    m_Rollers.set(0.0);
                })
            )
        );

         NamedCommands.registerCommand("PrepareShooters", 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunCommand(() -> {
                        m_Shooters.setControl(new DutyCycleOut(1.0));
                        m_Kicker.setControl(new DutyCycleOut(1.0));
                    }),
                    drivetrain.applyRequest(() -> brake)
                ).withTimeout(6.0),
                new InstantCommand(() -> {
                    m_Shooters.setControl(new DutyCycleOut(0));
                    m_Kicker.setControl(new DutyCycleOut(0));
                })
            )
        );

        NamedCommands.registerCommand("PrepareShooters2", 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunCommand(() -> {
                        m_Shooters.setControl(new DutyCycleOut(1.0));
                        m_Kicker.setControl(new DutyCycleOut(1.0));
                    }),
                    drivetrain.applyRequest(() -> brake)
                ).withTimeout(14.0),
                new InstantCommand(() -> {
                    m_Shooters.setControl(new DutyCycleOut(0));
                    m_Kicker.setControl(new DutyCycleOut(0));
                })
            )
        );


        NamedCommands.registerCommand("PrepareShootersWithoutBrake", 
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    m_Shooters.setControl(new DutyCycleOut(1.0));
                    m_Kicker.setControl(new DutyCycleOut(1.0));
                }).withTimeout(7.0),
                new InstantCommand(() -> {
                    m_Shooters.setControl(new DutyCycleOut(0.0));
                    m_Kicker.setControl(new DutyCycleOut(0.0));
                })
            )
        );


        NamedCommands.registerCommand("RollersActivate",
            new SequentialCommandGroup(
                new RunCommand(() -> m_Rollers.set(1.0))
                .withTimeout(5.0),
                new InstantCommand(() -> m_Rollers.set(0.0))
            )
        );

        NamedCommands.registerCommand("RollersRun", 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunCommand(() -> {  
                        m_Rollers.set(0.8);
                    })
                ).withTimeout(6.0),
                new InstantCommand(() -> {
                    m_Rollers.set(0.0);
                })
            )
        );

        NamedCommands.registerCommand("RunRollersSlower", 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunCommand(() -> {  
                        m_Rollers.set(0.8);
                    })
                ).withTimeout(9.0),
                new InstantCommand(() -> {
                    m_Rollers.set(0.0);
                })
            )
        );

        /*NamedCommands.registerCommand("Climb", 
            new SequentialCommandGroup(
                new RunCommand(() -> m_Climber.set(1.0)).withTimeout(4.0),
                new InstantCommand(() -> m_Climber.set(0.0))
            )
        ); */
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->  drive
                    .withVelocityX(MetersPerSecond.of(m_xLimiter.calculate(-joystick.getLeftY() * MaxSpeed)))
                    .withVelocityY(MetersPerSecond.of(m_yLimiter.calculate(-joystick.getLeftX() * MaxSpeed)))
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // --- DRIVER BINDINGS ---
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.x().whileTrue(
            drivetrain.applyRequest(() -> {
                var table = NetworkTableInstance.getDefault().getTable("limelight-rjra");
                double tv = table.getEntry("tv").getDouble(0);
                double tx = table.getEntry("tx").getDouble(0);
                if (tv < 1.0) {
                    return drive.withVelocityX(MetersPerSecond.of(m_xLimiter.calculate(-joystick.getLeftY() * MaxSpeed)))
                                .withVelocityY(MetersPerSecond.of(m_yLimiter.calculate(-joystick.getLeftX() * MaxSpeed)))
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
                }
                double rotationPower = -tx * kP_Aim;
                return drive.withVelocityX(MetersPerSecond.of(m_xLimiter.calculate(-joystick.getLeftY() * MaxSpeed * 0.8)))
                            .withVelocityY(MetersPerSecond.of(m_yLimiter.calculate(-joystick.getLeftX() * MaxSpeed * 0.8)))
                            .withRotationalRate(rotationPower * MaxAngularRate);
            })
        );

        /* joystick.leftBumper().whileTrue(new RunCommand(() -> m_Climber.set(1.0)))
            .onFalse(new RunCommand(() -> m_Climber.set(0.0)));
        joystick.leftTrigger().whileTrue(new RunCommand(() -> m_Climber.set(-1.0)))
            .onFalse(new RunCommand(() -> m_Climber.set(0.0))); */

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // --- OPERATOR BINDINGS ---
        operatorJoystick.povUp().onTrue(new InstantCommand(() -> {
            double current = shooterPowerEntry.getDouble(1.0);
            shooterPowerEntry.setDouble(Math.min(current + 0.1, 1.0));
        }));
        operatorJoystick.povDown().onTrue(new InstantCommand(() -> {
            double current = shooterPowerEntry.getDouble(1.0);
            shooterPowerEntry.setDouble(Math.max(current - 0.1, 0.0));
        }));

        operatorJoystick.x().whileTrue(new RunCommand(() -> {
            m_IntakeActivate.setControl(new DutyCycleOut(0.70));
            m_Rollers.set(1.0);
        })).onFalse(new InstantCommand(() -> {
            m_IntakeActivate.setControl(new DutyCycleOut(0.0));
            m_Rollers.set(0.0);
        }));

        operatorJoystick.a().whileTrue(new RunCommand(() -> {
            double power = shooterPowerEntry.getDouble(1.0);
            m_Shooters.setControl(new DutyCycleOut(power));
            m_Kicker.setControl(new DutyCycleOut(power));
            m_Rollers.set(1.0);
        })).onFalse(new InstantCommand(() -> {
            m_Shooters.setControl(new DutyCycleOut(0.0));
            m_Kicker.setControl(new DutyCycleOut(0.0));
            m_Rollers.set(0.0);
        }));

        operatorJoystick.b().whileTrue(new RunCommand(() -> {
            m_Shooters.setControl(new DutyCycleOut(-1.0));
            m_Kicker.setControl(new DutyCycleOut(-1.0));    
            m_Rollers.set(-1.0);
        })).onFalse(new InstantCommand(() -> {
            m_Kicker.setControl(new DutyCycleOut(0.0));   
            m_Shooters.setControl(new DutyCycleOut(0.0));
            m_Rollers.set(0.0);
        }));

        operatorJoystick.rightTrigger().whileTrue(new RunCommand(() -> {
           m_IntakeActivate.setControl(new DutyCycleOut(-1.0));
            m_Rollers.set(-1.0);
        })).onFalse(new InstantCommand(() -> {
           m_IntakeActivate.setControl(new DutyCycleOut(0.0));
            m_Rollers.set(0.0);
        }));

        operatorJoystick.leftTrigger().whileTrue(new RunCommand(() -> m_IntakeMover.set(0.75)))
            .onFalse(new InstantCommand(() -> m_IntakeMover.set(0.0)));
        operatorJoystick.leftBumper().whileTrue(new RunCommand(() -> m_IntakeMover.set(-0.75)))
            .onFalse(new InstantCommand(() -> m_IntakeMover.set(0.0)));

        operatorJoystick.rightBumper().whileTrue(new RunCommand(() -> {
             double power = shooterPowerEntry.getDouble(1.0);
             m_Shooters.setControl(new DutyCycleOut(power));
             m_Kicker.setControl(new DutyCycleOut(power));
        })).onFalse(new InstantCommand(() -> {
             m_Kicker.setControl(new DutyCycleOut(0.0));   
             m_Shooters.setControl(new DutyCycleOut(0.0));
        }));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Runs dashboard updates constantly in the background
        new RunCommand(this::updateDashboard).ignoringDisable(true).schedule();
    }
  
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}