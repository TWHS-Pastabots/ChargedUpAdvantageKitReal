package first.robot;

import first.robot.Subsystem;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.claw.Tilt;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.pivot.Pivot;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;

import first.robot.Constants;

public class RobotContainer {
    private final Subsystem subsystem = Subsystem.getInstance();

    private final Drive m_drivetrain = subsystem.getDrivetrain();
    private final Claw m_intake = subsystem.getIntake();
    private final Tilt m_tilt = subsystem.getTilt();
    private final Elevator m_elevator = subsystem.getElevator();
    private final Pivot m_pivot = subsystem.getPivot();

    private static final CommandXboxController m_cont0 = new CommandXboxController(0);
    private final CommandXboxController m_opertateController = new CommandXboxController(0);

     private final static double defaultSpeed = 0.8;
	private final static double slowSpeed = 0.2;
	private static double speedMod = defaultSpeed;

    
    public static Trigger slow = m_cont0.leftTrigger(0.3);

    public static Trigger cone = m_cont0.x();
    public static Trigger cube = cone.negate();

    public static Trigger front = m_cont0.rightTrigger(0.2);

    public static Trigger intaking = m_cont0.rightBumper();

    public static Trigger cubeIntakeFront = cube.and(front).and(intaking).and(slow);
    public static Trigger cubeIntakeBack = cube.and(front).negate().and(intaking).and(slow);

    public static Trigger coneIntakeFront = cone.and(front).and(intaking).and(slow);
    public static Trigger coneIntakeBack = cone.and(front).negate().and(intaking).and(slow);

    public static Trigger coneScoreState = cone.and(intaking).negate().and(slow);
    public static Trigger cubeScoreState = cube.and(intaking).negate().and(slow);

    public static Trigger aligning = m_cont0.leftStick();     

    public static Trigger scoreCone = m_cont0.leftBumper().and(cone).and(slow);
    public static Trigger scoreCube = m_cont0.leftBumper().and(cube).and(slow);

    public static Trigger intakingCone = intaking.and(cone);
    public static Trigger intakingCube = intaking.and(cube);

    public RobotContainer(){
        configureButtonBindings();

        // Configure default commands
        m_drivetrain.setDefaultCommand(
            Command.requiring(m_drivetrain).executing(
                coroutine -> {
                    m_drivetrain.drive(m_cont0.getLeftY(), m_cont0.getLeftX(), m_cont0.getRightX(), true);
                }
            ).named("defaultDrive")
        );
        m_tilt.setDefaultCommand(
            Command.requiring(m_tilt).executing(
                coro -> {
                    m_tilt.transitionState();
            }).named("claw default"));
        m_pivot.setDefaultCommand(
            Command.requiring(m_pivot).executing(
                coro -> {
                    m_pivot.transitionState();
            }).named("pivot default"));
        m_elevator.setDefaultCommand(
            Command.requiring(m_elevator).executing(
                coro -> {
                    m_elevator.transitionState();
            }).named("elevator default"));
    }

    private void configureButtonBindings(){
        cubeIntakeFront.onTrue(coneIntakeFront());
    }




//BASE COMMANDS

    private Command cubeScoreState(){
        return Command.requiring(m_pivot, m_elevator, m_tilt)
        .executing(coro -> {
            m_pivot.scoreConeState();
            m_tilt.scoreConeState();
            m_elevator.scoreConeState();
        })
        .named("Score Cone");
    }

    
    private Command coneScoreState(){
        return Command.requiring(m_pivot, m_elevator, m_tilt)
        .executing(coro -> {
            m_pivot.scoreConeState();
            m_tilt.scoreConeState();
            m_elevator.scoreConeState();
        })
        .named("Score Cone");
    }

    private Command coneIntakeBack(){
        return Command.requiring(m_pivot, m_elevator, m_tilt)
        .executing(coro -> {
            m_pivot.coneIntakeBack();
            m_tilt.coneIntakeBack();
            m_elevator.coneIntake();
        })
        .named("Score Cone");
    }

    
    private Command coneIntakeFront(){
        return Command.requiring(m_pivot, m_elevator, m_tilt)
        .executing(coro -> {
            m_pivot.coneIntakeFront();
            m_tilt.coneIntakeFront();
            m_elevator.coneIntake();
        })
        .named("Score Cone");
    }
        
    private Command cubeIntakeFront(){
        return Command.requiring(m_pivot, m_elevator, m_tilt)
        .executing(coro -> {
            m_pivot.cubeIntakeFront();
            m_tilt.cubeIntakeFront();
            m_elevator.cubeIntake();
        })
        .named("Score Cone");
    }
            
    private Command cubeIntakeBack(){
        return Command.requiring(m_pivot, m_elevator, m_tilt)
        .executing(coro -> {
            m_pivot.cubeIntakeBack();
            m_tilt.cubeIntakeBack();
            m_elevator.cubeIntake();
        })
        .named("Score Cone");
    }

            
    private Command transitionState(){
        return Command.requiring(m_pivot, m_elevator, m_tilt)
        .executing(coro -> {
            m_pivot.transitionState();
            m_tilt.transitionState();
            m_elevator.transitionState();
        })
        .named("Score Cone");
    }

    private Command outtake(){
        return Command.requiring(m_intake)
        .executing(coro -> {m_intake.setPower(1);}).named("Outtake");
    }

    private Command inttake(){
        return Command.requiring(m_intake)
        .executing(coro -> {m_intake.setPower(1);}).named("Outtake");
    }
}


