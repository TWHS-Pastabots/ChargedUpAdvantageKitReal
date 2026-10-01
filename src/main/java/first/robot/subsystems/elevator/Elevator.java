package first.robot.subsystems.elevator;
 
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import first.robot.Constants.ElevatorConstants.ElevatorState;

import static first.robot.Constants.ElevatorConstants.ElevatorState;

import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
 
public class Elevator implements Mechanism {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private ElevatorState state = ElevatorState.TRANSITION;
 
    private final Alert leaderDisconnectedAlert =
            new Alert("Eleveator Leader Disconnected","Elevator leader motor is disconnected.", Alert.Level.HIGH);
    private final Alert followerDisconnectedAlert =
            new Alert("Eleveator Leader Disconnected","Elevator follower motor is disconnected.", Alert.Level.HIGH);
 
    public Elevator(ElevatorIO io) {
        this.io = io;
        Scheduler.getDefault().addPeriodic(this::periodic);
    }
 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
 
        io.setPosition(state.angle);
        // Update alerts
        leaderDisconnectedAlert.set(!inputs.elevatorLeaderConnected);
        followerDisconnectedAlert.set(!inputs.elevatorFollowerConnected);
    }
 
    /** Commands the elevator to the given position using onboard closed-loop control. */
    public void setPosition(Rotation2d position) {
        io.setPosition(position);
    }
 
    /** Runs the elevator at the specified open-loop voltage. */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
 
    /** Stops the elevator. */
    public void stop() {
        io.stop();
    }

    public double getLeaderTemperature(){
      return inputs.elevatorLeaderTempCelsius;
    }

    public double getFollowerTemperature(){
      return inputs.elevatorFollowerTempCelsius;
    }
 
    /** Returns the current measured position of the elevator. */
    public Rotation2d getPosition() {
        return inputs.elevatorPosition;
    }
 
    /** Returns true if the elevator is within the given tolerance of the target position. */
    public boolean atSetpoint(Rotation2d target, Rotation2d tolerance) {
        return Math.abs(inputs.elevatorPosition.minus(target).getRadians()) < tolerance.getRadians();
    }

public void transitionState()
  {
    state = ElevatorState.TRANSITION;
  }
  public void scoreConeState()
  {
    state = ElevatorState.CONESCORE;
  }
  public void cubeIntake()
  {
    state = ElevatorState.CUBEINTAKE;
  }
  public void coneIntake()
  {
    state = ElevatorState.CONEINTAKE;
  }
  public void coneScoreState()
  {
    state = ElevatorState.CONESCORE;
  }
  public void cubeScoreState()
  {
    state = ElevatorState.CUBESCORE;
  }
}
