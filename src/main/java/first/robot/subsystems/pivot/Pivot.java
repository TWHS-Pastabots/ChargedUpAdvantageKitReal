package first.robot.subsystems.pivot;
 
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import first.robot.Constants.PivotConstants.PivotState;

import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import static first.robot.Constants.PivotConstants.PivotState;
 
public class Pivot implements Mechanism {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private PivotState state = PivotState.TRANSITION;
 
    private final Alert leaderDisconnectedAlert =
            new Alert("Pivot leader Disconnected","Pivot leader motor is disconnected.", Alert.Level.HIGH);
    private final Alert followerDisconnectedAlert =
            new Alert("Pivot follower Disconnected","Pivot follower motor is disconnected.", Alert.Level.HIGH);
 
    public Pivot(PivotIO io) {
        this.io = io;
        Scheduler.getDefault().addPeriodic(this::periodic);
    }
 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        io.setPosition(state.angle);
 
        // Update alerts
        leaderDisconnectedAlert.set(!inputs.leaderConnected);
        followerDisconnectedAlert.set(!inputs.followerConnected);
    }
 
    /** Commands the pivot to the given angle using onboard closed-loop control. */
    public void setPosition(Rotation2d position) {
        io.setPosition(position);
    }
    
 
    /** Runs the pivot at the specified open-loop voltage. */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
 
    /** Stops the pivot. */
    public void stop() {
        io.stop();
    }
 
    /** Returns the current measured angle of the pivot. */
    public Rotation2d getPosition() {
        return inputs.position;
    }
 
    /** Returns true if the pivot is within the given tolerance of the target angle. */
    public boolean atSetpoint(Rotation2d target, Rotation2d tolerance) {
        return Math.abs(inputs.position.minus(target).getRadians()) < tolerance.getRadians();
    }

    public double getLeaderTemperature(){
      return inputs.leaderTempCelsius;
    }

    public double getFollowerTemperature(){
      return inputs.followerTempCelsius;
    }

    
  public void transitionState()
  {
    state = PivotState.TRANSITION;
  }
  public void scoreConeState()
  {
    state = PivotState.CONESCORE;
  }
  public void cubeIntakeFront()
  {
    state = PivotState.CUBEINTAKEFRONT;
  }
  public void cubeIntakeBack()
  {
    state = PivotState.CUBEINTAKEBACK;
  }
  public void coneIntakeFront()
  {
    state = PivotState.CONEINTAKEFRONT;
  }
  public void coneIntakeBack()
  {
    state = PivotState.CONEINTAKEBACK;
  }
  public void coneScoreState()
  {
    state = PivotState.CONESCORE;
  }
  public void cubeScoreState()
  {
    state = PivotState.CUBESCORE;
  }
}