package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  TalonFX m_leftShooter = new TalonFX(31,"rio");
  TalonFX m_rightShooter = new TalonFX(32,"rio");
  TalonFX m_feeder = new TalonFX(33,"rio");
  TalonFX m_conveyor = new TalonFX(34,"rio");

  double shooterSpeed;
  double feederSpeed;
  double conveyorSpeed;


  public Shooter() {
    SmartDashboard.putNumber("shooterSpeed", 0);
    SmartDashboard.putNumber("feederSpeed", 0);
    SmartDashboard.putNumber("conveyorSpeed", 0);
  }


  public Command feedCommand(boolean spin) {
    
    return run(
        () -> {
          if(spin){
            m_feeder.set(feederSpeed);
          }else{
            m_feeder.set(0);
          }
        });
  }

  public Command conveyorCommand(boolean spin) {
    
    return run(
        () -> {
          if(spin){
            m_conveyor.set(conveyorSpeed);
          }else{
            m_conveyor.set(0);
          }
        });
  }

  public Command shooterCommand(boolean spin) {
    
    return run(
        () -> {
          if(spin){
            m_leftShooter.set(shooterSpeed);
            m_rightShooter.set(-shooterSpeed);
          }else{
            m_leftShooter.set(0);
            m_rightShooter.set(0);
          }
        });
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterSpeed = SmartDashboard.getNumber("shooterSpeed", 0);
    feederSpeed = SmartDashboard.getNumber("feederSpeed", 0);
    conveyorSpeed = SmartDashboard.getNumber("conveyorSpeed", 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}