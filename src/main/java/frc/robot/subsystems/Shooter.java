package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.firecontrol.ProjectileSimulator;
import frc.firecontrol.ProjectileSimulator.GeneratedLUT;
import frc.firecontrol.ProjectileSimulator.LUTEntry;
import frc.firecontrol.ProjectileSimulator.SimParameters;
import frc.firecontrol.ShotCalculator;

public class Shooter extends SubsystemBase {

  TalonFX m_leftShooter = new TalonFX(31,"rio");
  TalonFX m_rightShooter = new TalonFX(32,"rio");
  TalonFX m_feeder = new TalonFX(33,"rio");
  TalonFX m_conveyor = new TalonFX(34,"rio");

  double shooterSpeed;
  double feederSpeed;
  double conveyorSpeed;

  CommandSwerveDrivetrain swerve;
  SwerveDriveState swerveState;


  public Shooter(CommandSwerveDrivetrain inputDrivetrain) {
    SmartDashboard.putNumber("shooterSpeed", 0);
    SmartDashboard.putNumber("feederSpeed", 0);
    SmartDashboard.putNumber("conveyorSpeed", 0);
    
    this.swerve = inputDrivetrain;
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

  public Command autoShooter(){
    return run(
      () -> {
        m_leftShooter.set(shooterSpeed);
        m_rightShooter.set(-shooterSpeed);
        m_conveyor.set(conveyorSpeed);
      }
    );
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



  // Shot calculator stuff
  ShotCalculator shotCalc;
  public void initShotCalculator(){
    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = ShooterConstants.launcherOffsetX;  // meters forward of robot center
    config.launcherOffsetY = ShooterConstants.launcherOffsetY;   // meters left of center

    this.shotCalc = new ShotCalculator(config);

    // Calculate the physics parameters for the auto shooting
    SimParameters params = new SimParameters(
        0.215,   // ball mass kg
        0.1501,  // ball diameter m
        0.47,    // drag coeff (smooth sphere)
        0.2,     // Magnus coeff
        1.225,   // air density kg/m^3
        0.43,    // exit height from floor, measure from CAD
        0.1016,  // wheel diameter, measure with calipers
        1.83,    // target height, from game manual
        0.6,     // slip factor (0=no grip, 1=perfect), tune on robot
        45.0,    // launch angle degrees from horizontal
        0.001,   // sim timestep
        1500, 6000, 25, 5.0  // RPM range, search iters, max sim time
    );
    ProjectileSimulator sim = new ProjectileSimulator(params);
    GeneratedLUT lut = sim.generateLUT();
    for (LUTEntry entry : lut.entries()) {
        if (entry.reachable()) {
            System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
                entry.distanceM(), entry.rpm(), entry.tof());
        }else{
            System.out.println("Entry not reachable");
        }
    }

    // load your shooter LUT (from ProjectileSimulator or hand-tuned)
    for (var entry : lut.entries()) {
        if (entry.reachable()) {
            shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
        }else{
            System.out.println("Entry not reachable");
        }
    }
  }
}