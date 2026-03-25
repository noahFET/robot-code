package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    PneumaticHub pHub = new PneumaticHub(61);
    Solenoid solenoid = pHub.makeSolenoid(0);

    public Command setSolenoid(boolean set){
        return run(
            ()->{
                solenoid.set(set);
            }
        );
    }

    public boolean status(){
        return solenoid.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("solenoid 0", status());
    }
}
