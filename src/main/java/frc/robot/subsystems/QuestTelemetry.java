package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.questOdometry.QuestServer;

public class QuestTelemetry extends SubsystemBase {
    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable questStateTable = inst.getTable("QuestState");
    private final StructPublisher<Pose2d> slamPose = questStateTable.getStructTopic("SLAMPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> cvPose = questStateTable.getStructTopic("CVPose", Pose2d.struct).publish();

    QuestServer questServer;

    public QuestTelemetry(QuestServer qS){
        this.questServer = qS;
    }

    @Override
    public void periodic(){
        slamPose.set(questServer.getSlamPose());
        cvPose.set(questServer.getCVPose());
    }
}
