package frc.robot.questOdometry;

public class QuestConstants {
    public static final byte[] questIP = new byte[]{(byte)10,(byte)45,(byte)1,(byte)200};
    public static final int recievePort = 5805;
    public static final int syncPort = 5806;
    public static final int sendPort = 5806;
    public static final double clockSyncErrorMargin = 0.001;
    public static final double questPosOnRobotX = 0.1969;
    public static final double questPosOnRobotY = -0.3377;
    public static final double questYawOnRobot = Math.toRadians(-90);
    public static final double camPosOnQuestX = 0.0914;
    public static final double camPosOnQuestY = 0.03175;
    public static final double maxReprojError = 5;
    public static final int minTags = 3;

}
