package frc.robot.questOdometry;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class QuestServer {
    DatagramSocket socket;
    InetSocketAddress questAddr;
    //byte[] recBuffer;
    ByteBuffer recBB;
    //DatagramPacket recPacket;
    //DatagramPacket syncRecPacket;
    //DatagramPacket sendPacket;
    public double clkOffset;
    public boolean recieveData = true;

    double questTime;
    double cameraLag;
    int tagCount;
    Pose2d slamPose;
    Boolean cvData;
    Pose2d cvPose;
    double reprojError;
    double transLag;
    double[] slamPoseArr = new double[3];
    double[] cvPoseArr = new double[3];


    CommandSwerveDrivetrain csd;
    Pose2d slamOffset;


    DatagramSocket sendSocket;

    TimeInterpolatableBuffer<Pose2d> slamHistory = TimeInterpolatableBuffer.createBuffer(1.5);


    //double[] rotatedSlamXY = new double[2];

    Pose2d rotatedSlamPose;

    Pose2d fusedPose;

    public QuestServer(CommandSwerveDrivetrain commandSwerveDrivetrain){
        this.csd = commandSwerveDrivetrain;
        try{
            socket = new DatagramSocket(QuestConstants.recievePort);
            socket.setSoTimeout(500);
            sendSocket = new DatagramSocket(QuestConstants.syncPort);
            sendSocket.setSoTimeout(500);

            questAddr = new InetSocketAddress(InetAddress.getByAddress(QuestConstants.questIP), QuestConstants.sendPort);
            System.out.println("Started UDP server on port " + socket.getLocalPort());
            System.out.println("Attempting to connect to Quest at " + questAddr.getHostName() + ":" + questAddr.getPort());

            // New thread for UDP communication so it can run at maximum speed and not interrupt critical robot functions in case of a malfunction
            Thread t1 = new Thread(new Runnable() {
                public void run()
                {
                    // Clock sync
                    //System.out.println("attempting to sync clock");
                    //clockSync();
                    //System.out.println("clock sync success");

                    // Recieve data from Quest
                    byte[] recBuffer = new byte[53];
                    DatagramPacket recPacket = new DatagramPacket(recBuffer, 53);
                    recBB = ByteBuffer.wrap(recBuffer).order(ByteOrder.LITTLE_ENDIAN);
                    recieveData = true;
                    //slamOffset = Pose2d.kZero;
                    recieveData(recPacket);

                }
            });  
            t1.start();

            Thread t2 = new Thread(new Runnable() {
                public void run()
                {
                    // Clock sync
                    clockSync();
                }
            });  
            t2.start();

        }catch(SocketException e){
            //e.printStackTrace();
            System.out.println("oopsies no server :3");
        }catch(UnknownHostException e){
            System.out.println("oopsies ip address failure :3");
        }
    }


    double[] goodCVPoseArr = new double[3];
    Rotation2d rotBy;

    void recieveData(DatagramPacket recPacket){
        boolean recSuccess;
        Optional<Pose2d> tempHistoryPoseOptional;
        Pose2d tempHistoryPose;
        while(recieveData){
            try{
                socket.receive(recPacket);
                recSuccess = true;
            }catch(IOException e){
                recSuccess = false;
            }

            if(recSuccess){
                // First 8 bytes: Quest time, next 8 bytes: camera latency, next 12 bytes: Quest SLAM pose,
                // next byte: cvData, next 4 bytes: tag count, next 12 bytes: CV camera pose, next 8 bytes: mean reprojection error (65 bytes total)
                // all measurements in {x,z,yaw}
                questTime = recBB.getDouble();
                transLag = Timer.getFPGATimestamp()-questTime;
                cameraLag = recBB.getDouble();

                // This is converting to wpilib coordinates
                slamPoseArr[1] = -(double)recBB.getFloat();
                slamPoseArr[0] = (double)recBB.getFloat();
                slamPoseArr[2] = -Math.toRadians((double)recBB.getFloat());

                slamPose = new Pose2d(slamPoseArr[0],slamPoseArr[1],new Rotation2d(slamPoseArr[2]));

                //slamPose = new Pose2d((double)recBB.getFloat(),(double)recBB.getFloat(),new Rotation2d((double)Math.toRadians(recBB.getFloat())));

                // slam pose history interpolation
                slamHistory.addSample(questTime, slamPose);

                cvData = recBB.get()!=0;
                if(cvData){
                    tagCount = recBB.getInt();

                    // This is converting to wpilib coordinates
                    cvPoseArr[1] = -(double)recBB.getFloat();
                    cvPoseArr[0] = (double)recBB.getFloat();
                    cvPoseArr[2] = -Math.toRadians((double)recBB.getFloat());

                    //cvPose = new Pose2d((double)recBB.getFloat(),(double)recBB.getFloat(),new Rotation2d((double)Math.toRadians(recBB.getFloat())));


                    reprojError = recBB.getDouble();

                    if(tagCount>=QuestConstants.minTags && reprojError<=QuestConstants.maxReprojError){
                        tempHistoryPoseOptional = slamHistory.getSample(questTime-cameraLag);
                        if(tempHistoryPoseOptional.isPresent()){
                            tempHistoryPose = tempHistoryPoseOptional.get();
                            
                            slamOffset = new Pose2d(tempHistoryPose.getX(),tempHistoryPose.getY(),new Rotation2d(tempHistoryPose.getRotation().getRadians()));
                            rotBy = new Rotation2d(cvPoseArr[2]-tempHistoryPose.getRotation().getRadians());
                            goodCVPoseArr = cvPoseArr;
                        }
                    }

                }
            }

            if(slamPose!=null&&slamOffset!=null){
                //rotatedSlamPose = new Pose2d(slamPoseArr[0]-slamOffset.getX(),slamPoseArr[1]-slamOffset.getX(),new Rotation2d(slamPoseArr[2]-slamOffset.getRotation().getRadians())).rotateBy(new Rotation2d((slamPoseArr[2]-slamOffset.getRotation().getRadians())+goodCVPoseArr[2]));
                rotatedSlamPose = new Pose2d(slamPoseArr[0]-slamOffset.getX(),slamPoseArr[1]-slamOffset.getY(),new Rotation2d()).rotateBy(rotBy);
                fusedPose = new Pose2d(rotatedSlamPose.getX()+goodCVPoseArr[0], rotatedSlamPose.getY()+goodCVPoseArr[1], new Rotation2d((slamPoseArr[2]-slamOffset.getRotation().getRadians())+goodCVPoseArr[2]));
                csd.addVisionMeasurement(fusedPose, questTime);
            }
            //fusedPose = new Pose2d(slamPose.getX(), slamPose.getY(), new Rotation2d(slamPose.getRotation().getRadians()));
            recBB.position(0);
        }
    }

    void clockSync(){
        // Clock sync
        boolean isFinished = false;
        byte[] socketRecBuffer = new byte[9];
        byte[] sendBuffer = new byte[9];
        DatagramPacket sendPacket = new DatagramPacket(sendBuffer,9,questAddr);
        DatagramPacket syncRecPacket = new DatagramPacket(socketRecBuffer,9);
        double startTime = 0;
        double responseTime = 0;
        double secondResponseTime;
        int state = 0;
        ByteBuffer sendingBB = ByteBuffer.wrap(sendBuffer).order(ByteOrder.LITTLE_ENDIAN);

        while(!isFinished){
            switch (state) {
                case 0:
                    startTime = Timer.getFPGATimestamp();
                    sendBuffer[0] = (byte)0x01;
                    try{
                        sendSocket.send(sendPacket);
                        //System.out.println("sent sync request");
                        state = 1;
                    }catch (IOException e){
                        state = 0;
                    }
                    break;
                case 1:
                    try{
                        sendSocket.receive(syncRecPacket);
                    }catch(IOException e){
                        state = 0;
                        //System.out.println("timeout");
                        break;
                    }
                    if(socketRecBuffer[0] == (byte)0x01){
                        responseTime = Timer.getFPGATimestamp()-startTime;
                        startTime = Timer.getFPGATimestamp();
                        sendBuffer[0] = (byte)0x02;
                        sendingBB.putDouble(1, responseTime);
                        try {
                            sendSocket.send(sendPacket);
                        } catch (IOException e) {
                            state = 0;
                            break;
                        }
                        state = 2;
                    }
                    break;
                case 2:
                    try{
                        sendSocket.receive(syncRecPacket);
                    }catch(IOException e){
                        state = 0;
                        break;
                    }
                    if(socketRecBuffer[0]==(byte)0x02){
                        secondResponseTime = Timer.getFPGATimestamp()-startTime;
                        if(Math.abs(responseTime-secondResponseTime)<=QuestConstants.clockSyncErrorMargin){
                            clkOffset = Timer.getFPGATimestamp()+((responseTime+secondResponseTime)/2)/2;
                            sendBuffer[0] = (byte)0x03;
                            sendingBB.putDouble(1,clkOffset);
                            try {
                                sendSocket.send(sendPacket);
                            } catch (IOException e) {
                                state = 0;
                                break;
                            }
                            state = 0;
                        }else{
                            state = 0;
                        }
                    }else if(socketRecBuffer[0]==(byte)0x04){
                        state = 0;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    
    public Pose2d getSlamPose(){
        return slamPose;
    }

    public Pose2d getCVPose(){
        return new Pose2d(cvPoseArr[0],cvPoseArr[1],new Rotation2d(cvPoseArr[2]));
    }

    public double getQuestTime(){
        return questTime;
    }

    public double getTransLag(){
        return transLag;
    }

    public Pose2d getFusedPose(){
        return fusedPose;
    }

}
