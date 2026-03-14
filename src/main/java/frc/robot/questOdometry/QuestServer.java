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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class QuestServer {
    DatagramSocket socket;
    InetSocketAddress questAddr;
    byte[] recBuffer;
    ByteBuffer recBB;
    DatagramPacket recPacket;
    DatagramPacket sendPacket;
    volatile double clkOffset;
    public boolean recieveData = true;

    double questTime;
    double cameraLag;
    int tagCount;
    Pose2d slamPose;
    Boolean cvData;
    Pose2d cvPose;
    double reprojError;

    public QuestServer(){
        try{
            socket = new DatagramSocket(QuestConstants.recievePort);
            socket.setSoTimeout(500);
            questAddr = new InetSocketAddress(InetAddress.getByAddress(QuestConstants.questIP), QuestConstants.sendPort);
            System.out.println("Started UDP server on port " + socket.getLocalPort());
            System.out.println("Attempting to connect to Quest at " + questAddr.getHostName() + ":" + questAddr.getPort());

            // New thread for UDP communication so it can run at maximum speed and not interrupt critical robot functions in case of a malfunction
            Thread t1 = new Thread(new Runnable() {
                public void run()
                {
                    // Clock sync
                    clockSync();

                    // Recieve data from Quest
                    recBuffer = new byte[69];
                    recPacket = new DatagramPacket(recBuffer, 69);
                    recBB = ByteBuffer.wrap(recBuffer).order(ByteOrder.LITTLE_ENDIAN);
                    recieveData = true;
                    recieveData();

                }
            });  
            t1.start();

        }catch(SocketException e){
            //e.printStackTrace();
            System.out.println("oopsies no server :3");
        }catch(UnknownHostException e){
            System.out.println("oopsies ip address failure :3");
        }
    }

    void recieveData(){
        boolean recSuccess;
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
                cameraLag = recBB.getDouble();
                slamPose = new Pose2d((double)recBB.getFloat(),(double)recBB.getFloat(),new Rotation2d((double)Math.toRadians(recBB.getFloat())));
                cvData = recBB.get()!=0;
                if(cvData){
                    tagCount = recBB.getInt();
                    cvPose = new Pose2d((double)recBB.getFloat(),(double)recBB.getFloat(),new Rotation2d((double)Math.toRadians(recBB.getFloat())));
                    reprojError = recBB.getDouble();
                }
            }

            recBB.position(0);
        }
    }

    void clockSync(){
        // Clock sync
        boolean isFinished = false;
        byte[] socketRecBuffer = new byte[9];
        byte[] sendBuffer = new byte[9];
        sendPacket = new DatagramPacket(sendBuffer,9,questAddr);
        recPacket = new DatagramPacket(socketRecBuffer,9);
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
                        socket.send(sendPacket);
                        state = 1;
                    }catch (IOException e){
                        state = 0;
                    }
                    break;
                case 1:
                    try{
                        socket.receive(recPacket);
                    }catch(IOException e){
                        state = 0;
                        break;
                    }
                    if(socketRecBuffer[0] == (byte)0x01){
                        responseTime = Timer.getFPGATimestamp()-startTime;
                        startTime = Timer.getFPGATimestamp();
                        sendBuffer[0] = (byte)0x02;
                        sendingBB.putDouble(1, responseTime);
                        try {
                            socket.send(sendPacket);
                        } catch (IOException e) {
                            state = 0;
                            break;
                        }
                        state = 2;
                    }
                    break;
                case 2:
                    try{
                        socket.receive(recPacket);
                    }catch(IOException e){
                        state = 0;
                        break;
                    }
                    if(socketRecBuffer[0]==(byte)0x02){
                        secondResponseTime = Timer.getFPGATimestamp()-startTime;
                        if(Math.abs(responseTime-secondResponseTime)<=QuestConstants.clockSyncErrorMargin){
                            clkOffset = Timer.getFPGATimestamp()-((responseTime+secondResponseTime)/2)/2;
                            sendBuffer[0] = (byte)0x03;
                            try {
                                socket.send(sendPacket);
                            } catch (IOException e) {
                                state = 0;
                                break;
                            }
                            isFinished = true;
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
        return cvPose;
    }

}
