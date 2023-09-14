package Connection;


import java.net.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.io.*;



import astar.State;
import astar.Obstacle;
import astar.astarPathFinder;
import astar.AMap;
import astar.AMapCell;
import astar.Node;

/*
 * AlgoServer connects to RPI via the RPI wifi
 */

public class AlgoServerMDP {

    private int SLEEPO = 1000;

    private Socket socket = null;
    private InputStream inStream = null;
    private OutputStream outStream = null;
    private String address;
    private int port;

    Object[] pathInstructions;
    int instructionCount = 0;
    boolean robotStarted = false;
    int imageCaptureStep = 1;
    String OBSTACLE_ID = "";

    int ADJUSTMENT_COUNT = 0;
    boolean IS_ADJUSTING = false;
    int BACKWARD_COUNT = 0;
    boolean AFTER_ADJUST = false;

    public AlgoServerMDP(String address, int port) {
        this.address = address;
        this.port = port;
    }

    public void createSocket() {
        try {
            socket = new Socket(address, port);
            System.out.println("Connected");
            inStream = socket.getInputStream();
            outStream = socket.getOutputStream();
            createReadThread();
            createWriteThread();
        } catch (UnknownHostException u) {
            u.printStackTrace();
        } catch (IOException io) {
            io.printStackTrace();
        }
    }

    public void createReadThread() {
        Thread readThread = new Thread() {
            public void run() {
                while (socket.isConnected()) {

                    try {
                        byte[] readBuffer = new byte[200];
                        int num = inStream.read(readBuffer);

                        if (num > 0) {
                            byte[] arrayBytes = new byte[num];
                            System.arraycopy(readBuffer, 0, arrayBytes, 0, num);
                            String recvedMessage = new String(arrayBytes, "UTF-8");
                            System.out.println("WORKINGH!!!!!! /n");
                            System.out.println(recvedMessage);
                            
                            String message="1,5,0,1/n2,9,6,3/n3,11,8,1/n4,11,13,0/n5,5,18,3/n6,19,1,2/n";
                            System.out.println(message);
                            ArrayList<ArrayList<String>> FinalPath;
                            FinalPath=(ArrayList<ArrayList<String>>) astarPathFinder.executeFN1(recvedMessage);
                            String s;
                            s=arrayListToString(FinalPath);
                            
                            
                            //System.out.println(Arrays.toString(pathInstructions) + "\n\n\n");
                            String msg1 = "AND: Path creation unsuccessful. Check the values sent...";
                           //String internal = "Received wrong format for robot and obstacles from android\n\n";
                            try {
                                outStream.write(s.getBytes("UTF-8"));
                                //System.out.println(internal);
                            } catch (Exception e2) {
                            }
                            return;
                            
//                            System.out.println("From FINAL RPI NOW: " + recvedMessage);
//                            try {
//                                switch (recvedMessage.substring(0, 3)) {
//                                    case "AND":
//                                        androidHandler(recvedMessage);
//                                        break;
//                                    case "STM":
//                                        stmHandler(recvedMessage);
//                                        break;
//                                    case "IMG":
//                                        imgHandler(recvedMessage);
//                                        break;
//                                    default:
//                                }
//                            } catch (Exception e) {
//                            }

                        } /*
                           * else {
                           * // notify();
                           * }
                           */
                        ;
                        // System.arraycopy();
                    } catch (SocketException se) {
                        System.exit(0);

                    } catch (IOException i) {
                        i.printStackTrace();
                    }

                }
            }
        };
        readThread.setPriority(Thread.MAX_PRIORITY);
        readThread.start();
    }
    public String arrayListToString(ArrayList<ArrayList<String>> list) {
        StringBuilder sb = new StringBuilder();
        for (ArrayList<String> innerList : list) {
            for (String str : innerList) {
                sb.append(str);
            }
        }
        return sb.toString();
    }

    public void createWriteThread() {
        Thread writeThread = new Thread() {
            public void run() {
                while (socket.isConnected()) {

                    try {
                        BufferedReader inputReader = new BufferedReader(new InputStreamReader(System.in));
                        sleep(100);
                        //System.out.println("FUNC WOKRING");
                        String typedMessage = inputReader.readLine();
                        if (typedMessage != null && typedMessage.length() > 0) {
                            synchronized (socket) {
                                outStream.write(typedMessage.getBytes("UTF-8"));
                                System.out.println("FUNC WOKRING");
                            }
                        }
                        ;
                        // System.arraycopy();

                    } catch (IOException i) {
                        i.printStackTrace();
                    } catch (InterruptedException ie) {
                        ie.printStackTrace();
                    }

                }
            }
        };
        writeThread.setPriority(Thread.MAX_PRIORITY);
        writeThread.start();
    }

    private void androidHandler(String message) {
        // build arena instruction
        if (message.length() >= 10) {
            // reset exisiting pathInstructions, instruction count
            pathInstructions = new Object[] {};
            instructionCount = 0;
            imageCaptureStep = 1;
            try {
                // see if a path can actually be built
            	System.out.printf("Hello!!--");
            	
            	//(message.replace(",", "\n")).substring(4)
                //pathInstructions = PathPlanner.psuedoDubins((message.replace(",", "\n")).substring(4));
            } catch (Exception e1) {
                // if path cannot be built
                // send an error message to android
                synchronized (socket) {
                    System.out.println(Arrays.toString(pathInstructions) + "\n\n\n");
                    String msg1 = "AND: Path creation unsuccessful. Check the values sent...";
                    String internal = "Received wrong format for robot and obstacles from android\n\n";
                    try {
                        outStream.write(msg1.getBytes("UTF-8"));
                        System.out.println(internal);
                    } catch (Exception e2) {
                    }
                }
                return;
            }
            // if an empty path was created, it is probably due to impossible arrangement of obstacles
            if (pathInstructions.length == 0) {
                synchronized (socket) {
                    String msg1 = "AND: Path creation unsuccessful. The arrangement of obstacles do not allow movement";
                    String internal = "Impossible path\n\n";
                    try {
                        outStream.write(msg1.getBytes("UTF-8"));
                        //Thread.sleep(3000);
                        System.out.println(internal);
                    } catch (Exception e2) {
                    }
                }
 
            }
            // otherwise the path created was successful
            // send ack to android and first instruction to stm
            else {
                synchronized (socket) {
                    System.out.println(Arrays.toString(pathInstructions));
                    String msg1 = "AND: Path creation successful";
                    String toSTM = "STM:" + (String) pathInstructions[instructionCount];
                    instructionCount++; //increment the instruction index
                    String internal = "Received robot and obstacles from android\nSent ack to android\npath plan created\nFirst instruction sent to STM\n\n";
                    try {
                        outStream.write(msg1.getBytes("UTF-8"));
                        System.out.println("Waiting...\n");
                        Thread.sleep(SLEEPO);
                        outStream.write(toSTM.getBytes("UTF-8"));
                        System.out.println(internal);
                        System.out.println(Arrays.deepToString(pathInstructions) + "\n\n");

                    } catch (Exception e2) {
                    }
                }
            }
        }
    }

    private void stmHandler(String message) {

        // this one here is strictly for adjustments after poor image capture
        // take an image -> delay -> move back forward
        /* 
        if (message.matches("STM:&.*") & IS_ADJUSTING) {
            synchronized (socket) {
                String takePicture = "IMG:CAP";
                try {
                    outStream.write(takePicture.getBytes("UTF-8"));
                    Thread.sleep(500);
        
                    //'Thread.sleep(1000);
                    //Thread.sleep(3000);
                } catch (Exception e) {
                }
            }
            return;
        }*/

        // after successful robot movement and ack from stm...
        if (message.matches("STM:&*") & IS_ADJUSTING == false & AFTER_ADJUST == false) {
            // if there are still instructions left, send next instruction
            if (instructionCount < pathInstructions.length) {
                // if next instruction is image capture, ask robot to take picture, and send current coords to android
                if (((String) pathInstructions[instructionCount]).matches("CAP.*")) {

                    synchronized (socket) {
                        String toCamera = "IMG:CAP";
                        String[] splitInstr = ((String) pathInstructions[instructionCount]).split(",");
                        String toAndroid = "AND:ROBOT,"
                                + String.format("%s,%s,%s", splitInstr[2], splitInstr[3], splitInstr[4]);
                        OBSTACLE_ID = splitInstr[1]; // save the current obstacle id
                        imageCaptureStep++;
                        try {

                            outStream.write(toCamera.getBytes("UTF-8"));
                            outStream.write(toAndroid.getBytes("UTF-8"));
                            //'Thread.sleep(1000);
                            //Thread.sleep(3000);
                        } catch (Exception e) {
                        }
                    }

                } else {
                    // if it is not a image capture, then just send nex instruction
                    synchronized (socket) {
                        try {
                            String toSTM = "STM:" + (String) pathInstructions[instructionCount];
                            String internal = String.format("Sent move command %s to robot\n\n",
                                    (String) pathInstructions[instructionCount]);
                            System.out.println(internal);
                            Thread.sleep(SLEEPO);
                            outStream.write(toSTM.getBytes("UTF-8"));
                            //Thread.sleep(5000);

                        } catch (Exception e) {
                        }
                    }
                }
                instructionCount++;
            }
        }
        if (AFTER_ADJUST) {
            AFTER_ADJUST = false;
        }

        if (IS_ADJUSTING) {
            synchronized (socket) {
                try {
                    String toCamera = "IMG:CAP";
                    String internal = "Capture again";
                    System.out.println(internal);
                    Thread.sleep(SLEEPO);
                    outStream.write(toCamera.getBytes("UTF-8"));
                    //Thread.sleep(5000);

                } catch (Exception e) {
                }
            }
            return;
        }

    }

    private void imgHandler(String message) {
        // receive image from camera
        if (message.matches("IMG:CAP:.*")) {

            String imageId = "";
            try {
                // if the image recognition did not get anything, [], escpae the function
                if (message.matches("IMG:CAP:-1.*") || message.matches("IMG:CAP:-2.*")) {
                    System.out.println("No image capture. Performing readjustment...\n\n");
                    /*
                    if (ADJUSTMENT_COUNT < 2) {
                        readjustment();
                        return;
                    } else {
                        AFTER_ADJUST = true;
                        IS_ADJUSTING = false;
                        synchronized (socket) {
                            String toSTM = "\\fmf" + BACKWARD_COUNT * 10 + ";";
                            try {
                                Thread.sleep(SLEEPO);
                                outStream.write(toSTM.getBytes("UTF-8"));
                            } catch (Exception e) {
                            }
                        }
                        ADJUSTMENT_COUNT = 0;
                        BACKWARD_COUNT = 0;
                    
                        //moveBack();
                    } */
                }
                // otherwise...
                imageId = message.substring(8, 10);
                synchronized (socket) {
                    String toAndroid = String.format("AND:TARGET,%s,%s", OBSTACLE_ID, imageId);

                    try {
                        outStream.write(toAndroid.getBytes("UTF-8"));
                    } catch (Exception e) {
                    }
                }

                // String toAndroid = String.format("AND:TARGET,%s,%s", OBSTACLE_ID, imageId);
                // outStream.write(toAndroid.getBytes("UTF-8"));
                System.out.println("Printed imageId " + imageId + "\n\n");

            } catch (Exception e) {
            }

            /*
            synchronized (socket) {
                String toAndroid = String.format("AND:TARGET,%s,%s", OBSTACLE_ID, imageId);
                try {
                    outStream.write(toAndroid.getBytes("UTF-8"));
                    //Thread.sleep(3000);
                } catch (Exception e) {
                }
            } */

            // after which send next instruction to STM
            if (instructionCount < pathInstructions.length) {
                String toSTM = "STM:" + (String) pathInstructions[instructionCount];
                instructionCount++;
                String internal = String.format(
                        "Sent image to android\nSend next instructions %s to STM\n\n",
                        (String) pathInstructions[instructionCount]);
                synchronized (socket) {
                    try {
                        System.out.println(internal);
                        Thread.sleep(SLEEPO);
                        outStream.write(toSTM.getBytes("UTF-8"));
                        //Thread.sleep(3000);
                    } catch (Exception e) {
                    }
                }
            } else {
                String toCamera = "IMG:DONE";
                String internal = "No more instructions send done camera";
                synchronized (socket) {
                    try {
                        System.out.println(internal);
                        Thread.sleep(SLEEPO);
                        outStream.write(toCamera.getBytes("UTF-8"));
                        //Thread.sleep(3000);
                    } catch (Exception e) {
                    }
                }
            }
        }

    }

    private void readjustment() {
        ADJUSTMENT_COUNT++;
        BACKWARD_COUNT++;
        IS_ADJUSTING = true;
        synchronized (socket) {
            try {
                String toSTM = "STM:\\fmb10;";
                outStream.write(toSTM.getBytes("UTF-8"));
                //String toCamera = "IMG:CAP";
                //ystem.out.println("Try capture image again");
                //Thread.sleep(SLEEPO);
                //outStream.write(toCamera.getBytes("UTF-8"));

                //Thread.sleep(3000);
            } catch (Exception e) {
            }
        }
    }

    private void moveBack() {

    }

    public static void AlgoServerMDP(String address, int port) {
        AlgoServerMDP algoServer = new AlgoServerMDP(address, port);
        algoServer.createSocket();
    }
}


