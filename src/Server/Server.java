package Server;
import java.util.Arrays;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

//import Algorithm.DubinsPathDriver;
//import Algorithm.PathPlanner;
import Connection.*;
//import Simulator.Simulator3;

public class Server {
    public static void main(String[] args) {

        String address = "192.168.12.12";
        int port = 4444;
        AlgoServerMDP.AlgoServerMDP(address, port);
    }

}