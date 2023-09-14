package Simulator;

import astar.State;
import astar.Obstacle;
import astar.astarPathFinder;
import astar.AMap;
import astar.AMapCell;
import astar.Node;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;


public class Simulator extends JPanel {
	
	State start = new State(1, 1, Simulator.UP);

    // Robot's current position on map
    State robot;

    JLabel timeLabel = new JLabel();

    long startTime = System.nanoTime();

    public static int MOVEMENT_SPEED = 1000 ;
    public static final int LEFT = 2;
    public static final int RIGHT = 0;
    public static final int UP = 1;
    public static final int DOWN = 3;

    private int count;
    // map dimensions
    final int LENGTH_PER_BOX = 20;
    private final int MAP_DIM = LENGTH_PER_BOX * 20;

    // obstacles
    public static int MAX_OBSTACLES = 8;
    HashMap<String, State> obstacles = new HashMap<>();

    public Simulator() {
        drawUI(); 
        this.robot = getOrigin().copy();
        convertSingleUnitToMap(this.robot);
        convertRobotToCenter(this.robot);
    }

    public Simulator(State start, ArrayList<State> obstacles) {

        
    	for (int i = 0; i < obstacles.size(); i++) {
            convertSingleUnitToMap(obstacles.get(i));
            //String a=obstacles.get(i).getState();
           // System.out.printf("hellllllllo",a);
            addObstacles(obstacles.get(i));
        }
        
        convertSingleUnitToMap(start);
        convertRobotToCenter(start);
        this.robot = start;
        drawUI();
    }

    private State getOrigin() {
        return new State(1, 1, Simulator.UP);
    }

    private void drawUI() {
        setLayout(new BoxLayout(this, BoxLayout.X_AXIS));

    	add(Box.createRigidArea(new Dimension(15, 0)));
        timeLabel = new JLabel();
        timeLabel.setAlignmentY(Component.BOTTOM_ALIGNMENT);
        timeLabel.setText("Time: 0s");
        timeLabel.setFont(new Font("ARIAL", Font.BOLD, 15));
        timeLabel.setMaximumSize(new Dimension(100,40));
        
        add(timeLabel);
        add(Box.createRigidArea(new Dimension(15, 0)));

        // Buttons
        
        // start button 
        JButton startButton = new JButton("Start");
        add(startButton);
        
        startButton.setBackground(Color.BLACK);
        startButton.setForeground(Color.white);
        startButton.setMaximumSize(new Dimension(130, 40));
        startButton.setAlignmentY(Component.BOTTOM_ALIGNMENT);
        startButton.addActionListener(e -> {
            execute(); 
        });
        add(Box.createRigidArea(new Dimension(10, 0)));
        // Reset button
        JButton resetButton = new JButton("Reset");
        add(resetButton);
        resetButton.setBackground(Color.BLACK);
        resetButton.setForeground(Color.white);
        resetButton.setMaximumSize(new Dimension(130, 40));
        resetButton.setAlignmentY(Component.BOTTOM_ALIGNMENT);
        resetButton.addActionListener(e -> {
            reset();
        });


        // Listeners
        addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                // Double click
                if (e.getClickCount() == 2 && e.getButton() == MouseEvent.BUTTON1) {
                } else if (e.getClickCount() == 1 && e.getButton() == MouseEvent.BUTTON1) {              
                	onClickAddObstacle(e.getX(), e.getY());
                }
            }
        });
    }

    public static String intDirectionToString(int direction) {
        if (direction == LEFT)
            return "LEFT";
        else if (direction == RIGHT)
            return "RIGHT";
        else if (direction == UP)
            return "UP";
        else if (direction == DOWN)
            return "DOWN";
        return "UP";
    }


    public static int stringDirectionToInt(String direction) {
        if (direction.equalsIgnoreCase("left"))
            return LEFT;
        else if (direction.equalsIgnoreCase("right"))
            return RIGHT;
        else if (direction.equalsIgnoreCase("up"))
            return UP;
        else if (direction.equalsIgnoreCase("down"))
            return DOWN;
        return UP;
    }

    /**
     * Determines if robot is facing obstacles
     *
     * @param robot
     * @param obstacle
     * @return
     */
    private boolean isFacingObstacle(State robot, State obstacle) {
        // Check if faces are adjacent
        if (
                !(robot.dir == LEFT && obstacle.dir == RIGHT) &&
                        !(robot.dir == RIGHT && obstacle.dir == LEFT) &&
                        !(robot.dir == UP && obstacle.dir == DOWN) &&
                        !(robot.dir == DOWN && obstacle.dir == UP)
        ) return false;

        if (robot.dir == DOWN && obstacle.dir == UP && robot.y > obstacle.y) {
        	return false;
        } else if (robot.dir == UP && obstacle.dir == DOWN && robot.y < obstacle.y) {
        	return false;
        }else if (robot.dir == LEFT && obstacle.dir == RIGHT && robot.x < obstacle.x) {
        	return false;
        }else if (robot.dir == RIGHT && obstacle.dir == LEFT && robot.x > obstacle.x) {
        	return false;
        }
        /**
         * For robot directions left and right, set Y axis allowable offset to 40 and x offset to 60
         * For robot directions up and down, set X axis allowable offset to 60 and y offset to 40
         * alternative goals
         *
         */
        int allowableXOffset = 0;
        int allowableYOffset = 0;
        int xOffset = Math.abs(robot.x - obstacle.x);
        int yOffset = Math.abs(robot.y - obstacle.y);
        
        if (robot.dir == LEFT || robot.dir == RIGHT) {
            allowableXOffset = 5 * LENGTH_PER_BOX;
            allowableYOffset = 2 * LENGTH_PER_BOX;
        } else {
            allowableXOffset = 2 * LENGTH_PER_BOX;
            allowableYOffset = 5 * LENGTH_PER_BOX;
        }
        // Satisfy the offset conditions
        // Must have a min distance of 3 cells
        return (allowableXOffset - xOffset > -1 &&
                allowableXOffset - xOffset > 2 &&
                allowableYOffset - yOffset > -1 &&
                allowableYOffset - yOffset > 2
        );
    }


    /**
     * Handler for add new obstacles
     *
     * @param x
     * @param y
     */
    private void onClickAddObstacle(int x, int y) {
        // exceed bounds
        if (x > MAP_DIM || y > MAP_DIM) return;

        State state = new State(
                fitCoordinateToGrid(x),
                fitCoordinateToGrid(y),
                Simulator.UP
        );

        State exist = obstacles.get(state.getXYPair());
        // Change obstacle orientation
        if (exist != null) {
            exist.dir = rotateSequence(exist.dir);
            state = exist;
        }

        obstacles.put(state.getXYPair(), state);
        repaint();
    }

    /**
     * Returns next sequence of rotation in clockwise
     *
     * @param rotation
     * @return
     */
    private int rotateSequence(int rotation) {
        if (rotation == UP)
            return RIGHT;
        else if (rotation == RIGHT)
            return DOWN;
        else if (rotation == DOWN)
            return LEFT;
        else if (rotation == LEFT)
            return UP;
        return UP;
    }

    private int fitCoordinateToGrid(int a) {
        return a / 20 * 20;
    }

    /**
     * Add obstacles to gui map
     *
     * @param state
     */
    private void addObstacles(State state) {
        obstacles.put(state.getXYPair(), state);
    }

    /**
     * Convert btw primitive cell and gui mapping
     * 
     * i.e (5 4) -> (100, 400 - 80)
     *
     * @param s
     */
    private void convertSingleUnitToMap(State s) {
        convertSingleUnitToMapX(s);
        convertSingleUnitToMapY(s);
    }

    private void convertSingleUnitToMapX(State s) {
        s.x = (s.x * LENGTH_PER_BOX);
    }

    private void convertSingleUnitToMapY(State s) {
        s.y = MAP_DIM - (s.y * LENGTH_PER_BOX) - LENGTH_PER_BOX;
    }


    private void convertMapToSingleUnit(State s) {
        s.x = (s.x / LENGTH_PER_BOX);
        s.y = (MAP_DIM - s.y - LENGTH_PER_BOX) / LENGTH_PER_BOX;
        System.out.println(s.getXYPair());
    }
    

    private void convertRobotToCenter(State s) {
        convertRobotToCenterX(s);
        convertRobotToCenterY(s);
    }

    private void convertRobotToCenterX(State s) {
        // Robot is 3 by 3 so the center would be x + 1, y - 1
        // Swing takes the coordinates from top left
        s.x -= LENGTH_PER_BOX;
    }

    private void convertRobotToCenterY(State s) {
        // Robot is 3 by 3 so the center would be x + 1, y - 1
        // Minus is plus. because we're inverting.
        s.y -= LENGTH_PER_BOX;
    }

    /**
     * Converts center position coordinate to top left
     * Note: Robot takes 3 x 3 and the direct coordinate on top left
     *
     * @param s
     */
    private void convertRobotToTopLeft(State s) {
        // Robot is 3 by 3 so the center would be x + 1, y - 1
        // Swing takes the coordinates from top left
        s.x += LENGTH_PER_BOX;
        // Minus is plus. because we're inverting.
        s.y += LENGTH_PER_BOX;
    }

    
    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;

        g.setColor(Color.decode("#fcd581"));
        g2.fillRect(0, 0, 400, 400);
        g.setColor(Color.decode("#D3D3D3"));
        g2.fillRect(0, 320, 80, 80);
        g.setColor(Color.decode("#fff8e8"));
        for (int i = 0; i <= 20; i++) {
        	 g.drawLine(i * LENGTH_PER_BOX, 0, i * LENGTH_PER_BOX,  400);
             g.drawLine(0, i * LENGTH_PER_BOX, 400, i * LENGTH_PER_BOX);
        } 

        // Robot
        g.setColor((Color.decode("#0496ff")));
        g2.fill(new Ellipse2D.Double(robot.x, robot.y, LENGTH_PER_BOX * 3, LENGTH_PER_BOX * 3));
        // Face of Robot
        g.setColor(Color.BLACK);
        switch (robot.dir) {
            case UP:
                g2.fillRect(robot.x + 23, robot.y, 14, 14);
                break;
            case LEFT:
                g2.fillRect(robot.x, robot.y + 23, 14, 14);
                break;
            case DOWN:
                g2.fillRect(robot.x + 23, robot.y + 46, 14, 14);
                break;
            case RIGHT:
                g2.fillRect(robot.x + 46, robot.y + 23, 14, 14);
                break;
        }

        drawObstacle(g, g2);
    }

    private void drawObstacle(Graphics g, Graphics2D g2) {
        // Set obstacles
        obstacles.forEach((key, obstacle) -> {
            g.setColor(Color.BLACK);
            g2.fillRect(obstacle.x, obstacle.y, LENGTH_PER_BOX, LENGTH_PER_BOX);

            int heightOfImage = LENGTH_PER_BOX / 4;

            if (obstacle.visited)
                g.setColor(Color.decode("#a5be002"));
            else
                g.setColor(Color.decode("#ff0700"));

            if (obstacle.dir == UP)
                g2.fillRect(obstacle.x, obstacle.y, LENGTH_PER_BOX, heightOfImage);
            else if (obstacle.dir == DOWN)
                g2.fillRect(obstacle.x, obstacle.y + LENGTH_PER_BOX - 5, LENGTH_PER_BOX, heightOfImage);
            else if (obstacle.dir == LEFT)
                g2.fillRect(obstacle.x, obstacle.y, 5, LENGTH_PER_BOX);
            else if (obstacle.dir == RIGHT)
                g2.fillRect(obstacle.x + LENGTH_PER_BOX - 5, obstacle.y, heightOfImage, LENGTH_PER_BOX);
        });
    }

    /**
     * Updates the ui movement of the robot
     * @param newRobotState
     * @throws InterruptedException
     */
    private void updateMovement(State newRobotState) throws InterruptedException {
        // Check if robot is in front of obstacle isFacingObstacle
        obstacles.forEach((key, obstacle) -> {
            if (isFacingObstacle(newRobotState, obstacle)) {
                obstacle.visited = true;
            }
        });
        // update time
        long stopTime = System.nanoTime();
        long current = (stopTime - startTime) / (1000 * 1000 * 1000);
        timeLabel.setText("Time: " + current + " s");

        robot = newRobotState;
        // Update frame
        repaint();
        // Time delay to update frame
        TimeUnit.MILLISECONDS.sleep(MOVEMENT_SPEED);

    }


    public void reset() {
        obstacles = new HashMap<>();
        start = getOrigin();
        State initial = getOrigin();
        convertSingleUnitToMap(initial);
        convertRobotToCenter(initial);
        this.robot = initial;
        repaint();
    }
    
    public void execute() {

        State copyStart = start.copy();
        
        /*------------- robot start position and dir--------------*/
        Node bot = new Node(1,1);
        bot.setDir(1);

        /*---------------------- obstacles-------------------------*/
        ArrayList<Obstacle> obstacles = new ArrayList<>(1);
        HashMap<Integer, Obstacle> orderedObstacles = new HashMap<>(obstacles.size());
        
        ArrayList<State> obstacleList = new ArrayList<>();
        
        count = 1;
        this.obstacles.forEach((key, obstacle) -> {
        	
            State copyState = obstacle.copy();
            Obstacle copyState1 = new Obstacle (count,obstacle.x / LENGTH_PER_BOX,
            		(MAP_DIM - obstacle.y - LENGTH_PER_BOX) / LENGTH_PER_BOX, obstacle.dir);
            convertMapToSingleUnit(copyState);
            obstacleList.add(copyState);
            obstacles.add(copyState1);
            
            System.out.println(count);
            count++;
            

            
        });
        
        
        /*---------------------- EXE-------------------------*/
        final int MAP_SIZE = 20;
        final AMap amap = new AMap(MAP_SIZE, obstacles);
        //System.out.println("Hey,");
//        for (int i = 0; i < 3; i++) {
//        	System.out.println(obstacles.get(i));
//        }
        
        final AMapCell[][] map = amap.getMap();
        Map<Node, Node> IcameFrom =  new HashMap<>();
        astarPathFinder pathFinder = new astarPathFinder(MAP_SIZE, obstacles, map, IcameFrom);

        ArrayList<ArrayList<String>> final_path = pathFinder.execute3(bot);


        System.out.println("final path");
        for (ArrayList<String> subp: final_path){
            System.out.println("next trip");
            for(String s: subp) {
            	System.out.println(s);
            }

        }

        // Run simulation
        new Thread(() -> {
            simulatePath(final_path);
        }).start();
    }

    /**
     * Simulate atar path
     *
     * @param finalPath
     */
    public void simulatePath(ArrayList<ArrayList<String>> finalPath) {
        try {
            startTime = System.nanoTime();
            this.robot = start.copy();
            convertSingleUnitToMap(this.robot);
            convertRobotToCenter(this.robot);


            int obstacleCount = 0;
            for (ArrayList<String> subp : finalPath) {
                if(!(subp.isEmpty())) {
            		obstacleCount++;
                }
                else {
                	continue;
                }
                System.out.printf("Obstacle: %d\n", obstacleCount);
                //System.out.printf(subp.get(0));
                //System.out.printf(subp.get(0));
                System.out.print(subp.isEmpty());
                for (String n : subp) {
                    // convert string to states
                	String [] nextpostion = n.split(" ");
                	
                    State newRobotState = new State(Integer.parseInt(nextpostion[0]),
                    		Integer.parseInt(nextpostion[1]),Integer.parseInt(nextpostion[2]));
                    System.out.println(newRobotState.getState());
                    convertSingleUnitToMap(newRobotState);
                    convertRobotToCenter(newRobotState);
                    // Update new robot coordinates
                    updateMovement(newRobotState);
                   
                }
                System.out.printf("after Loop ran once");
                
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    public static void app() {
        // Run simulator
        JFrame f = new JFrame();
        f.setTitle("Algorithm Simulator");
        Simulator s = new Simulator();

        s.setBackground(Color.decode("#fff8e8"));

        f.add(s);
        f.setVisible(true);
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.setSize(600, 500);
    }

    public static void main(String[] args) {
        app();
  
    }
}


