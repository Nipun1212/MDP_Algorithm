package Simulator;

public class Robot {
    private double sizeX;
    private double sizeY;
    private double x;
    private double y;
    private int direction;  
    
    public Robot(double x, double y, double sizeX, double sizeY) {
        this.x = x;
        this.y = y;
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        this.direction = 0;
    }

    public double getX() {
        return this.x;
    }
  
    public double getY() {
        return this.y;
    }

    public double getSizeX() {
        return this.sizeX;
    }

    public double getSizeY() {
        return this.sizeY;
    }

    public int getDirection() {
        return this.direction;
    }

    public void setRobotLoc(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }
}
