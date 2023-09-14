package astar;


public class State {
    public int x;
    public int y;
    public int dir;
    public boolean visited;

    public State(int x, int y, int dir) {
        this.x = x;
        this.y = y;
        this.dir = dir;
    }

    public State(int x, int y, int dir, boolean visited) {
        this.x = x;
        this.y = y;
        this.dir = dir;
        this.visited = visited;
    }

    public String getState() {
        return ("(" + String.valueOf(x) + "." + String.valueOf(y) + "." + String.valueOf(dir) + ")");
    }

    public State copy() {
        return new State(this.x, this.y, this.dir, this.visited);
    }

    /**
     * Returns X Y coordinates with | delimiter
     *
     * @return
     */
    public String getXYPair() {
        return x + ", " + y;
    }

}
