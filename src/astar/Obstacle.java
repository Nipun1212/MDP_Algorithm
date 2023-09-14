package astar;

public class Obstacle implements Comparable<Obstacle>{
    Node goal;
    int id;
    int x;
    int y;
    int dir;
    double easeOfAccess;
    private int unit = 20;
    // initial value = 15

    public Obstacle(int id, int x, int y, int dir) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.dir = dir;
        this.goal = findGoal();
    }

    public void setMax(){
        this.easeOfAccess = Double.MAX_VALUE;
    }

    public double setEaseOfAccess(Node bot) {
        int dx = x - bot.x;
        int dy = y - bot.y;
        double dist = Math.sqrt(dx * dx + dy * dy);
        double turn = 0;
        if (bot.dir == 0 || bot.dir == 2){
            if (dy == 0){
                if (bot.dir == 0 && dx > 0 && dir == 2) turn = 0;
                else if (bot.dir == 2 && dx < 0 && dir == 0) turn = 0;
            }
            else if (dy > 0){
                turn = switch (dir) {
                    case 0 -> unit * 2;
                    case 1 -> unit * 3;
                    case 2 -> unit * 2;
                    case 3 -> unit * 1;
                    default -> turn;
                };
            }
            else{
                turn = switch (dir) {
                    case 0 -> unit * 2;
                    case 1 -> unit * 1;
                    case 2 -> unit * 2;
                    case 3 -> unit * 3;
                    default -> turn;
                };
            }
        }
        else{
            if (dx == 0){
                if (bot.dir == 1 && dy > 0 && dir == 3) turn = 0;
                else if (bot.dir == 3 && dy < 0 && dir == 1) turn = 0;
            }
            else if (dx > 0){
                turn = switch (dir) {
                    case 0 -> unit * 3;
                    case 1 -> unit * 2;
                    case 2 -> unit * 1;
                    case 3 -> unit * 2;
                    default -> turn;
                };
            }
            else{
                turn = switch (dir) {
                    case 0 -> unit * 1;
                    case 1 -> unit * 2;
                    case 2 -> unit * 3;
                    case 3 -> unit * 2;
                    default -> turn;
                };
            }

        }
        this.easeOfAccess = dist + turn;
        return dist + turn;
    }

    private Node findGoal(){
        Node g = null;
        if (dir == 0) {
            g = new Node(x + 5, y);
            g.setDir(2);
        }
        else if (dir == 1) {
            g = new Node(x, y + 5);
            g.setDir(3);
        }
        else if (dir == 2) {
            g = new Node(x - 5, y);
            g.setDir(0);
        }
        else if (dir == 3) {
            g = new Node(x, y - 5);
            g.setDir(1);
        }
        return g;
    }

    @Override
    public int compareTo(Obstacle o) {
        return this.easeOfAccess > o.easeOfAccess ? 1 : -1;
    }

    @Override
    public String toString() {
        return "Obstacle[" + "id=" + id + ", x=" + x + ", y=" + y + ", dir=" + dir + ']';
    }
}
