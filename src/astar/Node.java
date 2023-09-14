package astar;

import java.lang.*;

public class Node implements Comparable<Node> {
    double Gn = 0;
    double Hn = 0; // heuristic
    double Fn = 0; //f(n) = h(n) + g(n)
    int x, y;
    int dir;
    Node parent;
    String actionFromParent;

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        this.Gn = Double.MAX_VALUE;
        this.Hn = Double.MAX_VALUE;
        this.Fn = this.Gn + this.Hn;
    }

    public void setActionFromParent(String actionFromParent) {
        this.actionFromParent = actionFromParent;
    }

    public void setDir(int dir) {
        this.dir = dir;
    }

    public void setGn(double gn) {
        Gn = gn;
        Fn = Gn + Hn;
    }

    public void setHn(double hn) {
        Hn = hn;
        Fn = Gn + Hn;
    }

    public void setFn(double fn) {
        Fn = fn;
    }

    public void setParent(Node parent) {
        this.parent = parent;
    }

    @Override
    public int compareTo(Node n) {
        return this.Fn >= n.Fn ? 1:-1;
    }

    @Override
    public String toString() {
        return (this.x + " " +this.y + " " + this.dir + " ");
    }
}


