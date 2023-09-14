package astar;

import java.util.*;

public class astarPathFinder {
    public int MAP_SIZE ;
    public final ArrayList<Obstacle> obstacles;
    public ArrayList<Obstacle> dynamicObs;
    public AMapCell[][] map;
    public Map<Node, Node> cameFrom;
    public int cameFromSizelimit = 0;

    public astarPathFinder(int MAP_SIZE, ArrayList<Obstacle> obstacles, AMapCell[][] map, Map<Node, Node> cameFrom) {
        this.MAP_SIZE = MAP_SIZE;
        this.obstacles = obstacles;
        System.out.print("ASTAR:");
        System.out.print(obstacles.size());
        this.dynamicObs = new ArrayList<Obstacle>(obstacles.size());
        for(Obstacle o : obstacles) {
        	this.dynamicObs.add(o);
        }
        this.map = map; 
        this.cameFrom = cameFrom;
    }

    public ArrayList<Node> astar(Node start, Node goal){
        PriorityQueue<Node> open = new PriorityQueue<Node>();
        open.add(start);
        start.setGn(0);
        start.setFn(0);
        ArrayList<Node> nopath = new ArrayList<Node>();
        cameFromSizelimit += 60000;

        while(!open.isEmpty()){
            Node current = open.poll();
            /*-----------------------------------TEST PRINT-----------------------------------
            System.out.println("Expanding " + current.toString());
            System.out.println(open.size());
            -----------------------------------TEST PRINT-----------------------------------*/
            if(current.x == goal.x && current.y == goal.y && current.dir == goal.dir){
                System.out.println("Goal reached!");
                System.out.println("Goal: " + goal.toString());
                return reconstruct_path(current);
            }

            for (Node neighbor: getNeighbors(current, goal)){
                double tentative_Gn = current.Gn + path_cost(current, neighbor);
                if (tentative_Gn < neighbor.Gn){
                    cameFrom.put(neighbor, neighbor.parent);
                    /*-----------------------------------TEST PRINT-----------------------------------
                    System.out.println(cameFrom.size());
                    -----------------------------------TEST PRINT-----------------------------------*/
                    neighbor.setGn(tentative_Gn);
                    neighbor.setFn(tentative_Gn + neighbor.Hn);
                    if (!open.contains(neighbor)) {open.add(neighbor);}
                }
            }
            if(cameFrom.size() > cameFromSizelimit ) return  nopath;
        }
        return nopath;
        //return null;
    }
//    private boolean turnnotAccessible(int x, int y) {
//    	for(Obstacle K : obstacles) {
//        	if(K.dir==0 || (x==K.x+1)) return true;
//        	
//        	if(K.dir==2 ||(x==K.x-1)) return true;
//        	
//        	if(K.dir==1 ||(y==K.y+1)) return true;
//        	if(K.dir==3||(y==K.y-1)) return true;
//        	}
//		return  !map[x][y].isAccessible ;
//    }
    private boolean notAccessible(int x, int y){
        if (x < 0 || x > MAP_SIZE -1 || y < 0 || y > MAP_SIZE -1) return true;
        
        
        return !map[x][y].isAccessible;
    }

    private ArrayList<Node> getNeighbors(Node current, Node goal){
        // adjacent cell when rotation is not required, otherwise 90 degrees is like from (1,1) to (3,3), turning radius = 20
        // 6 immediate neighbors
        ArrayList<Node> neighbors = new ArrayList<>();
        Node front = frontNeighbor(current, goal);
        Node back = backNeighbor(current,goal);
        Node lf = LFturn(current, goal);
        Node rf = RFturn(current, goal);
        Node lb = LBturn(current, goal);
        Node rb = RBturn(current, goal);
        if (front != null){
            neighbors.add(front);
        }
        if (back != null){
            neighbors.add(back);
        }
        if (lf != null){
            neighbors.add(lf);
        }
        if (rf != null){
            neighbors.add(rf);
        }
        if (lb != null){
            neighbors.add(lb);
        }
        if (rb != null){
            neighbors.add(rb);
        }

        /*-----------------------------------TEST PRINT-----------------------------------
        System.out.println("Neighbor size of " + current.toString() + ": " + neighbors.size());
        for (Node n: neighbors){
            System.out.println(n.toString());
        }
        -----------------------------------TEST PRINT-----------------------------------*/
        return neighbors;
    }

    private Node frontNeighbor(Node current, Node goal){
        // move forward by 1 grid
        int x;
        int y;
        if (current.dir == 1){
            x = current.x;
            y = current.y + 1;
        }
        else if (current.dir == 2){
            x = current.x - 1;
            y = current.y;
        }
        else if (current.dir == 3){
            x = current.x;
            y = current.y - 1;
        }
        else{
            x = current.x + 1;
            y = current.y;
        }
        if (notAccessible(x, y)) return null;
        Node n = new Node(x, y);
        n.setParent(current);
        n.setDir(current.dir);
        n.setHn(heuristics(n, goal));
        n.setActionFromParent("Forward");
        return n;
    }

    private Node backNeighbor(Node current, Node goal){
        int x;
        int y;
        if (current.dir == 1){
            x = current.x;
            y = current.y - 1;
        }
        else if (current.dir == 2){
            x = current.x + 1;
            y = current.y;
        }
        else if (current.dir == 3){
            x = current.x;
            y = current.y + 1;
        }
        else{
            x = current.x - 1;
            y = current.y;
        }
        if (notAccessible(x, y)) return null;
        Node n = new Node(x, y);
        n.setParent(current);
        n.setDir(current.dir);
        n.setHn(heuristics(n, goal));
        n.setActionFromParent("Backward");
        return n;
    }

    private Node LFturn(Node current, Node goal){
        int x;
        int y;
        int dir;
        int XDiff=3;
        int YDiff=3;
        
        if (current.dir == 1){
            if (notAccessible(current.x, current.y+1) || notAccessible(current.x, current.y+2)
                    || notAccessible(current.x-1, current.y+3) || notAccessible(current.x-2, current.y+3)) return null;
            x = current.x - XDiff;
            y = current.y + YDiff ;
            dir = 2;
        }
        else if (current.dir == 2){
            if (notAccessible(current.x-1, current.y) || notAccessible(current.x-2, current.y)
                    || notAccessible(current.x-3, current.y-1) || notAccessible(current.x-3, current.y-2)) return null;
            x = current.x - XDiff;
            y = current.y - YDiff ;
            dir = 3;
        }
        else if (current.dir == 3){
            if (notAccessible(current.x, current.y-1) || notAccessible(current.x, current.y-2)
                    || notAccessible(current.x+1, current.y-3) || notAccessible(current.x+2, current.y-3)) return null;
            x = current.x + XDiff ;
            y = current.y - YDiff;
            dir = 0;
        }
        else{
            if (notAccessible(current.x+1, current.y) || notAccessible(current.x+2, current.y)
                    || notAccessible(current.x+3, current.y+1) || notAccessible(current.x+3, current.y+2)) return null;
            x = current.x + XDiff ;
            y = current.y + YDiff;
            dir = 1;
        }
        if (notAccessible(x, y)) return null;
        Node n = new Node(x, y);
        n.setParent(current);
        n.setDir(dir);
        n.setHn(heuristics(n, goal));
        n.setActionFromParent("LFturn");
        return n;
    }

    private Node RFturn(Node current, Node goal){
        int x;
        int y;
        int dir;
        int XDiff=3;
        int YDiff=3;
        
        if (current.dir == 1){
            if (notAccessible(current.x, current.y+1) || notAccessible(current.x, current.y+2)
                    || notAccessible(current.x+1, current.y+3) || notAccessible(current.x+2, current.y+3)) return null;
            x = current.x + XDiff;
            y = current.y + YDiff ;
            dir = 0;
        }
        else if (current.dir == 2){
            if (notAccessible(current.x-1, current.y) || notAccessible(current.x-2, current.y)
                    || notAccessible(current.x-3, current.y+1) || notAccessible(current.x-3, current.y+2)) return null;
            x = current.x - YDiff;
            y = current.y + XDiff ;
            dir = 1;
        }
        else if (current.dir == 3){
            if (notAccessible(current.x, current.y-1) || notAccessible(current.x, current.y-2)
                    || notAccessible(current.x-1, current.y-3) || notAccessible(current.x-2, current.y-3)) return null;
            x = current.x - XDiff;
            y = current.y - YDiff ;
            dir = 2;
        }
        else{
            if (notAccessible(current.x+1, current.y) || notAccessible(current.x+2, current.y)
                    || notAccessible(current.x+3, current.y-1) || notAccessible(current.x+3, current.y-2)) return null;
            x = current.x + YDiff;
            y = current.y - XDiff ;
            dir = 3;
        }
        if (notAccessible(x, y)) return null;
        Node n = new Node(x, y);
        n.setParent(current);
        n.setDir(dir);
        n.setHn(heuristics(n, goal));
        n.setActionFromParent("RFturn");
        return n;
    }

    private Node LBturn(Node current, Node goal){
        int x;
        int y;
        int dir;
        int XDiff=3;
        int YDiff=3;
        if (current.dir == 1){
            if (notAccessible(current.x, current.y-1) || notAccessible(current.x, current.y-2)
                    || notAccessible(current.x-1, current.y-3) || notAccessible(current.x-2, current.y-3)) return null;
            x = current.x - XDiff ;
            y = current.y - XDiff;
            dir = 0;
        }
        else if (current.dir == 2){
            if (notAccessible(current.x+1, current.y) || notAccessible(current.x+2, current.y)
                    || notAccessible(current.x+3, current.y-1) || notAccessible(current.x+3, current.y-2)) return null;
            x = current.x + XDiff ;
            y = current.y - XDiff ;
            dir = 1;
        }
        else if (current.dir == 3){
            if (notAccessible(current.x, current.y+1) || notAccessible(current.x, current.y+2)
                    || notAccessible(current.x+1, current.y+3) || notAccessible(current.x+2, current.y+3)) return null;
            x = current.x + XDiff ;
            y = current.y + XDiff ;
            dir = 2;
        }
        else{
            if (notAccessible(current.x-1, current.y) || notAccessible(current.x-2, current.y)
                    || notAccessible(current.x-3, current.y+1) || notAccessible(current.x-3, current.y+2)) return null;
            x = current.x - XDiff ;
            y = current.y + XDiff ;
            dir = 3;
        }
        if (notAccessible(x, y)) return null;
        Node n = new Node(x, y);
        n.setParent(current);
        n.setDir(dir);
        n.setHn(heuristics(n, goal));
        n.setActionFromParent("LBturn");
        return n;
    }

    private Node RBturn(Node current, Node goal){
        int x;
        int y;
        int dir;
        int XDiff=3;
        int YDiff=3;
        if (current.dir == 1){
            if (notAccessible(current.x, current.y-1) || notAccessible(current.x, current.y-2)
                    || notAccessible(current.x+1, current.y-3) || notAccessible(current.x+2, current.y-3)) return null;
            x = current.x + XDiff ;
            y = current.y - YDiff;
            dir = 2;
        }
        else if (current.dir == 2){
            if (notAccessible(current.x+1, current.y) || notAccessible(current.x+2, current.y)
                    || notAccessible(current.x+3, current.y+1) || notAccessible(current.x+3, current.y+2)) return null;
            x = current.x + XDiff;
            y = current.y + YDiff;
            dir = 3;
        }
        else if (current.dir == 3){
            if (notAccessible(current.x, current.y+1) || notAccessible(current.x, current.y+2)
                    || notAccessible(current.x-1, current.y+3) || notAccessible(current.x-2, current.y+3)) return null;
            x = current.x - XDiff ;
            y = current.y + YDiff;
            dir = 0;
        }
        else{
            if (notAccessible(current.x-1, current.y) || notAccessible(current.x-2, current.y)
                    || notAccessible(current.x-3, current.y-1) || notAccessible(current.x-3, current.y-2)) return null;
            x = current.x - XDiff ;
            y = current.y - YDiff ;
            dir = 1;
        }
        //paulani
        if (notAccessible(x, y)) return null;
        Node n = new Node(x, y);
        n.setParent(current);
        n.setDir(dir);
        n.setHn(heuristics(n, goal));
        n.setActionFromParent("RBturn");
        return n;
    }

    private double heuristics(Node bot, Node goal){
        int dx = goal.x - bot.x;
        int dy = goal.y - bot.y;
        double dist = Math.sqrt(dx * dx + dy * dy);
        double turn = 0;
        int unit = 4;
        if (bot.dir == 0 || bot.dir == 2){
            if (dy == 0){
                if (bot.dir == 0 && dx > 0 && goal.dir == 0) turn = 0;
                else if (bot.dir == 2 && dx < 0 && goal.dir == 2) turn = 0;
            }
            else if (dy > 0){
                turn = switch (goal.dir) {
                    case 0 -> unit * 2;
                    case 1 -> unit * 3;
                    case 2 -> unit * 2;
                    case 3 -> unit * 1;
                    default -> turn;
                };
            }
            else{
                turn = switch (goal.dir) {
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
                if (bot.dir == 1 && dy > 0 && goal.dir == 3) turn = 0;
                else if (bot.dir == 3 && dy < 0 && goal.dir == 1) turn = 0;
            }
            else if (dx > 0){
                turn = switch (goal.dir) {
                    case 0 -> unit * 3;
                    case 1 -> unit * 2;
                    case 2 -> unit * 1;
                    case 3 -> unit * 2;
                    default -> turn;
                };
            }
            else{
                turn = switch (goal.dir) {
                    case 0 -> unit * 1;
                    case 1 -> unit * 2;
                    case 2 -> unit * 3;
                    case 3 -> unit * 2;
                    default -> turn;
                };
            }

        }
        return dist + turn;
    }

    private double path_cost(Node current, Node neighbor){
        // (1,1, up) -> (1,2up) cost= 1
        // (1,1,up) -> (3,3,right) cost = 4
        double cost, turnCost;
        int dx = Math.abs(current.x - neighbor.x);
        int dy = Math.abs(current.y - neighbor.y);
        if (dx == 1 || dy == 1){
            cost = 1;
        }
        else{
            cost = 4;
        }
        return cost;
        //return cost + turnCost;
//    	int dx = Math.abs(current.x - neighbor.x);
//        int dy = Math.abs(current.y - neighbor.y);
//
//        // Calculate the straight line distance cost
//        double straightLineCost = 0;
//        if (dx == 1 || dy == 1) {
//            straightLineCost = 1;
//        } else {
//            straightLineCost = 4;
//        }
//
//        // Calculate the turn cost
//        double turnCost = 0;
//        if (current.dir != neighbor.dir) {
//            int deltaDir = Math.abs(current.dir - neighbor.dir);
//            if (deltaDir == 1 || deltaDir == 3) {
//                // 90 degree turn
//                turnCost = 2;
//            } else {
//                // 180 degree turn
//                turnCost = 4;
//            }
//        }

        // Return the total cost
        //return straightLineCost + turnCost;
    }

    public ArrayList<Node> reconstruct_path(Node goal){
        ArrayList<Node> total_path = new ArrayList<Node>();
        total_path.add(0, goal);
        while (cameFrom.containsKey(goal)){
            goal = cameFrom.get(goal);
            total_path.add(0, goal);
        }
        return total_path;
    }

    public PriorityQueue<Obstacle> dynamicOrderObstacles (ArrayList<Obstacle> dynamicObs, Node bot){
        PriorityQueue<Obstacle> open = new PriorityQueue<>();
        HashMap<Integer, Obstacle> dynamicOrderedObs = new HashMap<>(dynamicObs.size()); // <visit order, obstacle(with its goal inside)>
        for (Obstacle o: dynamicObs){
            o.setEaseOfAccess(bot);
            System.out.println("obstacle no." + o.id + " updated ease of access: " + o.easeOfAccess);
            open.add(o);
        }
        return open;
    }

    // inplace turn
    public ArrayList<ArrayList<String>> exe3Helper(ArrayList<Node> p, ArrayList<ArrayList<String>> final_path, int count, int oid){
        ArrayList<String> str_path = new ArrayList<>(p.size());

        for (int j=0; j<p.size()-1; j++){
            Node n = p.get(j);
            Node n1 = p.get(j+1);
            str_path.add(n.toString() + " - " + n1.actionFromParent + " " + oid + ",");
            //System.out.println(str_path.get(j));
        }
        Node fn = p.get(p.size()-1);
        str_path.add(fn.toString() + " - " + "Pause " + oid+ ",");
        final_path.add(count, str_path);

        return final_path;
    }

    // inplace turn
    private ArrayList<ArrayList<String>> inPlace(Node bot, Node goal, ArrayList<Node> p, ArrayList<ArrayList<String>> final_path, int count, int oid){
        String inPlace = null;
        if ((bot.dir == 1 && goal.dir == 0) || (bot.dir == 0 && goal.dir == 3) || (bot.dir == 2 && goal.dir == 1) || (bot.dir == 3 && goal.dir == 2)) {
            inPlace = "RightInPlace";
        }
        else if ((bot.dir == 1 && goal.dir == 2) || (bot.dir == 0 && goal.dir == 1) || (bot.dir == 2 && goal.dir == 3) || (bot.dir == 3 && goal.dir == 0)) {
            inPlace = "LeftInPlace";
        }

        ArrayList<String> str_path = new ArrayList<>(p.size() + 1);

        for (int j=0; j<p.size()-2; j++){
            Node n = p.get(j);
            Node n1 = p.get(j+1);
            str_path.add(n.toString() + " " + n1.actionFromParent + " " + oid);
            //System.out.println(str_path.get(j));
        }
        Node fn = p.get(p.size()-2);
        str_path.add(fn.toString() + " " + inPlace + " " + oid);
        str_path.add(goal.toString() + " " + "Pause" + " " + oid);
        final_path.add(count, str_path);

        return final_path;
    }

    // inplace turn
    public ArrayList<ArrayList<String>> execute3(Node start){
        // HashMap<Integer, Obstacle> orderObstacles = orderObstacles();
        Node bot = start;
        PriorityQueue<Obstacle> orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
        ArrayList<ArrayList<String>> final_path = new ArrayList<ArrayList<String>>(orderObstacles.size() + 1); // <visiting order, sub-path>
        ArrayList<String> foo = new ArrayList<>(1);
        ArrayList<Obstacle> nopathObs = new ArrayList<Obstacle>();
        
        for (int i = 0; i <= orderObstacles.size(); i++){
            final_path.add(foo);
        }

//        for (int i = 1; i <= orderObstacles.size(); i++){
//            System.out.println("Visit " + i);
//
//            Obstacle o = orderObstacles.get(i);
//            Node g = o.goal;
//            ArrayList<Node> p = astar(bot, g);
//            ArrayList<String> str_path = new ArrayList<>(p.size());
//            bot = g;
//
//
//            for (int j=0; j<p.size()-1; j++){
//                Node n = p.get(j);
//                Node n1 = p.get(j+1);
//                str_path.add(n.toString() + " " + n1.actionFromParent);
//                //System.out.println(str_path.get(j));
//            }
//            Node fn = p.get(p.size()-1);
//            str_path.add(fn.toString() + " " + "Pause");
//            final_path.add(i, str_path);
//            //System.out.println("------------------------");
//        }
        
        int count = 1;
        int forceQuit = 0;
        while(!orderObstacles.isEmpty()){
            System.out.println("Visit " + count);

            Obstacle o = orderObstacles.poll();

            System.out.println(count + ": obstacle no." + o.id + " with ease of access: " + o.easeOfAccess);
            int dir=o.dir;
            
            if(dir==0) { 
            	map[o.x+1][o.y].setInaccessible();
            	//System.out.printf("Set",o.x,",",o.y,"as invalid");
            	System.out.printf("Set (%d,%d) as invalid", o.x+1, o.y);
            }
        	
        	if(o.dir==2) {
        		map[o.x-1][o.y].setInaccessible();
        		//System.out.printf("Set",o.x-1+","+o.y+"as invalid");
        		System.out.printf("Set (%d,%d) as invalid", o.x-1, o.y);
        	}
        	
        	if(o.dir==1) {
        		map[o.x][o.y+1].setInaccessible();
        		//System.out.printf("Set",o.x+","+o.y+1+"as invalid");
        		System.out.printf("Set (%d,%d) as invalid", o.x, o.y+1);
        	}
        	if(o.dir==3) {
        		map[o.x][o.y-1].setInaccessible();
        		//System.out.printf("Set",o.x+","+o.y-1+"as invalid");
        		System.out.printf("Set (%d,%d) as invalid", o.x, o.y-1);
        		
        	}
        	
            Node g = o.goal;
            
            
           
            ArrayList<Node> p = astar(bot, g);

            if (p.size() != 0){
                bot = g;
                dynamicObs.remove(o);
                orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
                exe3Helper(p, final_path, count, o.id);
                count += 1;
//                for (int j=0; j<p.size()-1; j++){
//                    Node n = p.get(j);
//                    Node n1 = p.get(j+1);
//                    str_path.add(n.toString() + " " + n1.actionFromParent);
//                    //System.out.println(str_path.get(j));
//                }
//                Node fn = p.get(p.size()-1);
//                str_path.add(fn.toString() + " " + "Pause");
//                final_path.add(count, str_path);
            }
            else{
                System.out.println("Checking alternative goals and their paths");
                Node altgoal1 = g;
                Node altgoal2 = g;
                Node altgoal3 = g;
  
                if (g.dir == 2 ) {
                    if (g.y-1 >= 0 ) {
                        altgoal1 = new Node(g.x, g.y -1);
                        altgoal1.setDir(g.dir);
                        
                    }
                    //changed from else if
                    if (g.y+1 < 20 )  {
                        altgoal2 = new Node(g.x, g.y +1);
                        altgoal2.setDir(g.dir);
                    }
                    altgoal3 = new Node(g.x+1, g.y);
                    altgoal3.setDir(g.dir);

                }
                else if (g.dir == 0) {
                    if (g.y-1 >= 0 ) {
                        altgoal1 = new Node(g.x, g.y -1);
                        altgoal1.setDir(g.dir);
                    }
                    //changed from else if
                    if (g.y+1 < 20 )  {
                        altgoal2 = new Node(g.x, g.y +1);
                        altgoal2.setDir(g.dir);
                    }
                    altgoal3 = new Node(g.x-1, g.y);
                    altgoal3.setDir(g.dir);

                }
                else if(g.dir == 1 ) {
                    if (g.x-1 >= 0 ) {
                        altgoal1 = new Node(g.x-1, g.y);
                        altgoal1.setDir(g.dir);
                    }
                    if (g.x+1 < 20 )  {
                        altgoal2 = new Node(g.x+1, g.y);
                        altgoal2.setDir(g.dir);
                    }
                    altgoal3 = new Node(g.x, g.y-1);
                    altgoal3.setDir(g.dir);
                }
                else if(g.dir == 3) {
                    if (g.x-1 >= 0 ) {
                        altgoal1 = new Node(g.x-1, g.y);
                        altgoal1.setDir(g.dir);
                    }
                    if (g.x+1 < 20 )  {
                        altgoal2 = new Node(g.x+1, g.y);
                        altgoal2.setDir(g.dir);
                    }
                    altgoal3 = new Node(g.x, g.y+1);
                    altgoal3.setDir(g.dir);
                }
                ArrayList<Node> p1 = astar(bot, altgoal1);
                ArrayList<Node> p2 = astar(bot, altgoal2);
                ArrayList<Node> p3 = astar(bot, altgoal3);
                
                // paulani
                // changed from p1.size()!=0 && p.size()!= 0
//                else if (p1_inplace.size()!=0 && p2_inplace.size()!= 0) {
//                	if (p1_inplace.get(p1_inplace.size()-1).Fn < p2_inplace.get(p2_inplace.size()-1).Fn)
//                    {
//                        p = p1_inplace;
//                        g = altgoal1_inplace;
//                    }
//                	else {
//                		p = p2_inplace;
//                        g = altgoal2_inplace;
//                	}
//                	bot = g;
//                    dynamicObs.remove(o);
//                    orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
//                    exe3Helper(p, final_path, count, o.id);
//                    count += 1;
//                }
//                else if(p2_inplace.size() !=0) {
//                	p = p2_inplace;
//                    g = altgoal2_inplace;
//                    bot = g;
//                    dynamicObs.remove(o);
//                    orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
//                    exe3Helper(p, final_path, count, o.id);
//                    count += 1;
//                }
//                else if(p1_inplace.size() !=0) {
//                	p = p1_inplace;
//                    g = altgoal1_inplace;
//                    bot = g;
//                    dynamicObs.remove(o);
//                    orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
//                    exe3Helper(p, final_path, count, o.id);
//                    count += 1;
//                }
                if (p1.size()!=0 && p2.size()!= 0 && p3.size()!= 0) {
                	double minsize = p1.get(p1.size()-1).Fn;
                	if (minsize > p2.get(p2.size()-1).Fn)  minsize = p2.get(p2.size()-1).Fn;
                	if (minsize > p3.get(p3.size()-1).Fn)  minsize = p3.get(p3.size()-1).Fn;
                	
                    if (minsize == p1.get(p1.size()-1).Fn)
                    {
                        p = p1;
                        g = altgoal1;
                    }
                    else if (minsize == p2.get(p2.size()-1).Fn) {
                        p = p2;
                        g = altgoal2;
                    }
                    else if (minsize == p3.get(p3.size()-1).Fn) {
                        p = p3;
                        g = altgoal3;
                    }
                    bot = g;
                    dynamicObs.remove(o);
                    orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
                    exe3Helper(p, final_path, count, o.id);
                    count += 1;
                }
                else if (p2.size() !=0) {
                    p = p2;
                    g = altgoal2;
                    bot = g;
                    dynamicObs.remove(o);
                    orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
                    exe3Helper(p, final_path, count, o.id);
                    count += 1;
                }
                else if (p1.size() != 0) {
                    p = p1;
                    g = altgoal1;
                    bot = g;
                    dynamicObs.remove(o);
                    orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
                    exe3Helper(p, final_path, count, o.id);
                    count += 1;
                }
                else if (p3.size() != 0) {
                    p = p3;
                    g = altgoal3;
                    bot = g;
                    dynamicObs.remove(o);
                    orderObstacles = dynamicOrderObstacles(dynamicObs, bot);
                    exe3Helper(p, final_path, count, o.id);
                    count += 1;
                }
                else {
                    o.setMax();
                    orderObstacles.add(o);
                    forceQuit += 1;
                    if (forceQuit >= 10){
                        orderObstacles.remove(o);
                        System.out.println(o.toString() + " cannot be reached!");
                        nopathObs.add(o);
                        
                    }
                    continue;

                }
            }
//            else{
//                o.setMax();
//                orderObstacles.add(o);
//                continue;
//            }
            if(dir==0) { 
            	map[o.x+1][o.y].setaccessible();
            	//System.out.printf("Set",o.x,",",o.y,"as invalid");
            	System.out.printf("Set (%d,%d) as valid", o.x+1, o.y);
            }
        	
        	if(o.dir==2) {
        		map[o.x-1][o.y].setaccessible();
        		//System.out.printf("Set",o.x-1+","+o.y+"as invalid");
        		System.out.printf("Set (%d,%d) as valid", o.x-1, o.y);
        	}
        	
        	if(o.dir==1) {
        		map[o.x][o.y+1].setaccessible();
        		//System.out.printf("Set",o.x+","+o.y+1+"as invalid");
        		System.out.printf("Set (%d,%d) as valid", o.x, o.y+1);
        	}
        	if(o.dir==3) {
        		map[o.x][o.y-1].setaccessible();
        		//System.out.printf("Set",o.x+","+o.y-1+"as invalid");
        		System.out.printf("Set (%d,%d) as valid", o.x, o.y-1);
        		
        	}
        }
        // only one 
       System.out.println("no path obs"+ nopathObs);
       
       while (nopathObs.size() != 0) {

    	    Obstacle o = nopathObs.get(0);
            nopathObs.remove(0);
        	System.out.println("obstacle need to in-place turn"+ o.toString());
        	Node g = o.goal;
        	Node altgoal1_inplace = g;
            Node altgoal2_inplace = g;
            if (g.dir == 0 || g.dir == 2) {
               
                altgoal1_inplace = new Node(g.x, g.y);
                altgoal1_inplace.setDir(1);
                altgoal2_inplace = new Node(g.x, g.y);
                altgoal2_inplace.setDir(3);

            }
            else if(g.dir == 1 || g.dir == 3) {
                
                altgoal1_inplace = new Node(g.x, g.y);
                altgoal1_inplace.setDir(0);
                altgoal2_inplace = new Node(g.x, g.y);
                altgoal2_inplace.setDir(2);
            }
            ArrayList<Node> p1_inplace = astar(bot, altgoal1_inplace);
            ArrayList<Node> p2_inplace = astar(bot, altgoal2_inplace);
            ArrayList<Node> p;
            //System.out.println("inplace turn 1"+ " " + p1_inplace.size());
            //System.out.println("inplace turn 2"+ " " + p2_inplace.size());
            
            if (p1_inplace.size()!=0 && p2_inplace.size()!= 0) {
            	
            	if (p1_inplace.get(p1_inplace.size()-1).Fn < p2_inplace.get(p2_inplace.size()-1).Fn)
                {
                    p = p1_inplace;
                    g = altgoal1_inplace;
                }
            	else {
            		p = p2_inplace;
                    g = altgoal2_inplace;
            	}
            	bot = g;
            	//System.out.println("inplace path"+ "" + p.size());
                inPlace(bot, o.goal, p, final_path, count, o.id);
                //System.out.println("obstacle id"+ " " + o.id);
                count += 1;
            }
            else if(p2_inplace.size() !=0) {
            	p = p2_inplace;
                g = altgoal2_inplace;
                bot = g;
                //System.out.println("inplace path"+ "" + p.size());
                inPlace(bot, o.goal, p, final_path, count, o.id);
                count += 1;
            }
            else if(p1_inplace.size() !=0) {
            	p = p1_inplace;
                g = altgoal1_inplace;
                bot = g;
                //System.out.println("inplace path"+ "" + p.size());
                inPlace(bot, o.goal, p, final_path, count, o.id);
                count += 1;
            }
        }
        return final_path;
    }
    public static  ArrayList<ArrayList<String>> executeFN1(String message) {
    	Node bot = new Node(1,1);
        bot.setDir(1);

        /*---------------------- obstacles-------------------------*/
        ArrayList<Obstacle> obstacles1 = new ArrayList<>(1);
        HashMap<Integer, Obstacle> orderedObstacles = new HashMap<>(obstacles1.size());

        
//        Obstacle o1 = new Obstacle(1, 5, 0, 1);
//        Obstacle o2 = new Obstacle(2, 9, 6, 3);
//        Obstacle o3 = new Obstacle(3, 11 ,8, 1);
//        Obstacle o4 = new Obstacle(4, 11, 13,0);
//        Obstacle o5 = new Obstacle(5, 5, 18,3);
//        Obstacle o6 = new Obstacle(6, 19, 1 ,2);
        
        //String message="1,5,0,1/n2,9,6,3/n3,11,8,1/n4,11,13,0/n5,5,18,3/n6,19,1,2/n";
    	String[] obs=message.split("/n");
    	//System.out.println(obs.length);
    	//System.out.println(obs[1]);
    	int size = obs.length;
        //int [] arr = new int [size];
    	for(int i=0; i<size; i++) {
            String[] str=obs[i].split(",");
            int sizeL = str.length;
            //System.out.println(sizeL);
            int [] arr = new int [sizeL];
            for(int j=0; j<sizeL; j++) {
            	System.out.println(Integer.parseInt(str[j]));
            	arr[j] = Integer.parseInt(str[j]);
               
            }
      
            
            //ArrayList<Obstacle> obstacles = new ArrayList<>(1);
//        	System.out.println(arr[0]);
//        	System.out.println(arr[1]);
//        	System.out.println(arr[2]);
//        	System.out.println(arr[3]);

            Obstacle o1 = new Obstacle(arr[0], arr[1], arr[2], arr[3]);
            obstacles1.add(o1);
        	System.out.println(o1.toString());

             
                
             
            
            //System.out.println(Arrays.toString(arr));
         }


//        obstacles.add(o1);
//        obstacles.add(o2);
//        obstacles.add(o3);
//        obstacles.add(o4);
//        obstacles.add(o5);
//        obstacles.add(o6);
        /*---------------------- AMap & pathFinder instance-------------------------*/
        final int MAP_SIZE = 20;
        final AMap amap = new AMap(MAP_SIZE, obstacles1);
        final AMapCell[][] map = amap.getMap();
        Map<Node, Node> IcameFrom =  new HashMap<>();
        astarPathFinder pathFinder = new astarPathFinder(MAP_SIZE, obstacles1, map, IcameFrom);


        /*---------------------- EXE-------------------------*/
        ArrayList<ArrayList<String>> final_path = pathFinder.execute3(bot);
        System.out.println("----1st obs to visit");
        ArrayList<String> subp1 = final_path.get(1);
        ArrayList<String> subp20 = final_path.get(2);

        for (String s: subp1){
            System.out.println(s);
           // System.out.println("Hello");

        }
        //for (String s: subp20){
           // System.out.println(s);
            //System.out.println("YYYYY");

        //}
        System.out.println("----2nd obs to visit");
        ArrayList<String> subp2 = final_path.get(2);
        for (String s: subp2){
            System.out.println(s);
        }
        System.out.println("----3rd obs to visit");
        ArrayList<String> subp3 = final_path.get(3);
        for (String s: subp3){
            System.out.println(s);
        }
        System.out.println("-----4th obs to visit");
        ArrayList<String> subp4 = final_path.get(4);
        for (String s: subp4){
            System.out.println(s);
        }
        System.out.println("----5th obs to visit");
        ArrayList<String> subp5 = final_path.get(5);
        for (String s: subp5){
            System.out.println(s);
        }
        
    	
        return final_path;
        
    }
    	
    public static void main(String[] args){
    	
        /*------------- robot start position and dir--------------*/
        Node bot = new Node(1,1);
        bot.setDir(1);

        /*---------------------- obstacles-------------------------*/
        ArrayList<Obstacle> obstacles = new ArrayList<>(1);
        HashMap<Integer, Obstacle> orderedObstacles = new HashMap<>(obstacles.size());

        
//        Obstacle o1 = new Obstacle(1, 5, 0, 1);
//        Obstacle o2 = new Obstacle(2, 9, 6, 3);
//        Obstacle o3 = new Obstacle(3, 11 ,8, 1);
//        Obstacle o4 = new Obstacle(4, 11, 13,0);
//        Obstacle o5 = new Obstacle(5, 5, 18,3);
//        Obstacle o6 = new Obstacle(6, 19, 1 ,2);
        
        String message="1,6,16,2/n2,13,11,0/n3,18,2,2/n4,2,12,0/n5,18,19,3/n6,8,6,3/n";
    	String[] obs=message.split("/n");
    	//System.out.println(obs.length);
    	System.out.println(obs[1]);
    	int size = obs.length;
        //int [] arr = new int [size];
    	for(int i=0; i<size; i++) {
            String[] str=obs[i].split(",");
            int sizeL = str.length;
            //System.out.println(sizeL);
            int [] arr = new int [sizeL];
            for(int j=0; j<sizeL; j++) {
            	System.out.println(Integer.parseInt(str[j]));
            	arr[j] = Integer.parseInt(str[j]);
               
            }
      
            
            //ArrayList<Obstacle> obstacles = new ArrayList<>(1);
//        	System.out.println(arr[0]);
//        	System.out.println(arr[1]);
//        	System.out.println(arr[2]);
//        	System.out.println(arr[3]);

            Obstacle o1 = new Obstacle(arr[0], arr[1], arr[2], arr[3]);
            obstacles.add(o1);
        	System.out.println(o1.toString());

             
                
             
            
            //System.out.println(Arrays.toString(arr));
         }


//        obstacles.add(o1);
//        obstacles.add(o2);
//        obstacles.add(o3);
//        obstacles.add(o4);
//        obstacles.add(o5);
//        obstacles.add(o6);
        /*---------------------- AMap & pathFinder instance-------------------------*/
        final int MAP_SIZE = 20;
        final AMap amap = new AMap(MAP_SIZE, obstacles);
        final AMapCell[][] map = amap.getMap();
        Map<Node, Node> IcameFrom =  new HashMap<>();
        astarPathFinder pathFinder = new astarPathFinder(MAP_SIZE, obstacles, map, IcameFrom);


        /*---------------------- EXE-------------------------*/
        ArrayList<ArrayList<String>> final_path = pathFinder.execute3(bot);
        
        System.out.println("----1st obs to visit");
        ArrayList<String> subp1 = final_path.get(1);
        ArrayList<String> subp20 = final_path.get(2);

        for (String s: subp1){
            System.out.println(s);
           // System.out.println("Hello");

        }
        //for (String s: subp20){
           // System.out.println(s);
            //System.out.println("YYYYY");

        //}
        System.out.println("----2nd obs to visit");
        ArrayList<String> subp2 = final_path.get(2);
        for (String s: subp2){
            System.out.println(s);
        }
        System.out.println("----3rd obs to visit");
        ArrayList<String> subp3 = final_path.get(3);
        for (String s: subp3){
            System.out.println(s);
        }
        System.out.println("-----4th obs to visit");
        ArrayList<String> subp4 = final_path.get(4);
        for (String s: subp4){
            System.out.println(s);
        }
        System.out.println("----5th obs to visit");
        ArrayList<String> subp5 = final_path.get(5);
        for (String s: subp5){
            System.out.println(s);
        }
       // System.out.println(Arrays.toString());
         }
    
   

	public static void executeFN(String recvedMessage) {
		// TODO Auto-generated method stub
		
	}
	        
        
}	
    	

