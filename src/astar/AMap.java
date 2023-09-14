package astar;

import java.util.ArrayList;

public class AMap {
    private int MAP_SIZE;
    private AMapCell[][] map;
    private ArrayList<Obstacle> obstacles;

    public AMap(int MAP_SIZE, ArrayList<Obstacle> obstacles) {
        this.MAP_SIZE = MAP_SIZE;
        this.obstacles = obstacles;
        this.map = new AMapCell[MAP_SIZE][MAP_SIZE];

        for(int i=0; i<MAP_SIZE; i++){
            for(int j=0; j<MAP_SIZE; j++){
                map[i][j] = new AMapCell();
            }
        }

        for (int i=0; i<MAP_SIZE; i++){
            map[i][0].setInaccessible();
            map[0][i].setInaccessible();
        }
        

        for(Obstacle obs:obstacles){
            if (obs.x>=0 && obs.x<20 && obs.y>=0 && obs.y<20) map[obs.x][obs.y].setInaccessible();
            if (obs.x>=0 && obs.x<20 && obs.y-1>=0 && obs.y-1<20) map[obs.x][obs.y-1].setInaccessible();
            if (obs.x>=0 && obs.x<20 && obs.y+1>=0 && obs.y+1<20) map[obs.x][obs.y+1].setInaccessible();
            if (obs.x-1>=0 && obs.x-1<20 && obs.y>=0 && obs.y<20) map[obs.x-1][obs.y].setInaccessible();
            if (obs.x+1>=0 && obs.x+1<20 && obs.y>=0 && obs.y<20) map[obs.x+1][obs.y].setInaccessible();
            if (obs.x-1>=0 && obs.x-1<20 && obs.y-1>=0 && obs.y-1<20) map[obs.x-1][obs.y-1].setInaccessible();
            if (obs.x-1>=0 && obs.x-1<20 && obs.y+1>=0 && obs.y+1<20) map[obs.x-1][obs.y+1].setInaccessible();
            if (obs.x+1>=0 && obs.x+1<20 && obs.y+1>=0 && obs.y+1<20) map[obs.x+1][obs.y+1].setInaccessible();
            if (obs.x+1>=0 && obs.x+1<20 && obs.y-1>=0 && obs.y-1<20) map[obs.x+1][obs.y-1].setInaccessible();
            
            
        }
    }

    public AMapCell[][] getMap() {
        return map;
    }
}


