package astar;

public class AMapCell {
    boolean isAccessible;

    public AMapCell() {
        this.isAccessible = true;
    }

    public void setInaccessible() {
        this.isAccessible = false;
    }
    public void setaccessible() {
        this.isAccessible = true;
    }

}
