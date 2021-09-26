package org.firstinspires.ftc.teamcode.geometry;


import java.util.ArrayList;

/**
 * path for following complex movements easily
 */
public class path {

    // false unless the path is completed.  This is to be set by the path following algorithm when the final point in the path has reach an acceptable steady state
    public boolean pathHasBeenCompleted = false;

    protected ArrayList<pathVector3D> pathPositions = new ArrayList<>();

    /**
     * blank constructor pog moment
     */
    public path() {

    }

    /**
     * contructor with path
     * @param pathPositions the arraylist of paths lol
     */
    public path(ArrayList<pathVector3D> pathPositions) {
        this.pathPositions = pathPositions;
    }

    /**
     * appends a new pathPosition to the path
     * @param pos new pathPosition
     */
    public void addPosition(pathVector3D pos) {
        pathPositions.add(pos);
    }

    /**
     * appends a new position to the path as a pathPosition
     * @param pos new position
     */
    public void addPosition(Vector3D pos) {
        pathPositions.add(new pathVector3D(pos));
    }

    /**
     * gets the entire list of positions
     * @return pathPositions
     */
    public ArrayList<pathVector3D> getAllPathPositions() {
        return pathPositions;
    }

    /**
     * get pathPosition at a given index
     * @param index the index we want starting from 0
     * @return
     */
    public pathVector3D getPathPosition(int index) {
        return pathPositions.get(index);
    }


    /**
     * gets the first position of the path
      * @return the first pathPosition
     */
    public pathVector3D getFirstPosition() {
        return getPathPosition(0);
    }

    /**
     * gets the last position of the path
     * @return the last pathPosition
     */
    public pathVector3D getLastPosition() {
        return pathPositions.get(pathPositions.size() - 1);
    }

    public long pathSize() {
        return pathPositions.size();
    }
}
