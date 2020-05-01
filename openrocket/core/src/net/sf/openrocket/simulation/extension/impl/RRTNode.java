package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;

import static net.sf.openrocket.simulation.extension.impl.RRT.copy;

public class RRTNode {
    public SimulationStatus status = null;
    public RRTNode parent = null;
    public Coordinate directionZ = null;
    public RRT.Action action = null;

    RRTNode(SimulationStatus statusIn, RRTNode parentIn){
        statusIn = copy(statusIn);
        status = statusIn;
        parent = parentIn;
        action = new RRT.Action(0,0,0);
        Quaternion q = status.getRocketOrientationQuaternion();
        Coordinate directionZ = new Coordinate(0,0,1);
        directionZ = q.rotate(directionZ);
        this.directionZ = directionZ;
    }
}
