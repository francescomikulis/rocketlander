package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.rocketcomponent.FlightConfiguration;
import net.sf.openrocket.simulation.FlightDataBranch;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.ArrayList;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;

import static net.sf.openrocket.simulation.extension.impl.Boundaries.strictlyWithinBoundary;

import java.util.Random;

public class RRT {
    private RRTNode root = null;
    private ArrayList<RRTNode> nodes = null;
    private RRTNode target = null;
    public RRTNode current = null;
    private ArrayList<RRTNode> options = null;
    private Boundaries boundaries = new Boundaries();
    private Boundaries goal = new Boundaries("goal", true);
    int counter = 0;
    double globalMin = Double.MAX_VALUE;
    public double globalMax;
    public double originalGlobalMax;
    RRTNode minNode;
    int tries=10;
    int NUM_GOAL_TRIES = 100;
    Action a;
    public int numNodesExpanded = 0;
    private boolean isUsingLateralVelocityObjective = true;

    public void setIsUsingLateralVelocityObjective(boolean newValue) {
        isUsingLateralVelocityObjective = newValue;
        goal = new Boundaries("goal", isUsingLateralVelocityObjective);
    }

    RRT(RRTNode rootIn){
        root = rootIn;
        nodes = new ArrayList<>();
        options = new ArrayList<>();
        SimulationStatus s = copy(root.status);

        setOriginalGlobalMax(s);

        target = new RRTNode(s,null);
        nodes.add(root);
        minNode = root;
    }

    public void setOriginalGlobalMax(SimulationStatus s) {
        target = new RRTNode(s,null);
        setGoalTarget();

        double maxDist = distance(root, target);
        originalGlobalMax = maxDist;
        globalMax = maxDist;

        target = new RRTNode(s,null);
    }

    public static SimulationStatus copy(SimulationStatus status){
        //SimulationConditions sc = new SimulationConditions();
        // SimulationStatus s = new SimulationStatus(status.getConfiguration().clone(), status.getSimulationConditions().clone());
        SimulationStatus s = status.hackyCopy();
        assignStatus(status,s);
       // FlightConfiguration simulationConfig = simulationConditions.getRocket().getFlightConfiguration( this.fcid).clone();
        //final String branchName = simulationConfig.getRocket().getTopmostStage().getName();
        s.setFlightData(new FlightDataBranch("Sustainer", FlightDataType.TYPE_TIME));
        return s;
    }

    boolean checkGoal(RRTNode n){
        boolean angleX = strictlyWithinBoundary(goal.ax, n.directionZ.x);
        boolean angleY = strictlyWithinBoundary(goal.ay, n.directionZ.y);
        boolean angleZ = strictlyWithinBoundary(goal.az, n.directionZ.z);
        boolean velX = strictlyWithinBoundary(goal.vx, n.status.getRocketVelocity().x);
        boolean velY = strictlyWithinBoundary(goal.vy, n.status.getRocketVelocity().y);
        boolean velZ = strictlyWithinBoundary(goal.vz, n.status.getRocketVelocity().z);
        boolean angleVelX = strictlyWithinBoundary(goal.avx, n.status.getRocketRotationVelocity().x);
        boolean angleVelY =  strictlyWithinBoundary(goal.avy, n.status.getRocketRotationVelocity().y);
        boolean angleVelZ =  strictlyWithinBoundary(goal.avz, n.status.getRocketRotationVelocity().z);
        boolean posX =  strictlyWithinBoundary(goal.x, n.status.getRocketPosition().x);
        boolean posY =  strictlyWithinBoundary(goal.y, n.status.getRocketPosition().y);
        boolean posZ =  strictlyWithinBoundary(goal.z, n.status.getRocketPosition().z);

        boolean angleVerticalVelocityPositionGoal = angleX && angleY && velZ && posZ;
        if (isUsingLateralVelocityObjective)
            return angleVerticalVelocityPositionGoal && velX && velY;
        return angleVerticalVelocityPositionGoal;
    }

    void setGoalTarget(){
        Coordinate coord = target.status.getRocketPosition();
        coord = coord.setX(getRandom(goal.x));
        coord =coord.setY(getRandom(goal.y));
        coord =coord.setZ(getRandom(goal.z));
        target.status.setRocketPosition(coord);

        coord = target.status.getRocketVelocity();
        coord =coord.setX(getRandom(goal.vx));
        coord =coord.setY(getRandom(goal.vy));
        coord =coord.setZ(getRandom(goal.vz));
        target.status.setRocketVelocity(coord);

        coord = target.status.getRocketRotationVelocity();
        coord =coord.setX(getRandom(goal.avx));
        coord =coord.setY(getRandom(goal.avy));
        coord =coord.setZ(getRandom(goal.avz));
        target.status.setRocketRotationVelocity(coord);

        coord = target.directionZ;
        coord =coord.setX(getRandom(goal.ax));
        coord =coord.setY(getRandom(goal.ay));
        coord =coord.setZ(getRandom(goal.az));
        target.directionZ = coord;

        target.status.setSimulationTime(getMean(goal.t));
    }

    void setRandomTarget(){
        Coordinate coord = target.status.getRocketPosition();
        coord = coord.setX(getRandom(boundaries.x)+0*minNode.status.getRocketPosition().x);
        coord =coord.setY(getRandom(boundaries.y)+0*minNode.status.getRocketPosition().y);
        coord = coord.setZ(getRandom(boundaries.z)+0*minNode.status.getRocketPosition().z);
        target.status.setRocketPosition(coord);

        coord = target.status.getRocketVelocity();
        coord =coord.setX(getRandom(boundaries.vx));
        coord =coord.setY(getRandom(boundaries.vy));
        coord =coord.setZ(getRandom(boundaries.vz));
        target.status.setRocketVelocity(coord);

        coord = target.status.getRocketRotationVelocity();
        coord =coord.setX(getRandom(boundaries.avx));
        coord =coord.setY(getRandom(boundaries.avy));
        coord =coord.setZ(getRandom(boundaries.avz));
        target.status.setRocketRotationVelocity(coord);

        coord = target.directionZ;
        coord =coord.setX(getRandom(boundaries.ax));
        coord =coord.setY(getRandom(boundaries.ay));
        coord =coord.setZ(getRandom(boundaries.az));
        target.directionZ = coord;
        //target.status.setRocketPosition(coord);
        target.status.setSimulationTime(getRandom(boundaries.t));
    }

    double getRandom(Boundaries.Limits limits){
        return limits.min + Math.random() * (limits.max - limits.min);
    }

    double getRandomAction(Boundaries.Limits limits){
        int range = (int) Math.round((limits.max - limits.min) / limits.increment + 0.5);
        int incrementMultipler = (int) Math.round(Math.random() * range);
        return limits.min + incrementMultipler * (limits.increment);
    }

    double getMean(Boundaries.Limits limits){
        return 0.5* (limits.max + limits.min);
    }

    public static void assignStatus(SimulationStatus cur,SimulationStatus status){
        status.setRocketPosition(cur.getRocketPosition());
        status.setRocketVelocity(cur.getRocketVelocity());
        status.setRocketOrientationQuaternion(cur.getRocketOrientationQuaternion());
        status.setRocketRotationVelocity(cur.getRocketRotationVelocity());
        status.setSimulationTime(cur.getSimulationTime());
    }

    Action setStatus(SimulationStatus status){
        if(counter == tries ){ //pick best
            RRTNode n = getNearest(options, target);
            if (n!=null)
                addNode(n); // add the node which is closest to the target
            if (nodes.size()>5000){
                RRTNode tmp = root;
                nodes = new ArrayList<>();
                nodes.add(tmp);
                globalMin = 99999;
                globalMax = originalGlobalMax;
            }
          //  if (Math.random() < 0.5) {
           //     Random random = new Random();
            //    int d = Math.abs(random.nextInt()) % nodes.size();
             //   nodes.remove(d);
           // }
            options = new ArrayList<>(); //reset list
            current = null;
            counter = 0;
        }
        if (current == null){ // need to  sample new point

            if (Math.random()< 0.1){
                tries = NUM_GOAL_TRIES;
                setGoalTarget();
                RRTNode tmp = getNearest(nodes, target);

            } else {
                tries = 50;
                setRandomTarget();
            }
            current = getNearest(nodes, target);
            if (checkGoal(current)){ return null; }
        } else {
            counter++;
            if (status.getRocketPosition().z != 0.001) { //only add valid options
                RRTNode n = new RRTNode(status, current);
                n.action = a;
                options.add(n);
            }
        }
        assignStatus(current.status, status);
        a = generateRandomDiscreteAction();
        return a;
    }

    private Action generateRandomDiscreteAction() {
        double gimbleX = getRandomAction(boundaries.gimbleX);
        double gimbleY = getRandomAction(boundaries.gimbleY);
        double thrust = getRandomAction(boundaries.thrust);
        double lateralThrustX = 0;
        double lateralThrustY = 0;
        if (isUsingLateralVelocityObjective && (Math.random() < 0.5)) {
            gimbleX = 0; gimbleY = 0;
            lateralThrustX = getRandomAction(boundaries.lateralThrustX);
            lateralThrustY = getRandomAction(boundaries.lateralThrustY);
        }
        return new Action(gimbleX, gimbleY, thrust, lateralThrustX, lateralThrustY);
    }

    void addNode(RRTNode node){
        nodes.add(node);
        numNodesExpanded++;
    }

    RRTNode getNearest(ArrayList<RRTNode> nodeList, RRTNode node){
        double min = Double.MAX_VALUE;
        RRTNode ret = null;
        for (RRTNode rrtNode : nodeList) {
            double tmp = distance(node, rrtNode);
            if (tmp < min) {
                min = tmp;
                ret = rrtNode;
                if (min < globalMin && tries==NUM_GOAL_TRIES){
                    globalMin = min;
                    minNode = rrtNode;
                    /*
                    System.out.println(globalMin);
                    System.out.println("Position: x: "+minNode.status.getRocketPosition().x+
                            " y: "+minNode.status.getRocketPosition().y+
                            " z: "+minNode.status.getRocketPosition().z);
                    System.out.println("Velocity: x: "+minNode.status.getRocketVelocity().x+
                            " y: "+minNode.status.getRocketVelocity().y+
                            " z: "+minNode.status.getRocketVelocity().z);
                    System.out.println("Angle: x: "+minNode.status.getRocketOrientationQuaternion().rotateZ().x+
                            " y: "+minNode.status.getRocketOrientationQuaternion().rotateZ().y+
                            " z: "+minNode.status.getRocketOrientationQuaternion().rotateZ().z);
                     */
                    System.out.println((1.0 - globalMin/globalMax) * 100 + "%");
                }

            }
        }
      //  System.out.println(min);
        return ret;
    }

    double distance(RRTNode n1,RRTNode n2){
        Coordinate cord1 = n1.status.getRocketPosition();
        if (n2==null || n2.status==null){
            n2 = null;
        }
        Coordinate cord2 = n2.status.getRocketPosition();
        double dx = cord1.x - cord2.x;
        double dy = cord1.y - cord2.y;
        double dz = cord1.z - cord2.z;
        cord1 = n1.status.getRocketVelocity();
        cord2 = n2.status.getRocketVelocity();
        double dvx = cord1.x - cord2.x;
        double dvy = cord1.y - cord2.y;
        double dvz = cord1.z - cord2.z;
        cord1 = n1.directionZ;
        cord2 = n2.directionZ;
        double dax = cord1.x - cord2.x;
        double day = cord1.y - cord2.y;
        double daz = cord1.z - cord2.z;
        double dt = n1.status.getSimulationTime()-n2.status.getSimulationTime();

        double baseDistance = 5*dz*dz + 5*dvz*dvz + dax*dax*1000 + day*day*1000;
        if (isUsingLateralVelocityObjective) {
            baseDistance += 2*dvx*dvx + 2*dvy*dvy;
        }
        return baseDistance;
    }

    public static class Action{
        double thrust;
        double gimbleX;
        double gimbleY;
        double lateralThrustX;
        double lateralThrustY;
        Action(double gimbleX, double gimbleY, double thrust, double lateralThrustX, double lateralThrustY){
            this.thrust = thrust;
            this.gimbleX = gimbleX;
            this.gimbleY = gimbleY;
            this.lateralThrustX = lateralThrustX;
            this.lateralThrustY = lateralThrustY;
        }
    }
}
