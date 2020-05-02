package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.rocketcomponent.FlightConfiguration;
import net.sf.openrocket.simulation.FlightDataBranch;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.ArrayList;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;

import java.util.Random;

public class RRT {
    private RRTNode root = null;
    private ArrayList<RRTNode> nodes = null;
    private RRTNode target = null;
    public RRTNode current = null;
    private ArrayList<RRTNode> options = null;
    private Boundaries boundaries = new Boundaries();
    private Boundaries goal = new Boundaries("goal");
    int counter = 0;
    double globalMin = 1000;
    RRTNode minNode;
    int tries=10;
    Action a;

    RRT(RRTNode rootIn){
        root = rootIn;
        nodes = new ArrayList<>();
        options = new ArrayList<>();
        SimulationStatus s = copy(root.status);
        target = new RRTNode(s,null);
        nodes.add(root);
        minNode = root;
    }
    public static SimulationStatus copy(SimulationStatus status){
        //SimulationConditions sc = new SimulationConditions();
        SimulationStatus s = new SimulationStatus(status.getConfiguration().clone(), status.getSimulationConditions().clone());
        assignStatus(status,s);
       // FlightConfiguration simulationConfig = simulationConditions.getRocket().getFlightConfiguration( this.fcid).clone();
        //final String branchName = simulationConfig.getRocket().getTopmostStage().getName();
        s.setFlightData(new FlightDataBranch("Sustainer", FlightDataType.TYPE_TIME));
        return s;
    }

    boolean checkGoal(RRTNode n){
        boolean b1 =                  n.directionZ.z > goal.az.min            &&                         n.directionZ.z < goal.az.max;
        boolean b2 =  n.status.getRocketVelocity().x > goal.vx.min            &&         n.status.getRocketVelocity().x < goal.vx.max;
        boolean b3 =  n.status.getRocketVelocity().y > goal.vy.min            &&         n.status.getRocketVelocity().y < goal.vy.max;
        boolean b4 =  n.status.getRocketVelocity().z > goal.vz.min            &&         n.status.getRocketVelocity().z < goal.vz.max;
        boolean b5 =  n.status.getRocketRotationVelocity().x > goal.avx.min   && n.status.getRocketRotationVelocity().x < goal.avx.max;
        boolean b6 =  n.status.getRocketRotationVelocity().y > goal.avy.min   && n.status.getRocketRotationVelocity().y < goal.avy.max;
        boolean b7 =  n.status.getRocketRotationVelocity().z > goal.avz.min   && n.status.getRocketRotationVelocity().z < goal.avz.max;
        boolean b8 =  n.status.getRocketPosition().x > goal.x.min             &&         n.status.getRocketPosition().x < goal.x.max;
        boolean b9 =  n.status.getRocketPosition().y > goal.y.min             &&         n.status.getRocketPosition().y < goal.y.max;
        boolean b10 =  n.status.getRocketPosition().z > goal.z.min            &&         n.status.getRocketPosition().z < goal.z.max;
        //return (b1 && b2 && b3 && b4 && b5 && b6 && b7 && b8 && b9 && b10);
        return (b10 && b4 && b2 && b3);
    }

    void setGoalTarget(){
        Coordinate coord = target.status.getRocketPosition();
        coord = coord.setX(getMean(goal.x));
        coord =coord.setY(getMean(goal.y));
        coord =coord.setZ(0);
        target.status.setRocketPosition(coord);

        coord = target.status.getRocketVelocity();
        coord =coord.setX(getMean(goal.vx));
        coord =coord.setY(getMean(goal.vy));
        coord =coord.setZ(-.5);
        target.status.setRocketVelocity(coord);

        coord = target.status.getRocketRotationVelocity();
        coord =coord.setX(getMean(goal.avx));
        coord =coord.setY(getMean(goal.avy));
        coord =coord.setZ(getMean(goal.avz));
        target.status.setRocketRotationVelocity(coord);

        coord = target.directionZ;
        coord =coord.setX(0);
        coord =coord.setY(0);
        coord =coord.setZ(1);
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
            if (nodes.size()>15000){
                RRTNode tmp = root;
                nodes = new ArrayList<>();
                nodes.add(tmp);
                globalMin = 99999;
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

            if (Math.random()< 0.01){
                tries = 1001;
                setGoalTarget();
                RRTNode tmp = getNearest(nodes, target);

            } else {
                tries = 10;
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
        a = new Action(getRandom(boundaries.gimbleX),
                    getRandom(boundaries.gimbleY),
                    getRandom(boundaries.thrust));
        return a;
    }

    void addNode(RRTNode node){
        nodes.add(node);
    }

    RRTNode getNearest(ArrayList<RRTNode> nodeList, RRTNode node){
        double min = Double.MAX_VALUE;
        RRTNode ret = null;
        for (RRTNode rrtNode : nodeList) {
            double tmp = distance(node, rrtNode);
            if (tmp < min) {
                min = tmp;
                ret = rrtNode;
                if (min < globalMin && tries==1001){
                    globalMin = min;
                    minNode = rrtNode;
                    System.out.println(globalMin);
                    System.out.println("Position: x: "+minNode.status.getRocketPosition().x+
                            " y: "+minNode.status.getRocketPosition().y+
                            " z: "+minNode.status.getRocketPosition().z);
                    System.out.println("Velocity: x: "+minNode.status.getRocketVelocity().x+
                            " y: "+minNode.status.getRocketVelocity().y+
                            " z: "+minNode.status.getRocketVelocity().z);
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
        return  2*dz*dz + 1*dvz*dvz + 1*dvx*dvx + 1*dvy*dvy + daz*daz*10;// + dax*dax + day*day + daz*daz+dt*dt;+ dvx*dvx + dvy*dvy + 10*dx*dx +10*dy*dy
    }

    public static class Action{
        double gimbleX;
        double gimbleY;
        double thrust;
        Action(double gimbleX, double gimbleY, double thrust){
            this.thrust = thrust;
            this.gimbleX = gimbleX;
            this.gimbleY = gimbleY;
        }
    }
}
