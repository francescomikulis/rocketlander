package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.ArrayList;
import net.sf.openrocket.util.Coordinate;

public class Boundaries {
    public Boundaries() {

    }

    class Limits{
        public double min;
        public double max;
        public double increment;
        Limits(double minIn ,double maxIn){
            min = minIn;
            max = maxIn;
        }
        Limits(double minIn ,double maxIn, double increment){
            min = minIn;
            max = maxIn;
            this.increment = increment;
        }
    }
    public Limits t = new Limits(0,6);
    public Limits x = new Limits(-20,20);
    public Limits y = new Limits(-20,20);
    public Limits z = new Limits(-1,30);
    public Limits vx = new Limits(-3,3);
    public Limits vy = new Limits(-3,3);
    public Limits vz = new Limits(-15,-.1);
    public Limits ax = new Limits(-.3, .3);
    public Limits ay = new Limits(-.3, .3);
    public Limits az = new Limits(0.99, 1);
    public Limits avx = new Limits(-.3, .3);
    public Limits avy = new Limits(-.3, .3);
    public Limits avz = new Limits(0, 0);
    public Limits thrust = new Limits(0, 1, 0.25);
    public Limits gimbleX = new Limits(-3*Math.PI/180, 3*Math.PI/180, Math.PI/180);
    public Limits gimbleY = new Limits(-3*Math.PI/180, 3*Math.PI/180, Math.PI/180);
    public Limits lateralThrustX = new Limits(-.25, .25, .25);
    public Limits lateralThrustY = new Limits(-.25, .25, .25);
    Boundaries(String goalInidactor){
        x = new Limits(-40,40);
        y = new Limits(-40,40);
        z = new Limits(-0.5,0.5);
        vx = new Limits(-0.5,0.5);
        vy = new Limits(-0.5,0.5);
        vz = new Limits(-0.5,0.5);
        ax = new Limits(-0.0698, 0.0698);
        ay = new Limits(-0.0698, 0.0698);
        az = new Limits(0.99, 1);
        avx = new Limits(-.3, .3);
        avy = new Limits(-.3, .3);
        t = new Limits(2,6);
    }
}
