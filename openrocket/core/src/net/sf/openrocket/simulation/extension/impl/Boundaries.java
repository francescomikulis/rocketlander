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
        Limits(double minIn ,double maxIn){
            min = minIn;
            max = maxIn;
        }
    }
    public Limits t = new Limits(0,6);
    public Limits x = new Limits(-2,2);
    public Limits y = new Limits(-2,2);
    public Limits z = new Limits(-10,5);
    public Limits vx = new Limits(-1,1);
    public Limits vy = new Limits(-1,1);
    public Limits vz = new Limits(-20,0);
    public Limits ax = new Limits(-.3, .3);
    public Limits ay = new Limits(-.3, .3);
    public Limits az = new Limits(.92, 1);
    public Limits avx = new Limits(-.3, .3);
    public Limits avy = new Limits(-.3, .3);
    public Limits avz = new Limits(0, 0);
    public Limits thrust = new Limits(0, 1);
    public Limits gimbleX = new Limits(-5*Math.PI/180, 5*Math.PI/180);
    public Limits gimbleY = new Limits(-5*Math.PI/180, 5*Math.PI/180);
    Boundaries(String goalInidactor){
        x = new Limits(-3,3);
        y = new Limits(-3,3);
        z = new Limits(0,1);
        vx = new Limits(-1,1);
        vy = new Limits(-1,1);
        vz = new Limits(-1,1);
        az = new Limits(.7, 1);
        avx = new Limits(-.3, .3);
        avy = new Limits(-.3, .3);
        t = new Limits(2,6);
    }
}
