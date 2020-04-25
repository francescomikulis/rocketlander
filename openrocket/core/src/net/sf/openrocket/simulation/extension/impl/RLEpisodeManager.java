package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import net.sf.openrocket.simulation.extension.impl.RLModel.*;

public class RLEpisodeManager {
    private static ArrayList<HashMap<String, ArrayList<Float>>> episodesData = null;
    private static ArrayList<ArrayList<StateActionTuple>> episodesStateAction = null;

    private RLModel model = null;
    private RLObjectFileStore mof = null;

    private static volatile RLEpisodeManager instance;

    private RLEpisodeManager() {}

    public static RLEpisodeManager getInstance() {
        if (instance == null) { // first time lock
            synchronized (RLEpisodeManager.class) {
                if (instance == null) {  // second time lock
                    instance = new RLEpisodeManager();
                }
            }
        }
        return instance;
    }

    public void initializeEpisodeManager() {
        model = RLModel.getInstance();
        mof = RLObjectFileStore.getInstance();
    }

    public void setupParameters(SimulationStatus status) {
        // Double totalPropellantMass = status.getMotors().iterator().next().getPropellantMass();

        // does not actually work.  This only fucks things up.
        // status.getMotors().iterator().next().getConfig().setIgnitionDelay(2);
    }

    public ArrayList<StateActionTuple> initializeEmptyActionStateTuples() {
        return new ArrayList<>();
    }

    public void storeActionValueFunction() {
        System.out.println("SAVING ACTION VALUE FUNCTION!!!");
        initializeEpisodeManager();
        mof.storeActionValueFunctions(model.getValueFunctionTable());
    }

    /* SAFE INITIALIZERS */

    public void safeActionValueFunctionInitialization() {
        if (model.getValueFunctionTable() == null) {
            model.setValueFunctionTable(mof.readActionValueFunctionFromMethods(model.getMethods()));
        }
    }

    public String selectFileNameExtension() {
        return ".txt";
        /*
        String filenameExtension = "1D.txt";
        if (model.simulationType == SimulationType._1D) {
            filenameExtension = "1D.txt";
        } else if (model.simulationType == SimulationType._2D) {
            filenameExtension = "2D.txt";
        }
        else if (model.simulationType == SimulationType._3D) {
            filenameExtension = "3D.txt";
        }
        return filenameExtension;
         */
    }
}
