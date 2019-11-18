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
    private static ArrayList<HashMap<String, ArrayList<Double>>> episodesData = null;
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
        Double totalPropellantMass = status.getMotors().iterator().next().getPropellantMass();

        // does not actually work.  This only fucks things up.
        // status.getMotors().iterator().next().getConfig().setIgnitionDelay(2);
    }

    public ArrayList<StateActionTuple> initializeEmptyActionStateTuples() {
        return new ArrayList<>();
    }

    public void storeActionValueFunction() {
        System.out.println("CLOSING!!!");
        initializeEpisodeManager();
        if (model.getValueFunctionTable().size() != 0) {
            if (model.simulationType==SimulationType._1D) {
                mof.storeActionValueFunction(model.getValueFunctionTable(),"1D.txt");
            } else if (model.simulationType==SimulationType._3D){
                mof.storeActionValueFunction(model.getValueFunctionTable(),"3D.txt");
            }
        }
    }

    public void printAll() {
        initializeEpisodeManager();
        if (model.getValueFunctionTable() == null) return;
        for (Map.Entry<StateActionTuple, Double> entry: model.getValueFunctionTable().entrySet()) {
            StateActionTuple stateActionTuple = entry.getKey();
            double value = entry.getValue();
            System.out.println(stateActionTuple + ": " + value);
        }
    }

    /* SAFE INITIALIZERS */

    public void safeActionValueFunctionInitialization() {
        if (model.getValueFunctionTable() == null) {
            try {
                if (model.simulationType==SimulationType._1D) {
                    model.setValueFunctionTable(mof.readActionValueFunction("1D.txt"));
                } else if (model.simulationType==SimulationType._3D) {
                    model.setValueFunctionTable(mof.readActionValueFunction("3D.txt"));
                }
            } catch (Exception e) {
                model.resetValueFunctionTable();
            }
        }
    }

    private void safeEpisodesDataInitialization() {
        if (episodesData == null) {
            try {
                episodesData =  mof.readEpisodesData();
            } catch (Exception e) {
                episodesData = new ArrayList<>();
            }
        }
    }

    public static HashMap<String, ArrayList<Double>> initializeEmptyEpisode() {
        HashMap<String, ArrayList<Double>> episode = new HashMap<>();
        // run through all keys of dataKeys
        for (String key: dataKeys) {
            String type = getKeyField(key);
            ArrayList<String> components = getComponents(key);
            if (components.size() == 1) {
                episode.put(type, new ArrayList<>());
            } else {
                for (String component: components) {
                    episode.put(type + "_" + component, new ArrayList<>());
                }
            }
        }
        return episode;
    }

    public void addAndStoreEpisode(HashMap<String, ArrayList<Double>> episode) {
        episodesData.add(episode);
        mof.storeEpisodesData(episodesData);
    }

    /*
    Get components from status and add them to a HashMap episode -- NO NEED TO MODIFY.
     */


    public static Integer getMaxTimestepOfEpisode(HashMap<String, ArrayList<Double>> episode) {
        return episode.get("position_x").size();
    }

    // read the last timeStep that was added to an episode
    public HashMap<String, Double> readLastTimeStep(HashMap<String, ArrayList<Double>> episode) {
        return readTimeStep(episode, 0);
    }

    // read timeSteps that occurred before n time steps before the last of an episode
    public HashMap<String, Double> readTimeStep(HashMap<String, ArrayList<Double>> episode, int delta) {
        HashMap<String, Double> lastTimeStep = new HashMap<>();
        for (String key: dataKeys) {
            String type = getKeyField(key);
            ArrayList<String> components = getComponents(key);
            for (String component: components) {
                String compositeType = type;
                if (components.size() != 1) compositeType += "_" + component;
                assert (delta >= 0);
                int upperBound = episode.get(compositeType).size();
                int index = upperBound - delta - 1;
                index = Math.max(0, index);
                if (index < upperBound)
                    lastTimeStep.put(compositeType, episode.get(compositeType).get(index));
            }
        }
        return lastTimeStep;
    }

    /* ADD DATA DYNAMICALLY */

    public static void addData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode) {
        for (String key: dataKeys) {
            String type = getKeyField(key);
            ArrayList<String> components = getComponents(key);
            int numComponents = components.size();
            switch (numComponents) {
                case 1: add1ComponentData(status, episode, type); break;
                case 3: add3ComponentData(status, episode, type); break;
                case 4: addQuaternionData(status, episode, type); break;
            }
        }
    }

    private static void add1ComponentData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode, String type) {
        double data = 0.0;
        if (type.equals("thrust")) data = status.getMotors().iterator().next().getThrust(status.getSimulationTime());
        episode.get(type).add(data);
    }

    private static void add3ComponentData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode, String type) {
        Coordinate coordinate = new Coordinate(0, 0, 0 );
        if (type.equals("position")) coordinate = status.getRocketPosition();
        if (type.equals("velocity")) coordinate = status.getRocketVelocity();
        if (type.equals("rotationV")) coordinate = status.getRocketRotationVelocity();
        episode.get(type + "_x").add(coordinate.x);
        episode.get(type + "_y").add(coordinate.y);
        episode.get(type + "_z").add(coordinate.z);
    }

    public static void addQuaternionData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode, String type) {
        Quaternion quaternion = new Quaternion(0, 0, 0, 0);
        if (type.equals("orientation_quat")) quaternion = status.getRocketOrientationQuaternion();
        episode.get(type + "_w").add(quaternion.getW());
        episode.get(type + "_x").add(quaternion.getX());
        episode.get(type + "_y").add(quaternion.getY());
        episode.get(type + "_z").add(quaternion.getZ());
    }

    private static boolean isSingleComponent(String key) {
        return key.equals(getKeyField(key));
    }

    private static String getKeyField(String key) {
        if (!key.contains("_")) return key;
        return key.substring(0, key.lastIndexOf("_"));
    }

    private static ArrayList<String> getComponents(String key) {
        ArrayList<String> components = new ArrayList<String>();
        if (isSingleComponent(key)) {
            // if no components present (no _); then use the key itself
            components.add("");
        } else {
            String fields = key.substring(key.lastIndexOf("_") + 1);
            // loop for each component and initialize the HashMap
            for (Character letter: fields.toCharArray()) {
                components.add(letter.toString());
            }
        }
        return components;
    }

    /*
    Getters -- DO NOT MODIFY BELOW
     */

    public ArrayList<ArrayList<StateActionTuple>> getEpisodesStateAction() {
        return episodesStateAction;
    }

    public ArrayList<HashMap<String, ArrayList<Double>>> getEpisodesData() {
        return episodesData;
    }

    private static final ArrayList<String> dataKeys = new ArrayList<String>() {
        {
            //add("thrust");
            add("position_xyz");
            //add("velocity_xyz");
            //add("rotationV_xyz");
            add("orientation_quat_wxyz");
        }
    };

    public static int serialize_length() {
        return 7 * 9 + 1;
    }

    public static byte[] serialize_single_timestep(HashMap<String, ArrayList<Double>> episode) {
        byte[] bytes = new byte[serialize_length()];
        int offset = 0;
        int length = 8;

        for (Map.Entry<String, ArrayList<Double>> entry: episode.entrySet()) {
            String dataType = entry.getKey();
            char encoded_key = dataType.charAt(dataType.length() - 1);
            if (dataType.contains("orientation")) {
                encoded_key += (int)('a' - 'A');  // offset
            }

            bytes[offset] = (byte) encoded_key;

            byte[] doubleByte = new byte[8];
            ByteBuffer.wrap(doubleByte).putDouble(entry.getValue().get(0));
            System.arraycopy(doubleByte, 0, bytes, offset + 1, 8);

            offset += 9;
        }
        bytes[63] = '*';
        return bytes;
    }
}
