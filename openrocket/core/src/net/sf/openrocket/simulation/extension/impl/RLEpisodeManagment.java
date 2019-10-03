package net.sf.openrocket.simulation.extension.impl;

import com.sun.tools.javac.util.List;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;

import java.util.ArrayList;
import java.util.HashMap;

public class RLEpisodeManagment {
    public static ArrayList<HashMap<String, ArrayList<Double>>> episodesData = null;
    public static HashMap<StateActionTuple, Double> valueFunctionTable = null;
    RLMyObjectFileStore mof = new RLMyObjectFileStore();

    public RLEpisodeManagment() {
        if (episodesData == null) {
            try {
                ArrayList<HashMap<String, ArrayList<Double>>> fromFile = mof.readEpisodesData();
                episodesData = fromFile;
            } catch (Exception e) {
                episodesData = new ArrayList<>();
            }
        }
        if (valueFunctionTable == null) {
            try {
                HashMap<StateActionTuple, Double> fromFile = mof.readActionValueFunction();
                valueFunctionTable = fromFile;
            } catch (Exception e) {
                valueFunctionTable = new HashMap<>();
            }
        }
    }

    public static ArrayList<HashMap<String, ArrayList<Double>>> getEpisodesData() {
        return episodesData;
    }

    public static HashMap<StateActionTuple, Double> getValueFunctionTable() {
        return valueFunctionTable;
    }

    public void setupParameters(SimulationStatus status) {
        Double totalPropellantMass = status.getMotors().iterator().next().getPropellantMass();

        // does not actually work.  This only fucks things up.
        // status.getMotors().iterator().next().getConfig().setIgnitionDelay(2);
    }

    public HashMap<String, ArrayList<Double>> initializeEmptyEpisode() {
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




    public void addData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode) {
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

    private void add1ComponentData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode, String type) {
        double data = 0.0;
        if (type.equals("thrust")) data = status.getMotors().iterator().next().getThrust(status.getSimulationTime());
        episode.get(type).add(data);
    }

    private void add3ComponentData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode, String type) {
        Coordinate coordinate = new Coordinate(0, 0, 0 );
        if (type.equals("position")) coordinate = status.getRocketPosition();
        if (type.equals("velocity")) coordinate = status.getRocketVelocity();
        if (type.equals("rotationV")) coordinate = status.getRocketRotationVelocity();
        episode.get(type + "_x").add(coordinate.x);
        episode.get(type + "_y").add(coordinate.y);
        episode.get(type + "_z").add(coordinate.z);
    }

    public void addQuaternionData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode, String type) {
        Quaternion quaternion = new Quaternion(0, 0, 0, 0);
        if (type.equals("orientation_quat")) quaternion = status.getRocketOrientationQuaternion();
        episode.get(type + "_w").add(quaternion.getW());
        episode.get(type + "_x").add(quaternion.getX());
        episode.get(type + "_y").add(quaternion.getY());
        episode.get(type + "_z").add(quaternion.getZ());
    }

    public void addEpisode(HashMap<String, ArrayList<Double>> episode) {
        episodesData.add(episode);

        // NOTE: HERE THE LOOP WAS DISABLED.
        //if (episodesData.size() % 5 == 0) {
            mof.storeEpisodesData(episodesData);
            mof.storeActionValueFunction(valueFunctionTable);
        //}

        ArrayList<Double> velocities =  episode.get("velocity_z");
        System.out.println("Sim number: " + episodesData.size() + " " + velocities.get(velocities.size()-1));
    }

    private boolean isSingleComponent(String key) {
        return key.equals(getKeyField(key));
    }

    private String getKeyField(String key) {
        if (!key.contains("_")) return key;
        return key.substring(0, key.lastIndexOf("_"));
    }

    private ArrayList<String> getComponents(String key) {
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

    private static final ArrayList<String> dataKeys = new ArrayList<>(
            List.of("thrust", "position_xyz", "velocity_xyz", "rotationV_xyz", "orientation_quat_wxyz")
    );
}
