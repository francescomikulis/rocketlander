package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;

import java.util.ArrayList;
import java.util.HashMap;

public class EpisodeManagment {
    public static ArrayList<HashMap<String, ArrayList<Double>>> episodes = null;
    MyObjectFileStore mof = new MyObjectFileStore();

    public EpisodeManagment() {
        if (episodes == null) {
            try {
                ArrayList<HashMap<String, ArrayList<Double>>> fromFile = mof.readEpisodes();
                episodes = fromFile;
            } catch (Exception e) {
                episodes = new ArrayList<>();
            }
        }
    }

    public HashMap<String, ArrayList<Double>> initializeEmptyEpisode() {
        HashMap<String, ArrayList<Double>> episode = new HashMap<>();
        episode.put("position_x", new ArrayList<>());
        episode.put("position_y", new ArrayList<>());
        episode.put("position_z", new ArrayList<>());
        episode.put("velocity_x", new ArrayList<>());
        episode.put("velocity_y", new ArrayList<>());
        episode.put("velocity_z", new ArrayList<>());
        episode.put("quat_x", new ArrayList<>());
        episode.put("quat_y", new ArrayList<>());
        episode.put("quat_z", new ArrayList<>());
        episode.put("quat_w", new ArrayList<>());
        episode.put("rotationV_x", new ArrayList<>());
        episode.put("rotationV_y", new ArrayList<>());
        episode.put("rotationV_z", new ArrayList<>());
        return episode;
    }

    public void addData(SimulationStatus status, HashMap<String, ArrayList<Double>> episode) {
        episode.get("position_x").add(status.getRocketPosition().x);
        episode.get("position_y").add(status.getRocketPosition().y);
        episode.get("position_z").add(status.getRocketPosition().z);
        episode.get("velocity_x").add(status.getRocketVelocity().x);
        episode.get("velocity_y").add(status.getRocketVelocity().y);
        episode.get("velocity_z").add(status.getRocketVelocity().z);
        episode.get("quat_x").add(status.getRocketOrientationQuaternion().getX());
        episode.get("quat_y").add(status.getRocketOrientationQuaternion().getY());
        episode.get("quat_z").add(status.getRocketOrientationQuaternion().getZ());
        episode.get("quat_w").add(status.getRocketOrientationQuaternion().getW());
        episode.get("rotationV_x").add(status.getRocketRotationVelocity().x);
        episode.get("rotationV_y").add(status.getRocketRotationVelocity().y);
        episode.get("rotationV_z").add(status.getRocketRotationVelocity().z);
    }

    public void addEpisode(HashMap<String, ArrayList<Double>> episode) {
        episodes.add(episode);

        if (episodes.size() % 5 == 0) {
            mof.storeEpisodes(episodes);
        }

        ArrayList<Double> velocities =  episode.get("velocity_z");
        System.out.println("Sim number: " + episodes.size() + " " + velocities.get(velocities.size()-1));
    }
}
