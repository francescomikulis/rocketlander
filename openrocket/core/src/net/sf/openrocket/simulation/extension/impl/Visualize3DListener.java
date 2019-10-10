package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;

import java.io.IOException;
import java.lang.Double.*;
import java.util.ArrayList;
import java.util.HashMap;

public class Visualize3DListener extends AbstractSimulationListener {
	Visualize3D visualize3D;
	Client client = Client.getInstance();
	long curTime;

	Visualize3DListener(Visualize3D visualize3D) {
		this.visualize3D = visualize3D;
	}

	@Override
	public void startSimulation(SimulationStatus status) throws SimulationException {
		client.Connect();
	}

	@Override
	public void postStep(SimulationStatus status) throws SimulationException{
		if (!client.Connected()){
			client.Connect();
		} else{
			HashMap<String, ArrayList<Double>> data = RLEpisodeManager.initializeEmptyEpisode();
			RLEpisodeManager.addData(status, data);
			client.write(RLEpisodeManager.serialize_single_timestep(data), 0, RLEpisodeManager.serialize_length());
		}
		waitdt(status);
	}

	@Override
	public void endSimulation(SimulationStatus status, SimulationException exception) {
		client.close();
	}

	private void waitdt(SimulationStatus status){
		int timeStep;
		try {
			double val = 1000 * status.getPreviousTimeStep() / visualize3D.getTimeRate();
			timeStep = (int) Math.floor(val);
			long realTimeStep = (System.currentTimeMillis() - curTime);
			if (realTimeStep < timeStep)
				Thread.sleep(timeStep - realTimeStep);
			curTime = System.currentTimeMillis();

		} catch (InterruptedException e) {
			e.printStackTrace();
		}

	}
}

