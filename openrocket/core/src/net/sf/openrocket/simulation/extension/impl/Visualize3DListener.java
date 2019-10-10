package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;

import java.io.IOException;
import java.lang.Double.*;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

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
			//HashMap<String, ArrayList<Double>> data = RLEpisodeManager.initializeEmptyEpisode();
			//RLEpisodeManager.addData(status, data);
			//client.write(RLEpisodeManager.serialize_single_timestep(data), 0, RLEpisodeManager.serialize_length());
			byte[] bytes = serialize_single_timeStep(status);
			client.write(bytes, 0, bytes.length);
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

	private static byte[] getFloatBytes(double value) {
		byte[] floatByte = new byte[4];
		ByteBuffer.wrap(floatByte).putFloat((float) value);
		return floatByte;
	}

	private byte[] serialize_single_timeStep(SimulationStatus status) {
		byte[] bytes = new byte[28];
		int offset = 0;

		System.arraycopy(getFloatBytes(status.getRocketPosition().x), 0, bytes, offset, 4);
		offset += 4;
		System.arraycopy(getFloatBytes(status.getRocketPosition().y), 0, bytes, offset, 4);
		offset += 4;
		System.arraycopy(getFloatBytes(status.getRocketPosition().z), 0, bytes, offset, 4);
		offset += 4;
		System.arraycopy(getFloatBytes(status.getRocketOrientationQuaternion().getW()), 0, bytes, offset, 4);
		offset += 4;
		System.arraycopy(getFloatBytes(status.getRocketOrientationQuaternion().getX()), 0, bytes, offset, 4);
		offset += 4;
		System.arraycopy(getFloatBytes(status.getRocketOrientationQuaternion().getY()), 0, bytes, offset, 4);
		offset += 4;
		System.arraycopy(getFloatBytes(status.getRocketOrientationQuaternion().getZ()), 0, bytes, offset, 4);

		return bytes;
	}
}

