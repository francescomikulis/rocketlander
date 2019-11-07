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
		client.setConnectionString(visualize3D.getConnectionString());
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

	private static int arrayAdd(byte[] bytes, double value, int offset) {
		byte[] floatBytes = getFloatBytes(value);
		int length = 4;
		System.arraycopy(floatBytes, 0, bytes, offset, length);
		return offset + length;
	}

	private static byte[] getFloatBytes(double value) {
		byte[] floatByte = new byte[4];
		ByteBuffer.wrap(floatByte).putFloat((float) value);
		return floatByte;
	}

	private byte[] serialize_single_timeStep(SimulationStatus status) {
		byte[] bytes = new byte[40];
		int offset = 0;
		offset = arrayAdd(bytes, status.getRocketPosition().x, offset);
		offset = arrayAdd(bytes, status.getRocketPosition().y, offset);
		offset = arrayAdd(bytes, status.getRocketPosition().z, offset);
		offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getW(), offset);
		offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getX(), offset);
		offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getY(), offset);
		offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getZ(), offset);
		// thrust may not work
		double thrust = 0.0;
		try {
			thrust = status.getActiveMotors().iterator().next().getThrust(status.getSimulationTime());
		} catch (Exception e) {}
		offset = arrayAdd(bytes, thrust, offset);
		// gimble angles not yet present in the simulationStatus
		offset = arrayAdd(bytes, 0.0, offset);
		offset = arrayAdd(bytes, 0.0, offset);
		return bytes;
	}
}

