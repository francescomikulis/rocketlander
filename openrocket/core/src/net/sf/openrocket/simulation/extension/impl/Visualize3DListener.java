package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.simulation.listeners.SimulationListener;

import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.util.List;

public class Visualize3DListener extends AbstractSimulationListener {
	AbstractSimulationListenerSupportsVisualize3DListener listener = null;
	Visualize3D visualize3D;
	Client client = Client.getInstance();
	long curTime;
	private boolean visualizeDuringPostStep = true;

	Visualize3DListener(Visualize3D visualize3D) {
		this.visualize3D = visualize3D;
	}

	@Override
	public void startSimulation(SimulationStatus status) {
		client.setConnectionString(visualize3D.getConnectionString());
		client.Connect();
	}

	public void setListener(AbstractSimulationListenerSupportsVisualize3DListener listener) {
		this.listener = listener;
	}

	@Override
	public void postStep(SimulationStatus status) {
		if (visualizeDuringPostStep) {
			if (!client.Connected()) {
				client.Connect();
			} else {
				byte[] bytes = serialize_single_timeStep(status);
				client.write(bytes, 0, bytes.length);
			}
			waitdt(status);
		}
	}

	@Override
	public void endSimulation(SimulationStatus status, SimulationException exception) {
		if (visualizeDuringPostStep)
			closeClient();
	}

	public void closeClient() {
		client.close();
	}

	public void setVisualizeDuringPostStep(boolean visualize) {
		this.visualizeDuringPostStep = visualize;
	}

	private void waitdt(SimulationStatus status){
		int timeStep;
		double val = 1000 * status.getPreviousTimeStep() / visualize3D.getTimeRate();
		timeStep = (int) Math.floor(val);
		long realTimeStep = (System.currentTimeMillis() - curTime);

		// NOTE: intentional busy wait - required for certain uses of the visualizer (e.g. not in parallel with simulator)
		while (realTimeStep < timeStep)
			realTimeStep = (System.currentTimeMillis() - curTime);

		curTime = System.currentTimeMillis();
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
		byte[] bytes = new byte[4*12];  // 12 is the number of floats that will be inputted
		int offset = 0;
		offset = arrayAdd(bytes, status.getRocketPosition().x, offset);
		offset = arrayAdd(bytes, status.getRocketPosition().y, offset);
		offset = arrayAdd(bytes, status.getRocketPosition().z, offset);
		offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getW(), offset);
		offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getX(), offset);
		offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getY(), offset);
		offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getZ(), offset);
		// thrust doesn't always work
		double actualMotorThrust = 0.0;
		try {
			actualMotorThrust = status.getActiveMotors().iterator().next().getThrust(status.getSimulationTime());
		} catch (Exception e) {}
		// thurst and gimbal angles not yet present in the simulationStatus
		if (listener != null) {
			offset = arrayAdd(bytes, listener.getMaxMotorPower() * listener.getLastThrust(), offset);
			offset = arrayAdd(bytes, listener.getLastGimbalX(), offset);
			offset = arrayAdd(bytes, listener.getLastGimbalY(), offset);
			offset = arrayAdd(bytes, listener.getLastLateralThrustX(), offset);
			offset = arrayAdd(bytes, listener.getLastLateralThrustY(), offset);
		} else {
			offset = arrayAdd(bytes, actualMotorThrust, offset);
			offset = arrayAdd(bytes, 0.0, offset);
			offset = arrayAdd(bytes, 0.0, offset);
			offset = arrayAdd(bytes, 0.0, offset);
			offset = arrayAdd(bytes, 0.0, offset);
		}
		return bytes;
	}
}

