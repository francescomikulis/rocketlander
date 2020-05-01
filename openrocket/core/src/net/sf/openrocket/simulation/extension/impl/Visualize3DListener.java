package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.simulation.listeners.SimulationListener;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.util.List;

public class Visualize3DListener extends AbstractSimulationListener {
	SimulationListener rocketLanderListener = null;
	Visualize3D visualize3D;
	Client client = Client.getInstance();
	long curTime;

	Visualize3DListener(Visualize3D visualize3D) {
		this.visualize3D = visualize3D;
	}

	@Override
	public void startSimulation(SimulationStatus status) {
		client.setConnectionString(visualize3D.getConnectionString());
		client.Connect();

		// RocketLanderListener integration for gimbal
		List<SimulationListener> listeners = status.getSimulationConditions().getSimulationListenerList();
		for (SimulationListener listener: listeners) {
			if (listener.getClass().toString().contains("RocketLanderListener")) {
				rocketLanderListener = listener;
			}
		}
	}

	@Override
	public void postStep(SimulationStatus status) {
		if (!client.Connected()){
			client.Connect();
		} else{
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
		double actualMotorThrust = 0.0;
		try {
			actualMotorThrust = status.getActiveMotors().iterator().next().getThrust(status.getSimulationTime());
		} catch (Exception e) {}
		// thurst and gimbal angles not yet present in the simulationStatus
		offset = arrayAdd(bytes, actualMotorThrust * RLL.getThrust(rocketLanderListener), offset);
		offset = arrayAdd(bytes, RLL.getGimbalX(rocketLanderListener), offset);
		offset = arrayAdd(bytes, RLL.getGimbalY(rocketLanderListener), offset);
		return bytes;
	}

	/** Hacky code to avoid requiring dependencies
	 * between RocketLanderListener and this 3DVisualizer
	 **/

	public static class RLL {
		public static float getGimbalX(Object rocketLanderListener) {
			return getActionDoubleValue(rocketLanderListener, "gimbalX");
		}

		public static float getGimbalY(Object rocketLanderListener) {
			return getActionDoubleValue(rocketLanderListener, "gimbalY");
		}

		public static float getThrust(Object rocketLanderListener) {
			return getActionDoubleValue(rocketLanderListener, "thrust");
		}

		public static float getActionDoubleValue(Object rocketLanderListener, String field){
			if (rocketLanderListener == null)
				return 0.0f;

			Object action = getAction(rocketLanderListener);
			Object result = callMethod(action, action.getClass(), "getDouble", field);
			if (result == null) return 0.0f;
			return ((Double)result).floatValue();
		}

		public static Object getAction(Object rocketLanderListener) {
			return callMethod(rocketLanderListener, "getLastAction");
		}

		/*
		public static Object getField(Object object, String field){
			if (object == null) return null;
			Object result = null;
			try {
				Class<?> c = object.getClass();
				Field f = c.getDeclaredField(field);
				f.setAccessible(true);
				result = (Object) f.get(object);
			} catch (Exception e) {
				System.out.println(e.getMessage());
				e.printStackTrace();
			}
			return result;
		}
		*/

		public static Object callMethod(Object object, String methodName) {
			if (object == null) return null;

			Method method;
			try {
				method = object.getClass().getMethod(methodName);
			} catch (NoSuchMethodException e) {
				e.printStackTrace();
				return null;
			}

			try {
				method.setAccessible(true);
				return method.invoke(object);
			} catch (Exception e) {
				e.printStackTrace();
				return null;
			}
		}

		public static Object callMethod(Object object, Class theClass, String methodName, Object... args) {
			if (object == null) return null;

			Method method;
			try {
				method = theClass.getMethod(methodName, String.class);
			} catch (NoSuchMethodException e) {
				e.printStackTrace();
				return null;
			}

			try {
				method.setAccessible(true);
				return method.invoke(object, args);
			} catch (Exception e) {
				e.printStackTrace();
				return null;
			}
		}
	}
}

