package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.SpinnerEditor;
import net.sf.openrocket.gui.adaptors.DoubleModel;
import net.sf.openrocket.gui.adaptors.ValueColumn;
import net.sf.openrocket.gui.components.BasicSlider;
import net.sf.openrocket.gui.components.UnitSelector;
import net.sf.openrocket.gui.main.SimulationPanel;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.unit.UnitGroup;

import javax.swing.*;

@Plugin
public class Visualize3DConfigurator extends AbstractSwingSimulationExtensionConfigurator<Visualize3D> {

	public Visualize3DConfigurator() {
		super(Visualize3D.class);
	}
	
	@Override
	protected JComponent getConfigurationComponent(Visualize3D extension, Simulation simulation, JPanel panel) {
		panel.add(new JLabel("Real time rate:"));
		//panel.add(new JLabel("IP:"));
		DoubleModel m = new DoubleModel(extension, "TimeRate", UnitGroup.UNITS_NONE);
		String connectionString = JOptionPane.showInputDialog(panel, new Object[] {"address:port", "", panel}, "127.0.0.1:5000");
		JSpinner spin = new JSpinner(m.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "w 65lp!");
		extension.setConnectionString(connectionString);
		UnitSelector unit = new UnitSelector(m);
		panel.add(unit, "w 25");
		return panel;
	}
	
}
