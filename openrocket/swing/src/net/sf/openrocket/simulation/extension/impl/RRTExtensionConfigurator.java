package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.SpinnerEditor;
import net.sf.openrocket.gui.adaptors.DoubleModel;
import net.sf.openrocket.gui.components.BasicSlider;
import net.sf.openrocket.gui.components.UnitSelector;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.unit.UnitGroup;

import javax.swing.*;

@Plugin
public class RRTExtensionConfigurator extends AbstractSwingSimulationExtensionConfigurator<RRTExtension> {

	public RRTExtensionConfigurator() {
		super(RRTExtension.class);
	}
	
	@Override
	protected JComponent getConfigurationComponent(RRTExtension extension, Simulation simulation, JPanel panel) {
		panel.add(new JLabel("UsingLateralObjective (0 or 1)"));
		DoubleModel m = new DoubleModel(extension, "UsingLateralObjective", UnitGroup.UNITS_NONE);
		JSpinner spin = new JSpinner(m.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "w 65lp!");
		UnitSelector unit = new UnitSelector(m);
		panel.add(unit, "w 25");
		return panel;
	}
	
}
