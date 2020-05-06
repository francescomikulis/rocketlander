package net.sf.openrocket.gui.main;

import java.awt.*;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;

import javax.swing.*;
import javax.swing.table.DefaultTableCellRenderer;

import net.sf.openrocket.document.Definition;
import net.sf.openrocket.gui.simulation.MDPDefinitionEditDialog;
import net.sf.openrocket.simulation.extension.impl.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.RLModel;
import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import net.miginfocom.swing.MigLayout;
import net.sf.openrocket.document.OpenRocketDocument;
import net.sf.openrocket.document.events.DocumentChangeEvent;
import net.sf.openrocket.document.events.DocumentChangeListener;
import net.sf.openrocket.document.events.SimulationChangeEvent;
import net.sf.openrocket.formatting.RocketDescriptor;
import net.sf.openrocket.gui.adaptors.Column;
import net.sf.openrocket.gui.adaptors.ColumnTable;
import net.sf.openrocket.gui.adaptors.ColumnTableModel;
import net.sf.openrocket.gui.adaptors.ColumnTableRowSorter;
import net.sf.openrocket.gui.components.StyledLabel;
import net.sf.openrocket.gui.simulation.SimulationWarningDialog;
import net.sf.openrocket.l10n.Translator;
import net.sf.openrocket.rocketcomponent.ComponentChangeEvent;
import net.sf.openrocket.rocketcomponent.ComponentChangeListener;
import net.sf.openrocket.startup.Application;
import net.sf.openrocket.startup.Preferences;
import net.sf.openrocket.util.AlphanumComparator;

@SuppressWarnings("serial")
public class RLPanel extends JPanel {

    private static final Logger log = LoggerFactory.getLogger(RLPanel.class);
    private static final Translator trans = Application.getTranslator();


    private static final Color WARNING_COLOR = Color.RED;
    private static final String WARNING_TEXT = "\uFF01"; // Fullwidth exclamation mark

    private static final Color OK_COLOR = new Color(60, 150, 0);
    private static final String OK_TEXT = "\u2714"; // Heavy check mark


    private RocketDescriptor descriptor = Application.getInjector().getInstance(RocketDescriptor.class);


    private final OpenRocketDocument document;

    private final ColumnTableModel definitionTableModel;
    private final JTable definitionTable;

    private final JButton editButton;
    private final JButton disableButton;
    private final JButton simulationTypeButton;
    private final JButton simulationAxisButton;
    private final JButton simulationInitButton;
    private final JButton resetModelButton;
    private final JButton deleteButton;

    public RLPanel(OpenRocketDocument doc) {
        super(new MigLayout("fill", "[grow][][][][][][grow]"));

        this.document = doc;



        ////////  The simulation action buttons

        //// New MDPDefinition
        {
            JButton button = new JButton("New MDP Definition");
            //// Add a new simulation
            button.setToolTipText("Create a new MDP Definition");
            button.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    MDPDefinition defaultLander = ModelBaseImplementation.getDefaultLanderDefinition();
                    String stringDefinition = MDPDefinition.toJsonString(defaultLander);
                    Definition definition = new Definition(defaultLander.name, stringDefinition);

                    int n = document.definitions.size();
                    document.definitions.add(definition);
                    definitionTableModel.fireTableDataChanged();
                    definitionTable.clearSelection();
                    definitionTable.addRowSelectionInterval(n, n);

                    openDialog(definition);
                }
            });
            this.add(button, "skip 1, gapright para");
        }

        //// Edit simulation button
        editButton = new JButton("Edit MDP Definition");
        //// Edit the selected simulation
        editButton.setToolTipText("Modify an MDP Definition");
        editButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int[] selection = definitionTable.getSelectedRows();
                if (selection.length != 1) {
                    return;
                }

                selection[0] = definitionTable.convertRowIndexToModel(selection[0]);
                Definition definition = document.definitions.get(selection[0]);

                openDialog(definition);
            }
        });
        this.add(editButton, "gapright para");

        //// Edit simulation button
        disableButton = new JButton("Disable MDP Definition(s)");
        //// Edit the selected simulation
        disableButton.setToolTipText("Disable MDP Definition(s)");
        disableButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int[] selection = definitionTable.getSelectedRows();
                if (selection.length == 0) {
                    return;
                }
                for (int i = 0; i < selection.length; i++) {
                    selection[i] = definitionTable.convertRowIndexToModel(selection[i]);
                    Definition definition = document.definitions.get(selection[i]);
                    definition.setIgnore(!definition.getIgnore());
                }
                definitionTableModel.fireTableDataChanged();
            }
        });
        this.add(disableButton, "gapright para");

        //// Simulation type button
        simulationTypeButton = new JButton("SimulationType: ");
        //// Edit Simulation type
        simulationTypeButton.setToolTipText("Toggle Simulation Type");
        simulationTypeButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                RLModel.getInstance().stepNextSimulationType();
                reloadRLUIText();
            }
        });
        this.add(simulationTypeButton, "gapright para");

        //// Simulation 2D axis button
        simulationAxisButton = new JButton("Simulation2DAxis: ");
        //// Edit Simulation type
        simulationAxisButton.setToolTipText("Toggle Simulation 2D Axis");
        simulationAxisButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                RLModel.getInstance().stepNextSimulation2DAxis();
                reloadRLUIText();
            }
        });
        this.add(simulationAxisButton, "gapright para");

        //// Simulation initialization button
        simulationInitButton = new JButton("SimInitVariation: ");
        //// Edit Simulation initialization
        simulationInitButton.setToolTipText("Toggle Simulation Initialization Variation");
        simulationInitButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                RLModel.getInstance().stepNextInitialVariation();
                reloadRLUIText();
            }
        });
        this.add(simulationInitButton, "gapright para");

        // THIS IS CRITICAL FOR UI UPDATES OF RL MODEL SINGLETON PARAMETERS
        reloadRLUIText();

        //// Reset the stateActionValueFunction
        resetModelButton = new JButton(trans.get("simpanel.but.resetmodel"));
        //// Re-run the selected simulations
        resetModelButton.setToolTipText(trans.get("simpanel.but.ttip.resetmodel"));
        resetModelButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int[] selection = definitionTable.getSelectedRows();
                if (selection.length == 0) {
                    return;
                }
                Definition[] definitions = new Definition[selection.length];
                StringBuilder definitionNames = new StringBuilder();
                for (int i = 0; i < selection.length; i++) {
                    selection[i] = definitionTable.convertRowIndexToModel(selection[i]);
                    definitions[i] = document.definitions.get(selection[i]);
                    definitionNames.append(definitions[i].getName());
                    if (i != selection.length - 1)
                        definitionNames.append(", ");
                }

                JPanel panel = new JPanel(new MigLayout());
                int ret = JOptionPane.showConfirmDialog(RLPanel.this,
                        new Object[] {
                                "Are you sure you want to reset the model(s)?",
                                "<html><b>You will reset: " + definitionNames + "</b>",
                                "<html><i>This operation cannot be undone.</i>",
                                "",
                                panel },
                        "Reset Model",
                        JOptionPane.OK_CANCEL_OPTION,
                        JOptionPane.WARNING_MESSAGE);
                if (ret != JOptionPane.OK_OPTION)
                    return;
                // perform reset on those definitions
                MDPDefinition[] mdpDefinitions = new MDPDefinition[definitions.length];
                for (int i = 0; i < definitions.length; i++)
                    mdpDefinitions[i] = MDPDefinition.buildFromJsonString(definitions[i].getData());
                RLModel.getInstance().resetValueFunctionTable(mdpDefinitions);
            }
        });
        this.add(resetModelButton, "gapright para");

        //// Delete simulations button
        deleteButton = new JButton("Delete MDP Definition");
        //// Delete the selected simulations
        deleteButton.setToolTipText("Please be careful");
        deleteButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int[] selection = definitionTable.getSelectedRows();
                if (selection.length == 0) {
                    return;
                }
                // Verify deletion
                boolean verify = Application.getPreferences().getBoolean(Preferences.CONFIRM_DELETE_SIMULATION, true);
                if (verify) {

                    JPanel panel = new JPanel(new MigLayout());
                    //// Do not ask me again
                    JCheckBox dontAsk = new JCheckBox(trans.get("simpanel.checkbox.donotask"));
                    panel.add(dontAsk, "wrap");
                    //// You can change the default operation in the preferences.
                    panel.add(new StyledLabel(trans.get("simpanel.lbl.defpref"), -2));

                    int ret = JOptionPane.showConfirmDialog(RLPanel.this,
                            new Object[] {
                                    "Delete the MDPDefinitions?",
                                    //// <html><i>This operation cannot be undone.</i>
                                    trans.get("simpanel.dlg.lbl.DeleteSim2"),
                                    "",
                                    panel },
                            "Delete MDPDefinitions",
                            JOptionPane.OK_CANCEL_OPTION,
                            JOptionPane.WARNING_MESSAGE);
                    if (ret != JOptionPane.OK_OPTION)
                        return;

                    if (dontAsk.isSelected()) {
                        Application.getPreferences().putBoolean(Preferences.CONFIRM_DELETE_SIMULATION, false);
                    }
                }

                // Delete MDPDefinitions
                for (int i = 0; i < selection.length; i++) {
                    selection[i] = definitionTable.convertRowIndexToModel(selection[i]);
                }
                Arrays.sort(selection);
                for (int i = selection.length - 1; i >= 0; i--) {
                    document.definitions.remove(selection[i]);
                }
                definitionTableModel.fireTableDataChanged();
            }
        });
        this.add(deleteButton, "wrap para");



        ////////  The simulation table

        definitionTableModel = new ColumnTableModel(

                //// Simulation name
                //// Name
                new Column("Name") {
                    @Override
                    public Object getValueAt(int row) {
                        if (row < 0 || row >= document.definitions.size())
                            return null;

                        return document.definitions.get(row).getName();
                    }

                    @Override
                    public int getDefaultWidth() {
                        return 100;
                    }

                    @Override
                    public Comparator<String> getComparator() {
                        return new AlphanumComparator();
                    }
                },

                new Column("Ignored") {
                    @Override
                    public Object getValueAt(int row) {
                        if (row < 0 || row >= document.definitions.size())
                            return null;

                        return String.valueOf(document.definitions.get(row).getIgnore());
                    }

                    @Override
                    public int getDefaultWidth() {
                        return 50;
                    }

                    @Override
                    public Comparator<String> getComparator() {
                        return new AlphanumComparator();
                    }
                },

                new Column("Definition") {
                    @Override
                    public Object getValueAt(int row) {
                        if (row < 0 || row >= document.definitions.size())
                            return null;

                        return document.definitions.get(row).getData();
                    }

                    @Override
                    public int getDefaultWidth() {
                        return 1000;
                    }

                    @Override
                    public Comparator<String> getComparator() {
                        return new AlphanumComparator();
                    }
                }
                ) {

            private static final long serialVersionUID = 8686456963492628476L;

            @Override
            public int getRowCount() {
                return document.definitions.size();
            }
        };

        // Override processKeyBinding so that the JTable does not catch
        // key bindings used in menu accelerators
        definitionTable = new ColumnTable(definitionTableModel) {

            private static final long serialVersionUID = -5799340181229735630L;

            @Override
            protected boolean processKeyBinding(KeyStroke ks,
                                                KeyEvent e,
                                                int condition,
                                                boolean pressed) {
                return false;
            }
        };
        ColumnTableRowSorter simulationTableSorter = new ColumnTableRowSorter(definitionTableModel);
        definitionTable.setRowSorter(simulationTableSorter);
        definitionTable.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
        definitionTable.setDefaultRenderer(Object.class, new JLabelRenderer());
        definitionTableModel.setColumnWidths(definitionTable.getColumnModel());

        // Mouse listener to act on double-clicks
        definitionTable.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1 && e.getClickCount() == 2) {
                    int selectedRow = definitionTable.getSelectedRow();
                    if (selectedRow < 0) {
                        return;
                    }
                    int selected = definitionTable.convertRowIndexToModel(selectedRow);

                    int column = definitionTable.columnAtPoint(e.getPoint());
                    if (column == 0) {
                        // this will fail
                        SimulationWarningDialog.showWarningDialog(RLPanel.this, null);
                    } else {

                        definitionTable.clearSelection();
                        definitionTable.addRowSelectionInterval(selectedRow, selectedRow);

                        openDialog(document.definitions.get(selected));
                    }
                } else if (e.getButton() == MouseEvent.BUTTON3 && e.getClickCount() == 1){

                } else {
                    updateButtonStates();
                }
            }
        });

        document.addDocumentChangeListener(new DocumentChangeListener() {
            @Override
            public void documentChanged(DocumentChangeEvent event) {
                if (!(event instanceof SimulationChangeEvent))
                    return;
                definitionTableModel.fireTableDataChanged();
            }
        });




        // Fire table change event when the rocket changes
        document.getRocket().addComponentChangeListener(new ComponentChangeListener() {
            @Override
            public void componentChanged(ComponentChangeEvent e) {
                fireMaintainSelection();
            }
        });


        JScrollPane scrollpane = new JScrollPane(definitionTable);
        this.add(scrollpane, "spanx, grow, wrap rel");

        updateButtonStates();
    }

    public void reloadRLUIText() {
        simulationTypeButton.setText("SimulationType: " + String.valueOf(RLModel.getInstance().simulationType));
        simulationAxisButton.setText("Simulation2DAxis: " + RLModel.getInstance().symmetryAxis2D);
        simulationInitButton.setText("SimInitVariation: " + String.valueOf(RLModel.getInstance().initVariation));
    }

    private void updateButtonStates() {
        int[] selection = definitionTable.getSelectedRows();
        if (selection.length == 0) {
            editButton.setEnabled(false);
            disableButton.setEnabled(false);
            resetModelButton.setEnabled(false);
            deleteButton.setEnabled(false);
        } else {
            if (selection.length > 1) {
            } else {
            }
            editButton.setEnabled(true);
            disableButton.setEnabled(true);
            resetModelButton.setEnabled(true);
            deleteButton.setEnabled(true);
        }

    }

    public ListSelectionModel getSimulationListSelectionModel() {
        return definitionTable.getSelectionModel();
    }

    private void openDialog(final Definition... definitions) {
        MDPDefinitionEditDialog d = new MDPDefinitionEditDialog(SwingUtilities.getWindowAncestor(this), document, definitions);
        d.setVisible(true);
        fireMaintainSelection();
    }

    private void fireMaintainSelection() {
        int[] selection = definitionTable.getSelectedRows();
        definitionTableModel.fireTableDataChanged();
        for (int row : selection) {
            if (row >= definitionTableModel.getRowCount())
                break;
            definitionTable.addRowSelectionInterval(row, row);
        }
    }

    public static class CellTransferable implements Transferable {

        public static final DataFlavor CELL_DATA_FLAVOR = new DataFlavor(Object.class, "application/x-cell-value");

        private Object cellValue;

        public CellTransferable(Object cellValue) {
            this.cellValue = cellValue;
        }

        @Override
        public DataFlavor[] getTransferDataFlavors() {
            return new DataFlavor[]{CELL_DATA_FLAVOR};
        }

        @Override
        public boolean isDataFlavorSupported(DataFlavor flavor) {
            return CELL_DATA_FLAVOR.equals(flavor);
        }

        @Override
        public Object getTransferData(DataFlavor flavor) throws UnsupportedFlavorException, IOException {
            if (!isDataFlavorSupported(flavor)) {
                throw new UnsupportedFlavorException(flavor);
            }
            return cellValue;
        }

    }

    private class JLabelRenderer extends DefaultTableCellRenderer {

        @Override
        public Component getTableCellRendererComponent(JTable table,
                                                       Object value, boolean isSelected, boolean hasFocus, int row,
                                                       int column) {

            if (row < 0 || row >= document.definitions.size())
                return super.getTableCellRendererComponent(table, value,
                        isSelected, hasFocus, row, column);

            row = table.getRowSorter().convertRowIndexToModel(row);

            // A JLabel is self-contained and has set its own tool tip
            if (value instanceof JLabel) {
                JLabel label = (JLabel) value;
                if (isSelected)
                    label.setBackground(table.getSelectionBackground());
                else
                    label.setBackground(table.getBackground());
                label.setOpaque(true);
                return label;
            }

            return super.getTableCellRendererComponent(table, value,
                    isSelected, hasFocus, row, column);
        }
    }
}
