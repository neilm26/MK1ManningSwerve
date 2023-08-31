package frc.robot.Subsystems.Networking;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NetworkEntry {

    private GenericEntry entry;
    private String entryName;

    public NetworkEntry(ShuffleboardTab tab, String entryName, BuiltInWidgets widget, Map<String, Object> properties,
            Object val, String parentName) {

        try {
            if (parentName != null) {
                ShuffleboardLayout layout = tab.getLayout(parentName, BuiltInLayouts.kList).withSize(2, 2);
                entry = layout.add(entryName,
                        val).withWidget(widget).withProperties(properties).getEntry();
            } else {
                entry = tab.add(entryName,
                        val).withWidget(widget).withProperties(properties).getEntry();
            }

            this.entryName = entryName;

            NetworkTableContainer.entries.put(entryName, this);

        } catch (IllegalArgumentException | NullPointerException e) {
            // TODO: handle exception
            DriverStation.reportError("Widget cannot be constructed! " + e.toString(), true);
        }
    }

    public Object getNetworkTblValue() {
        return entry.get().getValue();
    }

    public void setNetworkEntryValue(Object value) {
        if (getNetworkTblValue().getClass() == value.getClass()){
            entry.setValue(value);
        }
    }

    public GenericEntry getEntry() {
        return entry;
    }

    public String getEntryName() {
        return entryName;
    }
}
