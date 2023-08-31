package frc.robot.Subsystems.Networking;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class NetworkTableContainer {

        public static HashMap<String, NetworkEntry> entries = new HashMap<>();

    public static void insertGlobalEntries() {
        NetworkTableContainer.entries.putAll(Map.of(
                "Built Widgets",
                new NetworkEntry(Shuffleboard.getTab("Swerve"),
                        "Widgets built: ",
                        BuiltInWidgets.kBooleanBox, null,
                        true, null),
                "Field Centric",
                new NetworkEntry(Shuffleboard.getTab("Swerve"),
                        "Field Centric",
                        BuiltInWidgets.kToggleButton,
                        null,
                        false, null),
                "Override Target Heading",
                new NetworkEntry(Shuffleboard.getTab("Swerve"),
                        "Override Target Heading",
                        BuiltInWidgets.kToggleButton,
                        null,
                        false, null),
                "Forward Output",
                new NetworkEntry(Shuffleboard.getTab("Swerve"),
                        "Forward Output",
                        BuiltInWidgets.kGraph,
                        null,
                        0, null),
                "Auto Forward Power",
                new NetworkEntry(Shuffleboard.getTab("Swerve"),
                        "Auto Forward Power",
                        BuiltInWidgets.kNumberSlider,
                        Map.of("min", -1200, "max", 1200),
                        0, null)));
    }
}
