package frc.lib.tuneables;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class TuneableCommand extends Command implements Tuneable {
    public Tuneable fullTuneable() {
        return (builder) -> {
            builder.setSendableType(SendableType.LIST);
            builder.addChild("run button", (Sendable) this);
            builder.addChild("tuning", (Tuneable) this);
        };
    };
}
