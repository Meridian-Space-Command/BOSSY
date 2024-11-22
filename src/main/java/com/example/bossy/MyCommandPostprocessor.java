package com.example.bossy;

import org.yamcs.YConfiguration;
import org.yamcs.cmdhistory.CommandHistoryPublisher;
import org.yamcs.commanding.PreparedCommand;
import org.yamcs.tctm.CcsdsSeqCountFiller;
import org.yamcs.tctm.CommandPostprocessor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.nio.ByteBuffer;


public class MyCommandPostprocessor implements CommandPostprocessor {

    private static final Logger log = LoggerFactory.getLogger(MyCommandPostprocessor.class);
    private CcsdsSeqCountFiller seqFiller = new CcsdsSeqCountFiller();
    private CommandHistoryPublisher commandHistory;
    @SuppressWarnings("unused")
    private final String yamcsInstance;

    public MyCommandPostprocessor(String yamcsInstance) {
        this(yamcsInstance, YConfiguration.emptyConfig());
    }

    public MyCommandPostprocessor(String yamcsInstance, YConfiguration config) {
        this.yamcsInstance = yamcsInstance;
        log.info("Command postprocessor initialized for instance {}", yamcsInstance);
    }

    @Override
    public void setCommandHistoryPublisher(CommandHistoryPublisher commandHistory) {
        this.commandHistory = commandHistory;
    }

    @Override
    public byte[] process(PreparedCommand pc) {
        byte[] binary = pc.getBinary();
        
        // Get current mission time from YAMCS
        long missionTime = pc.getGenerationTime();  // This gets the current mission time
        
        // Convert to milliseconds since epoch and pack into 4 bytes
        byte[] timestampedCommand = new byte[binary.length + 4];
        ByteBuffer.wrap(timestampedCommand, 0, 4).putInt((int)(missionTime & 0xFFFFFFFFL));
        System.arraycopy(binary, 0, timestampedCommand, 4, binary.length);
        
        return timestampedCommand;
    }
}
