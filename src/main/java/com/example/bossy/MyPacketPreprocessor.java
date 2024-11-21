package com.example.bossy;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.time.Instant;

import org.yamcs.TmPacket;
import org.yamcs.YConfiguration;
import org.yamcs.tctm.AbstractPacketPreprocessor;
import org.yamcs.utils.TimeEncoding;

public class MyPacketPreprocessor extends AbstractPacketPreprocessor {

    private Map<Integer, AtomicInteger> seqCounts = new HashMap<>();
    private final long missionEpoch;

    public MyPacketPreprocessor(String yamcsInstance) {
        this(yamcsInstance, YConfiguration.emptyConfig());
    }

    public MyPacketPreprocessor(String yamcsInstance, YConfiguration config) {
        super(yamcsInstance, config);
        
        // Get mission start time from environment variable
        String missionStartTime = System.getenv("MISSION_START_TIME");
        missionEpoch = Instant.parse(missionStartTime).toEpochMilli();
    }

    @Override
    public TmPacket process(TmPacket packet) {
        byte[] bytes = packet.getPacket();
        if (bytes.length < 10) {
            eventProducer.sendWarning("SHORT_PACKET",
                    "Short packet received, length: " + bytes.length + "; minimum required length is 10 bytes.");
            return null;
        }

        ByteBuffer buffer = ByteBuffer.wrap(bytes);

        // Read primary header
        int apidseqcount = buffer.getInt(0);
        int apid = (apidseqcount >> 16) & 0x07FF;
        int seq = (apidseqcount) & 0x3FFF;

        // Verify sequence continuity
        AtomicInteger ai = seqCounts.computeIfAbsent(apid, k -> new AtomicInteger());
        int oldseq = ai.getAndSet(seq);
        if (((seq - oldseq) & 0x3FFF) != 1) {
            eventProducer.sendWarning("SEQ_COUNT_JUMP",
                    "Sequence count jump for APID: " + apid + " old seq: " + oldseq + " newseq: " + seq);
        }

        // Read time from secondary header (4 bytes CUC)
        long timeValue = buffer.getInt(6) & 0xFFFFFFFFL;  // Get unsigned int

        // Convert CUC time (milliseconds since mission epoch) to YAMCS time
        long generationTime = TimeEncoding.fromUnixMillisec(missionEpoch + timeValue);
        packet.setGenerationTime(generationTime);

        // Use the full 32-bits for sequence count
        packet.setSequenceCount(apidseqcount);

        return packet;
    }
}
