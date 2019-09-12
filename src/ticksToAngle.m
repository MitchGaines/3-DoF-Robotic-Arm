function T = ticksToAngle(tick, offset)
    T = (tick * 360) / 4096 + offset;
end

