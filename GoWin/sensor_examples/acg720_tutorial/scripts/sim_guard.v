`timescale 1ns/1ps

// Many tutorial testbenches intentionally leave a free-running clock and are
// meant for manual waveform inspection. Run them for a bounded 1 ms so batch
// verification proves elaboration and stimulus execution without hanging.
module acg720_sim_guard;
    initial begin
        #1000000;
        $display("ACG720_SIM_GUARD_REACHED time=%0t", $time);
        $finish;
    end
endmodule
