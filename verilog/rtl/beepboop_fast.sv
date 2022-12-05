module beepboop_fast (
    input logic clock, // 2MHz
    input logic reset,
    input logic btn,
    output [5:0] io_out
);

    logic clock_slow;

    beepboop inst (
        .io_in({5'b0, btn, reset, clock}),
        .io_out(io_out)
    );

    logic [19:0] counter; // 10ms interval

    // Counter
    always_ff @(posedge clock) begin
        if (reset) begin
            counter <= 0;
            clock_slow <= 0;
        end
        else begin
            counter <= counter + 1;

            if (counter >= 9999) begin
                clock_slow <= ~clock_slow;
                counter <= 0;
            end
        end
    end
    
endmodule

