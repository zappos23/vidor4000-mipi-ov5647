module mkrvidor4000_top
(
    // system signals
    input CLK_48MHZ,
    input RESETn,
    input SAM_INT_IN,
    output SAM_INT_OUT,

    // SDRAM
    output SDRAM_CLK,
    output [11:0] SDRAM_ADDR,
    output [1:0] SDRAM_BA,
    output SDRAM_CASn,
    output SDRAM_CKE,
    output SDRAM_CSn,
    inout [15:0] SDRAM_DQ,
    output [1:0] SDRAM_DQM,
    output SDRAM_RASn,
    output SDRAM_WEn,

    // SAM D21 PINS
    inout MKR_AREF,
    inout [6:0] MKR_A,
    inout [14:0] MKR_D,

    // HDMI output
    output [2:0] HDMI_TX,
    output HDMI_CLK,
    inout HDMI_SDA,
    inout HDMI_SCL,

    input HDMI_HPD,

    // MIPI input
    input [1:0] MIPI_D,
    input MIPI_CLK,
    inout MIPI_SDA,
    inout MIPI_SCL,
    inout [1:0] MIPI_GP

);

// internal oscillator
wire OSC_CLK;
cyclone10lp_oscillator osc ( 
    .clkout(OSC_CLK),
    .oscena(1'b1)
);

mem_pll mem_pll(.inclk0(CLK_48MHZ), .c0(SDRAM_CLK));

wire clk_pixel_x5;
wire clk_pixel;
wire clk_audio;
hdmi_pll hdmi_pll(.inclk0(CLK_48MHZ), .c0(clk_pixel), .c1(clk_pixel_x5), .c2(clk_audio));

//JM pll for JTAG 120Mhz
wire clk120;
system_pll system_pll(.areset(1'b0), .inclk0(CLK_48MHZ), .c0(), .c1(clk120), .c2(), .c3(), .c4(), .locked());

localparam AUDIO_BIT_WIDTH = 16;
localparam AUDIO_RATE = 48000;
localparam WAVE_RATE = 240;

logic [AUDIO_BIT_WIDTH-1:0] audio_sample_word;
sawtooth #(.BIT_WIDTH(AUDIO_BIT_WIDTH), .SAMPLE_RATE(AUDIO_RATE), .WAVE_RATE(WAVE_RATE)) sawtooth (.clk_audio(clk_audio), .level(audio_sample_word));

logic [23:0] rgb;
logic [9:0] cx, cy, screen_start_x, screen_start_y;
hdmi #(.VIDEO_ID_CODE(1), .AUDIO_RATE(AUDIO_RATE), .AUDIO_BIT_WIDTH(AUDIO_BIT_WIDTH)) hdmi(
    .clk_pixel_x5(clk_pixel_x5),
    .clk_pixel(clk_pixel),
    .clk_audio(clk_audio),
    .rgb(rgb),
    .audio_sample_word('{audio_sample_word >> 9, audio_sample_word >> 9}),
    .tmds(HDMI_TX),
    .tmds_clock(HDMI_CLK),
    .cx(cx),
    .cy(cy),
    .screen_start_x(screen_start_x),
    .screen_start_y(screen_start_y)
);

logic [1:0] mode = 2'd0;
logic [1:0] resolution = 2'd3; // 640x480 @ 30FPS
logic format = 1'd0; // RAW8
logic ready;
logic model_err;
logic nack_err;

ov5647 #(.INPUT_CLK_RATE(48_000_000), .TARGET_SCL_RATE(100_000)) ov5647 (
    .clk_in(CLK_48MHZ),
    .scl(MIPI_SCL),
    .sda(MIPI_SDA),
    .mode(mode),
    .resolution(resolution),
    .format(format),
    .ready(ready),
    .power_enable(MIPI_GP[0]),
    .model_err(model_err),
    .nack_err(nack_err)
);

// JM RASPBERRY PI CAM V2.1
/*
imx219 #(.INPUT_CLK_RATE(48_000_000), .TARGET_SCL_RATE(100_000)) imx219 (
    .clk_in(CLK_48MHZ),
    .scl(MIPI_SCL),
    .sda(MIPI_SDA),
    .mode(mode),
    .resolution(resolution),
    .format(format),
    .ready(ready),
    .power_enable(MIPI_GP[0]),
    .model_err(model_err),
    .nack_err(nack_err)
);
*/

logic [7:0] image_data [3:0];
logic [5:0] image_data_type;
logic image_data_enable;
logic [15:0] word_count;
logic frame_start, line_start, interrupt, frame_end;
camera #(.NUM_LANES(2)) camera (
    .clock_p(MIPI_CLK),
    .data_p(MIPI_D),
    .image_data(image_data),
    .image_data_type(image_data_type),
    .image_data_enable(image_data_enable),
    .word_count(word_count),
    .frame_start(frame_start),
    .frame_end(frame_end),
    .line_start(line_start),
    .interrupt(interrupt)
);

// logic [7:0] raw [3:0];
// logic raw_enable;
// raw8 raw8 (.image_data(image_data), .image_data_enable(image_data_enable), .raw(raw), .raw_enable(raw_enable));

logic [25:0] camera_counter = 26'd0;
always @(posedge CLK_48MHZ)
    camera_counter <= camera_counter + 1'd1;

always @(posedge CLK_48MHZ)
    if (ready && mode == 2'd0)
        mode <= 2'd1;
    else if (ready && mode == 2'd1 && camera_counter + 1'd1 == 26'd0)
        mode <= 2'd2;

//JM pixel enable now is UART
logic pixel_enable;
logic pixel_enable_hdmi;
assign pixel_enable_hdmi = cx >= screen_start_x && cy >= screen_start_y;
logic [7:0] pixel;

arbiter arbiter (
    .pixel_clk(clk_pixel),
    .pixel_enable(pixel_enable_hdmi),       //JM: uart: pixel_enable, hdmi: pixel_enable_hdmi
    .pixel(pixel),
    .mipi_clk(MIPI_CLK),
    .mipi_data_enable(image_data_enable),
    .mipi_data(image_data),         // JM: image_data
    .frame_start(frame_start),
    .line_start(line_start),
    .interrupt(interrupt),
    .sdram_clk(SDRAM_CLK),
    .clock_enable(SDRAM_CKE),
    .bank_activate(SDRAM_BA),
    .address(SDRAM_ADDR),
    .chip_select(SDRAM_CSn),
    .row_address_strobe(SDRAM_RASn),
    .column_address_strobe(SDRAM_CASn),
    .write_enable(SDRAM_WEn),
    .dqm(SDRAM_DQM),
    .dq(SDRAM_DQ),
    .pixel_ready(pixel_ready)
);

/*
// JM Test logic to temporary supply a count rather than pixel
logic [7:0] counter = 8'd0;
logic [7:0] counter_data [3:0];

assign counter_data[0] = counter; 
assign counter_data[1] = counter; 
assign counter_data[2] = counter; 
assign counter_data[3] = counter;

always @(posedge MIPI_CLK) begin
    if (image_data_enable && interrupt)
        counter <= counter + 1'd1;
end
*/

logic [31:0] pre_debug0 = 32'b0;
logic [31:0] debug0 = 32'h0;
logic [31:0] pre_debug1;
logic [31:0] debug1 = 32'h0;


// JTAG
JTAG_TOP jtag0 (
   .iCLK_MAIN(clk120),
   .iREAD_0(debug0),
   .iREAD_1(debug1));

// UART TX to SAMD21
parameter CLK_HZ = 25200000;
//parameter BIT_RATE =   9600;
parameter BIT_RATE =   115200;
parameter PAYLOAD_BITS = 8;
logic uart_tx_busy;
logic pixel_ready;
// if less than 256 means there is empty slot to write.
logic [7:0] uart_tx_fifo_data_in_used;
// data_out_used is useless. It only incremental when data is read. Then it reaches the maximum limit then it stop
logic [7:0] uart_tx_fifo_data_out_used;
logic uart_tx_fifo_data_in_enable;
logic [7:0] uart_tx_fifo_data_out;
// to indicate to the fifo for a read.
logic uart_tx_data_out_acknowledge;
logic uart_tx_enable;
logic uart_tx_fifo_full;

// ready to write when sender fifo is not empty and receiver fifo is not full
assign uart_tx_fifo_data_in_enable = pixel_ready && (uart_tx_fifo_data_in_used != 8'b11111111);
assign pixel_enable = uart_tx_fifo_data_in_used != 8'b11111111;
// send a receive acknowledge to fifo to increment the read pointer when uart is free.
assign uart_tx_data_out_acknowledge = !uart_tx_busy;
assign uart_tx_enable = uart_tx_fifo_data_in_used != 8'd0;
// 1: indicates fifo is full
assign uart_tx_fifo_full = uart_tx_fifo_data_in_used == 8'b11111111;

// UART TX FIFO
fifo #(.DATA_WIDTH(8), .POINTER_WIDTH(8), .SENDER_DELAY_CHAIN_LENGTH(1), .RECEIVER_DELAY_CHAIN_LENGTH(1)) uart_tx_fifo(
    .sender_clock(clk_pixel),
    .data_in_enable(uart_tx_fifo_data_in_enable),
    .data_in_used(uart_tx_fifo_data_in_used),
    .data_in(pixel),
    .receiver_clock(clk_pixel),
    .data_out_used(uart_tx_fifo_data_out_used),
    .data_out_acknowledge(uart_tx_data_out_acknowledge),
    .data_out(uart_tx_fifo_data_out)
);

//
// UART Transmitter module.
//
uart_tx #(
.BIT_RATE(BIT_RATE),
.PAYLOAD_BITS(PAYLOAD_BITS),
.CLK_HZ  (CLK_HZ  )
) i_uart_tx(
.clk          (clk_pixel    ),
.resetn       (1'b1         ),
.uart_txd     (MKR_D[13]   ),
.uart_tx_en   (1'b1 ),
.uart_tx_busy (uart_tx_busy),
.uart_tx_data (uart_tx_fifo_data_out) 
);

always @(posedge clk_pixel)
    rgb <= {pixel, pixel, pixel};

endmodule
