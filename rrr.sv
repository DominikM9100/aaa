`timescale 1ns / 1ps

module receiver #(
    parameter int DATA_WIDTH = 128,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8,   // 16
    parameter int DEST_WIDTH = 4
)(
    input  logic                Clk,
    input  logic                Rst,
    axis_interface.slave        S_axis,
    axis_interface.master       Fifo_Desc_Hdr_Push,
    axis_interface.master       Fifo_Payload_Push,
    axis_interface.master       Fifo_Meta_Push
);

    // ----------------------------------------------------------------
    // Kodowanie stanów – tylko 3
    // ----------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_IDLE    = 2'd0,
        ST_HDR     = 2'd1,
        ST_PAYLOAD = 2'd2
    } state_t;

    state_t state, next_state;

    // ----------------------------------------------------------------
    // Rejestry nagłówka (wypełniane podczas ST_HDR)
    // ----------------------------------------------------------------
    logic [7:0]  hdr_len;          // IHL*4 (w bajtach)
    logic [7:0]  hdr_bytes_rcvd;   // ile bajtów nagłówka już odebrano
    logic [15:0] total_length;     // pole Total Length z IP
    logic        df_flag;          // Don't Fragment
    logic        hdr_parsed;       // czy IHL/Total/DF już sparsowane

    // ----------------------------------------------------------------
    // Sygnały kombinacyjne
    // ----------------------------------------------------------------
    logic [7:0]  hdr_rem;
    logic        is_split_beat;
    logic        is_last_hdr_beat;
    logic [4:0]  split_offset;
    logic [15:0] payload_len;

    assign hdr_rem          = hdr_len - hdr_bytes_rcvd;
    assign is_split_beat    = hdr_parsed && (hdr_rem > 8'd0) && (hdr_rem < 8'd16);
    assign is_last_hdr_beat = hdr_parsed && (hdr_rem <= 8'd16);
    assign split_offset     = is_split_beat ? hdr_rem[4:0] : 5'd16;
    assign payload_len      = total_length - {8'd0, hdr_len};

    // Maski TKEEP dla podziału taktu split
    logic [KEEP_WIDTH-1:0] hdr_keep_mask;
    logic [KEEP_WIDTH-1:0] payload_keep_mask;
    logic [KEEP_WIDTH:0]   mask_tmp;

    assign mask_tmp          = ({{KEEP_WIDTH{1'b0}}, 1'b1} << split_offset) - 1'b1;
    assign hdr_keep_mask     = mask_tmp[KEEP_WIDTH-1:0];
    assign payload_keep_mask = ({KEEP_WIDTH{1'b1}} >> split_offset);

    // Handshake S_AXIS
    logic s_axis_fire;
    assign s_axis_fire = S_axis.tvalid && S_axis.tready;

    // ----------------------------------------------------------------
    // Blok sekwencyjny: state <= next_state + rejestry nagłówka
    // ----------------------------------------------------------------
    always_ff @(posedge Clk) begin
        if (Rst) begin
            state <= ST_IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_ff @(posedge Clk) begin
        if (Rst) begin
            hdr_len        <= '0;
            hdr_bytes_rcvd <= '0;
            total_length   <= '0;
            df_flag        <= 1'b0;
            hdr_parsed     <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (s_axis_fire) begin
                        // Nowy pakiet – reset pól nagłówka
                        hdr_len        <= '0;
                        hdr_bytes_rcvd <= '0;
                        total_length   <= '0;
                        df_flag        <= 1'b0;
                        hdr_parsed     <= 1'b0;
                    end
                end

                ST_HDR: begin
                    if (s_axis_fire) begin
                        if (!hdr_parsed) begin
                            // Pierwszy takt nagłówka – parsowanie pól IP
                            // Bajt 0:    wersja[7:4] | IHL[3:0]
                            // Bajty 2-3: Total Length (big-endian)
                            // Bajt 6:    flagi (bit 6 = DF)
                            hdr_len        <= {S_axis.tdata[3:0], 2'b00};
                            total_length   <= {S_axis.tdata[23:16],
                                               S_axis.tdata[31:24]};
                            df_flag        <= S_axis.tdata[54];
                            hdr_parsed     <= 1'b1;
                            hdr_bytes_rcvd <= 8'd16;
                        end else begin
                            hdr_bytes_rcvd <= hdr_bytes_rcvd + 8'd16;
                        end
                    end
                end

                default: ;
            endcase
        end
    end

    // ----------------------------------------------------------------
    // Blok kombinacyjny: next_state + wszystkie wyjścia AXI
    // ----------------------------------------------------------------
    always_comb begin
        // --- wartości domyślne ---
        next_state = state;

        S_axis.tready             = 1'b0;

        Fifo_Desc_Hdr_Push.tvalid = 1'b0;
        Fifo_Desc_Hdr_Push.tdata  = S_axis.tdata;
        Fifo_Desc_Hdr_Push.tkeep  = S_axis.tkeep;
        Fifo_Desc_Hdr_Push.tlast  = 1'b0;
        Fifo_Desc_Hdr_Push.tdest  = S_axis.tdest;

        Fifo_Payload_Push.tvalid  = 1'b0;
        Fifo_Payload_Push.tdata   = S_axis.tdata;
        Fifo_Payload_Push.tkeep   = S_axis.tkeep;
        Fifo_Payload_Push.tlast   = 1'b0;
        Fifo_Payload_Push.tdest   = S_axis.tdest;

        Fifo_Meta_Push.tvalid     = 1'b0;
        Fifo_Meta_Push.tdata      = '0;
        Fifo_Meta_Push.tkeep      = {KEEP_WIDTH{1'b1}};
        Fifo_Meta_Push.tlast      = 1'b1;
        Fifo_Meta_Push.tdest      = '0;

        case (state)
            // =========================================================
            ST_IDLE: begin
                // Przekazanie deskryptora do FIFO Desc_Hdr
                Fifo_Desc_Hdr_Push.tvalid = S_axis.tvalid;
                S_axis.tready             = Fifo_Desc_Hdr_Push.tready;

                if (s_axis_fire)
                    next_state = ST_HDR;
            end

            // =========================================================
            ST_HDR: begin
                if (is_split_beat) begin
                    // Ostatni takt nagłówka z jednoczesnym początkiem payloadu
                    Fifo_Desc_Hdr_Push.tvalid = S_axis.tvalid && Fifo_Payload_Push.tready;
                    Fifo_Desc_Hdr_Push.tkeep  = S_axis.tkeep & hdr_keep_mask;

                    Fifo_Payload_Push.tvalid  = S_axis.tvalid && Fifo_Desc_Hdr_Push.tready;
                    Fifo_Payload_Push.tdata   = S_axis.tdata >> (split_offset * 8);
                    Fifo_Payload_Push.tkeep   = (S_axis.tkeep >> split_offset) &
                                                 payload_keep_mask;
                    Fifo_Payload_Push.tlast   = S_axis.tlast;

                    if (S_axis.tlast)
                        S_axis.tready = Fifo_Desc_Hdr_Push.tready &&
                                        Fifo_Payload_Push.tready &&
                                        Fifo_Meta_Push.tready;
                    else
                        S_axis.tready = Fifo_Desc_Hdr_Push.tready &&
                                        Fifo_Payload_Push.tready;
                end else begin
                    // Cały takt to nagłówek
                    Fifo_Desc_Hdr_Push.tvalid = S_axis.tvalid;

                    if (is_last_hdr_beat && S_axis.tlast)
                        S_axis.tready = Fifo_Desc_Hdr_Push.tready &&
                                        Fifo_Meta_Push.tready;
                    else
                        S_axis.tready = Fifo_Desc_Hdr_Push.tready;
                end

                // Wyjście ze stanu ST_HDR
                if (s_axis_fire && is_last_hdr_beat) begin
                    if (S_axis.tlast)
                        next_state = ST_IDLE;   // pakiet bez payloadu
                    else
                        next_state = ST_PAYLOAD;
                end
            end

            // =========================================================
            ST_PAYLOAD: begin
                Fifo_Payload_Push.tvalid = S_axis.tvalid;
                Fifo_Payload_Push.tlast  = S_axis.tlast;

                if (S_axis.tlast)
                    S_axis.tready = Fifo_Payload_Push.tready &&
                                    Fifo_Meta_Push.tready;
                else
                    S_axis.tready = Fifo_Payload_Push.tready;

                if (s_axis_fire && S_axis.tlast)
                    next_state = ST_IDLE;
            end

            default: next_state = ST_IDLE;
        endcase

        // ---------------------------------------------------------
        // Meta push – gdy kończy się pakiet (TLAST)
        // ---------------------------------------------------------
        if (s_axis_fire && S_axis.tlast && (state != ST_IDLE)) begin
            Fifo_Meta_Push.tvalid          = 1'b1;
            // Format metadanych (128 b):
            // [7:0]    hdr_len
            // [23:8]   total_length
            // [39:24]  payload_len
            // [40]     df_flag
            // [44:41]  IHL (hdr_len/4)
            Fifo_Meta_Push.tdata[7:0]      = hdr_len;
            Fifo_Meta_Push.tdata[23:8]     = total_length;
            Fifo_Meta_Push.tdata[39:24]    = payload_len;
            Fifo_Meta_Push.tdata[40]       = df_flag;
            Fifo_Meta_Push.tdata[44:41]    = hdr_len[5:2];
        end
    end

endmodule
