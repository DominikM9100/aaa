module Receiver #(
    parameter int    IFACES_NUMBER = 4,
    parameter string TYPE          = "IFACE",   // "PREFRAG" | "IFACE"
    parameter int    MAX_MTU       = 1500
)(
    input  logic                          Clk,
    input  logic                          Rst,
    input  logic [IFACES_NUMBER*4-1:0]    Ifaces_values,
    input  logic [IFACES_NUMBER*16-1:0]   Mtu_port,
    axi4streamif.slave                    S_axis,
    axi4streamif.master                   Fifo_Desc_Hdr,
    axi4streamif.master                   Fifo_Payload,
    axi4streamif.master                   Fifo_Meta
);

    // -----------------------------------------------------------------
    // FSM states
    // -----------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_IDLE,
        ST_HDR,
        ST_PAYLOAD
    } state_t;

    state_t state, next_state;

    // -----------------------------------------------------------------
    // Parsed values
    // -----------------------------------------------------------------
    logic [7:0]  iface_id_reg;
    logic [3:0]  ihl_reg;             // IHL in 32-bit words
    logic [15:0] total_len_reg;
    logic [15:0] id_reg;
    logic [15:0] flags_frag_reg;
    logic [15:0] mtu_desc_reg;        // for TYPE == "PREFRAG"

    logic [15:0] hdr_bytes_count;     // IP header bytes already received
    logic [15:0] hdr_bytes_total;     // = IHL * 4

    logic        df_flag;
    assign df_flag         = flags_frag_reg[14];
    assign hdr_bytes_total = {12'b0, ihl_reg, 2'b00};   // IHL * 4

    // -----------------------------------------------------------------
    // MTU selection
    // -----------------------------------------------------------------
    logic [15:0] mtu_selected;

    generate
        if (TYPE == "PREFRAG") begin : g_prefrag
            always_comb mtu_selected = mtu_desc_reg;
        end else begin : g_iface
            always_comb begin
                mtu_selected = '0;
                for (int i = 0; i < IFACES_NUMBER; i++) begin
                    if (Ifaces_values[4*i +: 4] == iface_id_reg[3:0])
                        mtu_selected = Mtu_port[16*i +: 16];
                end
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Metadata word
    // {mode[3], hdr_len[4], payload_len[16], ifaces[8], param_1[16], param_0[16]}
    // -----------------------------------------------------------------
    logic        frwd_flag, frag_flag, os_flag;
    logic [2:0]  mode_bits;
    logic [3:0]  hdr_len_meta;
    logic [15:0] payload_len_meta;
    logic [7:0]  ifaces_meta;
    logic [15:0] param_1_meta;
    logic [15:0] param_0_meta;
    logic [62:0] meta_word;

    assign frwd_flag = (mtu_selected < MAX_MTU);
    assign frag_flag = (mtu_selected > MAX_MTU) && !df_flag;
    assign os_flag   = (mtu_selected > MAX_MTU) &&  df_flag;

    assign mode_bits        = {frwd_flag, frag_flag, os_flag};
    assign hdr_len_meta     = ihl_reg;
    assign payload_len_meta = total_len_reg - hdr_bytes_total;
    assign ifaces_meta      = iface_id_reg;
    assign param_0_meta     = id_reg;
    assign param_1_meta     = flags_frag_reg;

    assign meta_word = {mode_bits, hdr_len_meta, payload_len_meta,
                        ifaces_meta, param_1_meta, param_0_meta};

    // -----------------------------------------------------------------
    // FSM current-state register
    // -----------------------------------------------------------------
    always_ff @(posedge Clk) begin
        if (Rst) state <= ST_IDLE;
        else     state <= next_state;
    end

    // -----------------------------------------------------------------
    // Parse values from descriptor / IP header
    // -----------------------------------------------------------------
    always_ff @(posedge Clk) begin
        if (Rst) begin
            iface_id_reg   <= '0;
            ihl_reg        <= '0;
            total_len_reg  <= '0;
            id_reg         <= '0;
            flags_frag_reg <= '0;
            mtu_desc_reg   <= '0;
        end else begin
            // Descriptor beat
            if (state == ST_IDLE && S_axis.tvalid && S_axis.tready) begin
                iface_id_reg <= S_axis.tdata[71:56];
                // Dla TYPE="PREFRAG" – przykładowe miejsce MTU w deskryptorze
                mtu_desc_reg <= S_axis.tdata[95:80];
            end

            // First header beat
            if (state == ST_HDR && hdr_bytes_count == 0 &&
                S_axis.tvalid && S_axis.tready) begin
                ihl_reg        <= S_axis.tdata[3:0];
                total_len_reg  <= S_axis.tdata[31:16];
                id_reg         <= S_axis.tdata[47:32];
                flags_frag_reg <= S_axis.tdata[63:48];
            end
        end
    end

    // -----------------------------------------------------------------
    // Header byte counter
    // -----------------------------------------------------------------
    always_ff @(posedge Clk) begin
        if (Rst) begin
            hdr_bytes_count <= '0;
        end else begin
            if (state == ST_IDLE && S_axis.tvalid && S_axis.tready)
                hdr_bytes_count <= '0;
            else if (state == ST_HDR && S_axis.tvalid && S_axis.tready)
                hdr_bytes_count <= hdr_bytes_count + 16'($countones(S_axis.tkeep));
        end
    end

    // -----------------------------------------------------------------
    // FSM next-state (combinational)
    // -----------------------------------------------------------------
    always_comb begin
        next_state = state;
        case (state)
            ST_IDLE: begin
                if (S_axis.tvalid && S_axis.tready)
                    next_state = ST_HDR;
            end
            ST_HDR: begin
                if (S_axis.tvalid && S_axis.tready &&
                    (hdr_bytes_count + 16'($countones(S_axis.tkeep)) >= hdr_bytes_total))
                    next_state = ST_PAYLOAD;
            end
            ST_PAYLOAD: begin
                if (S_axis.tvalid && S_axis.tready && S_axis.tlast)
                    next_state = ST_IDLE;
            end
        endcase
    end

    // -----------------------------------------------------------------
    // AXI-Stream routing (S_axis -> Fifo_Desc_Hdr / Fifo_Payload)
    // -----------------------------------------------------------------
    always_comb begin
        // defaults
        Fifo_Desc_Hdr.tvalid = 1'b0;
        Fifo_Desc_Hdr.tdata  = S_axis.tdata;
        Fifo_Desc_Hdr.tkeep  = S_axis.tkeep;
        Fifo_Desc_Hdr.tlast  = 1'b0;
        Fifo_Desc_Hdr.tstrb  = S_axis.tstrb;
        Fifo_Desc_Hdr.tdest  = S_axis.tdest;
        Fifo_Desc_Hdr.tid    = S_axis.tid;
        Fifo_Desc_Hdr.tuser  = S_axis.tuser;

        Fifo_Payload.tvalid  = 1'b0;
        Fifo_Payload.tdata   = S_axis.tdata;
        Fifo_Payload.tkeep   = S_axis.tkeep;
        Fifo_Payload.tlast   = 1'b0;
        Fifo_Payload.tstrb   = S_axis.tstrb;
        Fifo_Payload.tdest   = S_axis.tdest;
        Fifo_Payload.tid     = S_axis.tid;
        Fifo_Payload.tuser   = S_axis.tuser;

        S_axis.tready = 1'b0;

        case (state)
            ST_IDLE: begin
                Fifo_Desc_Hdr.tvalid = S_axis.tvalid;
                S_axis.tready        = Fifo_Desc_Hdr.tready;
            end
            ST_HDR: begin
                Fifo_Desc_Hdr.tvalid = S_axis.tvalid;
                Fifo_Desc_Hdr.tlast  =
                    (hdr_bytes_count + 16'($countones(S_axis.tkeep)) >= hdr_bytes_total);
                S_axis.tready        = Fifo_Desc_Hdr.tready;
            end
            ST_PAYLOAD: begin
                Fifo_Payload.tvalid  = S_axis.tvalid;
                Fifo_Payload.tlast   = S_axis.tlast;
                S_axis.tready        = Fifo_Payload.tready;
            end
        endcase
    end

    // -----------------------------------------------------------------
    // Metadata write to Fifo_Meta (single-beat packet)
    // -----------------------------------------------------------------
    logic        meta_pending;
    logic [62:0] meta_reg;

    always_ff @(posedge Clk) begin
        if (Rst) begin
            meta_pending <= 1'b0;
            meta_reg     <= '0;
        end else begin
            if (state == ST_HDR && S_axis.tvalid && S_axis.tready &&
                (hdr_bytes_count + 16'($countones(S_axis.tkeep)) >= hdr_bytes_total)) begin
                meta_reg     <= meta_word;
                meta_pending <= 1'b1;
            end
            else if (meta_pending && Fifo_Meta.tready) begin
                meta_pending <= 1'b0;
            end
        end
    end

    always_comb begin
        Fifo_Meta.tvalid      = meta_pending;
        Fifo_Meta.tdata       = '0;
        Fifo_Meta.tdata[62:0] = meta_reg;
        Fifo_Meta.tkeep       = '1;
        Fifo_Meta.tlast       = 1'b1;
        Fifo_Meta.tstrb       = '1;
        Fifo_Meta.tdest       = '0;
        Fifo_Meta.tid         = '0;
        Fifo_Meta.tuser       = '0;
    end

endmodule





















`timescale 1ns/1ps

// ============================================================================
// Bardzo prosty testbench dla Receiver.
// Wysyla 3 pakiety (deskryptor + IPv4), FIFO to zawsze-gotowe odbiorniki,
// monitory wypisuja beaty i porownuja meta oraz dlugosc payloadu z oczekiwaniami.
//
//  pkt 1: IFACE_ID=2 (MTU 1500), IHL=5, payload   8 B -> total  28 B -> frwd
//  pkt 2: IFACE_ID=1 (MTU  100), IHL=5, payload 180 B -> total 200 B, DF=0 -> frag
//  pkt 3: IFACE_ID=1 (MTU  100), IHL=6, payload 100 B -> total 124 B, DF=1 -> os
// ============================================================================
module tb_Receiver;

    localparam int IFACES_NUMBER = 2;

    // ------------------------------------------------------------------
    // Zegar 200 MHz i reset
    // ------------------------------------------------------------------
    logic clk = 1'b0;
    logic rst = 1'b1;
    always #2.5 clk = ~clk;

    // ------------------------------------------------------------------
    // Interfejsy
    // ------------------------------------------------------------------
    axi4streamif #(.TDATA(128), .TDEST(4)) s_axis        (clk, rst);
    axi4streamif #(.TDATA(128), .TDEST(4)) fifo_desc_hdr (clk, rst);
    axi4streamif #(.TDATA(128), .TDEST(4)) fifo_payload  (clk, rst);
    axi4streamif #(.TDATA(128), .TDEST(4)) fifo_meta     (clk, rst);

    // interfejs 0: IFACE_ID=1, MTU=100; interfejs 1: IFACE_ID=2, MTU=1500
    logic [IFACES_NUMBER*4-1:0]  ifaces_values = {4'd2, 4'd1};
    logic [IFACES_NUMBER*16-1:0] mtu_port      = {16'd1500, 16'd100};

    Receiver #(
        .IFACES_NUMBER (IFACES_NUMBER),
        .TYPE          ("IFACE"),
        .MAX_MTU       (1500)
    ) dut (
        .Clk           (clk),
        .Rst           (rst),
        .Ifaces_values (ifaces_values),
        .Mtu_port      (mtu_port),
        .S_axis        (s_axis),
        .Fifo_Desc_Hdr (fifo_desc_hdr),
        .Fifo_Payload  (fifo_payload),
        .Fifo_Meta     (fifo_meta)
    );

    // ------------------------------------------------------------------
    // Oczekiwania
    // ------------------------------------------------------------------
    logic [62:0] exp_meta   [$];
    int          exp_pl_len [$];

    // ------------------------------------------------------------------
    // Wyslanie jednego beatu (czeka na tready)
    // ------------------------------------------------------------------
    task automatic send_beat(input logic [127:0] d, input logic [15:0] k, input logic last);
        s_axis.tdata  <= d;
        s_axis.tkeep  <= k;
        s_axis.tstrb  <= k;
        s_axis.tlast  <= last;
        s_axis.tvalid <= 1'b1;
        do begin
            @(posedge clk);
        end while (!s_axis.tready);
        s_axis.tvalid <= 1'b0;
        s_axis.tlast  <= 1'b0;
    endtask

    // ------------------------------------------------------------------
    // Wyslanie pakietu: deskryptor (1 beat, 12 B) + naglowek IPv4 + payload
    // ------------------------------------------------------------------
    task automatic send_packet(input logic [7:0]  iface_id,
                               input int          ihl,          // slowa 32-bit
                               input int          payload_len,  // bajty
                               input logic [15:0] id,
                               input logic        df);
        byte unsigned pkt[$];
        int           hdr_len   = ihl * 4;
        int           total_len = hdr_len + payload_len;
        logic [127:0] d;
        logic [15:0]  k;
        logic [15:0]  mtu;
        logic [2:0]   mode;
        logic [15:0]  flags;

        // --- oczekiwana meta ---
        mtu   = (iface_id == 8'd1) ? 16'd100 : 16'd1500;
        flags = {1'b0, df, 14'd0};
        if (total_len <= mtu) mode = 3'b100;           // frwd
        else if (!df)         mode = 3'b010;           // frag
        else                  mode = 3'b001;           // os
        exp_meta.push_back({mode, 4'(ihl), 16'(payload_len), iface_id, flags, id});
        exp_pl_len.push_back(payload_len);

        // --- deskryptor: IFACE_ID w bitach [63:56] ---
        d = '0;
        d[63:56] = iface_id;
        send_beat(d, 16'h0FFF, 1'b0);

        // --- naglowek IPv4 ---
        for (int i = 0; i < hdr_len; i++) pkt.push_back(8'h00);
        pkt[0]  = {4'd4, 4'(ihl)};
        pkt[2]  = total_len[15:8];
        pkt[3]  = total_len[7:0];
        pkt[4]  = id[15:8];
        pkt[5]  = id[7:0];
        pkt[6]  = df ? 8'h40 : 8'h00;
        pkt[8]  = 8'd64;      // TTL
        pkt[9]  = 8'd17;      // UDP
        pkt[12] = 8'd10; pkt[15] = 8'd1;    // src 10.0.0.1
        pkt[16] = 8'd10; pkt[19] = 8'd2;    // dst 10.0.0.2

        // --- payload ---
        for (int i = 0; i < payload_len; i++) pkt.push_back(8'(160 + i));

        // --- beaty (bajt 0 -> tdata[7:0]) ---
        for (int base = 0; base < total_len; base += 16) begin
            d = '0;
            k = '0;
            for (int j = 0; j < 16; j++) begin
                if (base + j < total_len) begin
                    d[8*j +: 8] = pkt[base + j];
                    k[j]        = 1'b1;
                end
            end
            send_beat(d, k, (base + 16 >= total_len));
        end
    endtask

    // ------------------------------------------------------------------
    // Monitory
    // ------------------------------------------------------------------
    int pl_bytes = 0;

    always @(posedge clk) begin
        if (!rst) begin
            if (fifo_desc_hdr.tvalid && fifo_desc_hdr.tready)
                $display("%8t DESC_HDR data=%032h keep=%04h last=%b",
                         $time, fifo_desc_hdr.tdata, fifo_desc_hdr.tkeep, fifo_desc_hdr.tlast);

            if (fifo_payload.tvalid && fifo_payload.tready) begin
                $display("%8t PAYLOAD  data=%032h keep=%04h last=%b",
                         $time, fifo_payload.tdata, fifo_payload.tkeep, fifo_payload.tlast);
                pl_bytes += $countones(fifo_payload.tkeep);
                if (fifo_payload.tlast) begin
                    if (exp_pl_len.size() == 0)
                        $error("payload: nieoczekiwany pakiet");
                    else begin
                        int e;
                        e = exp_pl_len.pop_front();
                        if (pl_bytes != e) $error("payload: %0d B, oczekiwano %0d B", pl_bytes, e);
                        else               $display("         payload OK (%0d B)", pl_bytes);
                    end
                    pl_bytes = 0;
                end
            end

            if (fifo_meta.tvalid && fifo_meta.tready) begin
                $display("%8t META     %016h", $time, fifo_meta.tdata[62:0]);
                if (exp_meta.size() == 0)
                    $error("meta: nieoczekiwana");
                else begin
                    logic [62:0] e;
                    e = exp_meta.pop_front();
                    if (fifo_meta.tdata[62:0] !== e)
                        $error("meta: %h, oczekiwano %h", fifo_meta.tdata[62:0], e);
                    else
                        $display("         meta OK (mode=%03b hdr_len=%0d payload_len=%0d)",
                                 e[62:60], e[59:56], e[55:40]);
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Scenariusz
    // ------------------------------------------------------------------
    initial begin
        s_axis.tvalid = 1'b0;
        s_axis.tdata  = '0;
        s_axis.tkeep  = '0;
        s_axis.tstrb  = '0;
        s_axis.tlast  = 1'b0;
        s_axis.tid    = '0;
        s_axis.tdest  = '0;
        s_axis.tuser  = '0;

        fifo_desc_hdr.tready = 1'b1;
        fifo_payload.tready  = 1'b1;
        fifo_meta.tready     = 1'b1;

        repeat (5) @(posedge clk);
        rst <= 1'b0;
        repeat (2) @(posedge clk);

        send_packet(8'h02, 5,   8, 16'h1234, 1'b1);   // frwd, pakiet konczy sie w beacie granicznym
        send_packet(8'h01, 5, 180, 16'hBEEF, 1'b0);   // frag
        send_packet(8'h01, 6, 100, 16'hCAFE, 1'b1);   // os (IHL=6, offset payloadu 8 B)

        repeat (20) @(posedge clk);

        if (exp_meta.size() != 0 || exp_pl_len.size() != 0)
            $error("brakuje wyjsc: meta=%0d payload=%0d", exp_meta.size(), exp_pl_len.size());
        else
            $display("TB: koniec, wszystkie oczekiwania spelnione");
        $finish;
    end

endmodule





























// ============================================================================
// frag_top_example - przyklad uzycia axi4streamif w projekcie Vivado
//
//  * Top ma PLASKIE porty (interfejsy SV nie moga byc na porcie top-level).
//  * Interfejsy axi4streamif sa tworzone tutaj i podlaczane do Receivera.
//  * Miedzy Receiverem a reszta ukladu stoja IP "AXI4-Stream Data FIFO"
//    (porty plaskie s_axis_* / m_axis_*, reset aktywny NISKO -> ~Rst).
//
// Zalozona konfiguracja IP (nazwy komponentow = nazwy modulow ponizej):
//   axis_desc_hdr_fifo : 16 B, TKEEP, TLAST, TDEST=4, glebokosc >= najwiekszy pakiet w beatach
//   axis_payload_fifo  : jak wyzej
//   axis_meta_fifo     :  8 B, TLAST, bez TKEEP/TDEST, glebokosc np. 512
// Nieuzywane sygnaly (tid, tuser, tstrb) - w IP wylaczone, wiec nie sa podlaczone.
// ============================================================================
module frag_top_example #(
    parameter int IFACES_NUMBER = 4
)(
    input  logic                         Clk,
    input  logic                         Rst,            // aktywny '1'
    input  logic [IFACES_NUMBER*4-1:0]   Ifaces_values,
    input  logic [IFACES_NUMBER*16-1:0]  Mtu_port,

    // wejscie (plaskie)
    input  logic [127:0]                 s_tdata,
    input  logic [15:0]                  s_tkeep,
    input  logic [3:0]                   s_tdest,
    input  logic                         s_tlast,
    input  logic                         s_tvalid,
    output logic                         s_tready,

    // wyjscie: deskryptor + naglowek
    output logic [127:0]                 dh_tdata,
    output logic [15:0]                  dh_tkeep,
    output logic [3:0]                   dh_tdest,
    output logic                         dh_tlast,
    output logic                         dh_tvalid,
    input  logic                         dh_tready,

    // wyjscie: payload
    output logic [127:0]                 pl_tdata,
    output logic [15:0]                  pl_tkeep,
    output logic [3:0]                   pl_tdest,
    output logic                         pl_tlast,
    output logic                         pl_tvalid,
    input  logic                         pl_tready,

    // wyjscie: meta (63 bity)
    output logic [63:0]                  mt_tdata,
    output logic                         mt_tlast,
    output logic                         mt_tvalid,
    input  logic                         mt_tready
);

    // ------------------------------------------------------------------
    // Interfejsy
    // ------------------------------------------------------------------
    axi4streamif #(.TDATA(128), .TDEST(4)) s_axis        (Clk, Rst);
    axi4streamif #(.TDATA(128), .TDEST(4)) fifo_desc_hdr (Clk, Rst);
    axi4streamif #(.TDATA(128), .TDEST(4)) fifo_payload  (Clk, Rst);
    axi4streamif #(.TDATA(64),  .TDEST(1)) fifo_meta     (Clk, Rst);

    // plaskie porty top -> interfejs wejsciowy
    assign s_axis.tdata  = s_tdata;
    assign s_axis.tkeep  = s_tkeep;
    assign s_axis.tstrb  = s_tkeep;
    assign s_axis.tdest  = s_tdest;
    assign s_axis.tid    = '0;
    assign s_axis.tuser  = '0;
    assign s_axis.tlast  = s_tlast;
    assign s_axis.tvalid = s_tvalid;
    assign s_tready      = s_axis.tready;

    // ------------------------------------------------------------------
    // Receiver
    // ------------------------------------------------------------------
    Receiver #(
        .IFACES_NUMBER (IFACES_NUMBER),
        .TYPE          ("IFACE"),
        .MAX_MTU       (1500)
    ) u_receiver (
        .Clk           (Clk),
        .Rst           (Rst),
        .Ifaces_values (Ifaces_values),
        .Mtu_port      (Mtu_port),
        .S_axis        (s_axis),
        .Fifo_Desc_Hdr (fifo_desc_hdr),
        .Fifo_Payload  (fifo_payload),
        .Fifo_Meta     (fifo_meta)
    );

    // ------------------------------------------------------------------
    // FIFO IP (Receiver = master interfejsu, IP = slave; tready wraca do interfejsu)
    // ------------------------------------------------------------------
    axis_desc_hdr_fifo u_fifo_desc_hdr (
        .s_axis_aclk    (Clk),
        .s_axis_aresetn (~Rst),
        .s_axis_tvalid  (fifo_desc_hdr.tvalid),
        .s_axis_tready  (fifo_desc_hdr.tready),
        .s_axis_tdata   (fifo_desc_hdr.tdata),
        .s_axis_tkeep   (fifo_desc_hdr.tkeep),
        .s_axis_tlast   (fifo_desc_hdr.tlast),
        .s_axis_tdest   (fifo_desc_hdr.tdest),
        .m_axis_tvalid  (dh_tvalid),
        .m_axis_tready  (dh_tready),
        .m_axis_tdata   (dh_tdata),
        .m_axis_tkeep   (dh_tkeep),
        .m_axis_tlast   (dh_tlast),
        .m_axis_tdest   (dh_tdest)
    );

    axis_payload_fifo u_fifo_payload (
        .s_axis_aclk    (Clk),
        .s_axis_aresetn (~Rst),
        .s_axis_tvalid  (fifo_payload.tvalid),
        .s_axis_tready  (fifo_payload.tready),
        .s_axis_tdata   (fifo_payload.tdata),
        .s_axis_tkeep   (fifo_payload.tkeep),
        .s_axis_tlast   (fifo_payload.tlast),
        .s_axis_tdest   (fifo_payload.tdest),
        .m_axis_tvalid  (pl_tvalid),
        .m_axis_tready  (pl_tready),
        .m_axis_tdata   (pl_tdata),
        .m_axis_tkeep   (pl_tkeep),
        .m_axis_tlast   (pl_tlast),
        .m_axis_tdest   (pl_tdest)
    );

    axis_meta_fifo u_fifo_meta (
        .s_axis_aclk    (Clk),
        .s_axis_aresetn (~Rst),
        .s_axis_tvalid  (fifo_meta.tvalid),
        .s_axis_tready  (fifo_meta.tready),
        .s_axis_tdata   (fifo_meta.tdata),
        .s_axis_tlast   (fifo_meta.tlast),
        .m_axis_tvalid  (mt_tvalid),
        .m_axis_tready  (mt_tready),
        .m_axis_tdata   (mt_tdata),
        .m_axis_tlast   (mt_tlast)
    );

endmodule

















