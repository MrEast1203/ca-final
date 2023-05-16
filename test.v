// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;               // ok
    output [31:0] mem_addr_D ;               // ok
    output [31:0] mem_wdata_D;               // ok
    input  [31:0] mem_rdata_D;               // ok
    // For mem_I
    output [31:0] mem_addr_I ;               // ok
    input  [31:0] mem_rdata_I;               // ok
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              // ok
    reg    [31:0] PC_nxt      ;              // ok
    wire          regWrite    ;              // ok
    wire   [ 4:0] rs1, rs2, rd;              // ok
    wire   [31:0] rs1_data    ;              // ok
    wire   [31:0] rs2_data    ;              // ok
    reg    [31:0] rd_data     ;              // ok
    //---------------------------------------//

    // Todo: other wire/reg
    reg    [31:0] ALU_result  ;
    reg    [31:0] ImmGen      ;
    wire   [ 6:0] Control     ;
    wire   [ 4:0] ALU_control ;
    // mulDiv
    reg           state       ;
    reg           state_nxt   ;
    wire   [63:0] mul_out     ;
    wire   [ 1:0] mode        ;
    wire          ready       ;
    wire          valid       ; 
    // parameters
    // op
    parameter     beq    = 7'b1100011; // branch's op
    parameter     auipc  = 7'b0010111;
    parameter     jal    = 7'b1101111;
    parameter     jalr   = 7'b1100111;
    parameter     lw     = 7'b0000011; // load's   op
    parameter     sw     = 7'b0100011; // store's  op
    parameter     R_type = 7'b0110011;
    parameter     I_type = 7'b0010011;
    // funct
    parameter     add    = 5'b00000;
    parameter     sub    = 5'b10000;
    parameter     mul    = 5'b01000;
    parameter     addi   = 3'b000;
    parameter     slti   = 3'b010;
    parameter     slli   = 3'b001;
    parameter     srli   = 3'b101;
    // state
    parameter     Single = 1'b0;
    parameter     Multi  = 1'b1;
    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    mulDiv hw2(
        .clk(clk),
        .rst_n(rst_n),
        .valid(valid),
        .ready(ready),
        .mode(mode),
        .in_A(rs1_data),
        .in_B(rs2_data),
        .out(mul_out));
    
    assign mem_addr_I  = PC;
    assign mem_wen_D   = (Control == sw);
    assign mem_addr_D  = ALU_result;
    assign mem_wdata_D = rs2_data;
    assign regWrite    = !((Control == beq) | (Control == sw)) && (state_nxt != Multi);
    assign rs1         = mem_rdata_I[19:15];
    assign rs2         = mem_rdata_I[24:20];
    assign rd          = mem_rdata_I[11: 7];
    assign Control     = mem_rdata_I[ 6: 0];
    assign ALU_control = {mem_rdata_I[30], mem_rdata_I[25], mem_rdata_I[14:12]};
    // mulDiv
    assign mode = 2'b0; // only use MUL mode
    assign valid = ((state == Single) && (state_nxt == Multi))? 1'b1 : 1'b0;
    // Todo: any combinational/sequential circuit
    // PC
    always @(*) begin
        if (state_nxt == Multi) PC_nxt = PC;
        else begin
            case (Control)
            beq:     PC_nxt = (ALU_result == 32'b0)? (PC + (ImmGen << 1)) : (PC + 4);
            jal:     PC_nxt = ALU_result;
            jalr:    PC_nxt = ALU_result;
            default: PC_nxt = PC + 4;
            // R_type, I_type, lw, sw, auipc
            endcase 
        end      
    end
    // ImmGen
    always @(*) begin
        case (Control)
            beq:     ImmGen = {{21{mem_rdata_I[31]}}, mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8]};
            auipc:   ImmGen = {mem_rdata_I[31:12], 12'b0};
            jal:     ImmGen = {{12{mem_rdata_I[31]}}, mem_rdata_I[19:12], mem_rdata_I[20], mem_rdata_I[30:21], 1'b0};
            sw:      ImmGen=  {{21{mem_rdata_I[31]}}, mem_rdata_I[30:25], mem_rdata_I[11:7]};
            R_type:  ImmGen = 32'b0;
            I_type:  begin
                case (ALU_control[2:0])
                    slli:    ImmGen = {27'b0, mem_rdata_I[24:20]};
                    srli:    ImmGen = {27'b0, mem_rdata_I[24:20]};
                    default: ImmGen = {{21{mem_rdata_I[31]}}, mem_rdata_I[30:20]};
                    // addi, slti 
                endcase
            end
            default: ImmGen = {{21{mem_rdata_I[31]}}, mem_rdata_I[30:20]};
            // jalr, lw
        endcase 
    end
    // ALU 
    always @(*) begin
        case (Control)
            beq:     ALU_result = rs1_data - rs2_data;
            auipc:   ALU_result = PC + ImmGen;
            jal:     ALU_result = PC + ImmGen;
            R_type:  begin
                case (ALU_control)
                    add:     ALU_result = rs1_data + rs2_data;
                    sub:     ALU_result = rs1_data - rs2_data;
                    mul:     ALU_result = mul_out[31:0];
                    default: ALU_result = 32'b0;
                endcase
            end
            I_type:  begin
                case (ALU_control[2:0])
                    addi:    ALU_result = rs1_data + ImmGen;
                    slti:    ALU_result = {31'b0, (rs1_data < ImmGen)};
                    slli:    ALU_result = rs1_data << ImmGen;
                    srli:    ALU_result = rs1_data >> ImmGen;
                    default: ALU_result = 32'b0;
                endcase
            end
            default: ALU_result = rs1_data + ImmGen;
            // jalr, lw, sw 
        endcase
    end
    // rd_data
    always @(*) begin
        case (Control)
            jal:     rd_data = PC + 4;
            jalr:    rd_data = PC + 4;
            lw:      rd_data = mem_rdata_D;
            default: rd_data = ALU_result;
            // auipc, R_type, I_type (no use: beq, sw)
        endcase
    end
    // FSM(mul)
    always @(*) begin
        case (state)
            Single:  state_nxt = ((Control == R_type) && (ALU_control == mul))? Multi : Single;
            Multi:   state_nxt = ready? Single : Multi;
            default: state_nxt = Single;
        endcase
    end
    // Sequential Block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC    <= 32'h00010000; // Do not modify this value!!!
            state <= Single;          
        end
        else begin
            PC    <= PC_nxt;
            state <= state_nxt;           
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2
    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [ 1:0] mode; // mode: 0: mulu, 1: divu, 2: and, 3: or
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter AND  = 3'd3;
    parameter OR   = 3'd4;
    parameter OUT  = 3'd5;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;
    reg         ready;

    // Todo 5: Wire assignments
    assign out = shreg;

    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) state_nxt = {1'b0, mode} + 1;
                else       state_nxt = IDLE;
            end
            MUL: begin
                if (counter == 31) state_nxt = OUT;
                else               state_nxt = MUL;
            end
            DIV: begin
                if (counter == 31) state_nxt = OUT;
                else               state_nxt = DIV;
            end
            AND:     state_nxt = OUT;
            OR:      state_nxt = OUT;
            default: state_nxt = IDLE;
        endcase
    end

    // Todo 2: Counter
    always @(*) begin
        case(state)
            MUL: begin
                if (counter == 31) counter_nxt = 0;
                else               counter_nxt = counter + 1;
            end
            DIV: begin
                if (counter == 31) counter_nxt = 0;
                else               counter_nxt = counter + 1;
            end
            default: counter_nxt = 0;
        endcase
    end

    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT :    alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MUL:     alu_out = {1'b0, shreg[63:32]} + {1'b0, alu_in};
            DIV:     alu_out = {1'b1, shreg[62:31]} - {1'b0, alu_in};
            AND:     alu_out = {1'b0, shreg[31:0] & alu_in};
            OR:      alu_out = {1'b0, shreg[31:0] | alu_in};
            default: alu_out = 0;
        endcase
    end

    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) shreg_nxt = {32'b0, in_A};
                else       shreg_nxt = 0;
            end
            MUL: begin
                if (shreg[0] == 0) shreg_nxt = shreg >> 1;
                else               shreg_nxt = {alu_out, shreg[31:1]};                   
            end
            DIV: begin
                if (alu_out[32] == 0) shreg_nxt = shreg << 1; // Rem < 0
                else                  shreg_nxt = {alu_out[31:0], shreg[30:0], 1'b1};
            end
            AND:     shreg_nxt = {31'b0, alu_out};
            OR:      shreg_nxt = {31'b0, alu_out};
            default: shreg_nxt = 0;
        endcase
    end

    // Output control
    always @(*) begin
        case(state)
            OUT:     ready = 1;
            default: ready = 0;
        endcase
    end
    
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            counter <= 0;
            alu_in  <= 0;
            shreg   <= 0;
        end
        else begin
            state   <= state_nxt;
            counter <= counter_nxt;
            alu_in  <= alu_in_nxt;
            shreg   <= shreg_nxt;
        end
    end
endmodule