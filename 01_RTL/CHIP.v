//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,  //valid_inst                                                      //
    // data memory                                                                              //
        input               i_DMEM_stall,   //nop                                                   //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,  //valid_data                                                       //
        output              o_DMEM_wen,   // ready                                                      //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata                                                        //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration
        parameter       OP_AUIPC = 7'b0010111;
        parameter       OP_JAL   = 7'b1101111;
        parameter       OP_JALR  = 7'b1100111;

        // R_type
        parameter       R_type   = 7'b0110011;
        parameter       add      = 5'b00000;
        parameter       sub      = 5'b10000;
        parameter       mul      = 5'b01000;
        parameter       anD      = 5'b00111;
        parameter       xoR      = 5'b00100;

        // I_type
        parameter       I_type   = 7'b0010011;
        parameter       addi     = 3'b000;
        parameter       slli     = 3'b001;
        parameter       slti     = 3'b010;
        parameter       srai     = 3'b101;

        parameter       OP_LW    = 7'b0000011; // load's   op
        parameter       OP_SW    = 7'b0100011; // store's  op
        // SB_type
        parameter       SB_type  = 7'b1100011; //beq,bne,blt,bge
        parameter       beq      = 3'b000;
        parameter       bne      = 3'b001;
        parameter       blt      = 3'b100;
        parameter       bge      = 3'b101;
        // state      
        parameter       Single   = 1'b0;
        parameter       Multi    = 1'b1;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg   [  BIT_W-1:0] PC, next_PC;
        wire                mem_cen, mem_wen;  // regWrite, MemRead/Write
        wire  [  BIT_W-1:0] Read_data1, Read_data2; // rs1_data,rs2_data
        reg   [  BIT_W-1:0] Write_data; // rd_data 
        wire  [        4:0] Read_register1, Read_register2, Write_register; // rs1,rs2,rd

        wire  [        6:0] op;
        reg   [  BIT_W-1:0] ALU_result;
        reg   [  BIT_W-1:0] ImmGen;
        wire  [        4:0] ALU_Control;

        reg                 state,state_nxt;
        wire  [        1:0] mode;
        wire  [2*BIT_W-1:0] out;
        wire                ready;
        wire                valid; 
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
       // PC
        assign o_IMEM_addr = PC;

        // Registers
        assign Read_register1= i_IMEM_data[19:15];
        assign Read_register2 = i_IMEM_data[24:20];
        assign Write_register  = i_IMEM_data[11:7];
        assign op = i_IMEM_data[6:0];
        assign mem_cen = !((op == SB_type) | (op == OP_SW)) && (state_nxt != Multi);
        assign o_IMEM_cen = mem_cen;

        // ALU
        assign ALU_Control = {i_IMEM_data[30], i_IMEM_data[25], i_IMEM_data[14:12]};
        assign mode = 2'b0;
        assign valid = ((state == Single) && (state_nxt == Multi))? 1'b1 : 1'b0;

        // Data memory
        assign mem_wen = (op == OP_SW);
        assign o_DMEM_wen = mem_wen;
        assign o_DMEM_addr = ALU_result;
        assign o_DMEM_wdata = Read_data2;
        assign o_DMEM_cen = ((op == OP_LW) | (op == OP_SW));

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (mem_wen),          
        .rs1    (Read_register1),                
        .rs2    (Read_register2),                
        .rd     (Write_register),                 
        .wdata  (Write_data),             
        .rdata1 (Read_data1),           
        .rdata2 (Read_data2)
    );

    MULDIV_unit hw2(
        .i_clk  (i_clk),
        .i_rst_n(i_rst_n),
        .i_valid(valid),
        .o_done (ready),
        .i_inst (mode),
        .i_A    (Read_data1),
        .i_B    (Read_data2),
        .o_data (out)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit
    // PC
    always @(*) begin
        if (state_nxt == Multi) next_PC = PC;
        else begin
            case (op)
            SB_type: next_PC = (ALU_result == 32'b0)? (PC + (ImmGen << 1)) : (PC + 4);
            OP_JAL : next_PC = ALU_result;
            OP_JALR: next_PC = ALU_result;
            default: next_PC = PC + 4;
            // R_type, I_type, OP_LW, OP_SW, OP_AUIPC
            endcase 
        end      
    end

    // WB
    always @(*) begin
        case (op)
            OP_JAL:     Write_data = PC + 4;
            OP_JALR:    Write_data = PC + 4;
            OP_LW:      Write_data = i_DMEM_rdata;
            default:    Write_data = ALU_result;
        endcase
    end

    // ImmGen
    always @(*) begin
        case (op)
            SB_type :   ImmGen = {{21{i_IMEM_data[31]}}, i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8]};
            OP_AUIPC:   ImmGen = {i_IMEM_data[31:12], 12'b0};
            OP_JAL:     ImmGen = {{12{i_IMEM_data[31]}}, i_IMEM_data[19:12], i_IMEM_data[20], i_IMEM_data[30:21], 1'b0};
            OP_SW:      ImmGen=  {{21{i_IMEM_data[31]}}, i_IMEM_data[30:25], i_IMEM_data[11:7]};
            R_type:  ImmGen = 32'b0;
            I_type:  begin
                case (ALU_Control[2:0])
                    slli:    ImmGen = {27'b0, i_IMEM_data[24:20]};
                    srai:    ImmGen = {27'b0, i_IMEM_data[24:20]};
                    default: ImmGen = {{21{i_IMEM_data[31]}}, i_IMEM_data[30:20]};
                    // addi, slti 
                endcase
            end
            default: ImmGen = {{21{i_IMEM_data[31]}}, i_IMEM_data[30:20]};
            // OP_JALR, OP_LW
        endcase 
    end


    // ALU 
    always @(*) begin
        case (op)
            SB_type: begin
                case (ALU_Control[2:0])
                    beq:     ALU_result = Read_data1 - Read_data2;
                    bne:     ALU_result = (Read_data1 == Read_data2)? ALU_result : 32'b0;
                    blt:     ALU_result = (Read_data1 < Read_data2)? 32'b0 : ALU_result;
                    bge:     ALU_result = (Read_data1 < Read_data2)? ALU_result : 32'b0;
                    default: ALU_result = ALU_result;
                endcase
            end
            OP_AUIPC:   ALU_result = PC + ImmGen;
            OP_JAL:     ALU_result = PC + ImmGen;
            R_type: begin
                case (ALU_Control)
                    add:     ALU_result = Read_data1 + Read_data2;
                    sub:     ALU_result = Read_data1 - Read_data2;
                    anD:     ALU_result = Read_data1 & Read_data2;
                    xoR:     ALU_result = Read_data1 | Read_data2;
                    mul:     ALU_result = out[31:0];
                    default: ALU_result = 32'b0;
                endcase
            end
            I_type:  begin
                case (ALU_Control[2:0])
                    addi:    ALU_result = Read_data1 + ImmGen;
                    slli:    ALU_result = Read_data1 << ImmGen;
                    slti:    ALU_result = {31'b0, (Read_data1 < ImmGen)};
                    srai:    ALU_result = Read_data1 >> ImmGen;
                    default: ALU_result = 32'b0;
                endcase
            end
            default: ALU_result = Read_data1 + ImmGen;
            // OP_JALR, OP_LW, OP_SW
        endcase
    end
 

    // FSM(mul)
    always @(*) begin
        case (state)
            Single:  state_nxt = ((op == R_type) && (ALU_Control == mul))? Multi : Single;
            Multi:   state_nxt = ready? Single : Multi;
            default: state_nxt = Single;
        endcase
    end

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= Single;
        end
        else begin
            PC <= next_PC;
            state <= state_nxt;
        end
    end
endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
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

module MULDIV_unit(i_clk, i_rst_n, i_valid, i_A, i_B, i_inst, o_data, o_done);
    // TODO: port declaration
    parameter DATA_W = 32;
    input                       i_clk;
    input                       i_rst_n;

    input                       i_valid;
    input [DATA_W - 1 : 0]      i_A;
    input [DATA_W - 1 : 0]      i_B;
    input [         1 : 0]      i_inst;

    output [2*DATA_W - 1 : 0]   o_data;
    output                      o_done;
    // Todo: HW2
    // Parameters
    // Definition of states
    parameter S_IDLE = 2'd0;
    parameter S_MUL  = 2'd1;
    parameter S_OUT  = 2'd2;
    

// Wires & Regs
    // Todo
    reg        [            3 : 0]       state, next_state;
    reg        [            4 : 0]       count, next_count;
    reg        [ 2*DATA_W - 1 : 0]       shift;
    reg        [ 2*DATA_W - 1 : 0]       next_shift;
    reg        [      DATA_W  : 0]       output_data;
    reg        [   DATA_W - 1 : 0]       input_data;
    reg        [   DATA_W - 1 : 0]       next_input_data;
    reg                                  o_valid;

// Wire Assignments
    // Todo
    assign o_data = shift;
    assign o_done = o_valid;
    
// Always Combination
    // Todo: FSM
    always @(*) begin
        case(state)
            S_IDLE  : begin
                if (i_valid && i_inst==2'b0) begin
                    next_state <= S_MUL;
                end
                else begin
                    next_state <= S_IDLE;
                end
            end
            S_MUL   : begin
                if (count==31) begin
                    next_state <= S_OUT;
                end
                else begin
                    next_state <= S_MUL;
                end
            end
            S_OUT   : begin
                next_state <= S_IDLE;                  
            end
            default : next_state <= 0;
        endcase
    end

    // Todo: Counter
    always @(*) begin
        if(state==S_MUL && count<31) begin
            next_count = count + 1;
        end
        else begin
            next_count = 0;
        end
    end

    // Todo: ALU output
    always @(*) begin
        case(state)
            S_IDLE   : begin
                output_data = 0;
            end           
            S_MUL   : begin
                if (shift[0] == 1) begin
                    output_data = i_B + shift[63:32];
                end 
                else begin
                    output_data = shift[63:32];
                end
            end
            S_OUT   : begin
                    output_data = 0;                 
            end
            default : output_data = 0;
        endcase
    end

    // Todo: Shift register
    always @(*) begin
        if(state==S_OUT) o_valid = 1;
        else o_valid = 0;
    end
    always @(*) begin
        if (!i_valid) begin
            next_input_data = 0;
        end
        else begin
            next_input_data = i_B;
        end
    end
    always @(*) begin
        case(state)
            S_IDLE   : begin
                if (!i_valid) begin
                    next_shift[31:0] = 0;
                    next_shift[63:32] = 32'b0;
                end
                else begin
                    next_shift[31:0] = i_A;
                    next_shift[63:32] = 32'b0;
                end
            end
            S_MUL   : begin
                next_shift = {output_data, shift[31:1]};
            end
            S_OUT   : begin
                next_shift = shift;                 
            end
            default : next_shift = shift;
        endcase
    end
    
    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state <= S_IDLE;
            shift <= 0;
            count <= 0;
            input_data <= 0;
        end
        else begin
            state <= next_state;
            count <= next_count;
            shift <= next_shift;
            input_data <= next_input_data;
        end
    end
endmodule




module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W-1:0]  o_mem_wdata,
            input [BIT_W-1:0] i_mem_rdata,
            input i_mem_stall
    );

    //---------------------------------------//
    //          default connection           //
    assign o_mem_cen = i_proc_cen;        //
    assign o_mem_wen = i_proc_wen;        //
    assign o_mem_addr = i_proc_addr;      //
    assign o_mem_wdata = i_proc_wdata;    //
    assign o_proc_rdata = i_mem_rdata;    //
    assign o_proc_stall = i_mem_stall;    //
    //---------------------------------------//

    // Todo: BONUS
endmodule