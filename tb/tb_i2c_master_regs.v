/********1*********2*********3*********4*********5*********6*********7*********8
* File : tb_i2c_master_regs.v
*_______________________________________________________________________________
*
* Revision history
*
* Name          Date        Observations
* ------------------------------------------------------------------------------
* -            01/05/2023   First version.
* ------------------------------------------------------------------------------
*_______________________________________________________________________________
*
* Description
* Testbench to verify the I2C Control and Configuration Registers.
*_______________________________________________________________________________

* (c) Copyright Universitat de Barcelona, 2023
*
*********1*********2*********3*********4*********5*********6*********7*********/

`include "../misc/timescale.v"
`include "../rtl/i2c_master_defines.v"
`include "../misc/sys_model.v"
`include "../misc/dbus_master_model.v"

// verification level: RTL_LVL GATE_LVL
`define RTL_LVL

module tb_i2c_master_regs(); // module name (same as the file)
  //___________________________________________________________________________
  // input output signals for the DUT
  parameter         DWIDTH = 8;     // bus data size
  parameter         AWIDTH = 3;     // bus addres size

  // bus
  wire              clk;            // master clock input
  wire              rst_n;          // async active low reset
  wire [AWIDTH-1:0] addr;           // lower address bits
  wire [DWIDTH-1:0] dataIn;         // databus input
  reg  [DWIDTH-1:0] dataOut;        // databus output
  wire              wr;             // write enable input
  reg               int;            // interrupt request signal output

  // commands
  wire              start;          // generate (repeated) start condition
  wire              stop;           // generate stop condition
  wire              read;           // read from slave
  wire              write;          // write to slave
  wire              tx_ack;         // when a receiver, sent ACK (Txack='0') or NACK (Txack ='1')
  wire              rx_ack;         // received ack bit
  wire        [7:0] rx_data;        // data received from slave
  wire        [7:0] tx_data;        // data to be transmitted to slave
  wire        [7:0] prescale;       // I2C clock presacel value
  wire              i2C_busy;       // bus busy (start signal detected)
  wire              i2C_done;       // command completed, used to clear command register
  wire              i2C_en;         // enables the i2c core
  wire              i2C_al;         // I2C bus arbitration lost


  // test signals
  integer errors;                   // Accumulated errors during the simulation
  integer vExpected;                // expected value
  integer vObtained;                // obtained value
  reg  [DWIDTH-1:0] data2write; // data to load in the shift register
  reg    [AWIDTH:0] addr2write; // data to load in the shift register

  //___________________________________________________________________________
  // Instantiation of the module to be verified
  `ifdef RTL_LVL
  i2c_master_regs #(
    .DWIDTH (8),
    .AWIDTH (3)) DUT(
  `else
  i2c_master_regs DUT( // used by post-síntesis verification
  `endif
    // bus
    .Clk                 (clk),      
    .Rst_n               (rst_n),      
    .Addr                (addr),      
    .DataIn              (dataIn),      
    .DataOut             (dataOut),      
    .Wr                  (wr),      
    .Int                 (int),      

    // commands
    .Start               (start),      
    .Stop                (stop),      
    .Read                (read),      
    .Write               (write),      
    .Tx_ack              (tx_ack),      
    .Rx_ack              (rx_ack),      
    .Rx_data             (rx_data),      
    .Tx_data             (tx_data),      
    .Prescale            (prescale),      
    .I2C_busy            (i2C_busy),      
    .I2C_done            (i2C_done),      
    .I2C_en              (i2C_en),      
    .I2C_al              (i2C_al)      

   );


  //___________________________________________________________________________
  // hookup system data bus master
  dbus_master_model#(
    .DATA_WIDTH  (8),
    .ADDR_WIDTH  (8)) u_dbus(

    .Clk                 (clk),      
    .Rst_n               (rst_n),      
    .Addr                (addr),      
    .Dout                (dataIn),      
    .Din                 (dataOut),      
    .Wr                  (wr) 
    );



  // hookup system module that generates clock and reset
 sys_model#(
    .CLK_HALFPERIOD  (5),
    .DELAY  (8)
   ) u_sys(

    .Clk                 (clk),      
    .Rst_n               (rst_n)
   );
  //___________________________________________________________________________
  // signals and vars initialization
  initial begin

    // bus
    rst_n  = 1'b0;
    addr = {AWIDTH{1'b0}};
    dataIn = {DWIDTH{1'b0}};
    wr = 1'b0;
    int = 1'b0;

    // commands 
    start = 1'b0;
    stop = 1'b0;
    read = 1'b0;
    tx_ack = 1'b0;
    rx_ack = 1'b0;
    rx_data = 8'b0;
    tx_data = 8'b0;
    prescale = 8'b0;
    i2C_busy = 1'b0;
    i2C_done = 1'b0;
    i2C_en = 1'b0;
    i2C_al = 1'b0;
  end

  //___________________________________________________________________________
  // Test Vectors
  initial begin
   
    
    $timeformat(-9, 2, " ns", 10); // format for the time print
    errors = 0;                    // initialize the errors counter
    u_sys.reset(3);                // puts the DUT in a known stage
    u_sys.wait_cycles(5);          // waits 5 clock cicles

    $display("[Info- %t] Test Wr/Rd of registers through System Data Bus", $time);
    // TODO: open the dbus_master_model and complete the write and read tasks.
    addr2write = {1'b0,`I2C_TXR};
    while(!addr2write[ADDR_WIDTH]) begin
      data2write = 8'hAA;
      $display("[Info- %t] Test Wr/Rd %h to Reg[%h]", $time, data2write, addr2write);
      u_dbus.write(addr2write[ADDR_WIDTH-1:0], data2write);
      u_dbus.read(addr2write[ADDR_WIDTH-1:0], vObtained);
      vExpected = data2write;
      sync_check;
      data2write = 8'h55;
      $display("[Info- %t] Test Wr/Rd %h to Reg[%h]", $time, data2write, addr2write);
      u_dbus.write(addr2write[ADDR_WIDTH-1:0], data2write);
      u_dbus.read(addr2write[ADDR_WIDTH-1:0], vObtained);
      vExpected = data2write;
      sync_check;
      data2write = 8'h00;
      $display("[Info- %t] Test Wr/Rd %h to Reg[%h]", $time, data2write, addr2write);
      u_dbus.write(addr2write[ADDR_WIDTH-1:0], data2write);
      u_dbus.read(addr2write[ADDR_WIDTH-1:0], vObtained);
      vExpected = data2write;
      sync_check;
      addr2write = addr2write - 1'b1;
      $display("[Info- %t] New address to be writted[%h]", $time, addr2write[ADDR_WIDTH-1:0]);
    end
    check_errors;
    errors = 0; // reset the errors counter


    $display("[Info- %t] Test CR autoclear after tranfer ends", $time);
    // TODO: Generate the test vectors using the available tasks to check
    // the autoclear of the CR register bits when byte transfer ends
    tranfer_done(1);
    addr= 3'h3;
    vExpected = 8'd0;
    vObtained = dataOut;
    async_check;
    check_errors;
    errors = 0;  

    $display("[Info- %t] Test CR autoclear after arbitration is lost", $time);
    // TODO: Generate the test vectors using the available tasks to check
    // the autoclear of the CR register bits when arbitration is lost.
    // Additionaly it should check that the SR's al bit is set, clear it
    // with CR's al_ack bit is automaticaly and check that the al_ack is auto-cleared.
    arbitration_lost;
    addr= 3'h3;
    vExpected = 8'd0;
    vObtained = dataOut;
    async_check;
    check_errors;
    wait_cycles(1);
    addr= 3'h5;
    wait_cycles(1);
    addr= 3'h3;
    dataIn= 8'b00000010;
    i2C_en= 1'b1;
    wr=1'b1;
    wait_cycles(1);
    addr= 3'h5;
    wait_cycles(1);
    addr= 3'h3;
    vExpected = 8'd0;
    vObtained = dataOut;
    async_check;
    check_errors;
    errors = 0;  


    $display("[Info- %t] Test TIP flag", $time);
    // TODO: Generate the test vectors using the available tasks to check
    // the correct generation of the Transfer In Progress flag. It must
    // check the TIP assertion and deassertion.
    // TO BE COMPLETED BY THE STUDENT

    $display("[Info- %t] Test INT request generation", $time);
    // TODO: Generate the test vectors using the available tasks to check
    // the generation of the interreupt request.
    //    > Test all the posible generation sources
    //    > check the status bit and the interrupt request
    //    > the interrupt clear
    // TO BE COMPLETED BY THE STUDENT

    $display("[Info- %t] Test Prescale, Control, Command and Transmission registers outputs", $time);
    // TODO: Generate the test vectors using the available tasks to check
    // if all the prescale, control and commands signals outputs are correct.
    // TO BE COMPLETED BY THE STUDENT

    $display("[Info- %t] Test RXR and the rx_ack flag", $time);
    // TODO: check that the rx_data is acceccible through the RXR,
    // and the rx_ack from bit 7 of SR
    // TO BE COMPLETED BY THE STUDENT

    $display("[Info- %t] End of test", $time);
    $stop;
  end

  initial begin
    $monitor("[Info- %t] Status=%b", $time, u_dut.sr);
  end

  //___________________________________________________________________________
  // Test tasks and functions

  task check_interrupt;
  // TODO: This task checks the interrupt status for a detemined period of
  // time (=3 clock cycles). After it, If the interrupt has been asseted,
  // a successful message is displays. If not, after the the three clocks,
  // an error message is displayed and the error counter increased.
  // It should be implemented using the fork statment : f label  join and  disable f label;
 
    fork
       // evento 1
       begin
        addr = 3'h1;
        dataIn = 8'b11000000;
        wait_cycles(2);
        tranfer_done(1);
        wait_cycles(1);
        if (~|dataOut)
          $display("[Info- %t] Interrupt has been made successfully", $time);
        else begin
          $display("[Info- %t] Error, the interruption has faild", $time);
          errors = errors + 1;
        end
       end

       // evento 2
       begin
        addr = 3'h1;
        dataIn = 8'b11000000;
        wait_cycles(2);
        arbitration_lost;
        wait_cycles(1);
        if (~|dataOut)
          $display("[Info- %t] Interrupt has been made successfully", $time);
        else begin
          $display("[Info- %t] Error, the interruption has faild", $time);
          errors = errors + 1;
        end
       
       end


    join
  endtask


  task tranfer_done(input Ack);
  // TODO: This task generates an I2C_done pulse signal indicating that
  // a byte transfer is done, and set the rx_ack depending on the input
    begin
        rx_ack=Ack;
        i2C_done= 1'b1;
        wait_cycles (1);
        i2C_done= 1'b0;
    end
  endtask

  task arbitration_lost;
  // TODO: this task generates a I2C_al pulse simulating the arbitration lost
  // signal from the core.
    begin
      i2C_al=1'b1;
      wait_cycles(1);
      i2C_al=1'b0;
    end 
  endtask


  //___________________________________________________________________________
  // Basic tasks

  task sync_check;
  // Synchronous errors check
    begin
      u_sys.wait_cycles(1);
      if (vExpected != vObtained) begin
        $display("[Error! %t] The value is %h and should be %h", $time, vObtained, vExpected);
        errors = errors + 1;
      end else begin
        $display("[Info- %t] Successful check at time", $time);
      end
  end
  endtask

  task async_check;
  // Asynchronous errors check
    begin
      if (vExpected != vObtained) begin
        $display("[Error! %t] The value is %h and should be %h", $time, vObtained, vExpected);
        errors = errors + 1;
      end else begin
        $display("[Info- %t] Successful check", $time);
      end
    end
  endtask

  task check_errors;
  // Check for errors during the simulation
    begin
      if (errors==0) begin
        $display("********** TEST PASSED **********");
      end else begin
        $display("********** TEST FAILED **********");
      end
    end
  endtask
endmodule
