
//=======================================================
//  MyARM
//=======================================================

module MyDE0_Nano(

//////////// CLOCK //////////
input logic 		          		CLOCK_50,

//////////// LED //////////
output logic		     [7:0]		LED,

//////////// KEY //////////
input logic 		     [1:0]		KEY,

//////////// SW //////////
input logic 		     [3:0]		SW,

//////////// SDRAM //////////
output logic		    [12:0]		DRAM_ADDR,
output logic		     [1:0]		DRAM_BA,
output logic		          		DRAM_CAS_N,
output logic		          		DRAM_CKE,
output logic		          		DRAM_CLK,
output logic		          		DRAM_CS_N,
inout logic 		    [15:0]		DRAM_DQ,
output logic		     [1:0]		DRAM_DQM,
output logic		          		DRAM_RAS_N,
output logic		          		DRAM_WE_N,

//////////// EPCS //////////
output logic		          		EPCS_ASDO,
input logic 		          		EPCS_DATA0,
output logic		          		EPCS_DCLK,
output logic		          		EPCS_NCSO,

//////////// Accelerometer and EEPROM //////////
output logic		          		G_SENSOR_CS_N,
input logic 		          		G_SENSOR_INT,
output logic		          		I2C_SCLK,
inout logic 		          		I2C_SDAT,

//////////// ADC //////////
output logic		          		ADC_CS_N,
output logic		          		ADC_SADDR,
output logic		          		ADC_SCLK,
input logic 		          		ADC_SDAT,

//////////// 2x13 GPIO Header //////////
inout logic 		    [12:0]		GPIO_2,
input logic 		     [2:0]		GPIO_2_IN,

//////////// GPIO_0, GPIO_0 connect to GPIO Default : JP1 //////////
inout logic 		    [33:0]		GPIO_0_PI,
input logic 		     [1:0]		GPIO_0_PI_IN,

//////////// GPIO_1, GPIO_1 connect to GPIO Default : JP2 //////////
inout logic 		    [33:0]		GPIO_1,
input logic 		     [1:0]		GPIO_1_IN
);			 

//=======================================================
//  Clock | Reset
//=======================================================

	logic clk, reset; 
	//reg clk12 = 0;
	//reg clkcount[7:0] = 0; 
	assign clk   = CLOCK_50;
	
	//always @(posedge clk) begin
	//	if(clkcount == 4) begin
	//		clk12 <= ~clk12;
	//		clkcount <= 0;
	//		end
	//	else clkcount <= clkcount +1;
	//end 
	
	//always @(posedge clk) begin
//		if (counter2 == (DIVIDE_BY/2) - 1) begin
	//		i2c_clk <= ~i2c_clk;
		//	counter2 <= 0;
		//end
		//else counter2 <= counter2 + 1;
	//end 
	//assign reset = GPIO_0_PI[1];

//=======================================================
//  SPI
//=======================================================

	logic 		   spi_clk, spi_cs, spi_mosi, spi_miso;
	logic [7:0]    DataAddr;
	logic [31:0]   DataToRPi;

	spi_slave spi_slave_instance(
		.SPI_CLK    (spi_clk),		// input
		.SPI_CS     (spi_cs),		// input
		.SPI_MOSI   (spi_mosi),		// input
		.SPI_MISO   (spi_miso),		// output
		.DataAddr   (DataAddr),		// output : address of what we want to read form DE0 to RPi
		.DataToRPi  (DataToRPi),	// input  : data to send to the RPi
		.Clk        (clk)				// input
	);
	// Connect to JP1 on the DE0-Nano : CARBOT14
	assign spi_clk  		= GPIO_0_PI[11];	// SCLK = pin 16 = GPIO_11
	assign spi_cs   		= GPIO_0_PI[9];	// CE0  = pin 14 = GPIO_9
	assign spi_mosi     	= GPIO_0_PI[15];	// MOSI = pin 20 = GPIO_15
	
	assign GPIO_0_PI[13] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 18 = GPIO_13

    /*
	// Connect to JP1 on the DE0-Nano : MINIBOT
	assign spi_clk  		= GPIO_0_PI[22];	// SCLK = pin 27 = GPIO_22
	assign spi_cs   		= GPIO_0_PI[20];	// CE1  = pin 25 = GPIO_20
	assign spi_mosi     	= GPIO_0_PI[23];	// MOSI = pin 28 = GPIO_23

	assign GPIO_0_PI[21] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 26 = GPIO_21
	*/
	


//=======================================================
//  Memory
//=======================================================	

	logic [31:0] toSend;

	always_comb begin
		case(DataAddr)
			8'b0000: toSend = countRightEnc;
			8'b0001: toSend = countLeftEnc;
			8'b0010: toSend = countRightOdo;
			8'b0011: toSend = countLeftOdo;
			8'b0100: toSend = distance1;
			8'b0101: toSend = distance2;
			8'b0110: toSend = distance3;
			8'b0111: toSend = distance4;
			8'b1000: toSend = binaryData;
		default: toSend = 32'b0;
		endcase
	end

	assign DataToRPi = toSend;

//=======================================================
//  Encoder
//=======================================================

	/*
	---------- PINs ------ JP2 - GPIO-1
	ENC R - ENC 1A : pin 36 - GPIO_1[29]
	ENC R - ENC 1B : pin 38 - GPIO_1[31]
	ENC L - ENC 2A : pin 32 - GPIO_1[25]
	ENC L - ENC 2B : pin 34 - GPIO_1[27]

	ODO R - ODO 1A : pin 26 - GPIO_1[21]
	ODO R - ODO 1B : pin 28 - GPIO_1[23]
	ODO L - ODO 2A : pin 22 - GPIO_1[17]
	ODO L - ODO 2B : pin 24 - GPIO_1[19]

	ECHO   : pin   - GPIO_1[11]
	TRIGGER: pin   - GPIO_1[9]
	*/

   // Encoder - Odometer
   reg signed [31:0] countLeftEnc, countRightEnc, countLeftOdo, countRightOdo;

	logic leftEncA, leftEncB, rightEncA, rightEncB;
	assign rightEncA = GPIO_1[29];
	assign rightEncB = GPIO_1[31];
	
	assign leftEncA  = GPIO_1[25];
	assign leftEncB  = GPIO_1[27];

	logic leftOdoA, leftOdoB, rightOdoA, rightOdoB;
	assign rightOdoA = GPIO_1[21];
	assign rightOdoB = GPIO_1[23];
	assign leftOdoA  = GPIO_1[17];
	
	logic rstgr2;
	
	assign leftOdoB  = GPIO_1[19];
	

	encodergr2 leftEnc(clk, rstgr2, leftEncA, leftEncB, countLeftEnc);
	encodergr2 rightEnc(clk,rstgr2, rightEncA, rightEncB, countRightEnc);
	
	encodergr2 leftOdo(clk, rstgr2,leftOdoA, leftOdoB, countLeftOdo);
	encodergr2 rightOdo(clk,rstgr2, rightOdoA, rightOdoB, countRightOdo);
	
		
	//MySonar
	assign reset_s = ~KEY[1];
	
	logic [31:0] distance1;
	logic trigger1,echo1,reset_s, set;
	
	assign echo1	= GPIO_1[1];
	assign GPIO_1[3] = trigger1;
	
	MySonar sonar1(clk,reset_s,echo1,trigger1,distance1);
	
	logic [31:0] distance2;
	logic trigger2,echo2;
	assign echo2	= GPIO_1[5];
	assign GPIO_1[7] = trigger2;
	
	MySonar sonar2(clk,reset_s,echo2,trigger2,distance2);
	
	logic [31:0] distance3;
	logic trigger3,echo3;
	assign echo3	= GPIO_1[9];
	assign GPIO_1[11] = trigger3;
	
	MySonar sonar3(clk,reset_s,echo3,trigger3,distance3);
	
	logic [31:0] distance4;
	logic trigger4,echo4;
	assign echo4	= GPIO_1[15];
	assign GPIO_1[13] = trigger4;
	
	MySonar sonar4(clk,reset_s,echo4,trigger4,distance4);
	
	logic [31:0] binaryData;
	
	assign binaryData[0] = GPIO_1[32];//start
	assign binaryData[1] = GPIO_1[4];//led1
	assign binaryData[2] = ~GPIO_1[14];//led2
	assign binaryData[3] = ~GPIO_1[8];//led3

	
	assign GPIO_1[2] = 0;
	assign GPIO_1[10] = 0;
	assign GPIO_1[33] = 0;
	assign GPIO_1[16] = 0;
	
	
	assign LED[0] = binaryData[0];
	assign LED[1] = binaryData[1];
	assign LED[2] = binaryData[2];
	assign LED[3] = binaryData[3];

	

	

endmodule

