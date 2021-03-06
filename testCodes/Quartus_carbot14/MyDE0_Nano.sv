
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

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_0_PI,
input logic 		     [1:0]		GPIO_0_PI_IN,

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_1,
input logic 		     [1:0]		GPIO_1_IN
);			 

//=======================================================
//  Clock | Reset
//=======================================================

	logic clk, reset;
	
	assign clk   = CLOCK_50;
	//assign reset = GPIO_0_PI[1];

//=======================================================
//  SPI
//=======================================================

	logic 		   spi_clk, spi_cs, spi_mosi, spi_miso;
	logic [31:0]   DataToRPi;
	logic [7:0]    DataAddr;

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
	
	/*
	assign spi_clk  		= GPIO_0_PI[11];	// SCLK = pin 16 = GPIO_11
	assign spi_cs   		= GPIO_0_PI[9];	// CE0  = pin 14 = GPIO_9
	assign spi_mosi     	= GPIO_0_PI[15];	// MOSI = pin 20 = GPIO_15
	assign GPIO_0_PI[13] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 18 = GPIO_13
	*/
    
	// Connect to JP1 on the DE0-Nano : MINIBOT
	assign spi_clk  		= GPIO_0_PI[22];	// SCLK = pin 27 = GPIO_22
	assign spi_cs   		= GPIO_0_PI[20];	// CE1  = pin 25 = GPIO_20
	assign spi_mosi     	= GPIO_0_PI[23];	// MOSI = pin 28 = GPIO_23

	assign GPIO_0_PI[21] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 26 = GPIO_21
	
	


//=======================================================
//  Memory
//=======================================================	

	logic [31:0] count;

	always_comb begin
		case(DataAddr[2:0])
			3'b000: count = countLeftEnc;
			3'b001: count = countRightEnc;
			3'b010: count = countLeftOdo;
			3'b011: count = countRightOdo;
			//3'b100: count = distance;
		default: count = 32'b0;
		endcase
	end
	
	assign DataToRPi = count;

	/*always_comb begin
		case(DataAddr[3:2])
			2'b10: set = 1;
			2'b11: set = 0;
			default: set = 0;
		endcase
	end*/
	
//=======================================================
//  Encoder
//=======================================================

	/*
	---------- PINs ----------  A MODIF
	ENC 1A : pin 4  - GPIO_0_PI[1]
	ENC 1B : pin 6  - GPIO_0_PI[3]
	ENC 2A : pin 5  - GPIO_0_PI[2]
	ENC 2B : pin 8  - GPIO_0_PI[5]

	ODO 1A : pin 13 - GPIO_0_PI[8]
	ODO 1B : pin 14 - GPIO_0_PI[9]
	ODO 2A : pin 15 - GPIO_0_PI[10]
	ODO 2B : pin 16 - GPIO_0_PI[11]

	ECHO   : pin 21  - GPIO_0_PI[16]
	TRIGGER: pin 22  - GPIO_0_PI[17]
	*/

   // Encoder - Odometer
   logic [31:0] countLeftEnc, countRightEnc, countLeftOdo, countRightOdo;

	logic leftEncA, leftEncB, rightEncA, rightEncB;
	assign rightEncA = GPIO_0_PI[1];
	assign rightEncB = GPIO_0_PI[3];
	assign leftEncA  = GPIO_0_PI[2];
	assign leftEncB  = GPIO_0_PI[5];

	logic leftOdoA, leftOdoB, rightOdoA, rightOdoB;
	assign rightOdoA = GPIO_0_PI[8];
	assign rightOdoB = GPIO_0_PI[9];
	assign leftOdoA  = GPIO_0_PI[10];
	assign leftOdoB  = GPIO_0_PI[11];
	

	Encoder leftEnc(clk, leftEncA, leftEncB, countLeftEnc);
	Encoder rightEnc(clk, rightEncA, rightEncB, countRightEnc);

	Encoder leftOdo(clk, leftOdoA, leftOdoB, countLeftOdo);
	Encoder rightOdo(clk, rightOdoA, rightOdoB, countRightOdo);
		
	//sonar
	
	logic [31:0] distance;
	logic reset_s,launch;
	//logic [6:0] Adr;
	assign Adr = 7'hEC;
	assign GPIO_1[12] = i2c_sda;
	assign GPIO_1[14] = i2c_scl;
	assign launch = ~KEY[0];
	
	MySonarI2C sonar1(clk,reset_s,launch,Adr,distance,i2c_sda,i2c_scl);
	
	//MySonar
	//logic [31:0] distance;
	//logic trigger,echo,reset_s, set;
	//assign echo		= GPIO_0_PI[16];
	//assign GPIO_0_PI[17] = trigger;
	//MySonar sonar(clk,reset_s,echo,trigger,distance);
	//assign LED[0] = 1;
	
	//assign LED[0] = reset_s;
	//assign LED[1] = set;
	//assign LED[7:3] = distance;
	// Sonar
	assign LED[0] = launch;
	assign LED[7:1] = distance;


endmodule



