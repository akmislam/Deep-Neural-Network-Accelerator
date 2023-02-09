/*
   Akm Islam
   ESE 507
   Project 3 - Generator C++ program (based upon provided handout code)
   
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <assert.h>
#include <math.h>
using namespace std;

// Function prototypes
void printUsage();                                                                                 // Function prototype to print how to use each modes.
void genFCLayer(int M, int N, int T, int R, int P, vector<int>& constvector, string modName, ofstream &os);                   // Function prototype to generate single FC layer.
void genNetwork(int N, int M1, int M2, int M3, int T, int R, int B, vector<int>& constVector, string modName, ofstream &os);  // Function prototype to generate network of FC layers.
void readConstants(ifstream &constStream, vector<int>& constvector);                               // Function prototype to read W matrix values.
void genROM(vector<int>& constVector, int bits, string modName, ofstream &os, int P, int M, int N, string layer);             // Function prototype to generate ROMs.
void genMultipier(int T, ofstream &os);                                                            // Function prototype to generate Multipliers.
void genAdder(int T, ofstream &os);                                                                // Function prototype to generate Adders.
void genDatapath(int T, int M, int N, string romModeName, ofstream &os, int j, string layer);      // Function prototype to generate Datapaths.
void genReLU(int T, ofstream &os);                                                                 // Function prototype to generate ReLUs.
void genOutputMUX(int T, int P, ofstream &os, string layer);                                       // Function prototype to generate Output MUXes.
bool canBeParallelized(int M, int P, int B);                                                       // Function prototype to determine if a layer can be further parallelized.
int *parallelizeNetwork(int N, int M1, int M2, int M3, int B);                                     // Function prototype to determine best parallelization for network.

/*
   Main Function
   Inputs: argc (argument count), argv (array of strings)
   Ouputs: Generated .SV file for requested parameters.
*/
int main(int argc, char* argv[]) 
{
   if (argc < 7)                    // If less than expected parameters passed,
   {
      printUsage();                 // Print helpful message.
      return 1;                     // Quit.
   }

   int mode = atoi(argv[1]);        // Extract the mode of operation.

   ifstream const_file;             // Input file for W matrix.
   ofstream os;                     // Output file for fc_M_N_T_R,sv file.
   vector<int> constVector;         // Vector to hold W matrix values.

   // Generator for Parts 1 and 2.
   if (((mode == 1) && (argc == 7)) || ((mode == 2) && (argc == 8)))              // Check mode and arguments passed. 
   {
      int M = atoi(argv[2]);                                                      // Extract value of M.
      int N = atoi(argv[3]);                                                      // Extract value of N.
      int T = atoi(argv[4]);                                                      // Extract value of T.
      int R = atoi(argv[5]);                                                      // Extract value of R.
      int P;                                                                      // Declare parameter variable.

      if (mode == 1)                                                              // Mode 1
      {
         P=1;                                                                     // Use P=1.
         const_file.open(argv[6]);                                                // Get W values from 7th argument.
      }
      else                                                                        // In other modes,
      {
         P = atoi(argv[6]);                                                       // Get P value from 7th argument.
         const_file.open(argv[7]);                                                // Get W values from 8th argument.
      }

      if (const_file.is_open() != true)                                           // If error opening W matrix values file.
      {
         cout << "ERROR reading constant file " << argv[6] << endl;               // Print error message.
         return 1;                                                                // Quit.
      }

      readConstants(const_file, constVector);                                     // Read W matrix values and paste into vector.

      string out_file = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P) + ".sv";  // Assemble output SV file name.

      os.open(out_file);                                                          // Open SV file.
      if (os.is_open() != true)                                                   // If error opening SV file,
      {
         cout << "ERROR opening " << out_file << " for write." << endl;           // Print error message.
         return 1;                                                                // Quit.
      }

      string modName = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P);           // Assemble name for module.
      os << "`include \"fc_M_N_T_R_1.sv\"" << endl;                               // Include contents of file for building blocks.
      genFCLayer(M, N, T, R, P, constVector, modName, os);                        // Generate single FC layer with requested parameters.
   }

   // Generator for Part 3
   else if ((mode == 3) && (argc == 10)) 
   {      
      int N  = atoi(argv[2]);                                                       // Extract value of N.
      int M1 = atoi(argv[3]);                                                       // Extract value of M1.
      int M2 = atoi(argv[4]);                                                       // Extract value of M2.
      int M3 = atoi(argv[5]);                                                       // Extract value of M3.
      int T  = atoi(argv[6]);                                                       // Extract value of T.
      int R  = atoi(argv[7]);                                                       // Extract value of R.
      int B  = atoi(argv[8]);                                                       // Extract value of B.

      const_file.open(argv[9]);                                                     // Open W matrix values.

      if (const_file.is_open() != true)                                             // If error opening W matrix values file.
      {
         cout << "ERROR reading constant file " << argv[8] << endl;                 // Print error message.
         return 1;                                                                  // Quit.
      }

      readConstants(const_file, constVector);                                       // Paste W matrix values into vector.

      string out_file = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(B)+ ".sv";    // Assemble name for SV file. 

      os.open(out_file);                                                            // Open file.

      if (os.is_open() != true)                                                     // If error opening W matrix values file
      {
         cout << "ERROR opening " << out_file << " for write." << endl;             // Print error message.
         return 1;                                                                  // Quit.
      }

      string mod_name = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(B);           // Assemble name for module.

      genNetwork(N, M1, M2, M3, T, R, B, constVector, mod_name, os);                // Generate FC network with requested parameters.

   }

   else                 // Otherwise,
   {
      printUsage();     // Print how to use generator.
      return 1;         // Quit.
   }

   os.close();          // Close output stream.

   return 0;            // Quit.
}


/*
   Function to Read Matrix Values
   Inputs: constStream (input stream), constvector (vector of ints to store values)
   Outputs: None (fills constvector)
*/
void readConstants(ifstream &constStream, vector<int>& constvector) 
{
   string constLineString;                         // Variable to capture each line of file.
   while(getline(constStream, constLineString))    // While there are values to capture
   {
      int val = atoi(constLineString.c_str());     // Capture the value and convert to integer.
      constvector.push_back(val);                  // Insert value into constvector.
   }
}


/*
   Function to Generate ROM
   Inputs: constVector (vector of matrix values), bits (precision of values), modName (naming of ROM), os (output stream), P (parallelism), M (size of output vector), N (size of input vector), layer (name to differentiate from multiple layers)
   Outputs: None (generates ROM via os)
*/
void genROM(vector<int>& constVector, int bits, string modName, ofstream &os, int P, int M, int N, string layer)
{
   int numWords = constVector.size();           // Determine number of words needed.
   int addrBits = ceil(log2(numWords));         // Determine number of bits needed to address words.

   int j;
   vector<int>::iterator it;
   for (j=0; j<P; j++)                          // Create P instances.
   {
      os << "module " << modName << "_" << layer << j << "(clk, addr, z);"    << endl;
      os << "   input clk;"                                     << endl;
      os << "   input [" << addrBits-1 << ":0] addr;"           << endl;
      os << "   output logic signed [" << bits-1 << ":0] z;"    << endl;
      os                                                        << endl;
      os << "   always_ff @(posedge clk) begin"                 << endl;
      os << "      case(addr)"                                  << endl;
      int i=0;
      it = constVector.begin();
      for (i=0, it += j*N; i < M*N/P; it += (P-1)*N)  // Address from 0 to M*N/P. After every N values, skip P-1 rows.
      {
         for (int k=0; k<N; it++, i++, k++)           // Increase address and address from const file one by one.
         {
            if (*it < 0)
               os << "        " << i << ": z <= -" << bits << "'d" << abs(*it) << ";" << endl;
            else
               os << "        " << i << ": z <= "  << bits << "'d" << *it      << ";" << endl;
         }
      }

      os << "        default: z <= 'Z;"                         << endl;
      os << "      endcase" << endl << "   end" << endl << "endmodule" << endl << endl;
   }

}


/*
   Function to Generate Multiplier
   Inputs: T (precision of values), os (output stream)
   Outputs: None (generates a multiplier via os)
*/
void genMultiplier(int T, ofstream &os)
{
   long max = (((long) 1 << T-1) - 1);          // Determine max value.
   long min = ((long) 1 << (T-1));              // Determine min value.

   os << "module multiplier(a, b, ab);"                                    << endl;
   os << "    parameter                                 T=" << T << ";"    << endl;
   os << "    input logic signed [T-1:0]                a, b;"             << endl;
   os << "    logic signed [2*T-1:0]                    intermediate;"     << endl;
   os << "    output logic signed [T-1:0]               ab;"               << endl;
   os                                                                      << endl;
   os << "    always_comb begin"                                           << endl;
   os << "        intermediate = a * b;"                                   << endl;
   os << "        if (intermediate > " << T+1 << "'sd" << max << ")"       << endl;
   os << "            ab = " << T << "'sd" << max << ";"                   << endl;
   os << "        else if (intermediate < -" << T+1 << "'sd" << min << ")" << endl;
   os << "            ab = -" << T << "'sd" << min << ";"                  << endl;
   os << "        else"                                                    << endl;
   os << "            ab = intermediate[" << T-1 << ":0];"                 << endl;
   os << "    end"                                                         << endl;
   os << "endmodule"                                                       << endl;
   os << endl;
}


/*
   Function to Generate Adder
   Inputs: T (precision of values), os (output stream)
   Outputs: None (generates an adder via os)
*/
void genAdder(int T, ofstream &os)
{
   long max = (((long) 1 << T-1) - 1);          // Determine max value.
   long min = ((long) 1 << (T-1));              // Determine min value.         

   os << "module adder(ab, f, acc);"                                       << endl;
   os << "    parameter                                 T=" << T << ";"    << endl;
   os << "    input signed [T-1:0]                      ab, f;"            << endl;
   os << "    logic signed [T:0]                        intermediate;"     << endl;
   os << "    output logic signed [T-1:0]               acc;"              << endl;
   os                                                                      << endl;
   os << "    always_comb begin"                                           << endl;
   os << "        intermediate = f + ab;"                                  << endl;
   os << "        if (intermediate > " << T+1 << "'sd" << max << ")"       << endl;
   os << "            acc = " << T << "'sd" << max << ";"                  << endl;
   os << "        else if (intermediate < -" << T+1 << "'sd" << min << ")" << endl;
   os << "            acc = -" << T << "'sd" << min << ";"                 << endl;
   os << "        else"                                                    << endl;
   os << "            acc = intermediate[" << T-1 << ":0];"                << endl;
   os << "    end"                                                         << endl;
   os << "endmodule"                                                       << endl;
   os << endl;
}

/*
   Function to Generate Datapath
   Inputs: T (precision of values), M (rows of matrix), N (columns of matrix), romModName (name of ROM), os (output stream), j (j-th datapath), layer (name to differentiate from other layers)
   Outputs: None (generates a Datapath via os)
*/
void genDatapath(int T, int M, int N, string romModName, ofstream &os, int j, string layer)
{
   os << "module DataPath" << "_" << layer << j << "(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, en_acc, output_data);" << endl;
   os << "    parameter                      T=" << T << ", M=" << M << ", N=" << N << ";"   << endl;
   os << "    localparam                     SIZE_x=$clog2(N), SIZE_w=$clog2(M*N);"          << endl;
   os << "    input                          clk, reset;"                                    << endl;
   os << "    input[T-1:0]                   input_data;"                                    << endl;
   os << "    input                          wr_en_x, clear_acc, en_acc;"                    << endl;
   os << "    input logic [SIZE_x-1:0]       addr_x;"                                        << endl;
   os << "    input logic [SIZE_w-1:0]       addr_w;"                                        << endl;
   os                                                                                        << endl;
   os << "    output logic [T-1:0]           output_data;"                                   << endl;
   os << "    logic [T-1:0]                  mem_x_out, mem_w_out;"                          << endl;       
   os << "    logic [T-1:0]                  multiplier_out, acc;"                           << endl;        
   os                                                                                        << endl;
   os << "    memory #(T,N) mem_x(.clk(clk), .data_in(input_data), .data_out(mem_x_out), .addr(addr_x), .wr_en(wr_en_x));" << endl;
   os << "    " << romModName << "_" << layer << j << " mem_w(.clk(clk), .addr(addr_w), .z(mem_w_out));"     << endl;
   os << "    multiplier multiplier0(.a(mem_x_out), .b(mem_w_out), .ab(multiplier_out));"    << endl;
   os << "    adder adder0(.ab(multiplier_out), .f(output_data), .acc(acc));"                << endl;
   os << "    f_ff #(T) f_ff0(.clk(clk), .reset(reset), .clear(clear_acc), .enable_f(en_acc), .acc(acc), .f(output_data));" << endl;

   os << "endmodule" << endl;
   os << endl;
}


/*
   Function to Generate ReLU
   Inputs: T (precision of values), os (output stream)
   Outputs: None (generates a ReLU via os)
*/
void genReLU(int T, ofstream &os)
{
   os << "module ReLU(incoming, outgoing);"                                << endl;
   os << "    parameter                                 T=" << T << ";"    << endl;
   os << "    input signed [T-1:0]                      incoming;"         << endl;
   os << "    output logic signed [T-1:0]               outgoing;"         << endl;
   os                                                                      << endl;
   os << "    always_comb begin"                                           << endl;
   os << "        if (incoming < 0)"                                       << endl;
   os << "            outgoing = 0;"                                       << endl;
   os << "        else"                                                    << endl;
   os << "            outgoing = incoming;"                                << endl;
   os << "    end"                                                         << endl;
   os << "endmodule"                                                       << endl;
   os << endl;
}


/*
   Function to Generate Output MUX
   Inputs: T (precision of values), P (parallelism), os (output stream), layer (name to differentiate from other layers)
   Outputs: None (generates an Output MUX)
*/
void genOutputMUX(int T, int P, ofstream &os, string layer)
{
   os << "module OutputMUX" << "_" << layer << "(clk, reset, output_data_inside, output_valid_inside, output_data, output_valid, output_ready, output_ready_inside);" << endl;
   os << "    parameter                                 T=" << T << ", P=" << P << ";" << endl;
   os << "    localparam                                SIZE_turn=$clog2(P);"          << endl;
   os << "    input                                     clk, reset;"                   << endl;
   os << "    input [P-1:0]                             output_valid_inside;"          << endl;
   os << "    input signed [P-1:0][T-1:0]               output_data_inside;"           << endl;
   os << "    output logic signed [T-1:0]               output_data;"                  << endl;
   os << "    output logic                              output_valid;"                 << endl;
   os <<                                                                                  endl;
   os << "    input                                     output_ready;"                 << endl;
   os << "    output logic [P-1:0]                      output_ready_inside;"          << endl;
   os <<                                                                                  endl;
   os << "    logic [SIZE_turn-1:0]                     turn;"                         << endl;
   os << "    always_comb begin"                                                       << endl;
   os << "        if (turn == 0) begin"                                                << endl;
   os << "            output_data = output_data_inside[0];"                            << endl;
   os << "            output_valid = output_valid_inside[0]; "                         << endl;
   os << "            output_ready_inside = (output_ready << 0);"                      << endl;
   os << "        end"                                                                 << endl;
   for (int i=1; i<P; i++)
   {
      os << "        else if (turn == " << i << ") begin"                              << endl;
      os << "            output_data = output_data_inside[" << i << "];"               << endl;
      os << "            output_valid = output_valid_inside[" << i << "];"             << endl;
      os << "            output_ready_inside = (output_ready << " << i << ");"         << endl;
      os << "        end"                                                              << endl;
   }

   os << "        else begin"                                                          << endl;
   os << "            output_data = '0;"                                               << endl;
   os << "            output_valid = 0;"                                               << endl;
   os << "            output_ready_inside = '0;"                                       << endl;
   os << "        end"                                                                 << endl;
   os << "    end"                                                                     << endl;
   os <<                                                                                  endl;
   os << "    always_ff @ (posedge clk) begin"                                         << endl;
   os << "        if (reset)"                                                          << endl;
   os << "            turn <= 0;"                                                      << endl;
   os << "        else if (output_ready && output_valid) begin"                        << endl;
   os << "            if (turn == P-1)"                                                << endl;
   os << "                turn <= 0;"                                                  << endl;
   os << "            else"                                                            << endl;
   os << "                turn <= turn + 1;"                                           << endl;
   os << "        end"                                                                 << endl;
   os << "    end"                                                                     << endl;
   os << "endmodule"                                                                   << endl;
   os <<                                                                                  endl;               
}


/*
   Function to Generate FC Layer (Parts 1 and 2)
   Inputs: M (rows of matrix), N (columns of matrix), T (precision of values), R (include ReLU), P (parallelism), constVector (vector of Matrix values), modName (name of top module), os (output stream)
   Outputs: None (generates a FC Layer)
*/
void genFCLayer(int M, int N, int T, int R, int P, vector<int>& constVector, string modName, ofstream &os) {

   string romModName = modName + "_W_rom";                                 // Assemble ROM naming.               
   
   if (M*N != constVector.size())                                          // If rows * columns does not match size of vector
   {
      cout << "ERROR: constVector does not contain correct amount of data for the requested design" << endl;   // Print error message.
      cout << "The design parameters requested require " << M*N << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;    // Inform how many values are required.
      assert(false);                                                  
   }

   genROM(constVector, T, romModName, os, P, M, N, modName);               // Generate a ROM (for W) with constants 0 through M*N-1, with T bits
   genMultiplier(T, os);                                                   // Generate a multiplier with T bits for inputs and outputs.
   genAdder(T, os);                                                        // Generate an adder with T bits for inputs and outputs.
   if (1 < P) genOutputMUX(T, P, os, modName);                             // If parallelism requested, generate output mux.
   
   int i;
   for (i=0; i<P; i++)
      genDatapath(T, M, N, romModName, os, i, modName);                    // Generate Datapath with T bits, M rows, N columns.
   
   if (R == 1)                                                             // If ReLU is required,
      genReLU(T, os);                                                      // Generate ReLU.
   
   // Generate I/O
   os << "// Top level module"                                                             << endl;
   os << "module " << modName << "(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);" << endl;
   os << "   parameter                     T=" << T << ", M=" << M << ", N=" << N << ", P=" << P << ";"   << endl;
   os << "   localparam                    SIZE_x=$clog2(N), SIZE_w=$clog2(M*N);"          << endl;
   os << "   input                         clk, reset, input_valid, output_ready;"         << endl;
   os << "   input signed [T-1:0]          input_data;"                                    << endl;
   os << "   output logic signed [T-1:0]   output_data;"                                   << endl;
   os << "   output logic                  output_valid, input_ready;"                     << endl;
   os                                                                                      << endl;
   
   if (P == 1) // For Part 1
   {
      i=0;
      // Makes all internal wires for each control and datapath modules. 
      os << "   logic                         wr_en_x" << i << ", clear_acc" << i << ", en_acc" << i << ";" << endl;
      os << "   logic[SIZE_x-1:0]             addr_x" << i << ";"                          << endl;
      os << "   logic[SIZE_w-1:0]             addr_w" << i << ";"                          << endl;
      os                                                                                   << endl;
   
      // Instantiate control module with given parameters.
      os << "   control" << " #(T,M,N,P) control" << i << "(.clk(clk), .reset(reset), .input_valid(input_valid), .output_ready(output_ready), .addr_x(addr_x" << i << "), .wr_en_x(wr_en_x" << i << "), .addr_w(addr_w" << i << "), .clear_acc(clear_acc" << i << "), .en_acc(en_acc" << i << "), .input_ready(input_ready), .output_valid(output_valid));" << endl;
      os                                                                                   << endl;
      
      // Instantiate datapath with/without ReLU.
      if (R == 1)  // If ReLU is required, generate wire to connect datapath output to ReLU, datapath, and ReLU.
      {
         os << "   logic signed [T-1:0] datapathout" << i << ";"                          << endl;
         os << "   DataPath" << "_" << modName << i << " datapathinstance" << i << "(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x" << i << "), .wr_en_x(wr_en_x" << i << "), .addr_w(addr_w" << i << "), .clear_acc(clear_acc" << i << "), .en_acc(en_acc" << i << "), .output_data(datapathout" << i << "));" << endl;
         os << "   ReLU ReLU" << i << "(.incoming(datapathout"<< i << "), .outgoing(output_data));"      << endl;
      }
      else         // Otherwise, generate only datapath and directly connect to output.
         os << "   DataPath" << "_" << modName << i << " datapathinstance" << i << "(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x" << i << "), .wr_en_x(wr_en_x" << i << "), .addr_w(addr_w" << i << "), .clear_acc(clear_acc" << i << "), .en_acc(en_acc" << i << "), .output_data(output_data));" << endl;
      os                                                                                  << endl; 
   }
   else        // For Part 2
   {
      // Makes all internal wires for each control and datapath modules. 
      for (i=0; i<P; i++)
      {
         os << "   logic                         wr_en_x" << i << ", clear_acc" << i << ", en_acc" << i << ";" << endl;
         os << "   logic[SIZE_x-1:0]             addr_x" << i << ";"                          << endl;
         os << "   logic[SIZE_w-1:0]             addr_w" << i << ";"                          << endl;
         os                                                                                   << endl;

      }

      // Declare signals to connect control module to output MUX
      os << "   parameter[1:0]                STATE_INIT=0, STATE_STORE_X=1, STATE_COMPUTE=2, STATE_STALL=3;"  << endl;
      os << "   logic[1:0]                    state;"                                                        << endl;
      os << "   logic [P-1:0]                 input_ready_inside, output_valid_inside, output_ready_inside;" << endl;
      os << "   logic [P-1:0][T-1:0]          output_data_inside;"                                           << endl;
      os <<                                                                                                     endl;

      // Assign input_ready as a logical AND of all control modules' input_ready signals.
      os << "   assign input_ready = ";
      for (i=0; i<P-1; i++)
      {
         os << "input_ready_inside[" << i << "] & ";
      }

      // Last signal
      os << "input_ready_inside[" << P-1 << "];" << endl;
      os <<                                         endl;

      // Makes all instantiations of control modules needed.
      for (i=0; i<P-1; i++)
      {
         // Instantiate control module with given parameters.
         os << "   control" << " #(T,M,N,P) control" << i << "(.clk(clk), .reset(reset), .input_valid(input_valid && (state != STATE_COMPUTE && state != STATE_STALL)), .output_ready(output_ready_inside[" << i << "]), .addr_x(addr_x" << i << "), .wr_en_x(wr_en_x" << i << "), .addr_w(addr_w" << i << "), .clear_acc(clear_acc" << i << "), .en_acc(en_acc" << i << "), .input_ready(input_ready_inside[" << i << "]), .output_valid(output_valid_inside[" << i << "]));" << endl;
         os                                                                                   << endl;
      }

      os << "   control" << " #(T,M,N,P) control" << i << "(.clk(clk), .reset(reset), .input_valid(input_valid && (state != STATE_COMPUTE && state != STATE_STALL)), .output_ready(output_ready_inside[" << i << "]), .addr_x(addr_x" << i << "), .wr_en_x(wr_en_x" << i << "), .addr_w(addr_w" << i << "), .clear_acc(clear_acc" << i << "), .en_acc(en_acc" << i << "), .input_ready(input_ready_inside[" << i << "]), .output_valid(output_valid_inside[" << i << "]), .state(state));" << endl;
      os << endl;

      // Makes all instantiations of datapath modules needed.
      for (i=0; i<P; i++)
      {
         // Instantiate datapath with/without ReLU.
         if (R == 1)  // If ReLU is required, generate wire to connect datapath output to ReLU, datapath, and ReLU.
         {
            os << "   logic signed [T-1:0] datapathout" << i << ";"                          << endl;
            os << "   DataPath" << "_" << modName << i << " datapathinstance" << i << "(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x" << i << "), .wr_en_x(wr_en_x" << i << "), .addr_w(addr_w" << i << "), .clear_acc(clear_acc" << i << "), .en_acc(en_acc" << i << "), .output_data(datapathout" << i << "));" << endl;
            os << "   ReLU ReLU" << i << "(.incoming(datapathout"<< i << "), .outgoing(output_data_inside[" << i << "]));"      << endl;
         }
         else         // Otherwise, generate only datapath and directly connect to output.
            os << "   DataPath" << "_" << modName << i << " datapathinstance" << i << "(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x" << i << "), .wr_en_x(wr_en_x" << i << "), .addr_w(addr_w" << i << "), .clear_acc(clear_acc" << i << "), .en_acc(en_acc" << i << "), .output_data(output_data_inside[" << i << "]));" << endl;
         os                                                                                  << endl;
      }

      // Makes instantiation of output MUX to interface each MAC to ports
      os << "   OutputMUX" << "_" << modName << " outputmux(.clk(clk), .reset(reset), .output_data_inside(output_data_inside), .output_valid_inside(output_valid_inside), .output_data(output_data), .output_valid(output_valid), .output_ready(output_ready), .output_ready_inside(output_ready_inside));" << endl;
      os <<                                                                                     endl;   
   }

   os << "endmodule" << endl << endl;  
}


/*
   Function to Generate Network (Part 3)
   Inputs: N (input vector layer 1), M1 (input vector layer 2), M2, (input vector layer 3), M3 (output vector of network), T (precision of values), R (include ReLU), B (budget of MAC units), P (parallelism), constVector (vector of Matrix values), modName (name of top module), os (output stream)
   Outputs: None (generates a fully connected 3 Layer network)
*/
void genNetwork(int N, int M1, int M2, int M3, int T, int R, int B, vector<int>& constVector, string modName, ofstream &os) {

   int *P = parallelizeNetwork(N, M1, M2, M3, B);        // Run optimization function.
   int P1 = P[0]; // replace this with your optimized value
   int P2 = P[1]; // replace this with your optimized value
   int P3 = P[2]; // replace this with your optimized value
   
   if (M1%P1 || M2%P2 || M3%P3)
   {
      cout << "Error, one of parallel values does not evenly divide output sizes" << endl;
      return;
   }

   int start = 0;
   int stop = M1*N;
   vector<int> constVector1(&constVector[start], &constVector[stop]);   // Capture W matrix values for layer 1.

   start = stop;
   stop = start+M2*M1;
   vector<int> constVector2(&constVector[start], &constVector[stop]);   // Capture W matrix values for layer 2.

   start = stop;
   stop = start+M3*M2;
   vector<int> constVector3(&constVector[start], &constVector[stop]);   // Capture W matrix values for layer 3.

   if (stop > constVector.size())                                       // If size is less than expected values
   {
      os << "ERROR: constVector does not contain enough data for the requested design" << endl;                                                                // Print error message.
      os << "The design parameters requested require " << stop << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;  // Print values.
      assert(false);                                                
   }

   os << "`include \"fc_M_N_T_R_1.sv\"" << endl;                      // Include contents of file for building blocks.
   
   string subModName1 = "l1_fc_" + to_string(M1) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P1);       // Assemble name for layer 1.
   string subModName2 = "l2_fc_" + to_string(M2) + "_" + to_string(M1) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P2);      // Assemble name for layer 2.
   string subModName3 = "l3_fc_" + to_string(M3) + "_" + to_string(M2) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P3);      // Assemble name for layer 3.

   genFCLayer(M1, N, T, R, P1, constVector1, subModName1, os);    // Generate layer 1.
   genFCLayer(M2, M1, T, R, P2, constVector2, subModName2, os);   // Generate layer 2.
   genFCLayer(M3, M2, T, R, P3, constVector3, subModName3, os);   // Generate layer 3.

   // Create top-level module
   os << "module " << modName << "(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);" << endl;
   os << "   parameter                     T=" << T << ", N=" << N << ", M1=" << M1 << ", M2=" << M2 << ", M3=" << M3 << ";"  << endl;
   os << "   input                         clk, reset, input_valid, output_ready;"         << endl;
   os << "   input signed [T-1:0]          input_data;"                                    << endl;
   os << "   output logic signed [T-1:0]   output_data;"                                   << endl;
   os << "   output logic                  output_valid, input_ready;"                     << endl;
   os                                                                                      << endl;
   os << "   logic[T-1:0]                  dataL12, dataL23;"                              << endl;
   os << "   logic                         validL12, validL23, readyL12, readyL23;"        << endl;
   os <<                                                                                      endl;
   
   // Instantiate all layers and wire them.
   os << "   " << subModName1 << " " << subModName1+"0" << "(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .input_data(input_data), .output_valid(validL12), .output_ready(readyL12), .output_data(dataL12));" << endl;
   os << "   " << subModName2 << " " << subModName2+"0" << "(.clk(clk), .reset(reset), .input_valid(validL12), .input_ready(readyL12), .input_data(dataL12), .output_valid(validL23), .output_ready(readyL23), .output_data(dataL23));" << endl;
   os << "   " << subModName3 << " " << subModName3+"0" << "(.clk(clk), .reset(reset), .input_valid(validL23), .input_ready(readyL23), .input_data(dataL23), .output_valid(output_valid), .output_ready(output_ready), .output_data(output_data));" << endl;

   os << "endmodule" << endl;
}


/*
   Function to Determine if Layer can be further Parallelized
   Inputs: M (output size of layer), P (current parallel units), B (budget of MACs)
   Output: True/False if Layer can be parallelized further
*/
bool canBeParallelized(int M, int P, int B)
{
   for (int i=P+1; i<=B; i++)
   {
      if (M % i == 0)
         return true;
   }

   return false;
}


/*
   Function to Best Parallelize a Network
   Inputs: N (input vector layer 1), M1 (input vector layer 2), M2, (input vector layer 3), M3 (output vector of network), B (budget of MAC units)
   Outputs: Array of Parallel values
*/
int *parallelizeNetwork(int N, int M1, int M2, int M3, int B)
{
   int *P = (int*) malloc (sizeof(int)*3);      // Create new array for parallel values.
   for (int i=0; i<3; i++)                      // Initialize all P values
      P[i] = 1;                                 // to 1.

   if (B == 3)                                  // If budget only allows 1 MAC per layer,
      return P;                                 // Network cannot be parallelized further. Return P.

   B -= 3;                                      // Remove 3 MAC's from budget (one for each layer).

   int cycles[3];                               // Create array to hold how cycles needed to compute all outputs for each layer.
   int MP[3] = {M1, M2, M3};                    // Create array to hold how parallelized layer currently is.
   int M[3] = {M1, M2, M3};                     // Create array to hold output size for each layer.
   bool change = true;                          // Create a bool variable to indicate a change has been made.

   while(change)                                // Keep going if the last iteration made a change.
   {
      change = false;                           // Mark change as false. If no changes are made in this iteration, loop will break.
      int max=0, j=-1;                          // Variables to hold max cycles in a layer and in which layer.
      cycles[0] = N  + (M1/P[0])*(N +2)+P[0];   // Calculate cycles needed in layer 1.
      cycles[1] = M1 + (M2/P[1])*(M1+2)+P[1];   // Calculate cycles needed in layer 2.
      cycles[2] = M2 + (M3/P[2])*(M2+2)+P[2];   // Calculate cycles needed in layer 3.

      for (int i=0; i<3; i++)                   // Iterate through all cycles.
      {
         if (cycles[i] > max && canBeParallelized(M[i], P[i], B+P[i]))  // If cycles is greater than the max and the layer can be further parallelized,
         {
            j = i;                              // Record its layer
            max = cycles[i];                    // Record its count.
         }
      }

      int multiple;                             // Create variable to hold new parallel value.

      for(int i=P[j]+1; i<=MP[j]; i++)          // Starting from last parallel value up to M/P, check for valid parallel values.
      {
         if (MP[j] % i == 0 && (i-P[j] <= B))   // If current i divides M/P evenly and is within the remaining MAC budget,
         {
            change = true;                      // Record that a change has been made.
            multiple = i;                       // Record multiple value.
         }
      }

      if (change)                               // If a change has been made,
      {
         B -= multiple-P[j];                    // Take away new parallel value from MAC budget.
         P[j] = multiple;                       // Assign new parallel value.
         MP[j] = M[j]/P[j];                     // Record new M/P value.
      }

   }

   return P;
}


void printUsage() {
  cout << "Usage: ./gen MODE ARGS" << endl << endl;

  cout << "   Mode 1 (Part 1): Produce one neural network layer (unparallelized)" << endl;
  cout << "      ./gen 1 M N T R const_file" << endl << endl;

  cout << "   Mode 2 (Part 2): Produce one neural network layer (parallelized)" << endl;
  cout << "      ./gen 2 M N T R P const_file" << endl << endl;

  cout << "   Mode 3 (Part 3): Produce a system with three interconnected layers" << endl;
  cout << "      ./gen 3 N M1 M2 M3 T R B const_file" << endl;
}