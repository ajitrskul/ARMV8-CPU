# Single Cyle ARMV8 CPU
Complete and functioning CPU in Verilog that models the ARMV8 instruction set and relevant data paths for a single cycke CPU. This CPU uses 64-bit memory addressing and has 32 64-bit registers. It is coded to handle the most common assembly instructions including: MOV, ADD, SUB, AND, OR, ADDI, SUBI, LDUR and STUR. 

## Testing
Although unoptimized, the test bench was created to test basic functionalities of each instruction and ensure that they work as intended with the most common cases. Additional testing could still be made to test for edge cases including any affects that each instruction may have on each other. In addition, more features could be included to handle possible user error including overcapacity in registers.
