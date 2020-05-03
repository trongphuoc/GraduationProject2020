# TCL File Generated by Component Editor 13.0
# Wed Aug 28 12:15:47 CST 2013
# DO NOT MODIFY


# 
# LOG_Generate "log generator" v1.0
# Dee Zeng 2013.08.28.12:15:47
# generate log
# 

# 
# request TCL package from ACDS 13.0
# 
package require -exact qsys 13.0


# 
# module LOG_Generate
# 
set_module_property DESCRIPTION "generate log"
set_module_property NAME LOG_Generate
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property AUTHOR "Dee Zeng"
set_module_property DISPLAY_NAME "log generator"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property ANALYZE_HDL AUTO
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL log_generator
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
add_fileset_file log_generator.v VERILOG PATH log_generator.v TOP_LEVEL_FILE
add_fileset_file log_rom.v VERILOG PATH log_rom.v
add_fileset_file log_fifo.v VERILOG PATH log_fifo.v
add_fileset_file log.mif MIF PATH log.mif

add_fileset SIM_VERILOG SIM_VERILOG "" ""
set_fileset_property SIM_VERILOG TOP_LEVEL log_generator
set_fileset_property SIM_VERILOG ENABLE_RELATIVE_INCLUDE_PATHS false
add_fileset_file log_fifo.v VERILOG PATH log_fifo.v
add_fileset_file log.mif MIF PATH log.mif
add_fileset_file log_generator.v VERILOG PATH log_generator.v
add_fileset_file log_rom.v VERILOG PATH log_rom.v


# 
# parameters
# 
add_parameter BACK_COLOR STD_LOGIC_VECTOR 1 ""
set_parameter_property BACK_COLOR DEFAULT_VALUE 1
set_parameter_property BACK_COLOR DISPLAY_NAME BACK_COLOR
set_parameter_property BACK_COLOR WIDTH 26
set_parameter_property BACK_COLOR TYPE STD_LOGIC_VECTOR
set_parameter_property BACK_COLOR UNITS None
set_parameter_property BACK_COLOR DESCRIPTION ""
set_parameter_property BACK_COLOR HDL_PARAMETER true
add_parameter FRONT_COLOR STD_LOGIC_VECTOR 16777215 ""
set_parameter_property FRONT_COLOR DEFAULT_VALUE 16777215
set_parameter_property FRONT_COLOR DISPLAY_NAME FRONT_COLOR
set_parameter_property FRONT_COLOR WIDTH 26
set_parameter_property FRONT_COLOR TYPE STD_LOGIC_VECTOR
set_parameter_property FRONT_COLOR UNITS None
set_parameter_property FRONT_COLOR DESCRIPTION ""
set_parameter_property FRONT_COLOR HDL_PARAMETER true
add_parameter MIF_FILE_LOCATION STRING ./log.mif ""
set_parameter_property MIF_FILE_LOCATION DEFAULT_VALUE ./log.mif
set_parameter_property MIF_FILE_LOCATION DISPLAY_NAME MIF_FILE_LOCATION
set_parameter_property MIF_FILE_LOCATION TYPE STRING
set_parameter_property MIF_FILE_LOCATION ENABLED false
set_parameter_property MIF_FILE_LOCATION UNITS None
set_parameter_property MIF_FILE_LOCATION DESCRIPTION ""
set_parameter_property MIF_FILE_LOCATION HDL_PARAMETER true
add_parameter LOG_WIDTH INTEGER 1024 ""
set_parameter_property LOG_WIDTH DEFAULT_VALUE 1024
set_parameter_property LOG_WIDTH DISPLAY_NAME LOG_WIDTH
set_parameter_property LOG_WIDTH TYPE INTEGER
set_parameter_property LOG_WIDTH ENABLED false
set_parameter_property LOG_WIDTH UNITS None
set_parameter_property LOG_WIDTH ALLOWED_RANGES -2147483648:2147483647
set_parameter_property LOG_WIDTH DESCRIPTION ""
set_parameter_property LOG_WIDTH HDL_PARAMETER true
add_parameter LOG_HEIGHT INTEGER 249 ""
set_parameter_property LOG_HEIGHT DEFAULT_VALUE 249
set_parameter_property LOG_HEIGHT DISPLAY_NAME LOG_HEIGHT
set_parameter_property LOG_HEIGHT TYPE INTEGER
set_parameter_property LOG_HEIGHT UNITS None
set_parameter_property LOG_HEIGHT ALLOWED_RANGES -2147483648:2147483647
set_parameter_property LOG_HEIGHT DESCRIPTION ""
set_parameter_property LOG_HEIGHT HDL_PARAMETER true


# 
# display items
# 


# 
# connection point reset
# 
add_interface reset reset end
set_interface_property reset associatedClock clock_sink
set_interface_property reset synchronousEdges DEASSERT
set_interface_property reset ENABLED true
set_interface_property reset EXPORT_OF ""
set_interface_property reset PORT_NAME_MAP ""
set_interface_property reset SVD_ADDRESS_GROUP ""

add_interface_port reset reset reset Input 1


# 
# connection point clock_sink
# 
add_interface clock_sink clock end
set_interface_property clock_sink clockRate 0
set_interface_property clock_sink ENABLED true
set_interface_property clock_sink EXPORT_OF ""
set_interface_property clock_sink PORT_NAME_MAP ""
set_interface_property clock_sink SVD_ADDRESS_GROUP ""

add_interface_port clock_sink clock clk Input 1


# 
# connection point dout
# 
add_interface dout avalon_streaming start
set_interface_property dout associatedClock clock_sink
set_interface_property dout associatedReset reset
set_interface_property dout dataBitsPerSymbol 8
set_interface_property dout errorDescriptor ""
set_interface_property dout firstSymbolInHighOrderBits true
set_interface_property dout maxChannel 0
set_interface_property dout readyLatency 1
set_interface_property dout ENABLED true
set_interface_property dout EXPORT_OF ""
set_interface_property dout PORT_NAME_MAP ""
set_interface_property dout SVD_ADDRESS_GROUP ""

add_interface_port dout dout_ready ready Input 1
add_interface_port dout dout_valid valid Output 1
add_interface_port dout dout_data data Output 24
add_interface_port dout dout_sop startofpacket Output 1
add_interface_port dout dout_eop endofpacket Output 1
add_interface_port dout dout_empty empty Output 2

