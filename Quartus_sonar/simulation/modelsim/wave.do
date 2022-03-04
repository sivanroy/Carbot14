onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -expand -group Utility /MyTestbench/clk
add wave -noupdate -expand -group Utility /MyTestbench/reset
add wave -noupdate -expand -group Utility -radix decimal /MyTestbench/dut/counter
add wave -noupdate -expand -group Utility /MyTestbench/launch
add wave -noupdate -expand -group Utility /MyTestbench/dut/running
add wave -noupdate -expand -group Data_in -color {Sea Green} -itemcolor {Lime Green} -radix hexadecimal /MyTestbench/dut/data_in
add wave -noupdate -expand -group Data_in -color {Sea Green} -itemcolor {Lime Green} /MyTestbench/dut/rw
add wave -noupdate -expand -group Data_in -color {Sea Green} -itemcolor {Lime Green} -radix hexadecimal /MyTestbench/Adr
add wave -noupdate -expand -group Data_in -color {Sea Green} -itemcolor {Lime Green} -radix hexadecimal /MyTestbench/dut/addr
add wave -noupdate -expand -group a -radix hexadecimal /MyTestbench/dut/data_out
add wave -noupdate -expand -group a -color White /MyTestbench/ready
add wave -noupdate -expand -group a -color White -itemcolor White /MyTestbench/dut/enable
add wave -noupdate -expand -group Status -color Red -itemcolor Red /MyTestbench/dut/state
add wave -noupdate -expand -group Status -color Red -itemcolor Red -radix decimal /MyTestbench/dut/done
add wave -noupdate -expand -group Status -color Red -itemcolor Red /MyTestbench/dut/i2c_end
add wave -noupdate -expand -group Data -radix decimal /MyTestbench/dut/distance
add wave -noupdate -expand -group Data -radix hexadecimal /MyTestbench/dut/highByte
add wave -noupdate -expand -group Data -radix hexadecimal /MyTestbench/dut/lowByte
add wave -noupdate -expand -group Data -radix hexadecimal /MyTestbench/dut/total_distance
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {170973 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 100
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {241500 ps}
