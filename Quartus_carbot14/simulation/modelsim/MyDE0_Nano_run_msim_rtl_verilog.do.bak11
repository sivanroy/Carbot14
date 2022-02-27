transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -sv -work work +incdir+D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14 {D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14/MySonar.sv}
vlog -sv -work work +incdir+D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14 {D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14/MyDE0_Nano.sv}
vlog -sv -work work +incdir+D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14 {D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14/MySPI.sv}
vlog -sv -work work +incdir+D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14 {D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14/Encoder.sv}

vlog -sv -work work +incdir+D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14 {D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14/MyTestbench.sv}
vlog -sv -work work +incdir+D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14 {D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_carbot14/MySonar.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cycloneive_ver -L rtl_work -L work -voptargs="+acc"  MyTestbench

add wave *
view structure
view signals
run 10 sec
