transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -sv -work work +incdir+D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_sonar {D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_sonar/MySonarI2C.sv}

vlog -sv -work work +incdir+D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_sonar {D:/UCL/Q7-Q8_LELME2002/Carbot14/Quartus_sonar/MyTestbench.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cycloneive_ver -L rtl_work -L work -voptargs="+acc"  MyTestbench

add wave *
view structure
view signals
run 1000 us
