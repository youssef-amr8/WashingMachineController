vlib work
vlog newdesign.v nnewtbb.v +cover -covercells
vsim -voptargs=+acc work.tb_top_washing_machine -cover
add wave -r /*
add wave /tb_top_washing_machine/dut/controller/assert__Any_state_to_off
add wave /tb_top_washing_machine/dut/controller/assert__DRN_TO_SPN
add wave /tb_top_washing_machine/dut/controller/assert__OFF_TO_INIT
add wave /tb_top_washing_machine/dut/controller/assert__PAUSED_TO_temp_state
add wave /tb_top_washing_machine/dut/controller/assert__SPN_TO_DRN
add wave /tb_top_washing_machine/dut/controller/assert__WASH_TO_PAUSED
add wave /tb_top_washing_machine/dut/controller/assert__WTR_FIL_TO_WASH
coverage save washing123.ucdb -onexit
run -all 
vcover report washing123.ucdb -details -annotate -all > coverageReport.txt