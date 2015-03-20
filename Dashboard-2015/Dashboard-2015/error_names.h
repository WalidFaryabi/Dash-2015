/*
 * error_names.h
 *
 * Created: 12.02.2015 16:52:57
 *  Author: Will
 */ 


#ifndef ERROR_NAMES_H_
#define ERROR_NAMES_H_


const char *ecu_error_names[10] = {"No error","Lost BMS communication", "Inverter voltage < 0.9*batt pck voltage",
	 "Lost inverter communication", "Internal error in inverter", 
	"Lost torque sensor communication", "Lost speed sensor communication", 
	"FRG_RUN is low", "AIR_PLUS is low", "BSPD loss signal"};
	
	
const char *bms_fault_names[20] = {"No fault", "Driving off while plugged in", "Interlock is tripped", 
	"Comm fault with bank or cell", "Charge overcurrent", "Discharge overcurrent", "Over-temperature",
	 "Under voltage", "Over voltage", "No battery voltage", "High voltage B- leak to chassis", 
	 "High voltage B+ leak to chassis", "Relay K1 i shorted", "Contactor K2 is shorted", 
	 "Contactor K3 is shorted", "Open K1 or K3, or shorted K2", "Open K2", "Excessive precharge time", 
	 "EEPROM stack overflow", "Loss of CAN from HVFE"};
	 
	 
const char *bms_warning_names[9] = {"No warning", "Isolation fault", "Low SOH", "High temperature", "Low temperature",
	 "Discharge overcurrent", "Charge overcurrent", "High voltge", "Low voltage"};


#endif /* ERROR_NAMES_H_ */