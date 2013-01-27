/*
 * pwm.h
 *
 *  Created on: Dec 24, 2012
 *      Author: Donald R. Poole, Jr.
 */

#ifndef PWM_H_
#define PWM_H_

/********************************************/
/* L4 Peripheral Clock Module Configuration */
/********************************************/
#define CM_PER_REG_START        0x44E00000
#define CM_PER_REG_END          0x44E03FFF
#define CM_PER_REG_SIZE         (CM_PER_REG_END - CM_PER_REG_START)

#define CM_PER_EPWMSS0_CLKCTRL  0xD4
#define CM_PER_EPWMSS1_CLKCTRL  0xCC
#define CM_PER_EPWMSS2_CLKCTRL  0xD8

/***************************/
/* PWM Subsystem Addresses */
/***************************/

// PWM Subsystem 0
#define PWM_SUB_0_CONFIG_START  0x48300000
#define PWM_SUB_0_CONFIG_END    0x483000FF
#define PWM_SUB_0_ECAP_START    0x48300100
#define PWM_SUB_0_ECAP_END      0x4830017F
#define PWM_SUB_0_EQEP_START    0x48300180
#define PWM_SUB_0_EQEP_END      0x483001FF
#define PWM_SUB_0_EPWM_START    0x48300200
#define PWM_SUB_0_EPWM_END      0x4830025F
#define PWM_SUB_0_CONFIG_SIZE   (PWM_SUB_0_CONFIG_END - PWM_SUB_0_CONFIG_START)
#define PWM_SUB_0_ECAP_SIZE     (PWM_SUB_0_ECAP_END - PWM_SUB_0_ECAP_START)
#define PWM_SUB_0_EQEP_SIZE     (PWM_SUB_0_EQEP_END - PWM_SUB_0_EQEP_START)
#define PWM_SUB_0_EPWM_SIZE     (PWM_SUB_0_EPWM_END - PWM_SUB_0_EPWM_START)

// PWM Subsystem 1
#define PWM_SUB_1_CONFIG_START  0x48302000
#define PWM_SUB_1_CONFIG_END    0x483020FF
#define PWM_SUB_1_ECAP_START    0x48302100
#define PWM_SUB_1_ECAP_END      0x4830217F
#define PWM_SUB_1_EQEP_START    0x48302180
#define PWM_SUB_1_EQEP_END      0x483021FF
#define PWM_SUB_1_EPWM_START    0x48302200
#define PWM_SUB_1_EPWM_END      0x4830225F
#define PWM_SUB_1_CONFIG_SIZE   (PWM_SUB_1_CONFIG_END - PWM_SUB_1_CONFIG_START)
#define PWM_SUB_1_ECAP_SIZE     (PWM_SUB_1_ECAP_END - PWM_SUB_1_ECAP_START)
#define PWM_SUB_1_EQEP_SIZE     (PWM_SUB_1_EQEP_END - PWM_SUB_1_EQEP_START)
#define PWM_SUB_1_EPWM_SIZE     (PWM_SUB_1_EPWM_END - PWM_SUB_1_EPWM_START)

// PWM Subsystem 2
#define PWM_SUB_2_CONFIG_START  0x48304000
#define PWM_SUB_2_CONFIG_END    0x483040FF
#define PWM_SUB_2_ECAP_START    0x48304100
#define PWM_SUB_2_ECAP_END      0x4830417F
#define PWM_SUB_2_EQEP_START    0x48304180
#define PWM_SUB_2_EQEP_END      0x483041FF
#define PWM_SUB_2_EPWM_START    0x48304200
#define PWM_SUB_2_EPWM_END      0x4830425F
#define PWM_SUB_2_CONFIG_SIZE   (PWM_SUB_2_CONFIG_END - PWM_SUB_2_CONFIG_START)
#define PWM_SUB_2_ECAP_SIZE     (PWM_SUB_2_ECAP_END - PWM_SUB_2_ECAP_START)
#define PWM_SUB_2_EQEP_SIZE     (PWM_SUB_2_EQEP_END - PWM_SUB_2_EQEP_START)
#define PWM_SUB_2_EPWM_SIZE     (PWM_SUB_2_EPWM_END - PWM_SUB_2_EPWM_START)

/**********************************/
/* PWM Subsystem Register Offsets */
/**********************************/
#define PWMSS_CONFIG_IDVER      0x0
#define PWMSS_CONFIG_SYSCONFIG  0x4
#define PWMSS_CONFIG_CLKCONFIG  0x8
#define PWMSS_CONFIG_CLKSTATUS  0xC
// ePWM Module - Time-Based Submodule Registers
#define EPWM_TBCTL              0x0
#define EPWM_TBSTS              0x2
#define EPWM_TBPHSHR            0x4
#define EPWM_TBPHS              0x6
#define EPWM_TBCNT              0x8
#define EPWM_TBPRD              0xA
// ePWM Module - Counter-Compare Submodule Registers
#define EPWM_CMPCTL             0xE
#define EPWM_CMPAHR             0x10
#define EPWM_CMPA               0x12
#define EPWM_CMPB               0x14
// ePWM Module - Action-Qualifier Submodule Registers
#define EPWM_AQCTLA             0x16
#define EPWM_AQCTLB             0x18
#define EPWM_AQSFRC             0x1A
#define EPWM_AQCSFRC            0x1C
// ePWM Module - Dead-Band Generator Submodule Registers
#define EPWM_DBCTL              0x1E
#define EPWM_DBRED              0x20
#define EPWM_DBFED              0x22
// ePWM Module - Trip-Zone Submodule Registers
#define EPWM_TZSEL              0x24
#define EPWM_TZCTL              0x28
#define EPWM_TZEINT             0x2A
#define EPWM_TZFLG              0x2C
#define EPWM_TZCLR              0x2E
#define EPWM_TZFRC              0x30
// ePWM Module - Event-Trigger Submodule Registers
#define EPWM_ETSEL              0x32
#define EPWM_ETPS               0x34
#define EPWM_ETFLG              0x36
#define EPWM_ETCLR              0x38
#define EPWM_ETFRC              0x3A
// ePWM Module - PWM-Chopper Submodule Registers
#define EPWM_PCCTL              0x3C
// ePWM Module - High-Resolution PWM (HRPWM) Submodule Registers
#define EPWM_HRCTL              0x40

#endif
