#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Robot-Guinedor-Wemaere.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Robot-Guinedor-Wemaere.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=main.c ChipConfig.c IO.c timer.c robot.c toolbox.c PWM.c ADC.c etat.c CB_TX1.c CB_RX1.c UART.c UART_Protocol.c QEI.c Utilities.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.o ${OBJECTDIR}/ChipConfig.o ${OBJECTDIR}/IO.o ${OBJECTDIR}/timer.o ${OBJECTDIR}/robot.o ${OBJECTDIR}/toolbox.o ${OBJECTDIR}/PWM.o ${OBJECTDIR}/ADC.o ${OBJECTDIR}/etat.o ${OBJECTDIR}/CB_TX1.o ${OBJECTDIR}/CB_RX1.o ${OBJECTDIR}/UART.o ${OBJECTDIR}/UART_Protocol.o ${OBJECTDIR}/QEI.o ${OBJECTDIR}/Utilities.o
POSSIBLE_DEPFILES=${OBJECTDIR}/main.o.d ${OBJECTDIR}/ChipConfig.o.d ${OBJECTDIR}/IO.o.d ${OBJECTDIR}/timer.o.d ${OBJECTDIR}/robot.o.d ${OBJECTDIR}/toolbox.o.d ${OBJECTDIR}/PWM.o.d ${OBJECTDIR}/ADC.o.d ${OBJECTDIR}/etat.o.d ${OBJECTDIR}/CB_TX1.o.d ${OBJECTDIR}/CB_RX1.o.d ${OBJECTDIR}/UART.o.d ${OBJECTDIR}/UART_Protocol.o.d ${OBJECTDIR}/QEI.o.d ${OBJECTDIR}/Utilities.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.o ${OBJECTDIR}/ChipConfig.o ${OBJECTDIR}/IO.o ${OBJECTDIR}/timer.o ${OBJECTDIR}/robot.o ${OBJECTDIR}/toolbox.o ${OBJECTDIR}/PWM.o ${OBJECTDIR}/ADC.o ${OBJECTDIR}/etat.o ${OBJECTDIR}/CB_TX1.o ${OBJECTDIR}/CB_RX1.o ${OBJECTDIR}/UART.o ${OBJECTDIR}/UART_Protocol.o ${OBJECTDIR}/QEI.o ${OBJECTDIR}/Utilities.o

# Source Files
SOURCEFILES=main.c ChipConfig.c IO.c timer.c robot.c toolbox.c PWM.c ADC.c etat.c CB_TX1.c CB_RX1.c UART.c UART_Protocol.c QEI.c Utilities.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Robot-Guinedor-Wemaere.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP512GM306
MP_LINKER_FILE_OPTION=,--script=p33EP512GM306.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/574b7e62811073668b066085b665bc07b4449249 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/ChipConfig.o: ChipConfig.c  .generated_files/flags/default/e3331ab453c324fc842b97d0399e9a82f151b463 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ChipConfig.o.d 
	@${RM} ${OBJECTDIR}/ChipConfig.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ChipConfig.c  -o ${OBJECTDIR}/ChipConfig.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/ChipConfig.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/IO.o: IO.c  .generated_files/flags/default/26e245a88faa1d25660de32c14b207b89717371a .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/IO.o.d 
	@${RM} ${OBJECTDIR}/IO.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  IO.c  -o ${OBJECTDIR}/IO.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/IO.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/timer.o: timer.c  .generated_files/flags/default/b3e2771c6eb6f75d4ec8950c5cce2ba0a94dd40c .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/timer.o.d 
	@${RM} ${OBJECTDIR}/timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  timer.c  -o ${OBJECTDIR}/timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/robot.o: robot.c  .generated_files/flags/default/42a974652e476e11048b006ae1e5ce9431adbb7f .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/robot.o.d 
	@${RM} ${OBJECTDIR}/robot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  robot.c  -o ${OBJECTDIR}/robot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/robot.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/toolbox.o: toolbox.c  .generated_files/flags/default/725941ef2f49c6b16f5714ad7e2c0647ca74c51b .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/toolbox.o.d 
	@${RM} ${OBJECTDIR}/toolbox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  toolbox.c  -o ${OBJECTDIR}/toolbox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/toolbox.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/PWM.o: PWM.c  .generated_files/flags/default/2a4aaa4c3c35f671812d2cd823e63faafd12a7f2 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/PWM.o.d 
	@${RM} ${OBJECTDIR}/PWM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PWM.c  -o ${OBJECTDIR}/PWM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/PWM.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/ADC.o: ADC.c  .generated_files/flags/default/e562394f4609538e805e2faba2dc2ebe07cb6e83 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ADC.o.d 
	@${RM} ${OBJECTDIR}/ADC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ADC.c  -o ${OBJECTDIR}/ADC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/ADC.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/etat.o: etat.c  .generated_files/flags/default/3774f6939dfa9e4c9e64aa95395df54574142595 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/etat.o.d 
	@${RM} ${OBJECTDIR}/etat.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  etat.c  -o ${OBJECTDIR}/etat.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/etat.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/CB_TX1.o: CB_TX1.c  .generated_files/flags/default/d5da10c183dc97e27bf14925b6a3085dbecb6e07 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/CB_TX1.o.d 
	@${RM} ${OBJECTDIR}/CB_TX1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  CB_TX1.c  -o ${OBJECTDIR}/CB_TX1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/CB_TX1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/CB_RX1.o: CB_RX1.c  .generated_files/flags/default/8ee553c79000aab3edeee8ac23548d42aee810ae .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/CB_RX1.o.d 
	@${RM} ${OBJECTDIR}/CB_RX1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  CB_RX1.c  -o ${OBJECTDIR}/CB_RX1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/CB_RX1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/UART.o: UART.c  .generated_files/flags/default/5cd5c20382ec17e048c0531cc5e273d69f1c4c50 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UART.o.d 
	@${RM} ${OBJECTDIR}/UART.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UART.c  -o ${OBJECTDIR}/UART.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/UART.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/UART_Protocol.o: UART_Protocol.c  .generated_files/flags/default/829879395246fbce30a7b14dafb55c4962c56f72 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UART_Protocol.o.d 
	@${RM} ${OBJECTDIR}/UART_Protocol.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UART_Protocol.c  -o ${OBJECTDIR}/UART_Protocol.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/UART_Protocol.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/QEI.o: QEI.c  .generated_files/flags/default/d695edb7727aae17adbfa2663771eb01f0f094b5 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/QEI.o.d 
	@${RM} ${OBJECTDIR}/QEI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  QEI.c  -o ${OBJECTDIR}/QEI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/QEI.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Utilities.o: Utilities.c  .generated_files/flags/default/4b59e959d628c4010503287f0e50dc4467410f92 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Utilities.o.d 
	@${RM} ${OBJECTDIR}/Utilities.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Utilities.c  -o ${OBJECTDIR}/Utilities.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/Utilities.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/63636aa1e334ce4ecd662470b2a870068e7189cc .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/ChipConfig.o: ChipConfig.c  .generated_files/flags/default/48ef17fc192b009599a3a653a29a47b804956a48 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ChipConfig.o.d 
	@${RM} ${OBJECTDIR}/ChipConfig.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ChipConfig.c  -o ${OBJECTDIR}/ChipConfig.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/ChipConfig.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/IO.o: IO.c  .generated_files/flags/default/f9e5cdffc46bbb84a9b14ffb28f93ff64dcd39c4 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/IO.o.d 
	@${RM} ${OBJECTDIR}/IO.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  IO.c  -o ${OBJECTDIR}/IO.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/IO.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/timer.o: timer.c  .generated_files/flags/default/d5f12b91ca8110c0e0b66873b84d5ff62427b4cd .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/timer.o.d 
	@${RM} ${OBJECTDIR}/timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  timer.c  -o ${OBJECTDIR}/timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/timer.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/robot.o: robot.c  .generated_files/flags/default/1d556c7db21e7538ec78341f8c999e1b15505b66 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/robot.o.d 
	@${RM} ${OBJECTDIR}/robot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  robot.c  -o ${OBJECTDIR}/robot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/robot.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/toolbox.o: toolbox.c  .generated_files/flags/default/c68a16d649bb0cca606890c9805cb40ed07b85a7 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/toolbox.o.d 
	@${RM} ${OBJECTDIR}/toolbox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  toolbox.c  -o ${OBJECTDIR}/toolbox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/toolbox.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/PWM.o: PWM.c  .generated_files/flags/default/fc28965440dd76ce7a7b5510358edfa933850cb4 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/PWM.o.d 
	@${RM} ${OBJECTDIR}/PWM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PWM.c  -o ${OBJECTDIR}/PWM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/PWM.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/ADC.o: ADC.c  .generated_files/flags/default/6f107e2ba7985042091f8069be396510fe181d2e .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ADC.o.d 
	@${RM} ${OBJECTDIR}/ADC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ADC.c  -o ${OBJECTDIR}/ADC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/ADC.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/etat.o: etat.c  .generated_files/flags/default/373e02cccdb10e1efec6e1f019de2014af7e0878 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/etat.o.d 
	@${RM} ${OBJECTDIR}/etat.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  etat.c  -o ${OBJECTDIR}/etat.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/etat.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/CB_TX1.o: CB_TX1.c  .generated_files/flags/default/5a0d06a6853680be18a187028542350659efa137 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/CB_TX1.o.d 
	@${RM} ${OBJECTDIR}/CB_TX1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  CB_TX1.c  -o ${OBJECTDIR}/CB_TX1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/CB_TX1.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/CB_RX1.o: CB_RX1.c  .generated_files/flags/default/e3c4bdae7854c94e4ea5baf0a0b41f31facabc8 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/CB_RX1.o.d 
	@${RM} ${OBJECTDIR}/CB_RX1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  CB_RX1.c  -o ${OBJECTDIR}/CB_RX1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/CB_RX1.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/UART.o: UART.c  .generated_files/flags/default/a3122f9aabebe4842ed9810250ee774dde072b91 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UART.o.d 
	@${RM} ${OBJECTDIR}/UART.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UART.c  -o ${OBJECTDIR}/UART.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/UART.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/UART_Protocol.o: UART_Protocol.c  .generated_files/flags/default/285034934ecfd1eefdb59c66f944ba5a550805fd .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UART_Protocol.o.d 
	@${RM} ${OBJECTDIR}/UART_Protocol.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UART_Protocol.c  -o ${OBJECTDIR}/UART_Protocol.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/UART_Protocol.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/QEI.o: QEI.c  .generated_files/flags/default/9a781a8923fe36412a80ae0a83d1f538a23cf2ef .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/QEI.o.d 
	@${RM} ${OBJECTDIR}/QEI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  QEI.c  -o ${OBJECTDIR}/QEI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/QEI.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Utilities.o: Utilities.c  .generated_files/flags/default/42f8bfcf8b122856e972d5df6450020294dddee3 .generated_files/flags/default/d0069a406b199ddf13f961b25b13067240c1ddd8
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Utilities.o.d 
	@${RM} ${OBJECTDIR}/Utilities.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Utilities.c  -o ${OBJECTDIR}/Utilities.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/Utilities.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Robot-Guinedor-Wemaere.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Robot-Guinedor-Wemaere.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_ICD4=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Robot-Guinedor-Wemaere.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Robot-Guinedor-Wemaere.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Robot-Guinedor-Wemaere.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   -mdfp="${DFP_DIR}/xc16" 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
