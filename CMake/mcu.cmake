set(CPU -mcpu=cortex-m4)
set(FPU -mfpu=fpv4-sp-d16)
set(FLOAT_ABI -mfloat-abi=hard )
set(FLOAT_PRINTF -u _printf_float)

set(CPU_COMPILE_OPTIONS 
	${CPU}
	-mthumb
	${FPU}
	${FLOAT_ABI}
	${FLOAT_PRINTF}
)
