#pragma once

/* disabled: CONFIG_LIB_SEL4_INLINE_INVOCATIONS */
/* disabled: CONFIG_LIB_SEL4_DEFAULT_FUNCTION_ATTRIBUTES */
/* disabled: CONFIG_LIB_SEL4_PUBLIC_SYMBOLS */
#define CONFIG_LIB_SEL4_FUNCTION_ATTRIBUTE inline
#define CONFIG_LIB_SEL4_INLINE_INVOCATIONS 1 /* LibSel4FunctionAttributeInline=ON */
/* disabled: CONFIG_LIB_SEL4_STUBS_USE_IPC_BUFFER_ONLY */
#define CONFIG_LIB_SEL4_PRINT_INVOCATION_ERRORS 0
