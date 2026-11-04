################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

TEST ?= 0
ifneq ($(filter $(TEST),0 1),$(TEST))
$(error Unknown TEST '$(TEST)'; use 0 or 1)
endif

# Reference and host tests must never enter the robot image.
APP_DIRS := ../app_program ../app_program/app ../app_program/board \
    ../app_program/drivers ../app_program/platform \
    $(wildcard ../app_program/modules/*)
APP_SRCS := $(foreach directory,$(APP_DIRS),$(wildcard $(directory)/*.c))
APP_TEST_SRCS := $(foreach directory,$(APP_DIRS),$(wildcard $(directory)/test_*.c))
APP_SRCS := $(filter-out $(APP_TEST_SRCS),$(APP_SRCS))

ifeq ($(TEST),1)
APP_SRCS += $(APP_TEST_SRCS) ../app_program/tests/test_report.c
EXE_FILE := $(EXE_FILE)_test
endif

# Separate object trees prevent stale APP_TEST values when TEST is switched.
APP_OBJ_DIR := mtkernel_3/app_program/obj_test$(TEST)
APP_OBJS := $(patsubst ../app_program/%.c,$(APP_OBJ_DIR)/%.o,$(APP_SRCS))
APP_DEPS := $(APP_OBJS:.o=.d)

OBJS += $(APP_OBJS)
C_DEPS += $(APP_DEPS)

PYTHON ?= python3
.PHONY: app_standard
app_standard:
	$(PYTHON) ../tools/check_app_standard.py

$(APP_OBJS): | app_standard

$(APP_OBJ_DIR)/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p "$(dir $@)"
	$(GCC) $(filter-out -std=%,$(CFLAGS)) -std=c99 -Wall -Wextra \
	    -D$(TARGET) -DAPP_TEST=$(TEST) $(INCPATH) \
	    -I"../app_program" -I"../device/include" \
	    -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
