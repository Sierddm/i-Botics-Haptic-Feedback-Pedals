SHELL:=/bin/bash

#
# Folder definitions.
#
CUR_DIR:=$(shell pwd)
SOURCE_DIR:=$(CUR_DIR)/src
INCLUDE_DIR:=$(CUR_DIR)/include
BUILD_DIR:=$(CUR_DIR)/build
OBJECT_DIR:=$(BUILD_DIR)/obj
BIN_DIR:=$(BUILD_DIR)/bin
UTILITY_DIR:=$(OBJECT_DIR)/gpmc_fpga_user/utility
UTILITY_BIN_DIR:=$(BIN_DIR)/gpmc_fpga_user/utility
LIBRARY_BIN_DIR:=$(BIN_DIR)/ramstix
UNIT_TESTS_DIR:=$(OBJECT_DIR)/ramstix/unit-tests
UNIT_TESTS_BIN_DIR:=$(BIN_DIR)/ramstix/unit-tests

#
# Utility declarations.
#
SOURCE_FILES_UTILITY=$(SOURCE_DIR)/gpmc_fpga_user/utility/%_util.c
OBJECT_FILES_UTILITY=$(patsubst %.c,%.o,$(patsubst $(SOURCE_DIR)/%, $(OBJECT_DIR)/%, $(SOURCE_FILES_UTILITY)))
UTILITIES:=$(patsubst $(SOURCE_DIR)/gpmc_fpga_user/utility/%.c, %, $(wildcard $(SOURCE_DIR)/gpmc_fpga_user/utility/*_util.c))
OBJECT_FILES_GPMC=$(patsubst %.c,%.o,$(patsubst $(SOURCE_DIR)/%, $(OBJECT_DIR)/%, $(SOURCE_DIR)/gpmc_fpga_user/gpmc_fpga_user.c))

#
# Unit Test declarations.
#
SOURCE_FILES_UNIT_TESTS=$(SOURCE_DIR)/ramstix/unit-tests/%_test.c
OBJECT_UNIT_TESTS=$(patsubst %.c,%.o,$(patsubst $(SOURCE_DIR)/%, $(OBJECT_DIR)/%, $(SOURCE_FILES_UNIT_TESTS)))
UNIT_TESTS:=$(patsubst $(SOURCE_DIR)/ramstix/unit-tests/%.c, %, $(wildcard $(SOURCE_DIR)/ramstix/unit-tests/*_test.c))

#
# RaMstix userspace libary.
#
SOURCE_FILES_RAMSTIX:=$(wildcard $(SOURCE_DIR)/ramstix/ramstix_*.c) $(SOURCE_DIR)/gpmc_fpga_user/gpmc_fpga_user.c
OBJECT_FILES_RAMSTIX:=$(patsubst %.c,%.o,$(patsubst $(SOURCE_DIR)/%, $(OBJECT_DIR)/%, $(SOURCE_FILES_RAMSTIX)))

#
# Build flags
#
INTERNAL_CFLAGS=-I$(INCLUDE_DIR) -Wall -fPIC
INTERNAL_CPPFLAGS=
INTERNAL_LDFLAGS=

#
# Build commands
#
ifeq ($(V), s)
  PRINT_OUTPUT:=
else
  PRINT_OUTPUT:=@
endif
RM?=rm -f
MKDIR?=mkdir -p
CP=cp
CD=cd
FIND=find
ECHO=echo
CAT=cat
CC=gcc
CXX=g++


# Colored indication strings.
OK_COLOR=\e[32m
ERROR_COLOR=\e[31m
C_BLUE=\e[34m
C_CYAN=\e[36m
NO_COLOR=\e[39m
ERROR_STRING=[$(ERROR_COLOR)ERRORS$(NO_COLOR)]
OK_STRING=[$(OK_COLOR)OK$(NO_COLOR)]
CC_STRING=[$(C_CYAN)CC$(NO_COLOR)]
CXX_STRING=[$(C_CYAN)CXX$(NO_COLOR)]
CLEAN_STRING=[$(C_CYAN)CLEAN$(NO_COLOR)]
DISTCLEAN_STRING=[$(C_CYAN)DISTCLEAN$(NO_COLOR)]

#
# Build targets
#
# prevent non-execution
.PHONY: all clean distclean print

# prevent object file deletion.
.PRECIOUS: %.o
all: libramstix staticlibramstix utilities unit-tests
utilities: $(UTILITIES)
unit-tests: $(UNIT_TESTS)
libramstix: $(OBJECT_FILES_RAMSTIX)
	$(PRINT_OUTPUT)$(MKDIR) $(BUILD_DIR) $(BIN_DIR) $(UNIT_TESTS_BIN_DIR)
	@$(ECHO) -n -e "[$(C_BLUE)LD$(NO_COLOR)] $@"
	$(PRINT_OUTPUT)$(CC) $^ $(LDFLAGS) $(INTERNAL_LDFLAGS) -shared -o $(LIBRARY_BIN_DIR)/libramstix.so 2> temp.log || touch temp.errors
	@if test -e temp.errors; then $(ECHO) -e " $(ERROR_STRING)" && $(CAT) temp.log; elif test -s temp.log; then $(ECHO) "$(WARN_STRING)" && $(CAT) temp.log; else $(ECHO) -e " $(OK_STRING)"; fi;
	$(PRINT_OUTPUT)$(RM) temp.errors temp.log

staticlibramstix: $(OBJECT_FILES_RAMSTIX)
	$(PRINT_OUTPUT)$(MKDIR) $(BUILD_DIR) $(BIN_DIR) $(UNIT_TESTS_BIN_DIR)
	@$(ECHO) -n -e "[$(C_BLUE)LD$(NO_COLOR)] $@"
	$(PRINT_OUTPUT)$(AR) rcs libramstix.a $^ 2> temp.log || touch temp.errors
	@if test -e temp.errors; then $(ECHO) -e " $(ERROR_STRING)" && $(CAT) temp.log; elif test -s temp.log; then $(ECHO) "$(WARN_STRING)" && $(CAT) temp.log; else $(ECHO) -e " $(OK_STRING)"; fi;
	$(PRINT_OUTPUT)$(RM) temp.errors temp.log

%_util: $(OBJECT_FILES_GPMC) $(OBJECT_FILES_UTILITY)
	$(PRINT_OUTPUT)$(MKDIR) $(BUILD_DIR) $(BIN_DIR) $(UTILITY_BIN_DIR)
	@$(ECHO) -n -e "[$(C_BLUE)LD$(NO_COLOR)] $@"
	$(PRINT_OUTPUT)$(CC) $^ $(INTERNAL_LDFLAGS) -o $(UTILITY_BIN_DIR)/$@ 2> temp.log || touch temp.errors
	@if test -e temp.errors; then $(ECHO) -e " $(ERROR_STRING)" && $(CAT) temp.log; elif test -s temp.log; then $(ECHO) "$(WARN_STRING)" && $(CAT) temp.log; else $(ECHO) -e " $(OK_STRING)"; fi;
	$(PRINT_OUTPUT)$(RM) temp.errors temp.log
	
%_test: $(OBJECT_FILES_RAMSTIX) $(OBJECT_UNIT_TESTS)
	$(PRINT_OUTPUT)$(MKDIR) $(BUILD_DIR) $(BIN_DIR) $(UNIT_TESTS_BIN_DIR)
	@$(ECHO) -n -e "[$(C_BLUE)LD$(NO_COLOR)] $@"
	$(PRINT_OUTPUT)$(CC) $^ -lzmq $(LDFLAGS) $(INTERNAL_LDFLAGS) -o $(UNIT_TESTS_BIN_DIR)/$@ 2> temp.log || touch temp.errors
	@if test -e temp.errors; then $(ECHO) -e " $(ERROR_STRING)" && $(CAT) temp.log; elif test -s temp.log; then $(ECHO) "$(WARN_STRING)" && $(CAT) temp.log; else $(ECHO) -e " $(OK_STRING)"; fi;
	$(PRINT_OUTPUT)$(RM) temp.errors temp.log
    
#
# Generic object file targets
#
$(OBJECT_DIR)/%.o: $(SOURCE_DIR)/%.c
	$(PRINT_OUTPUT)$(MKDIR) $(BUILD_DIR) $(OBJECT_DIR)/ramstix $(UNIT_TESTS_DIR) $(UTILITY_DIR)
	@$(ECHO) -e -n "$(CC_STRING) $(patsubst $(OBJECT_DIR)/%,%,$@)"	
	$(PRINT_OUTPUT)$(CC) -c -lzmq $(CFLAGS) $(INTERNAL_CFLAGS) -o $@ $< 2> temp.log || touch temp.errors
	@if test -e temp.errors; then $(ECHO) -e " $(ERROR_STRING)" && $(CAT) temp.log; elif test -s temp.log; then $(ECHO) "$(WARN_STRING)" && $(CAT) temp.log; else $(ECHO) -e " $(OK_STRING)"; fi;
	$(PRINT_OUTPUT)$(RM) temp.errors temp.log

$(OBJECT_DIR)/%.o: $(SOURCE_DIR)/%.cpp
	$(PRINT_OUTPUT)$(MKDIR) $(BUILD_DIR) $(OBJECT_DIR)/ramstix $(UNIT_TESTS_DIR) $(UTILITY_DIR)
	@$(ECHO) -e -n "$(CXX_STRING) $(patsubst $(OBJECT_DIR)/%,%,$@)"	
	$(PRINT_OUTPUT)$(CXX) -c $(CFLAGS) $(INTERNAL_CFLAGS) $(CPPFLAGS) $(INTERNAL_CPPFLAGS) -o $@ $< 2> temp.log || touch temp.errors
	@if test -e temp.errors; then $(ECHO) -e " $(ERROR_STRING)" && $(CAT) temp.log; elif test -s temp.log; then $(ECHO) "$(WARN_STRING)" && $(CAT) temp.log; else $(ECHO) -e " $(OK_STRING)"; fi;
	$(PRINT_OUTPUT)$(RM) temp.errors temp.log
	

#
# Clean environment
#
clean: tools-clean

tools-clean:
	@$(ECHO) -e "$(CLEAN_STRING)"
	$(PRINT_OUTPUT)$(FIND) $(OBJECT_DIR) -name "*.o" -type f -delete 2> /dev/null || touch temp.errors
	$(PRINT_OUTPUT)$(RM) temp.errors
	
distclean:
	@$(ECHO) -e "$(DISTCLEAN_STRING)"
	$(PRINT_OUTPUT)$(RM) -r $(BUILD_DIR)
	
# Debug prints	
print:
	@$(ECHO) $(CUR_DIR)
	@$(ECHO) $(SOURCE_DIR)
	@$(ECHO) $(INCLUDE_DIR)
	@$(ECHO) $(BUILD_DIR)
	@$(ECHO) $(OBJECT_DIR)
	@$(ECHO) $(BIN_DIR)
	@$(ECHO) $(UNIT_TESTS_DIR)
	@$(ECHO) $(UTILITY_DIR)
	@$(ECHO) $(UNIT_TESTS_BIN_DIR)
	@$(ECHO) $(UTILITY_BIN_DIR)
	@$(ECHO) $(UTILITIES)
	@$(ECHO) $(UNIT_TESTS)
#	@$(ECHO) $(CC)$(CROSS_COMPILE)	

