SHELL := /bin/bash  # Use bash syntax
#	Compiler
CPP = g++
#	Compiler Flags
FLAGS = -Wall -Werror
#	Compiler Libraries
LIBS = 
#	Name of the Server
EXECUTABLE := phoenix



SRCDIR = src
OBJDIR = obj

#	Sources
SRC = $(wildcard $(SRCDIR)/*.cpp)
#	Headers
HDRS = $(wildcard $(SRCDIR)/*.hpp)
#	Creats Objects names
FILES := $(SRC:$(SRCDIR)/%.cpp=$(OBJDIR)/%)
#	Creats Objects names
OBJ := $(SRC:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


# ${variable%pattern} is like $variable, minus shortest matching pattern from the back-end;
# ${variable##pattern} is like $variable, minus the longest matching pattern from front-end.

$(FILES):
	@mkdir -p $(OBJDIR)
	@for f in $(FILES) ;\
	do \
		$(CPP) $(FLAGS) -c $(SRCDIR)/$${f##*/}.cpp -o $(OBJDIR)/$${f##*/}.o ;\
	done

$(EXECUTABLE):
	@$(CPP) $(FLAGS) $(LIBS) $(SRC) -o $(EXECUTABLE)

# pre define number of clients
C := 2

# command to run
CMD := ./$(EXECUTABLE)

# compiles
all:  $(EXECUTABLE)

# gets info
info:
		@echo executable : $(EXECUTABLE)
		@echo source     : $(SRC)
		@echo headers    : $(HDRS)
		@echo objects    : $(OBJ)
		@echo libraries	 : $(LIBS)

# runs executable
run:
		@$(CMD)

# creats multiple client
# e.g. make client C=3 CMD="./phoenix g l 2"
client:
		@num=1 ; while [[ $$num -le $(C) ]] ; do \
			x-terminal-emulator -hold -e "$(CMD)" & \
			((num = num + 1)) ; \
		done

# to run x-terminal-emulator without terminating, add: -hold

    # x-terminal-emulator -e "./phoenix" & \
# deletes the executable and the objects
clean:
		@rm	-rf	$(OBJDIR) $(EXECUTABLE)
		@echo cleaned

# cat -e -t -v Makefile
# dmesg

# for further reading: https://stackoverflow.com/questions/25362847/cpp-to-o-files-in-makefile