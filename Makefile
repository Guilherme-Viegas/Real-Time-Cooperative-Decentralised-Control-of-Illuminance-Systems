SHELL := /bin/bash  # Use bash syntax
#	Compiler
CXX = g++
#	Compiler Flags
CXXFLAGS = -Wall -Werror -std=c++11 -g
#	Compiler Libraries
LIBS = -lboost_system -pthread
#	Name of the Client
CLIENT := client_exe
#	Name of the Server
SERVER := server_exe
# path of the executables
CLIDIR = client_code
SERVDIR = server_code
# path to the objects
CLIOBJDIR = $(CLIDIR)/obj
SERVOBJDIR = $(SERVDIR)/obj
#	Sources
CLISRC = $(wildcard $(CLIDIR)/*.cpp)
SERVSRC = $(wildcard $(SERVDIR)/*.cpp)
#	Headers
CLIHDRS = $(wildcard $(CLIDIR)/*.hpp)
SERVHDRS = $(wildcard $(SERVDIR)/*.hpp)
#	Creats Objects names
CLIFILES := $(CLISRC:$(CLIDIR)/%.cpp=$(CLIOBJDIR)/%)
SERVFILES := $(SERVSRC:$(SERVDIR)/%.cpp=$(SERVOBJDIR)/%)
#	Creats Objects files
CLIOBJ := $(CLISRC:$(CLIDIR)/%.cpp=$(CLIOBJDIR)/%.o)
SERVOBJ := $(SERVSRC:$(SERVDIR)/%.cpp=$(SERVOBJDIR)/%.o)

# pre define number of clients
C := 2

# command to run the client
CMD := ./$(CLIENT)

# ${variable%pattern} is like $variable, minus shortest matching pattern from the back-end;
# ${variable##pattern} is like $variable, minus the longest matching pattern from front-end.


# gets info
info:
		clear
		@echo client  👨‍💻💡  : $(CLIENT)
		@echo server  💻🍓   : $(SERVER)
		@echo "The Makefile was successfully compiled!🏁🏎"

# get all
all: client server

# creates objects and the executable
client:
	clear
	@mkdir -p $(CLIOBJDIR)
	@for f in $(CLIFILES) ;\
	do \
		$(CXX) $(CXXFLAGS) -c $(CLIDIR)/$${f##*/}.cpp -o $(CLIOBJDIR)/$${f##*/}.o ;\
	done
	@$(CXX) $(CXXFLAGS) $(LIBS) $(CLIOBJ) -o $(CLIENT)

server:
	clear
	@mkdir -p $(SERVOBJDIR)
	@for f in $(SERVFILES) ;\
	do \
		$(CXX) $(CXXFLAGS) -c $(SERVDIR)/$${f##*/}.cpp -o $(SERVOBJDIR)/$${f##*/}.o ;\
	done
	@$(CXX) $(CXXFLAGS) $(LIBS) $(SERVOBJ) -o $(SERVER)


# runs the executable
run_client:
		clear
		@echo "              ___________________________________________________                "
		@echo "             /                                                    \         	    "
		@echo "             |    _____________________________________________    |				"
		@echo "             |   |                             ||              |   |				"
		@echo "             |   |  Welcome client.          ,-..-,            |   |				"
		@echo "             |   |                           {_.='}            |   |				"
		@echo "             |   |                      \    {_.='}    /       |   |				"
		@echo "             |   |                          ,.____.,           |   |				"
		@echo "             |   |                         /   ()   \          |   |				"
		@echo "             |   |                   -    |    \/    |    -    |   |				"
		@echo "             |   |                         \   /\   /          |   |				"
		@echo "             |   |                          '' __ ''           |   |				"
		@echo "             |   |                      /              \       |   |				"
		@echo "             |   |                             |               |   |				"
		@echo "             |   |                                             |   |				"
		@echo "             |   |                                             |   |				"
		@echo "             |   |_____________________________________________|   |				"
		@echo "             |                                                     |				"
		@echo "             \____________________________________________________/				"
		@echo "                    \_______________________________________/					"
		@echo "               _______________________________________________					"
		@echo "            _-'    .-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.  --- '-_				"
		@echo "          -'.-.-. .---.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--.  .-.-.'-_			    "
		@echo "        -'.-.-.-. .---.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-'__'. .-.-.-.'-_	    	"
		@echo "     _-'.-.-.-.-. .-----.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-----. .-.-.-.-.'-_		"
		@echo "  _-'.-.-.-.-.-. .---.-. .-----------------------------. .-.---. .---.-.-.-.'-_	"
		@echo " :-----------------------------------------------------------------------------:	"
		@echo " '---._.-----------------------------------------------------------------._.---'	"
		@$(CMD)


# creats multiple client
# e.g. make client C=3 CMD="./phoenix g l 2"
run_n_client:
		@num=1 ; while [[ $$num -le $(C) ]] ; do \
			x-terminal-emulator -hold -e "$(CMD)" & \
			((num = num + 1)) ; \
		done

# runs the server
run: # run_server:
		clear

		@echo "       .~~.   .~~.                                                                   "
		@echo "      '. \ ' ' / .'                                                                  "
		@echo "       .~ .~~~..~.           Server Type                :     Media Service          "
		@echo "      : .~.'~'.~. :                                                                  "
		@echo "     ~ (   ) (   ) ~         Machine                    :     Raspberry pi 3 model B "
		@echo "    ( : '~'.~.'~' : )        Operation System           :     Raspian TODO           "
		@echo "     ~ .~ (   ) ~. ~         Clients                    :     Arduinos               "
		@echo "      (  : '~' :  )          Type of Communications     :     Serial                 "
		@echo "       '~ .~~~. ~'                                                                   "
		@echo "           '~'                                                                       "
		@./$(SERVER)

# to run x-terminal-emulator without terminating, add: -hold

    # x-terminal-emulator -e "./phoenix" & \
# deletes the executable and the objects
clean:
		clear
		@rm	-rf	$(CLIOBJDIR) $(CLIENT) $(SERVOBJDIR) $(SERVER)
		@echo "You got ride of both executables and their obejcts!🧹"


# cat -e -t -v Makefile
# dmesg

# for further reading: https://stackoverflow.com/questions/25362847/cpp-to-o-files-in-makefile