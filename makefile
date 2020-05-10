
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = Miniprojet

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ../lib/e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
		./robot_management.c \
		./process_image.c \
		./audio_processing.c\
		./led_gestion.c\

#Header folders to include
INCDIR += 

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile