#general
STEPS_PER_ROTATION = 400 # Represents the number of steps (Slices) that the scanner will make

#step motor config
DIR_PIN  = 17 #Step motor GPIO pins
STEP_PIN = 27
M2_PIN   = 16
M1_PIN   = 22
M0_PIN   = 24

STEP_DELAY = 0.002          # delay between steps
MOVE_MICROSTEPS = int(800 / STEPS_PER_ROTATION) # how far the motor moves per service call

#data_service config
OUTPUT_PATH = "/output/scan.pcd" # Path of the output file



