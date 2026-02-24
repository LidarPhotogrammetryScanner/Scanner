# RPi_GPIO_Mock.py

# GPIO mode constants
BOARD = "BOARD"
BCM = "BCM"

# GPIO setup constants
IN = "IN"
OUT = "OUT"
PUD_UP = "PUD_UP"
PUD_DOWN = "PUD_DOWN"

# GPIO edge detection constants
RISING = "RISING"
FALLING = "FALLING"
BOTH = "BOTH"

# Mock functions
def setmode(mode):
    print(f"[MOCK] setmode({mode}) called")

def getmode():
    print("[MOCK] getmode() called")
    return None

def setup(channel, mode, pull_up_down=None):
    print(f"[MOCK] setup(channel={channel}, mode={mode}, pull_up_down={pull_up_down}) called")

def output(channel, value):
    print(f"[MOCK] output(channel={channel}, value={value}) called")

def input(channel):
    print(f"[MOCK] input(channel={channel}) called")
    return 0  # always return LOW for testing

def cleanup(channel=None):
    if channel:
        print(f"[MOCK] cleanup(channel={channel}) called")
    else:
        print("[MOCK] cleanup() called")

def add_event_detect(channel, edge, callback=None, bouncetime=None):
    print(f"[MOCK] add_event_detect(channel={channel}, edge={edge}, callback={callback}, bouncetime={bouncetime}) called")

def remove_event_detect(channel):
    print(f"[MOCK] remove_event_detect(channel={channel}) called")
