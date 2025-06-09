
"""
Andrea Favero 20250608



'Door at the door' project
Code for the stationary device (essentially a LoRa sensor)
Rev 0.1

################################################################################
##### had to remove the display functions, as I mechanically broke it during ###
##### the last assembly. Cut the flat cable to prevent more issues.          ###
################################################################################

Hardware:
- Heltec WiFi LoRa 32 V3 board (based on ESP32-S3)
- Radar HLK LD2010C (set via the app HLKRadarTool)

Logic:
- Stationary device keeps monitoring the radar signal (output pin)
- When it receives a "check" via the LoRa:
    - if the radar does not sense, it replies with 'no_cats_text' via LoRa.
    - if the radar senses the BLE scans for BLE tags, ad replies via LoRa with 'r' + cats-status.
    - if initial idling, it replies with 'd' + cats-status ('d'eactivated radar).
    - N times (4) out of M (80) it prepend the MCU temperature 'txx' in deg. Celsius.

Setting to apply to the radar, via the HLKRadarTool app:
  HLK LD2010C: set output pin active LOW
  HLK LD2010C: set "Unmanned duration" to 20s; This forces the output pin signal to stay active for at least this period

"""

###########################################################################################
# Libraries import ########################################################################
###########################################################################################

from machine import Pin, SoftI2C, PWM, lightsleep, WDT
from esp32 import mcu_temperature      # import for mcu temperature check
from sx1262 import SX1262              # import for LoRa communication
import sys, utime, json, bluetooth



###########################################################################################
# System controls #########################################################################
###########################################################################################

# setting for the Watch Dog Timer
WDT_ENABLED = True



###########################################################################################
# Constants ###############################################################################
###########################################################################################

WHITE_LED_PIN = Pin(35, Pin.OUT, value=0 )   # output pin for the onboard white led, initially set low (led off)
RADAR_PIN =     Pin(6, Pin.IN, Pin.PULL_UP)  # input pin as Pull UP prevents active signal when the HLK LD2010C radar is un-powered

FILE_NAME_CONFIG = "config.json"         # file storing overall settings
FILE_NAME_TAGS = "ble_tags.json"         # file storing BLE tags mappings

OLED_REFRESH_PERIOD_MS = 2000            # cadence for the info refreshment at oled display

MCU_TEMP_RUN_CADENCE = 80                # cadence of sending mcu temperature via LoRa ( 4 out of this Value)

# settings for dynamic adustment of the LoRa transmission power
MIN_LORA_RSSI_TARGET = -100              # min LoRa rssi signal level at receiver, to feedback the sender
MAX_LORA_RSSI_TARGET = -80               # max LoRa rssi signal level at receiver, to feedback the sender
LORA_TX_POWER_MIN = -9                   # min possible LoRa power for SX1262
LORA_TX_POWER_MAX = 22                   # max possible LoRa power for SX1262
MIN_MISSED_PACKETS = 4    # min number of missed LoRa connections, to bring the LoRa power to MAX
MAX_MISSED_PACKETS = 15   # max number of missed LoRa connections to give up (bring the LoRa power to NOM)



###########################################################################################
# settings ################################################################################
###########################################################################################

# set WDT related variables
if WDT_ENABLED:                          # case the WDT_ENABLED is set True
    wdt = WDT(timeout=60000)             # 60s watchdog
    radar_detection = 0                  # flag to track the radar output status

oled_disp_enabled = False                # this variable gets updated from the config.json setting

oled_brightness = {0:65475, 1:65465, 2:65455, 3:60450, 4:55450} # 0=min, 4 = max

prev_radar = 0                           # initial radar variables

mcu_temp = 0                             # mcu temperature is initially set to zero

lora_rssi = None                         # lora_rssi initially set to None, later a negative integer
missed_lora_packets = 0                  # counter of consecutive missed LoRa packages
initial_lora_tx_power = 0                # initial LoRa power, later get update by the config.json
lora_rssi_sender = None                  # lora_rssi_sender initially set to None, later from 0 to 2
lora_sender_action = ""                  # (one char) string for LoRa tx power adjustment at sender
toa_us = 0                               # Time-on-Air is microseconds

###########################################################################################
# functions ###############################################################################
###########################################################################################


def load_config(fname):
    """Load configuration from JSON file"""
    print("\nLoading the configuration ...")
    try:
        with open(fname, 'r') as f:
            config = json.load(f)
        print("Configuration loaded\n")
        return config
    except Exception as e:
        print(f"Error while loading {fname} : {e}")
        sys.exit(1)



def load_tags_ids(fname):
    """Function to load tags IDs"""
    print("\nLoading the BLE tags mapping ...")
    try:
        with open(fname, "r") as f:
            print("BLE tags mapping loaded\n")
            return json.load(f)
    except Exception as e:
        print(f"Error while loading {fname}: {e}")
        sys.exit(1)
    


def decode_my_ble_tags(my_ble_tags):
    """Decode my_ble_tags dictionary"""
    cats_idx, cats_names, cats_macs, no_cats_text = [], [], [], ""
    
    for idx in range(len(my_ble_tags)):
        cats_idx.append(idx)
        cats_names.append(my_ble_tags[str(idx)]["name"])
        cats_macs.append(my_ble_tags[str(idx)]["mac"])
        no_cats_text+= str(idx) + "0,"
    no_cats_text = no_cats_text[:-1]    # removal of the last comma
    
    return cats_idx, cats_names, cats_macs, no_cats_text



def init_backlight(config, oled_brightness, bright_level=3):
    """Initialize the backlight control (pwm)"""
    try:    
        # PWM init (note the max acceptable frequency is only 2445)
        oled_backlight = PWM(Pin(config['oled_display']['oled_backlight_pin']),
                             freq=2445,
                             duty_u16=oled_brightness[bright_level])
        return oled_backlight
    except Exception as e:
        print(f"Error while loading init_backlight as PWM: {e}")
        sys.exit(1)



def init_oled_display_rst_pin(config):
    """Initialize the Oled Display pin"""
    try:
        oled_rst_pin = Pin(config['oled_display']['oled_rst_pin'], Pin.OUT, value=1)   # GPIO setting, keeps reset forset to high (value=1)
        return oled_rst_pin
    except Exception as e:
        print(f"Error while loading init_oled_display_rst_pin {e}")
        sys.exit(1)



def init_i2c(config):
    """Initialize I2C for the OLED display"""
    print("Activating I2C function ...")
    try:
        # Set I2C pins in open_drain mode
        oled_sda_pin = Pin(config['oled_display']['oled_sda_pin'], Pin.OPEN_DRAIN, pull=Pin.PULL_UP)
        oled_scl_pin = Pin(config['oled_display']['oled_scl_pin'], Pin.OPEN_DRAIN, pull=Pin.PULL_UP)
        print("I2C function activated\n")
        return SoftI2C(sda=oled_sda_pin, scl=oled_scl_pin)
    except Exception as e:
        print(f"Error while activating I2C {e}")
        sys.exit(1)



def init_ble():
    """Initialize Blue Tootyh Low energy (BLE)."""
    print("Activating BLE function ...")
    try:
        attempts = 5
        for i in range(attempts):
            try:   
                ble = bluetooth.BLE()
                ble.active(False)       # Ensure Bluetooth is off before start_time_sing
                utime.sleep(0.5)
                ble.active(True)
                print("BLE function activated\n")
                return ble
            except:
                if i <= attempts-2:
                    utime.sleep(1)       # wait before new attempt
    
    except Exception as e:
        print(f"Error while activating BLE {e}")
        sys.exit(1)




def scan_ble():
    """
    Function to scan for BLE tags, and filter out those with RSSI below threshold.
    A scanning timeout applies.
    """
    detected_ble_tags = {}
    unique_identifiers = set()  # Track unique ble_tags using their identifiers
    
    def callback(*args):
        if len(args) >= 2:
            event, data = args[0], args[1]
            if event == 5:  # Event 5 indicates a scan result
                addr_type, addr, adv_type, ble_rssi, adv_data = data
                addr_bytes = bytes(addr)
                adv_data_bytes = bytes(adv_data)
                mac_address = ":".join("{:02X}".format(b) for b in addr_bytes)
                
                # check if the signal is stronger than threshold, and tag not listed yet
                if ble_rssi > ble_rssi_threshold and mac_address not in detected_ble_tags.keys():
#                     print(f"MAC Address: {mac_address}, BLE RSSI: {ble_rssi} dBm")
                    detected_ble_tags[mac_address] = ble_rssi  # store the rssi of detected ble tag
                    
    print("\nScanning for BLE Tags...")
    ble.irq(callback)
    ble.gap_scan(None)
    utime.sleep(1)
    ble.gap_scan(ble_scan_period_s * 1000, ble_scan_period_s * 1000, ble_scan_period_s * 1000)
    
    start_time_s_time = utime.time()
    while utime.time() - start_time_s_time < 0.5 + ble_scan_period_s:
        utime.sleep(0.5)

    ble.gap_scan(None)
    return detected_ble_tags





def analyze_ble_tags(detected_ble_tags, cats_idx, cats_names, cats_macs):
    """Compares the detected BLE tag with those of the cats."""
    ret = {}
    
    printout = False
    if printout:
        print()
    
    for mac_addr in cats_macs:
        if mac_addr in detected_ble_tags.keys():
            idx = cats_macs.index(mac_addr)
            if printout:
                print(f"{cats_names[idx]} is at the door, signal RSSI = {detected_ble_tags[mac_addr]} dB")
            ret[idx] = [1, detected_ble_tags[mac_addr]]
    
    if printout:
        print()
    
    return ret



def init_lora(config):
    "Setup the Lora communication."
    
    print("Activating LoRa ...")
    
    lora_tx_power = int(config['lora']['tx_power'])  # TX power in dBm
    initial_lora_tx_power = lora_tx_power
    print(f"Initial LoRa transmission power: {lora_tx_power} dB")
    

    try:
        lora_sx = SX1262(spi_bus = int(config['lora']['spi_bus']),
                         clk =     Pin(int(config['lora']['clk_pin'])),
                         mosi =    Pin(int(config['lora']['mosi_pin'])),
                         miso =    Pin(int(config['lora']['miso_pin'])),
                         cs =      Pin(int(config['lora']['cs_pin'])),
                         irq =     Pin(int(config['lora']['irq_pin'])),
                         rst =     Pin(int(config['lora']['rst_pin'])),
                         gpio =    Pin(int(config['lora']['dio1_pin']))
                         )

        # configure LoRa parameters
        lora_sx.begin(freq =   int(config['lora']['frequency_MHz']),  # Frequency in MHz
                      bw =     float(config['lora']['bw_KHz']),       # Bandwidth in kHz
                      sf =     int(config['lora']['spread_factor']),  # Spreading factor
                      power =  lora_tx_power,                         # TX power in dBm
                      cr=7,                  # Coding rate (8 is stronger forward error correction)
                      syncWord=0x12,         # Sync word
                      currentLimit=60.0,     # Current limit in mA
                      preambleLength=6,      # Preamble length  (6 is min, 8 is rather common)
                      implicit=False,        # Implicit header mode
                      implicitLen=0xFF,      # Implicit header length
                      crcOn=True,            # CRC enabled
                      txIq=False,            # TX IQ inversion
                      rxIq=False,            # RX IQ inversion
                      tcxoVoltage=1.7,       # TCXO voltage
                      useRegulatorLDO=False, # Use LDO regulator
                      blocking=True          # Blocking mode
                      )

        lora_sx.setBlockingCallback(False, cb)
        print("LoRa activated\n")
        return lora_sx, lora_tx_power, initial_lora_tx_power
    
    except Exception as e:
        print(f"Error while activating LoRa {e}")
        sys.exit(1)



def adjust_lora_tx_power(lora_rssi_sender, lora_tx_power, missed_lora_packets):
    """
    Dynamically adjust the LoRa transmission power, based on feedback from the sender.
    The sender appends one char to the LoRa query (or reply):
    - "<" when the sender rssi is below the min LoRa rssi target
    - ">" when the sender rssi is above the max LoRa rssi target
    
    Four main cases are considered:
    
    Case 1) No lora_rssi value, likely a communication lost:
    - If this happens more than a min number of times, LoRa power is set to MAX.
      This is an attempt to establish communication again.
    - If this happens more than a max number of times, LoRa power is set to nominal.
      This considers one of the device is still powered off.
    
    Case 2) Weak lora_rssi value at sender (< min LoRa rssi target):
    - LoRa power increased by one level at the time, until max is eventually reached.
    
    Case 3) Strong lora_rssi value at sender(> max LoRa rssi target):
    - LoRa power reduced by one level at the time, until min is eventually reached.
    
    Case 4) lora_rssi value within badwidth at sender or no feedback:
    - No action requested to the sender.
    """
    
    # based on the LoRa rssi on this device, assign at action (one char) for the sender
    lora_sender_action = ""
    if lora_rssi is not None:
        missed_lora_packets = 0
        if lora_rssi < MIN_LORA_RSSI_TARGET:
            lora_sender_action = "<"                               
        elif lora_rssi > MAX_LORA_RSSI_TARGET:
            lora_sender_action = ">"
    
    # case the LoRa rssi of this device is none, the occurrence is checked before reacting
    elif lora_rssi is None:
        missed_lora_packets += 1
        if missed_lora_packets > MIN_MISSED_PACKETS and missed_lora_packets <= MAX_MISSED_PACKETS:
            if lora_tx_power < LORA_TX_POWER_MAX:
                lora_tx_power = LORA_TX_POWER_MAX
                lora_sx.setOutputPower(lora_tx_power)
                print(f"\nIncreasing LoRa TX power to max MAX: {lora_tx_power} dBm")
        
        elif missed_lora_packets > MAX_MISSED_PACKETS:
            lora_sx.setOutputPower(initial_lora_tx_power)
            lora_tx_power = initial_lora_tx_power
            print(f"\nSet LoRa TX power to default value: {initial_lora_tx_power} dBm")
    
    if lora_rssi_sender is not None and lora_rssi is not None:
        # case of week LoRa rssi at sender, and receiver not yet to MAX
        if lora_rssi_sender == 0 and lora_tx_power < LORA_TX_POWER_MAX:
            lora_tx_power += 1
            lora_sx.setOutputPower(lora_tx_power)
            print(f"\nIncreasing LoRa TX power to {lora_tx_power} dBm")
        
        # case of strong LoRa rssi at sender, and receiver not yet to MIN
        elif lora_rssi_sender == 1 and lora_tx_power > LORA_TX_POWER_MIN:
            lora_tx_power -= 1
            lora_sx.setOutputPower(lora_tx_power)
            print(f"\nDecreasing LoRa TX power to {lora_tx_power} dBm")

    return lora_tx_power, missed_lora_packets, lora_sender_action
 
    

def cb(events):
    """Function called by the LoRa interrupt."""
    
    global lora_rssi, lora_rssi_sender, toa_us

    if events & SX1262.RX_DONE:
        msg, err = lora_sx.recv()
        error = SX1262.STATUS[err]
        lora_rssi = lora_sx.getRSSI()
        msg = msg.decode("utf-8")
#         print('Received: {}, LoRa RSSI:{}, {}'.format(msg, lora_rssi, error))
        if msg[:2] == "ck":
            lora_sx.send(bytes(str(cats_status), 'utf-8'))
            toa_us = lora_sx.getTimeOnAir(len(cats_status))
        
        # check if any LoRa rssi signal feedback from the sender
        if msg[-1] == "<":           # case the sender adds "<" after "ck"
            lora_rssi_sender = 0     # lora_rssi_sender set to zero (=weak)
        elif msg[-1] == ">":         # case the sender adds ">" after "ck"
            lora_rssi_sender = 1     # lora_rssi_sender set to 1 (=strong)
        else:                        # case no "<" or ">" added to after "ck"
            lora_rssi_sender = None  # lora_rssi_sender set

    elif events & SX1262.TX_DONE:
#         print(f"sent cats_status: {cats_status}\n")
        pass



def get_reset_reason():
    """Checks the reason for the MCU boot, in essence what happened at the power off (WDT, DeepSleep, etc)."""
    
    import machine
    reset_reason = machine.reset_cause()  # Get reset cause
    
    reset_message = {
        machine.PWRON_RESET: "POWER-ON RESET",
        machine.HARD_RESET: "HARD RESET",
        machine.WDT_RESET: "WATCHDOG RESET",
        machine.DEEPSLEEP_RESET: "DEEPSLEEP WAKE",
        machine.SOFT_RESET: "SOFT RESET"
        # machine.BROWNOUT_RESET is missing in some MicroPython versions
    }.get(reset_reason, f"???: {reset_reason}")
    return reset_message

  

def display_backlight_on():
    """Set the oled didplay backlight ON"""
    
    global oled_brightness, bright_level
    PWM(Pin(config['oled_display']['oled_backlight_pin']),
        freq=2445,
        duty_u16=oled_brightness[bright_level])
    
        
        
def display_backlight_off():
    """Set the oled didplay backlight ON"""
    
    global oled_backlight
    PWM(Pin(config['oled_display']['oled_backlight_pin']),
        freq=2445,
        duty_u16=65535)
    
        
    
def display_flash(display, n=2):
    """Flashes the oled display N times"""
    
    sleep_time = 0.1
    for i in range(n):
        display_backlight_off()
        display.fill(1)
        display.show()
        display_backlight_on()
        utime.sleep(sleep_time)
        
        display_backlight_off()
        display.fill(0)
        display.show()
        display_backlight_on()
        utime.sleep(sleep_time)
        


def display_plot_radar(display, radar_detection, time_now, show_time=1):
    """Print the radar output to the oled display"""
    
    display_backlight_on()
    
    display.fill(0)
    display.text(time_now, 0, 5, 1)
    display.text("RADAR RESULT:", 0, 25, 1)
    if radar_detection:
        display.text("DETECTED", 10, 38, 1)
    else:
        display.text("scanning...", 10, 38, 1)
    display.show()
    utime.sleep(show_time)
    


def display_plot_time(display, time_now):
    """Print time to the oled display."""
    
    display_backlight_on()
    display.fill(0)
    display.text(time_now, 0, 5, 1)    
    display.show()



def display_plot_tags(display, analyzed_ble_tags, cats_names, show_time=1):
    """Print cat name and BLE rssi to the oled display"""
    
    cats_number = len(analyzed_ble_tags)
    if cats_number > 0:
        display.fill(0)
        row_pitch = int(54/cats_number)
        row = 0
        for idx, data in analyzed_ble_tags.items():
            y = row * row_pitch + 2 +int(28/cats_number)
            ble_rssi = str(data[1])
            text = f"{cats_names[idx]}  ({ble_rssi})"
            display.text(text, 0, y, 1)
            row+=1
        
        display_backlight_on()
        display.show()
        utime.sleep(show_time)



def display_reset_reason(display, reset_msg):
    """Print the MCU reset reason to the oled display."""
    
    display.fill(0)  # Clear screen
    display.text("RESET:", 0, 20)
    display.text(reset_msg, 0, 40)
    display.show()
    display_backlight_on()
    utime.sleep(1)
    display_backlight_off()



def check_time(start_time_s):
    """Return the elapsed time since arg, in text format"""
    
    hours, remainder = divmod(utime.time() - start_time_s, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
            


def flash_white_led(n=2, time_s=1):
    """Flase the white LED N times"""
    
    for i in range(n):
        WHITE_LED_PIN.value(1)
        utime.sleep(time_s)
        WHITE_LED_PIN.value(0)
        if i < n-1:
            utime.sleep(time_s)
    


def get_mcu_temp_str():
    """Return the MCU temperature in string format, with a 't' in front"""
    
    mcu_temp = mcu_temperature()
    if mcu_temp > 0 and mcu_temp < 125:
        return f"t{str(mcu_temp)},"
    else:
        return "t0,"
         


################################################################################################
################################################################################################
################################################################################################
#                                                                                              #
#                                       main program                                           #
#                                                                                              #
################################################################################################
################################################################################################
################################################################################################


# Load configuration
config = load_config(FILE_NAME_CONFIG)

oled_disp_enabled = bool(config["oled_display"]["oled_enabled"]) # oled display gets eventually enabled

if oled_disp_enabled:
    from lib import ssd1306
    print("\nDisplay is activated")
else:
    print("\nDisplay is not activated")
    

# Load my_tags data
my_ble_tags = load_tags_ids(FILE_NAME_TAGS)

# decode the 
cats_idx, cats_names, cats_macs, no_cats_text = decode_my_ble_tags(my_ble_tags)
# print(f"\nText for no-cats detection: {no_cats_text}  (info: first digit is cat_idx, second digit means absence)")


if oled_disp_enabled:    # added at the last moment, as I broke the display during assembly :-( 
    
    # Initialize I2C
    i2c = init_i2c(config)

    # brightness level of the oled display
    bright_level = config['oled_display']['oled_brightess_level']

    # Initialize the oled backlight pin
    oled_backlight = init_backlight(config, oled_brightness=oled_brightness, bright_level=bright_level)

    # Initialize the Oled Display pin
    oled_rst_pin = init_oled_display_rst_pin(config)

    # Initialize the OLED display
    display = ssd1306.SSD1306_I2C(128, 64, i2c)

    # display flashes a few times
    display_flash(display, n=5)



# Check reset reason
reset_msg = get_reset_reason()
print(f"Last reset reason:{reset_msg}\n" )
if oled_disp_enabled:
    display_reset_reason(display, reset_msg)


# Initial response for LoRa
cats_status = no_cats_text


# Initialize the SX1262 LoRa module
lora_sx, lora_tx_power, initial_lora_tx_power = init_lora(config)


# Activate BLE
ble  = init_ble()


# settings from the config file 
ble_scan_period_s =             int(config['scanners']['ble_scan_time_secs'])            # scanning period, in secs, for BLE devices
ble_rssi_threshold =            int(config['scanners']['ble_rssi_threshold'])            # ignore devices with RSSI below this value
radar_scan_time_interval_secs = int(config['scanners']['radar_scan_time_interval_secs']) # time interval to check the radar output pin
radar_ignor_period_s =          int(config['scanners']['radar_ignor_period_s'])          # period (s) ignoring radar signal (like right after booting)
radar_keep_signal_period_s =    int(config['scanners']['radar_keep_signal_period_s'])    # period (s) keeping radar signal ON after OFF --> ON chanhe

radar_scan_time_interval_ms = 1000 * radar_scan_time_interval_secs
radar_keep_signal_period_ms = 1000 * radar_keep_signal_period_s

# visual feedback the program is active, via the white led, in caase the display is not in use :-(
if not oled_disp_enabled:
    flash_white_led(n=20, time_s=0.05)


# initial MCU temperature
mcu_temp = mcu_temperature()
print(f"mcu_temp: {mcu_temp} degC\n")
    

# initial runs counter is set to MCU_TEMP_RUN_CADENCE + 5
run = MCU_TEMP_RUN_CADENCE + 5


# initial time reference 
start_time_s = utime.time()            # epoch time from RTC of the board is assigned to start_time_s
start_time_ms = utime.ticks_ms()       # time in milliseconds since booting
last_radar_check_ms = utime.ticks_ms() # time in milliseconds since booting
radar_sensed_time_ms = -1000 * radar_keep_signal_period_s # last time the radar sensed something



print(f"\n{'#' * 60}")
n = max(0, 9 - len(str(radar_ignor_period_s)))
print(f"{'#' * 8}   Stationary device: idle for {radar_ignor_period_s} secs ...   {'#' * n}")
print(f"{'#' * 60}\n")


# prepend 'd' to cats_status to 'd'eactivate the cats_status interpretation at the portable device
# add mcu_temperature, that looks like "t44,"
# remaining info are cats related
cats_status = f"d{get_mcu_temp_str()}{no_cats_text}" # 'd' means the radar is temporarily "d"isabled


# intertaining print to terminal while the idle period (radar_ignor_period_s) elapses 
dot = 0
while utime.time() < start_time_s + radar_ignor_period_s:
    if WDT_ENABLED:                   # case the WDT_ENABLED is set True
        wdt.feed()                    # Reset the watchdog timer
    
    print('.', end='')
    dot += 1
    
    if dot <= 30:
        # update cat status with MCU temp; 'd' means the radar is temporarily "d"isabled
        cats_status = f"d{get_mcu_temp_str()}{no_cats_text}"
    elif dot > 30 and dot < 60:
        cats_status = f"d{no_cats_text}" # update cat status without MCU temp; 'd' means the radar is temporarily "d"isabled
    elif dot >= 60:
        dot = 0
        print()
    utime.sleep(1)
    
print("\n"*2)

cats_status = f"d{no_cats_text}"       # update cat status without MCU temp; 'd' means the radar is temporarily "d"isabled
start_time_s = utime.time()            # epoch time from RTC of the board is assigned to start_time_s
start_time_ms = utime.ticks_ms()       # time in milliseconds since booting
last_radar_check_ms = utime.ticks_ms() # time in milliseconds since booting


if oled_disp_enabled:              # case the oled_display is enabled
    display_flash(display, n=5)    # display flashes again, indicating the 


print(f"\n{'#' * 60}")
print(f"{'#' * 11}   Stationary device is active ...   {'#' * 12}")
print(f"{'#' * 60}\n")



################################################################################################
################################################################################################
################################################################################################
#                                                                                              #
#                              repeating part of the main program                              #
#                                                                                              #
################################################################################################
################################################################################################
################################################################################################


while True:                       # infinite loop

    if WDT_ENABLED:               # case the WDT_ENABLED is set True
        wdt.feed()                # Reset the watchdog timer
    
    now = utime.time()            # current time in seconds (rely on ESP RTC)
    now_ms = utime.ticks_ms()     # current time in ms since booting
    
    
    if now_ms > last_radar_check_ms + radar_scan_time_interval_ms:  # case it is time to check the radar
    
        last_radar_check_ms = now_ms                 # time reference is ms for last radar scan
        
        # status of the radard module pin OUT is checked
        radar = not RADAR_PIN.value()                # read the radar output pin
        
        # falsing the radar output for debug purpose
#         radar = False
#         radar = True
        
        # check if the radar output has changed from not sensing to sensing
        if radar and not prev_radar:                 # case the radar pin has changed from OFF to ON
            radar_detection = True                   # radar detection flag is changed to True
            radar_sensed_time_ms = utime.ticks_ms()  # time reference in ms of last radar change from OFF to ON
        
        prev_radar = radar                           # the last radar status is assigned to the previous one
        
        
        # case the radar output if OFF and elapsed time from last sensing is > radar_keep_signal_period_ms
        if not radar and utime.ticks_ms() > radar_sensed_time_ms + radar_keep_signal_period_ms:
            radar_detection = False                  # radar_detection is set False

        
        if not radar_detection:                      # case no radar detection
            
            if run >= MCU_TEMP_RUN_CADENCE and run <= 4 + MCU_TEMP_RUN_CADENCE:  # case to share temperature
                cats_status = get_mcu_temp_str() + no_cats_text # mcu_temp is prepend to no_cats
            else:                                    # case the iteration < threshold
                cats_status = no_cats_text           # only no_cats is assigned to cats_status
            cats_status += lora_sender_action        # add indication for lora power adjustment at sender
                
                
            last_radar_check_ms = utime.ticks_ms()   # current time in ms is assigned to last radar check ms
            time_now = check_time(start_time_s)      # current time (string format) is assigned to time_now
            print(f"{time_now}   lora RSSI:{lora_rssi}   lora_tx_power:{lora_tx_power}   cats_status:{cats_status}   ToA:{toa_us}us")
            if oled_disp_enabled:                    # case oled display is enabled
                display_plot_radar(display, radar_detection, time_now, show_time=5) # plots some info
                display_backlight_off()              # oled display backlight is set off
        
        
        elif radar_detection:                        # case of radar detection
            time_now = check_time(start_time_s)      # current time (string format) is assigned to time_now
            if oled_disp_enabled:                    # case oled display is enabled
                display_plot_radar(display, radar_detection, time_now) # plots some info
            
            detected_ble_tags = scan_ble()           # ble scanning
            
            # the bles scanning result is compared to the cats info
            analyzed_ble_tags = analyze_ble_tags(detected_ble_tags, cats_idx, cats_names, cats_macs)

            last_radar_check_ms = now_ms             # time reference of last radar check
            
            cats_status= "r"                         # 'r' means the 'r'adar has sensed something
            
            if run >= MCU_TEMP_RUN_CADENCE and run <= 4 + MCU_TEMP_RUN_CADENCE:  # case to share temperature
                cats_status += get_mcu_temp_str()    # mcu_temp is added

            if len(analyzed_ble_tags) == 0:          # case no cats where in ble range
                for cat in cats_idx:                 # iteration for the number on cats
                    cats_status += str(cat) + "0" + "," # a zero is assigned to each cat idx
                cats_status = cats_status[:-1]       # last comma is removed
                cats_status += lora_sender_action    # add indication for lora power adjustment at sender
                print(f"{time_now}   LoRa RSSI:{lora_rssi}   lora_tx_power:{lora_tx_power}   cats_status:{cats_status}   ToA:{toa_us}us")
            
            else:                                    # case at least one cat was in ble range
                for cat, data in analyzed_ble_tags.items():  # iteration over the detected ble tags
                    
                    # a string with cat idx + 1 + ble rssi is assigned
                    cats_status += str(cat) + str(data[0]) + str(data[1]) + "," 
                cats_status = cats_status[:-1]       # last comma is removed
                cats_status += lora_sender_action    # add indication for LoRa power adjustment at sender
                
                print(f"{time_now}   LoRa RSSI:{lora_rssi}   lora_tx_power:{lora_tx_power}   cats_status:{cats_status}   ToA:{toa_us}us")
                
                last_radar_check_ms = utime.ticks_ms() # time reference in ms of last radar check
                
                if oled_disp_enabled:                # case oled display is enabled
                    display_plot_tags(display, analyzed_ble_tags, cats_names, show_time=5) # plots some info
                    display_backlight_off()          # oled display backlight is set off
        
        # check and eventually adjust the loRa trasnmission power
        lora_tx_power, missed_lora_packets, lora_sender_action = adjust_lora_tx_power(lora_rssi_sender,
                                                                                      lora_tx_power,
                                                                                      missed_lora_packets)
        
        
        # preventing LoRa power adjustments based on old lora_rssi values
        lora_rssi = None    # set lora_rssi to None
        
        
        # iterator run is used to send mcu temperature via loRa
        # this is only done 5 times every the predefined cadence MCU_TEMP_RUN_CADENCE
        # once the temperature has been sent 5 times, the iterator is reset
        run += 1                                     # the run iterator is increased
        if run > 4 + MCU_TEMP_RUN_CADENCE:           # case run iterator above upper theshold
            run = 0                                  # iteration is set to zero
        
        
        if not oled_disp_enabled:                    # case oled display is disabled
            flash_white_led(n=1, time_s=0.25)        # the white led is flash as run feedback
        
        

    else:                                            # case it is not time to check the radar
    
        if oled_disp_enabled:                        # case oled display is enabled
            time_now = check_time(start_time_s)      # current time (string format) is assigned to time_now
            display_plot_time(display, time_now)     # plots the time
            utime.sleep_ms(OLED_REFRESH_PERIOD_MS // 2)  # sleep time for half OLED_REFRESH_PERIOD_MS
            display_backlight_off()                      # oled display backlight is set off
            utime.sleep_ms(OLED_REFRESH_PERIOD_MS // 2)  # sleep time for half OLED_REFRESH_PERIOD_MS
        
        elif not oled_disp_enabled:                  # case oled display is disabled
            utime.sleep_ms(50)                       # very small sleep
                

# end of code

