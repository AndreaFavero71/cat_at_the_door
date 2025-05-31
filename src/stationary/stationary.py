
"""
Andrea Favero 3



'Door at the door' project
Code for the stationary device (essentially a LoRa sensor)
Rev 0.0

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
    - if the radar senses the BLE scans for BLE tags, ad replies via LoRa with 'r' + cats-status

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
LORA_RSSI_TARGET = -80                   # dBm, desired lora signal level at receiver
LORA_RSSI_HYSTERESIS = 5                 # dB of histeresys to avoid over-steering
LORA_TX_POWER_MIN = 7                    # min possible for SX1262 (in theory down to -9)
LORA_TX_POWER_MAX = 22                   # max possible for SX1262
MIN_MISSED_PACKETS = 4    # min quantity of lost LoRa connections, to bring the LoRa power to MAX
MAX_MISSED_PACKETS = 15   # max quantity of lost LoRa connections to give up (bring the LoRa power to NOM)



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

lora_rssi = None                         # lora_rssi is initially set to None
lora_rssi_values = []                    # list holding the last N values of lora_rssi
missed_lora_packets = 0                  # counter of consecutive missed LoRa packages
initial_lora_tx_power = 0                # initial LoRa power, later get update by the config.json

###########################################################################################
# functions ###############################################################################
###########################################################################################

# Load configuration from JSON file
def load_config(fname):
    print("\nLoading the configuration ...")
    try:
        with open(fname, 'r') as f:
            config = json.load(f)
        print("Configuration loaded\n")
        return config
    except Exception as e:
        print(f"Error while loading {fname} : {e}")
        sys.exit(1)


# Function to load tags IDs
def load_tags_ids(fname):
    print("\nLoading the BLE tags mapping ...")
    try:
        with open(fname, "r") as f:
            print("BLE tags mapping loaded\n")
            return json.load(f)
    except Exception as e:
        print(f"Error while loading {fname}: {e}")
        sys.exit(1)
    

# decode my_ble_tags dictionary
def decode_my_ble_tags(my_ble_tags):
    cats_idx, cats_names, cats_macs, no_cats_text = [], [], [], ""
    
    for idx in range(len(my_ble_tags)):
        cats_idx.append(idx)
        cats_names.append(my_ble_tags[str(idx)]["name"])
        cats_macs.append(my_ble_tags[str(idx)]["mac"])
        no_cats_text+= str(idx) + "0,"
    no_cats_text = no_cats_text[:-1]    # removal of the last comma
    
    return cats_idx, cats_names, cats_macs, no_cats_text



# Initialize the backlight control (pwm)
def init_backlight(config, oled_brightness, bright_level=3):
    try:    
        # PWM init (note the max acceptable frequency is only 2445)
        oled_backlight = PWM(Pin(config['oled_display']['oled_backlight_pin']),
                             freq=2445,
                             duty_u16=oled_brightness[bright_level])
        return oled_backlight
    except Exception as e:
        print(f"Error while loading init_backlight as PWM: {e}")
        sys.exit(1)


# Initialize the Oled Display pin
def init_oled_display_rst_pin(config):
    try:
        oled_rst_pin = Pin(config['oled_display']['oled_rst_pin'], Pin.OUT, value=1)   # GPIO setting, keeps reset forset to high (value=1)
        return oled_rst_pin
    except Exception as e:
        print(f"Error while loading init_oled_display_rst_pin {e}")
        sys.exit(1)


# Initialize I2C for the OLED display
def init_i2c(config):
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
    
    global lora_tx_power
    
    print("Activating LoRa ...")
    
    lora_tx_power = int(config['lora']['tx_power'])       # TX power in dBm
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
        return lora_sx, initial_lora_tx_power
    
    except Exception as e:
        print(f"Error while activating LoRa {e}")
        sys.exit(1)



def adjust_lora_tx_power(lora_sx, initial_lora_tx_power, lora_rssi, lora_rssi_values, lora_tx_power, missed_lora_packets):
    """
    Dynamically adjust the LoRa transmission power, based on the RSSI.
    In theory, the received signal RSSI should be used to adjust the sender's
    device power; In this case, each device adjust its own trasmission power
    based on the RSSI level (despite being determined by the sender device power).
    Three main cases are considered:
    
    Case 1) No lora_rssi value, likely communication lost:
    - If this happens more than a min number of times, LoRa power is set to MAX.
      This is an attempt to establish communication again
    - If this happens more than a max number of times, LoRa power is set to nominal.
      This considers one of the device is still powered off
    
    Case 2) Weak lora_rssi value (< target - hysteresys):
    - real time rssi value is considered.
    - LoRa power increased by one level at the time.
    
    Case 3) Strong lora_rssi value (> target + hysteresys):
    - Average of last N rssi values is considered
    - LoRa power reduced by one level at the time.
    """

    #AF
#     print("adjust_lora_tx_power; lora_rssi:", lora_rssi)
#     print("missed_lora_packets:", missed_lora_packets)
    #AF
    
    if lora_rssi is None:
        missed_lora_packets += 1
        if missed_lora_packets > MIN_MISSED_PACKETS and missed_lora_packets <= MAX_MISSED_PACKETS:
            if lora_tx_power < LORA_TX_POWER_MAX:
                lora_tx_power = LORA_TX_POWER_MAX
                lora_sx.setOutputPower(lora_tx_power)
                print(f"\nIncreasing LoRa TX power to max MAX: {lora_tx_power} dBm")
        
        elif missed_lora_packets > MAX_MISSED_PACKETS:
            lora_sx.setOutputPower(initial_lora_tx_power)
            print(f"\nSet LoRa TX power to default value: {initial_lora_tx_power} dBm")
        
        if missed_lora_packets > 2:
            lora_rssi_values = []
        
        return lora_rssi_values, lora_tx_power, missed_lora_packets

    else:
        missed_lora_packets = 0
    
    
    # case of week rssi
    if lora_rssi < LORA_RSSI_TARGET - LORA_RSSI_HYSTERESIS:
        if lora_tx_power < LORA_TX_POWER_MAX:
            lora_tx_power += 1
            lora_sx.setOutputPower(lora_tx_power)
            print(f"\nIncreasing LoRa TX power to {lora_tx_power} dBm (average RSSI={lora_rssi})")
    
    # case of strong rssi, average the last N lora_rssi values is used
    else:
        lora_rssi_value_n = 5
        lora_rssi_avg = LORA_RSSI_TARGET
        lora_rssi_values.append(lora_rssi)
        if len(lora_rssi_values) > lora_rssi_value_n:
            lora_rssi_values = lora_rssi_values[-lora_rssi_value_n : ]
        
        if len(lora_rssi_values) > 0:
            lora_rssi_avg = sum(lora_rssi_values)/len(lora_rssi_values)
            
        if len(lora_rssi_values) >= lora_rssi_value_n:
            
            if lora_rssi_avg > LORA_RSSI_TARGET + LORA_RSSI_HYSTERESIS and lora_tx_power > LORA_TX_POWER_MIN:
                lora_tx_power -= 1
                lora_sx.setOutputPower(lora_tx_power)
                print(f"\nDecreasing LoRa TX power to {lora_tx_power} dBm (average RSSI={lora_rssi})")
    
    #AF
#     print(f"Last LoRa RSSi = {lora_rssi},   lora_rssi_values = {lora_rssi_values} ")
    #AF

    return lora_rssi_values, lora_tx_power, missed_lora_packets
    
    


def cb(events):
    global lora_sx, cats_status, lora_rssi

    if events & SX1262.RX_DONE:
        msg, err = lora_sx.recv()
        error = SX1262.STATUS[err]
        lora_rssi = lora_sx.getRSSI()
        msg = msg.decode("utf-8")
#         print('Received: {}, LoRa RSSI:{}, {}'.format(msg, lora_rssi, error))
        if msg == "ck":
            lora_sx.send(bytes(str(cats_status), 'utf-8'))

    elif events & SX1262.TX_DONE:
#         print(f"sent cats_status: {cats_status}\n")
        pass



# Get reset reason
def get_reset_reason():
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
    global oled_brightness, bright_level
    PWM(Pin(config['oled_display']['oled_backlight_pin']),
        freq=2445,
        duty_u16=oled_brightness[bright_level])
    
        
        
def display_backlight_off():
    global oled_backlight
    PWM(Pin(config['oled_display']['oled_backlight_pin']),
        freq=2445,
        duty_u16=65535)
    
        
    
# Print time and radar output to the display
def display_flash(display, n=2):
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
        


# Print time and radar output to the display
def display_plot_radar(display, radar_detection, time_now, show_time=1):
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
    


# Print time and radar output to the display
def display_plot_time(display, time_now):
    display_backlight_on()
    display.fill(0)
    display.text(time_now, 0, 5, 1)    
    display.show()


# Print time and radar output to the display
def display_plot_tags(display, analyzed_ble_tags, cats_names, show_time=1):
    
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
    display.fill(0)  # Clear screen
    display.text("RESET:", 0, 20)
    display.text(reset_msg, 0, 40)
    display.show()
    display_backlight_on()
    utime.sleep(1)
    display_backlight_off()


def check_time(start_time_s):
    hours, remainder = divmod(utime.time() - start_time_s, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
            

def flash_white_led(n=2, time_s=1):
    for i in range(n):
        WHITE_LED_PIN.value(1)
        utime.sleep(time_s)
        WHITE_LED_PIN.value(0)
        if i < n-1:
            utime.sleep(time_s)
    

def get_mcu_temp_str():
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
lora_sx, initial_lora_tx_power = init_lora(config)


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
    

# initial runs counter is set to MCU_TEMP_RUN_CADENCE
run = MCU_TEMP_RUN_CADENCE


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
cats_status = "d" + get_mcu_temp_str()  + no_cats_text  # 'd' means the radar is temporarily "d"isabled


# intertaining print to terminal while the idle period (radar_ignor_period_s) elapses 
dot = 0
while utime.time() < start_time_s + radar_ignor_period_s:
    if WDT_ENABLED:                   # case the WDT_ENABLED is set True
        wdt.feed()                    # Reset the watchdog timer
    print('.', end='')
    dot += 1
    if dot >= 60:
        dot = 0
        print()
    utime.sleep(1)
print("\n"*2)


cats_status = get_mcu_temp_str() + no_cats_text   # cats_status is reset to its default status
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
                cats_status = no_cats_text           # onlt no_cats is assigned to cats_status
            
            last_radar_check_ms = utime.ticks_ms()   # current time in ms is assigned to last radar check ms
            time_now = check_time(start_time_s)      # current time (string format) is assigned to time_now
            print(f"{time_now}   lora RSSI:{lora_rssi}   lora_tx_power:{lora_tx_power}   cats_status:{cats_status}")
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
                print(f"{time_now}   lora RSSI:{lora_rssi}   lora_tx_power:{lora_tx_power}   cats_status:{cats_status}")
            
            else:                                    # case at least one cat was in ble range
                for cat, data in analyzed_ble_tags.items():  # iteration over the detected ble tags
                    
                    # a string with cat idx + 1 + ble rssi is assigned
                    cats_status += str(cat) + str(data[0]) + str(data[1]) + "," 
                cats_status = cats_status[:-1]       # last comma is removed
                
                print(f"{time_now}   lora RSSI:{lora_rssi}   lora_tx_power:{lora_tx_power}   cats_status:{cats_status}")
                
                last_radar_check_ms = utime.ticks_ms() # time reference in ms of last radar check
                
                if oled_disp_enabled:                # case oled display is enabled
                    display_plot_tags(display, analyzed_ble_tags, cats_names, show_time=5) # plots some info
                    display_backlight_off()          # oled display backlight is set off
        
        # check and eventually adjust the loRa trasnmission power
        lora_rssi_values, lora_tx_power, missed_lora_packets = adjust_lora_tx_power(lora_sx,
                                                                                    initial_lora_tx_power,
                                                                                    lora_rssi,
                                                                                    lora_rssi_values,
                                                                                    lora_tx_power,
                                                                                    missed_lora_packets)
        
        # preventing adjustments based on old lora_rssi values
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

