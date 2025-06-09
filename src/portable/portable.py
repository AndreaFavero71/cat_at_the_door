"""
Andrea Favero, 20250608

'Door at the door' project
Code for the portable device
Rev 0.2


Hardware:
- Heltec WiFi LoRa 32 V3 board (based on ESP32-S3)
- e-ink 2in9V2 display (B&W) from Waveshare
- LiPo battery
- Encoder
- buzzer


High level Logic:
- Portable device periodically inquires the stationary device, via LoRa.
- The stationary device replies with info whether 'cats at the door' are detected.
- If cats are detected, the buzzer is activated and info are plot to the screen.
- Via the encoder push-button the buzzer can be stopped.
- 'cats at the door' reminder comes back to screen.
- 'cats at the door' reminder can be ackowledged by a long push-button pressing.


Energy:
- Portable device uses about 80 mWh, when LoRa updates every 20s.
- In between LoRa connections:
   - LoRa chip (SX1262) goes in sleep mode (settings retention).
   - MCU goes in lightsleep, if radar doesn't detect something.
- MCU goes deepsleep if timeout is reached (device forgotten 'ON').
- Inform the other device to adjust the LoRa signal power when too low or too high.


EPD Display:
- LoRa icon, if latest coomunication was established, and signal strenght.
- WiFi icon, if latest WiFI connection was established.
- Battery icon, with charge level periodically checked on the battery voltage.
- Bel icon if enabled, and related number of buzzer repeats
- Date and time.
- Cat icon when the stationary's device radar has sensed something.
- Cat's name and BLE tag RSSI level, when radar senses and BLE in range.
- Timeout icon right before deepsleep, if timeout is reached.


Config:
- Most of the settings in config.json file.
- Cats'names and related ble mac addresses in ble_tags.json
"""




###########################################################################################
# prints to the Shell to show the code is running #########################################
###########################################################################################

print(f"\n{'#' * 60}")
print(f"{'#' * 12}    Portable device started ...    {'#' * 13}")
print(f"{'#' * 60}\n")



###########################################################################################
# EPD library import and fast feedback on display #########################################
###########################################################################################

from epd2in9_V2 import EPD, Display                 # import edp driver

with open("lib/welcome_icon.bin", "rb") as f:       # opens the binary file with welcome bmp image
    welcome_icon = bytearray(f.read())              # makes a bytearray from the image

epd = EPD(width=128, height=296, orientation=90)    # valid orientations are 0, 90 and 180
epd.init_Fast()           # wake the epd up from sleep
epd.buffer = welcome_icon # fills the framebuffer with the welcome_icon
epd.get_buffer()          # rotates buffer and send it to EPD (portrait write to EPD, taking ca 60ms)
epd.display()             # plots the buffer to the display (takes ca 1.8 secs)
epd.sleep()               # prevents display damages on the long run (command takes ca 100ms)

welcome_icon = None       # release welcome_icon to free up some memory 



###########################################################################################
# Libraries import ########################################################################
###########################################################################################

from machine import Pin, ADC, PWM, RTC, Timer, deepsleep, lightsleep, WDT, SoftI2C
from machine import reset_cause, PWRON_RESET, HARD_RESET, WDT_RESET, DEEPSLEEP_RESET, SOFT_RESET

import sys, network, ntptime, json, ubluetooth, gc, ssd1306, utime, struct

from datetime_converter import DateTimeConverter    # date time conversion w/wo internet
from rotary_irq_esp import RotaryIRQ   # import for the encoder knob
from esp32 import mcu_temperature      # import for mcu temperature check
from sx1262 import SX1262              # import for LoRa communication

# import for edp display
from icons import Icons                # import custom icons
from writer import Writer              # library for text
import helvetica22bold                 # import for edp display
import helvetica26bold                 # import for edp display



###########################################################################################
# System controls #########################################################################
###########################################################################################

# setting for the Watch Dog Timer
WDT_ENABLED = True



###########################################################################################
# Constants ###############################################################################
###########################################################################################


# hardware setup for rotary Encoder
PUSH_BUTTON = Pin(4, Pin.IN, Pin.PULL_UP)  # pin for the push button of the encode


# hardware setup for buzzer
BUZZER = PWM(Pin(5), freq=1000, duty_u16=65535)  # Start OFF (transistor inverts logic)
buzzer_timer = Timer(2)                  # Virtual timer if -1


# Set a NTP server, for date time synchronization (should preferibly consider area of living)
ntptime.host = "europe.pool.ntp.org"
ntptime.timeout = 10                     # timeout in seconds


# settings for different time related controls
NTP_UPDATE_INTERVAL_S = 1800             # period (in secs) in between the RTC alignmento to NTP
BATTERY_CHECK_INTERVAL_S = 300           # period (in secs) in between battery level check


# settings file
FILE_NAME_CONFIG = "config.json"         # file storing most of the settings


# settings for BLE tags scanner
FILE_NAME_TAGS = "ble_tags.json"         # file to store the BLE tags mappings
BLE_SCAN_TIME = 10                       # seconds to scan for BLE devices
BLE_RSSI_THRESHOLD = -80                 # ignore devices with RSSI below this value


# settings for battery voltage check
ADC_CTRL = Pin(37, Pin.OUT)              # GPIO37 enables battery voltage measurement
ADC_IN = Pin(1)                          # GPIO1 reads battery voltage
adc_bat = ADC(ADC_IN)                    # adc object
adc_bat.atten(ADC.ATTN_11DB)             # 11dB attenuation (input range up to ~3.3V)
V_REF = 3.3                              # ADC reference voltage (assuming ESP32 powered at 3.3V)
DIVIDER_RATIO = 4.9                      # voltage divider ratio: (R14 + R13) / R14 = (100 + 390) / 100
VBAT_READINGS = 20                       # number of readings for averaging
CORRECTION = 1.048                       # correction of adc reading vs measured (multimeter)


# settings for dynamic adustment of the LoRa transmission power
MIN_LORA_RSSI_TARGET = -100              # min LoRa rssi signal level, to provide feedback to sender
MAX_LORA_RSSI_TARGET = -80               # max LoRa rssi signal level, to provide feedback to sender
LORA_TX_POWER_MIN = -9                   # min possible for SX1262
LORA_TX_POWER_MAX = 22                   # max possible for SX1262
MIN_MISSED_PACKETS = 4  # lower number of LoRa missed connections to bring the LoRa power to MAX
MAX_MISSED_PACKETS = 15 # upper number of LoRa missed connections to give up (bring the LoRa power to NOM)   

###########################################################################################
# settings ################################################################################
###########################################################################################

# setting for the oled display usage
oled_disp_enabled = False           # this variable gets updated from the config.json setting


# setting for the wifi usage
wifi_enabled = False                # this variable gets updated from the config.json setting


# push button settings
debounce_timer = Timer(0)           # timer-based debounce
push_button_block_timer = Timer(1)  # a second timer for the bush button debouncing
push_button_blocked = False         # flag to prevent repeated triggers
button_pressed = False              # flag to track if the encoder push button is pressed


# variables related to LoRa communication
lora_connect_status = False         # flag tracking the LoRa connection status
lora_connect_counter = 0            # counter tracking the number of succesfull LoRa connections
prev_lora_connect_counter = 0       # counter holding the number of succesfull LoRa connections at previous cycle
lora_last_connect_time_ms = 0       # (global) holds latest LoRa connection time (ms) to stationary device
lora_rssi = None                    # lora_rssi is initially set to None
missed_lora_packets = 0             # counter of consecutive missed LoRa packages
lora_tx_power = 0                   # LoRa power, later get update by the config.json
initial_lora_tx_power = 0           # initial LoRa power, later get update by the config.json
lora_rssi_sender = None             # lora_lora_rssi_sender is initially set to None
lora_sender_pwr_action = ""         # (one char) string for LoRa tx power adjustment at sender
query = ""                          # query string send buy the portable device 
toa_us = 0                          # Time-on-Air is microseconds

# variables related to the radar detection status (info via LoRa communication)
radar_detection = False             # gets updated based on responder device info


# cat detection related initial variables
cat_at_door_flag = False            # flag to track the presence of cats at the door
cat_at_door_reminder = False        # flag to track the presence of cats at the door has happened
cats_at_the_door = []               # list of cat's names detected via BLE, reset via button_pressed
cats_lbe_rssi = []                  # list of lbe rssi detected via BLE
cat_at_door_time = 0                # time used to idle after user aknowledges cat_at_the_door notification
buzzer_done = 0                     # counter of the times the buzzer has been activated


# battery voltage initial variabile
battery_voltage_list = []           # initialize list holdist the last voltage measurements
battery_voltage = 0                 # initialize variable holding the battery voltage
battery_level = 0                   # initialize variable holding the battery level
last_battery_check_time = 0         # initialize variable holding the last battery check time


# system related initial variables
wifi_connected = False              # flag to track the wifi connection status
oled_display_menu = False           # flag to track when dealing with the menu on screen


# mcu temperature of portable device is used as initial mcu temp of the stationary device
last_stat_mcu_temp = mcu_temperature()


###########################################################################################
# functions ###############################################################################
###########################################################################################

def load_config(fname):
    """Load configuration from JSON file"""
    
    try:
        with open(fname, 'r') as f:
            config = json.load(f)
        return config
    except e:
        print(f"Error while loading {fname} : {e}")
        sys.exit(1)
        


def load_tag_ids(fname):
    """Function loading tags related info, from JSON file"""
    
    try:
        with open(fname, "r") as f:
            return json.load(f)
    except e:
        print(f"Error while loading {fname} : {e}")
        sys.exit(1)



def decode_my_ble_tags(my_ble_tags):
    """
    Decodes the tags dictionary into 3 lists.
    Each cat gets an idx number, for faster LoRa communication.
    One string for 'no cats at the door' is also returned   
    """
    
    cats_idx     = []
    cats_names   = []
    cats_macs    = []
    no_cats_text = ""
    cats_number = len(my_ble_tags)
    
    for idx in range(cats_number):
        if my_ble_tags[str(idx)]["name"] != "" :
            cats_idx.append(idx)
            cats_names.append(my_ble_tags[str(idx)]["name"])
            cats_macs.append(my_ble_tags[str(idx)]["mac"])
            no_cats_text+= str(idx) + "0,"
    no_cats_text = no_cats_text[:-1]    # removal of the last comma
    
    return cats_number, cats_idx, cats_names, cats_macs, no_cats_text



def init_oled_backlight(config):
    """Initialize the backlight control pin for oled display"""
    
    oled_backlight_pin = Pin(config['oled_display']['oled_backlight_pin'], Pin.OUT, value=1)  # Start with backlight off
    return oled_backlight_pin



def get_reset_reason():
    """Checks the reason for the MCU boot, in essence what happened at the power off (WDT, DeepSleep, etc)."""
    
    reset_reason = reset_cause()          # Get reset cause
    reset_message = {
        PWRON_RESET:     "POWER-ON RESET",
        HARD_RESET:      "HARD RESET",
        WDT_RESET:       "WATCHDOG RESET",
        DEEPSLEEP_RESET: "DEEPSLEEP WAKE",
        SOFT_RESET:      "SOFT RESET"
        # machine.BROWNOUT_RESET is missing in some MicroPython versions
    }.get(reset_reason, f"???: {reset_reason}")
    
    return reset_reason, reset_message



def load_counter():
    """
    Load a counter from the RTC memory.
    This is used to properly count the timeout also in case of unwanted reboots, like WDT.
    """
    
    data = rtc.memory()          # reads the RTC memory
    if len(data) != 2:           # check is two bytes
        return None              # Memory invalid
    try:                         # tenattive approach
        val = struct.unpack(">H", data)[0]  # rom bytes to integer
        return val               # counter is returned
    except Exception:            # case of exceptions
        return None              # corrupt format



def save_rtc_counter(val):
    """Save the counter to the RTC memory"""
    
    rtc.memory(struct.pack(">H", val))



def get_wlan(config):
    """
    Connects to the WLAN.
    It attempts for 20 secs.
    It return the wlan object and a flag about connection result (succcess or failure).
    """
    
    print("Activating WLAN function ...")
    wlan_connected = False
    print_once = True
    max_time_ms = 20000
    t_ref = utime.ticks_ms()

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    while utime.ticks_ms() - t_ref < max_time_ms:
        if not wlan.isconnected():
            try:
                wlan.connect(config['wifi']['ssid'], config['wifi']['password'])
            except OSError as e:
                print(e)
                break

            while not wlan.isconnected():
                if utime.ticks_ms() - t_ref > max_time_ms:
                    print(f"Timeout of {int(max_time_ms/1000)} secs on getting WiFi connection")
                    break
        
        elif wlan.isconnected():
            wlan_connected = True
            print("Wlan is active")
            print(f"Network config: {wlan.ifconfig()}\n")
            return wlan, wlan_connected
    
    if not wlan_connected:
        print("Network connection filed")
        return None, False



def is_wifi_connected():
    "Check if WLAN is connected"
    
    wlan = network.WLAN(network.STA_IF)
    return wlan.isconnected()



def sync_time(utc_shift, dst_shift):
    "Synchronize date and time via WLAN and NTP"
    
    print("Updating RTC from NTP ...")
    try:
        ntptime.settime()                   # Get UTC time
        local_time = utime.localtime()      # Convert to structured time
        yearday = local_time[7]             # Get day of the year

        # Determine the correct time shift
        if 85 <= yearday < 300:             # Roughly summer (DST)
            hour_offset = utc_shift + dst_shift  # hour shifter considers utc and dst
        else:                               # Roughly winter (standard time)
            hour_offset = utc_shift         # hour shifter considers utc

        # Apply time correction
        local_time = list(utime.localtime(utime.mktime(utime.localtime()) + hour_offset * 3600))
        print("RTC updated via NTP")
        return True, local_time[:6]         # Return (year, month, day, hours, minutes, seconds)

    except Exception as e:
        print(f"Error on updating RTC from NTP: {e}")
        return False, ()



def get_date_time(converter, ntp_reached, date_time, utc_shift, dst_shift):
    """
    In between NPT connections, it converts from epoch to date and time individual fields.
    Right after an NTP connection, it separates the date and time fields from a list.
    """
    
    if ntp_reached:                        # Case the NTP was reachable
        year, month, day, hours, minutes, seconds = date_time
        print(f"{day:02d}/{month:02d}/{str(year)[2:]}  {hours:02d}:{minutes:02d}\n")
    else:                                  # Case the NTP was not reachable
        now = utime.time()                 # Epoch time from RTC of the board
        # time is split via the converter.unix_to_datetime (no libraries, ca 2ms)
        year, month, day, hours, minutes, seconds = converter.unix_to_datetime(now, utc_shift, dst_shift)
    return year, month, day, hours, minutes, seconds



def update_time(now, last_ntp_time_update, NTP_UPDATE_INTERVAL_S, first_time = False):
    "Updates the time, and handles exceptions"
    
    ntp_reached = False
    
    if wifi_enabled and is_wifi_connected() and (first_time or now > last_ntp_time_update + NTP_UPDATE_INTERVAL_S):
        ntp_reached, date_time = sync_time(utc_shift, dst_shift)  # Get the time from NTP (corrected by uts nad dst shifts)
    
    if ntp_reached:
        year, month, day, hours, minutes, seconds = get_date_time(converter, True, date_time, utc_shift, dst_shift)
        now = utime.time()            # Epoch time is assigned to now
        last_ntp_time_update = now    # now is assigned to last_ntp_time_update  
    else:
        year, month, day, hours, minutes, seconds = get_date_time(converter, False, '', utc_shift, dst_shift)
    
    date_time = f"{day:02d}/{month:02d}/{str(year)[2:]}  {hours:02d}:{minutes:02d}:{seconds:02d}"
    return last_ntp_time_update, date_time



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

        lora_sx.setBlockingCallback(False, lora_callback)
        print("LoRa is active\n")
        return lora_sx, lora_tx_power, initial_lora_tx_power
    
    except Exception as e:
        print(f"Error while activating LoRa{e}")
        sys.exit(1)



def validate_lora_interval_s(duty_cycle_percent, lora_interval_s, cat_num):
    """
    Validate the lora_interval_s against the duty_cycle and LoRa settings.
    duty_cycle_percent must be in percentage: 1 for 1%, 0.1 for 0.1%.
    The ToA is regulated per single appliance: The stationary device uses
    longer strings than the portable.
    Calculation based on the portable device.
    """
    
    print("Validating the lora_interval_s settings against the duty_cycle_percent and LoRa settings ...")
        
    # message from the portable device ("ck" or "ck>" makes no difference)
    msg1 = "ck>"               # this string is independant from number of cats

    # possible messages from the stationary device (example refer to two cats)
    msg2 = "xx" * cat_num + "," * (cat_num - 1)            # '00,10'
    msg3 = msg2 + ">"                                      # '00,10>'
    msg4 = "r" + msg2                                      # 'r00,10'
    msg5 = "r" + msg3                                      # 'r00,10>'
    msg6 = "tyy," + msg2                                   # 't45,00,10'
    msg7 = "tyy," + msg3                                   # 't45,00,10>'
    msg8 = "r" + "cc-kkk" * cat_num + "," * (cat_num - 1)  # 'r01-100,11-100'
    msg9 = msg8 + ">"                                      # 'r01-100,11-100>'

    messages = (msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9)
    
    # weight is an estmation of how often the msg strings are likely to happen
    # for instance, msg1 (the query) happens 100%
    weight = (1, 0.35, 0.35, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05)
    
    toa_stationary_sum_us = 0
    for i, msg in enumerate(messages):
        if i == 0:                                       # portable device 
            toa_msg1_us = lora_sx.getTimeOnAir(len(msg)) # toa is calculated via the lora library
        else:                                            # stationary devices
            w = weight[i]                                # weighting factor
            toa_us = lora_sx.getTimeOnAir(len(msg))      # ToA is calculated via the lora library
            toa_stationary_sum_us += + toa_us * w                   # weighted ToA 
    
    avg_toa_ms = toa_stationary_sum_us/1000
    toa_ms_allowed_hourly = int(3600*1000*duty_cycle_percent/100)
    max_lora_connections_per_hour = toa_ms_allowed_hourly/avg_toa_ms
    
    min_lora_interval_s = 60                             # arbitrary value, to initialize the variable
    if max_lora_connections_per_hour > 0:                # preventing division by zero
        min_lora_interval_s = 3600/max_lora_connections_per_hour
    
    if int(min_lora_interval_s) == min_lora_interval_s:
        min_lora_interval_s = int(min_lora_interval_s)
    else:
        min_lora_interval_s = 1 + int(min_lora_interval_s)

    if min_lora_interval_s <= lora_interval_s:
        print(f"Setting min_lora_interval_s = {lora_interval_s}, in config.json file,"
              f"warrants the {duty_cycle_percent}% duty cycle")
        print(f"Minimum time interval allowed at 'lora_interval_s': {min_lora_interval_s} s")
        return lora_interval_s
    else:
        print(f"\nAverage ToA per LoRa transmission: {int(round(avg_toa_ms,0))} ms")
        print(f"Max ToA allowed per hour due to max duty-cycle: {toa_ms_allowed_hourly} ms")
        print(f"Max number of LoRa connection per hour: {int(round(max_lora_connections_per_hour,0))}")
        print(f"Minimum time interval allowed at 'lora_interval_s': {min_lora_interval_s} s")
        print(f"\nSetting min_lora_interval_s = {lora_interval_s}, in config.json file, must be increase to"
              f"at least {min_lora_interval_s} to warrent the {duty_cycle_percent}% duty cycle")
        print(f"CAT AT THE DOOR system proceeds by using min_lora_interval_s = {min_lora_interval_s}")
        return min_lora_interval_s



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
    
    lora_sender_pwr_action = ""
    
    # based on the LoRa rssi on this receiver device, assign at action (one char) for the LoRa sender
    if lora_rssi is not None:
        missed_lora_packets = 0
        if lora_rssi < MIN_LORA_RSSI_TARGET:
            lora_sender_pwr_action = "<"                               
        elif lora_rssi > MAX_LORA_RSSI_TARGET:
            lora_sender_pwr_action = ">"
    
    # case the lora rssi of this device is none, the occurrence is checked before reacting
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
        # case of week LoRa rssiat sender, and receiver not yet to its MAX power
        if lora_rssi_sender == 0 and lora_tx_power < LORA_TX_POWER_MAX:
            lora_tx_power += 1
            lora_sx.setOutputPower(lora_tx_power)
            print(f"\nIncreasing LoRa TX power to {lora_tx_power} dBm")
        
        # case of strong LoRa rssi at sender, and receiver not yet to its MIN power
        elif lora_rssi_sender == 1 and lora_tx_power > LORA_TX_POWER_MIN:
            lora_tx_power -= 1
            lora_sx.setOutputPower(lora_tx_power)
            print(f"\nDecreasing LoRa TX power to {lora_tx_power} dBm")

    return lora_tx_power, missed_lora_packets, lora_sender_pwr_action



def lora_callback(events):
    """Function called by the LoRa interrupt."""
    
    global lora_rssi, lora_connect_counter, lora_last_connect_time_ms, cats_status
    
    if events & SX1262.RX_DONE:
        lora_last_connect_time_ms = utime.ticks_ms()
        msg, err = lora_sx.recv()
        error = SX1262.STATUS[err]
        lora_rssi = int(lora_sx.getRSSI())
        if error == "ERR_NONE":
            cats_status = msg.decode("utf-8")
            lora_connect_counter += 1
            print(f"query: {query} ; ToA:{toa_us//1000}ms ; LoRa power:{lora_tx_power} ; LoRa RSSI:{lora_rssi} ; cats_status: {cats_status}")
        else:
            print("LoRa conection error")

    elif events & SX1262.TX_DONE:
#         print("TX done\n")
        pass



def read_battery_voltage(adc_avg=0, bat_voltage=0):
    """Monitor the battery voltage"""
    try:
        ADC_CTRL.value(1)              # Enable battery voltage measurement
        utime.sleep_ms(20)             # Stabilization delay
        adc_avg = adc_bat.read()       # First ADC reading
        utime.sleep_ms(5)              # Short sleep time

        for _ in range(VBAT_READINGS): # Iterating VBAT_READINGS times
            adc_avg += adc_bat.read()  # Adds raw ADC value (0-4095) for VBAT_READINGS times
            utime.sleep_ms(5)          # Short sleep time

        adc_avg /= (VBAT_READINGS + 1) # average the VBAT_READINGS readings
        bat_voltage = CORRECTION * (adc_avg / 4095) * V_REF * DIVIDER_RATIO # convertion to batt voltage and correction
        ADC_CTRL.value(0)              # Disable measurement to save power
        return bat_voltage             # returns the measured battery voltag

    except Exception as e:
        print(f"Error reading battery voltage: {e}")
        return None



def get_battery_percentage(voltage):
    """
    Returns the battery percentage corresponding to the closest voltage level.
    Args:
        voltage (float): The measured battery voltage.
    Returns:
        int: The battery percentage that best matches the voltage.
    """
    voltage_levels = [4.1, 3.93, 3.82, 3.75, 3.7, 3.65, 3.6]
    percent_levels = [100, 80,   60,   40,   20,  10,   0]

    # Find the index of the voltage level closest to the measured voltage
    closest_index = min(range(len(voltage_levels)), key=lambda i: abs(voltage - voltage_levels[i]))
    return percent_levels[closest_index]



def check_battery(now, battery_voltage, battery_voltage_list, battery_level, last_battery_check_time,
                  BATTERY_CHECK_INTERVAL_S, first_time = False):

    if first_time or now > last_battery_check_time + BATTERY_CHECK_INTERVAL_S:
        battery_voltage = round(read_battery_voltage(),2) # battery voltage is measured   
        battery_voltage_list.append(battery_voltage)
        
        if len(battery_voltage_list) > 1:
            battery_voltage = sum(battery_voltage_list) / len(battery_voltage_list)
        elif len(battery_voltage_list) > 5:
            battery_voltage_list = battery_voltage_list[-5:]
            
        battery_level = get_battery_percentage(battery_voltage)
        
        print(f"\nBattery voltage = {battery_voltage:.2f}V,  Battery level = {battery_level}%\n")
        last_battery_check_time = utime.time()
    
    return battery_voltage, battery_voltage_list, battery_level, last_battery_check_time



def check_encoder(last_encoder_value, oled_display_menu):
    encoder_value = rotary.value()
    if encoder_value != last_encoder_value:
        print("Rotary Encoder Value:", encoder_value)
        last_encoder_value = encoder_value
        oled_display_menu = True
    return encoder_value, last_encoder_value, oled_display_menu



def unblock_push_button(timer):
    global push_button_blocked
    push_button_blocked = False  # Allow new triggers after 200ms



def debounced_handler(timer):
    global push_button_blocked, button_pressed
    if PUSH_BUTTON.value() == 0 and not push_button_blocked:  # Ensure button is still pressed & not push_button_blocked
        button_pressed = True
        print("Button Pressed!")
        push_button_blocked = True  # Block further presses
        push_button_block_timer.init(mode=Timer.ONE_SHOT, period=200, callback=unblock_push_button)  # unblock_push_button after 200ms



def btn_pressed(pin):
    if not push_button_blocked:  # Ignore if push_button_blocked
        debounce_timer.init(mode=Timer.ONE_SHOT, period=50, callback=debounced_handler)  # Debounce



# # Initialize I2C for the OLED display
def get_i2c(config):
    print("Activating I2C function ...")
    from machine import SoftI2C
    try:
        oled_sda_pin = Pin(config['oled_display']['oled_sda_pin'], Pin.OPEN_DRAIN, pull=Pin.PULL_UP) # Set I2C pins in open_drain mode
        oled_scl_pin = Pin(config['oled_display']['oled_scl_pin'], Pin.OPEN_DRAIN, pull=Pin.PULL_UP) # Set I2C pins in open_drain mode
        oled_rst_pin = Pin(config['oled_display']['oled_rst_pin'], Pin.OUT, value=1) # Initialize the OLED reset pin
        i2c = SoftI2C(sda=oled_sda_pin, scl=oled_scl_pin)
        print("I2C is active\n")
        return i2c
    except Exception as e:
        print(f"Error while activating I2C {e}")
        sys.exit(1)



# Choose between I2C and BLE
def get_either_epd_ble_wlan(choice, config,  ubluetooth, network):
    return



def analyze_cats_status(cats_status):
    """
    cat_status, for 2 cats, looks like: t44,r01-60,11-50<
    t44 = Stationary device mcu temperature 44 degC
    r = radar has detected something (very likely a cat)
    01-60 = 0:cat0, 1:at the door, -60: ble rssi
    11-50 = 1:cat1, 1:at the door, -50: ble rssi
    Ending character '<' or '>' only when necessary, otherwise absent
    < means the receiver gets a weak rssi signal
    > means the receiver gets a strong rssi signal
    """

    at_the_door = []                            # empty list to store cats name when at the door
    lbe_rssi = []                               # empty list to store lbe rssi when at the door
    detected = cats_status.replace(" ", "")     # cats_status is assigned to local variable detected
    lora_rssi_sender = None                     # rssi level at receiver (0=weak, 1=ok, 2=strong)
    stat_mcu_temp = ""                          # empty string is assign to local stat_mcu_temp variable
    
    if detected[-1] == '<':                     # case detected ends with 'r'
        lora_rssi_sender = 0                    # rssi level of the receiver set to 0 (=weak)
        detected = detected[-1]                 # last character is removed from detected

    elif detected[-1] == '>':                   # case detected ends with '>'
        lora_rssi_sender = 1                    # rssi level of the receiver set to 1 (=strong)
        detected = detected[-1]                 # last character is removed from detected
        
    if detected[0] == 'r':                      # case detected starts with 'r'
        detected = detected[1:]                 # initial 'r' is removed to detected
    
    if detected[0] == 't':                      # case sliced detected starts with 't'
        comma_pos = detected.find(",")          # position of first comma in string
        stat_mcu_temp = detected[1 : comma_pos] # string with the stationary mcu temperature
        detected = detected[1 + comma_pos :]    # detected is liced by removing stat mcu temp
    
    if detected.count(',') > 0:                 # at least 2 cats
        cats_number = detected.count(',') + 1   # number of cats
    else:                                       # one or zero cats
        if len(detected) >= 2:                  # at least 2 characters is one cat
            cats_number = 1                     # 1 cat
        else:                                   # less than 2 characters
            cats_number = 0                     # no cats
            
    
    for idx in range(cats_number):              # iteration for the number of cats
        if ',' in detected:                     # case there is a comma in detected
            split = detected.find(',')          # chars index of the split (comma)
        else:                                   # case there is not a comma in detected
            split = len(detected) + 1           # split is one unit more than string lenght
        
        cat = detected[:split]                  # one cat is taken from the string

        if cat[1] == '1':                       # this cat is at the door
            cat_name = cats_names[int(cat[0])]  # name of the cat at the door
            at_the_door.append(cat_name)        # cat name added to at_the_door
            lbe_rssi.append(str(cat[2:]))       # lbe_rssi of that tag is assigned

        detected = detected[split+1:]           # remaining cats in the string
        
        if idx == cats_number -2:               # case of last cat in string
            idx += 1                            # index is updated (not needed a new iteration)
            if detected[1] == '1':              # this cat is at the door
                cat_name = cats_names[int(detected[0])]  # name of the cat at the door
                at_the_door.append(cat_name)    # cat name added to at_the_door
                lbe_rssi.append(detected[2:])   # lbe_rssi of that tag is assigned
            
            # the return break the iteration
            return at_the_door, lbe_rssi, stat_mcu_temp, lora_rssi_sender
        
    # return the empty lists when no cats at the door
    return at_the_door, lbe_rssi, stat_mcu_temp, lora_rssi_sender 



def stop_beep(buzzer_timer=None):
    """Stop buzzer and cleanup timer. Pass timer object as arg to avoid global."""
    
    BUZZER.duty_u16(65535)  # Transistor OFF
    if buzzer_timer:
        buzzer_timer.deinit()

def start_beep(duration_ms=200, freq=1000, volume=5):
    """Start a beep with auto-stop. Adaptive volume on frequency."""
    
    stop_beep(buzzer_timer) # Stop any previous beep
    scaled_volume = max(1, min(100,int(freq * 0.023 + volume * 3.05 + 2 )))  # dynamic volume scaling
    duty = max(250, min(65535, int(65535 - (scaled_volume * 655.35))))       # reversing PWM (transistor), and ceiling it
    freq = max(20, min (4200, freq))                                         # ceiling frequency
#     print("freq:", freq, "  volume:", scaled_volume, "   duty:", duty)
    
    BUZZER.freq(freq)      # Apply settings
    BUZZER.duty_u16(duty)  # Apply settings
    buzzer_timer.init(mode=Timer.ONE_SHOT,
                      period=duration_ms,  # Auto-stop after duration_ms
                      callback=lambda t: stop_beep(buzzer_timer))


def activate_buzzer():
    """Activate the buzzer with a few short beeps."""
    for i in range(4):
        period = 150 + i*10
        start_beep(duration_ms=50+i*10, freq=700, volume=buzzer_volume)
        utime.sleep_ms(period)
    
    
    
def clean_mem():
    """Function to clean the memory"""
    
    print("gc.mem_alloc():", gc.mem_alloc())
    print("gc.mem_free():" , gc.mem_free())
    gc.collect()
    utime.sleep(5)
    print("gc.mem_alloc():", gc.mem_alloc())
    print("gc.mem_free():" , gc.mem_free())
    print()



def get_display_epd(epd):
    """Function preparing a Display oject based on Framebuffer."""
    
    epd.init_Fast()     # wake the epd up from sleep
    epd.new_buffer()    # makes a new HLSB buffer (protrait display orientation ...)
    epd.fb.fill(0xff)   # generates the white background (epd works inverted)
    
    # Display framebuffer is needed for the Writer
    return Display(epd.height, epd.width, epd.buffer)  # note it uses VLSB




def header_epd(epd, display_epd, lora_connect_status, wifi_connected, battery_level, date_time,
               last_stat_mcu_temp, lora_iterations, second_row=True):
    """
    Plots the one or two headers rows on the EPD display.
    The plotted infor are informative about the system.
    """
    
    # plot info related to the LoRa connection
    if lora_connect_status:                         # case of LoRa connection success
        display_epd.blit(Icons.lora_icon_fb, 0, 0)  # plots LoRa icon to the VLSB frame
            
    # plot info related to the wifi connection
    if wifi_connected:                              # case the wifi is connected
        # plot the wifi symbol on the first row
        display_epd.blit(Icons.wifi_icon_fb, 68, 0) # plots WiFi icon to the VLSB frame
    
    # plot info related to the buzzer
    if buzzer_enabled:                               # case the buzzer is enabled
        # plot the bell icon on the first row
        display_epd.blit(Icons.bell_enabled_fb, 154, 0) # plots WiFi icon to the VLSB frame
        
        # plot the volume set in the config
        Writer.set_textpos(display_epd, 2, 180)
        wri = Writer(display_epd, helvetica22bold, verbose=False)
        wri.printstring(str(buzzer_volume), invert=True)
    
    elif not buzzer_enabled:                          # case the buzzer is disabled
        # plot the crossed bell icon
        display_epd.blit(Icons.bell_disabled_fb, 162, 0)

        
    # plots the battery icon on the first row
    display_epd.blit(Icons.battery_icon[battery_level], 108, 4)
    
    
    # plot the time on the first row
    Writer.set_textpos(display_epd, 2, 206)     # writer coordinates first y then x
    wri = Writer(display_epd, helvetica22bold, verbose=False)
    wri.printstring(date_time[10:], invert=True)
    
    
    # plots when the second_row is set True (no radar or cats detections)
    if second_row:                                 # case the second_row is set True
        
        if lora_connect_status:                    # case of LoRa connection success
            Writer.set_textpos(display_epd, 29, 5) # writer coordinates first y then x
            wri = Writer(display_epd, helvetica22bold, verbose=False)
            wri.printstring(str(lora_rssi), invert=True)
        
        if wifi_connected:                         # case the wifi is connected
            # plot the date on the second row
            Writer.set_textpos(display_epd, 29, 206)  # writer coordinates first y then x
            wri = Writer(display_epd, helvetica22bold, verbose=False)
            wri.printstring(date_time[:8], invert=True)
        
        if buzzer_enabled:                         # case the second_row is set True
            # plot the buzzer repeats (R in front) set in the config
            txt = f"R {buzzer_repeats}"
            txt_len = len(txt)
            x = 150 if txt_len>=4 else 158
            Writer.set_textpos(display_epd, 29, x)
            wri = Writer(display_epd, helvetica22bold, verbose=False)
            wri.printstring(txt, invert=True)
        
        # plot the mcu temperature (alternates Stationary and Portable devices' mcu)
        if lora_iterations % 2 == 0:               # even LoRa iterations
            if last_stat_mcu_temp != "":           # case last MCU temp Stationary device is known
                temp_txt = f"S: {last_stat_mcu_temp}C" # MCU temp of the Stationary device
            else:                                  # case last MCU temp Stationary device is unknown
                temp_txt = f"P: {mcu_temperature()}C"  # MCU temp of the Portable device
        else:                                      # odd LoRa iterations
            temp_txt = f"P: {mcu_temperature()}C"  # MCU temp of the Portable device
        Writer.set_textpos(display_epd, 29, 70)    # writer coordinates first y then x
        wri = Writer(display_epd, helvetica22bold, verbose=False)
        wri.printstring(temp_txt, invert=True)     # plot the string
        
        # plot the LoRa connections counter
        connections_txt = f"SCAN {lora_iterations}"  # string with the lora_iterations
        if not lora_connect_status:                # case the (last) LoRa connection fails
            connections_txt += "(NO LoRa)"         # a note is added to the text message
        
        #AF
        connections_txt += " dB " + str(lora_tx_power)
        #AF
        
        Writer.set_textpos(display_epd, 105, 0)    # writer coordinates first y then x
        wri = Writer(display_epd, helvetica22bold, verbose=False)
        wri.printstring(connections_txt, invert=True)
        
    
    # drawing separation lines after icons and text (prevent lines to be overwritten)
    
    # horizontal lines separating the first "row" of the display from the rest
    epd.hline(0, 25, 297, 0x00)           # plot the horizontal line
    
    if second_row:                        # case the second_row is set True
        # horizontal lines separating the second "row" of the display from the rest
        epd.hline(0, 50, 297, 0x00)       # plot the horizontal line
        
    v_len = 50 if second_row else 25      # vertical line lenght
    
    # vertical lines separating the fields
    for x in (65, 103, 144, 202):         # x coordinate for the vertical lines
        if x == 103:                      # case x of the 2nd vertical line
            epd.vline(x, 0, 25, 0x00)     # plot the vertical line
        else:                             # not the sencond vertical line
            epd.vline(x, 0, v_len, 0x00)  # plot the vertical line



    
def show_info_epd(epd, wifi_connected, battery_level, date_time, last_stat_mcu_temp,
                  radar_detection, cats_at_the_door, cat_at_door_reminder,
                  cat_at_door_time, cats_lbe_rssi, lora_iterations):
    
    """
    Recall the one or two headers rows on the EPD display.
    Plots the main info, like the cats names when detected.
    """
    
    # plots the basic info (header), like the connections icons, battery, buzzer, date and time, etc
    display_epd = get_display_epd(epd)
    
    # set plt_2nd_row as True on every iteration, and get later set False if other info must be plotted
    plt_2nd_row = True
    
    # number of cats
    cats_number = len(cats_at_the_door)                  # number of detected cats (via ble tags)
    
    # plot a cat icon when the radar detects something and ninfo not yet aknowledged
    if radar_detection:  # case the radar detects (likely) a cats
        display_epd.blit(Icons.cat_icon_fb, 0, 32)       # plots cat icon to the VLSB frame (it takes ca 60ms)
        plt_2nd_row = False                              # the second header row is not plotted
    
    # updates cats names and BLE signal strenght
    if cats_number > 0:                                  # case there is at least one cat
        plt_2nd_row = False                              # the second header row is not plotted
        
        wri = Writer(display_epd, helvetica26bold, verbose=False) # writer coordinates first y then x               
        
        # variables to center the cats namea on the display
        font_size = 26                                   # font size
        min_y = 28                                       # min y position to preserve the header
        max_y = 128 - font_size                          # max y position for the text to prevent overflow
        
        # in case of (too) many cats, the text fill partially overlap
        if cats_number * font_size > font_size + max_y - min_y and cats_number > 1:
            font_size = int((font_size + max_y - min_y) / cats_number)
        
        # y gap in between the text rows, to equally distribute it in y direction
        y_gap = ((font_size + max_y - min_y) - cats_number * font_size) / cats_number
        
        row = 0                                          # set the initial row reference for the first 
        for idx, cat in enumerate(cats_at_the_door):     # iterates over the cats_at_the_door list of names
            y = int(min_y + y_gap/2 + row * (font_size + y_gap)) # y coordinate is calculated
            y = max(min_y, min(max_y, y))                # y coordinated is clamped between min and max
            
            Writer.set_textpos(display_epd, y, 85)       # defines the text position
            wri.printstring(cat, invert=True)            # print the cat name
            
            if len(cats_lbe_rssi) == cats_number:        # check cats_lbe_rssi coherence with cats_number
                Writer.set_textpos(display_epd, y, 240)  # defines the text position
                wri.printstring(str(cats_lbe_rssi[idx]), invert=True)  # print the cats_lbe_rssi
            row += 1                                     # increase the row reference
    
    
    # reminder remaining on screen after aknowledging the cat presente
    # case the cats presence has been aknowledged
    if cat_at_door_reminder and utime.time() > cat_at_door_time + radar_min_signal_time_s:
        Writer.set_textpos(display_epd, 70, 10) 
        wri = Writer(display_epd, helvetica22bold, verbose=False)
        wri.printstring("REMEMBER THE CAT(S) ?", invert=True)
    
    # plot the first header row, eventually also the second one, with various info
    header_epd(epd, display_epd, lora_connect_status, wifi_connected, battery_level,
               date_time, last_stat_mcu_temp, lora_iterations, second_row = plt_2nd_row)
    
    epd.get_buffer()    # rotates the buffer to send it to the display (portrait wrinting, resulting (takes ca 60ms)
    epd.display()       # plots the buffer to the display (takes ca 1.8 secs)
    epd.sleep()         # prevents display damages on the long run (command takes ca 100ms)



def show_ack_epd(epd, wifi_connected, battery_level, date_time,
                 last_stat_mcu_temp, lora_iterations, show_time=5):
    """
    Plots an AKNOWLEDGEMENT feedback on the EPD display.
    """
    # plots the basic info (header), like the connections icons, battery, date nd time, etc
    display_epd = get_display_epd(epd)
    header_epd(epd, display_epd, lora_connect_status, wifi_connected,
               battery_level, date_time, last_stat_mcu_temp, lora_iterations)

    wri = Writer(display_epd, helvetica26bold, verbose=False) # writer coordinates first y then x
    Writer.set_textpos(display_epd, 68, 40)                   # defines the text position
    wri.printstring("AKNOWLEDGED", invert=True)               # print the ack
    
    epd.get_buffer()    # rotates the buffer to send it to the display (portrait wrinting, resulting (takes ca 60ms)
    epd.display()       # plots the buffer to the display (takes ca 1.8 secs)
    epd.sleep()         # prevents display damahges on the long run (command takes ca 100ms)
    
    if show_time > 0:
        utime.sleep(show_time)  # sleep time to let the message visible
        clear_epd               # epd display is cleared




def show_deepsleep_epd():
    """
    Plots a sleeping icon on the EPD display.
    """
    
    # plots deepsleep image to the EPD before before entering in deepsleep
    with open("lib/deepsleep_icon.bin", "rb") as f:       # opens the binary file with welcome bmp image
        deepsleep_icon = bytearray(f.read())              # makes a bytearray from the image

    epd = EPD(width=128, height=296, orientation=0)       # valid is 0 for this image
    epd.init()                  # wake the epd up from sleep
    epd.buffer = deepsleep_icon # fills the framebuffer with the welcome_icon
    epd.display()               # plots the buffer to the display (takes ca 1.8 secs)
    epd.sleep()                 # prevents display damages on the long run (command takes ca 100ms)
    utime.sleep(5)              # sleep time ensuring the display gets updated
    
    
    
    
    
def clear_epd(epd, wifi_connected, battery_level, date_time):
    """Clears the EPD display."""
    
    # plots the basic info (header), like the connections icons, battery, date nd time, etc
    display_epd = get_display_epd(epd)
    header_epd(epd, display_epd, lora_connect_status, wifi_connected, battery_level, date_time)
    
    epd.get_buffer()    # rotates the buffer to send it to the display (portrait wrinting, resulting (takes ca 60ms)
    epd.display()       # plots the buffer to the display (takes ca 1.8 secs)
    epd.sleep()         # prevents display damahges on the long run (command takes ca 100ms)




# Function to display text on the OLED screen
def oled_display_text(oled_display, oled_backlight_pin, text_lines, keep_active=False, disp_time_ms=1000):
    """
    Display multiple lines of text on the OLED screen.
    
    :param oled_display:       The OLED display object.
    :param oled_backlight_pin: The backlight control pin.
    :param text_lines:         A list of tuples containing (text, x, y, color).
    :param keep_active:        If True, the oled_display will remain active without sleeping.
    :param disp_time_ms:       Time in milliseconds to display the text (only used if keep_active is False).
    """

#     oled_display_clean(oled_display)             # Clear the oled_display
    oled_display.fill(0)                      # clear the screen (all pixels to black)
    for text, x, y, color in text_lines:
        oled_display.text(text, x, y, color)
    oled_display.show()
    oled_backlight_pin.value(0)               # Drive low to enable backlight
    
    if not keep_active:                       # Only sleep if keep_active is False
        utime.sleep_ms(disp_time_ms)
        oled_display.fill(0)                  # clear the screen (all pixels to black)
        oled_display.show()
        oled_backlight_pin.value(1)           # Drive high to disable backlight



def goto_deepsleep():
    """Stops the periferies, calls the sleep icon for the EPD dispaly and call the MCU deepsleep."""
    
    print("\nPreparing for MCU deepsleep")
    
    if wifi_enabled:
        try:
            wlan.active(False)
            print("Turned the wifi off")
        except:
            pass
    
    try:
        lora_sx.sleep(retainConfig=False)
        print("Set the LoRa chip in sleep mode")
    except:
        pass
    
    try:
        lora_sx.spi.deinit()
        print("Set the SPI bus off")
    except:
        pass
    
    print("\nMCU deepsleep ...\n\n")
    utime.sleep(0.5)         # short delay
    
    show_deepsleep_epd()     # epd display plots a sleeping feedback
    deepsleep()              # device goes into deepsleep




################################################################################################
################################################################################################
################################################################################################
#                                                                                              #
#                                       main program                                           #
#                                                                                              #
################################################################################################
################################################################################################
################################################################################################



# prints the last reset reason
reset_reason, reset_msg = get_reset_reason()
print(f"\nLast reset reason: {reset_msg}\n")


# preserving memory when power OFF
# this is used for timeout monitoring (ignores evental WDT reboots if code freezes)
rtc = RTC()                              # instantiation of RTC


# load the lora_iterations counter from the RTC memory
lora_iterations = load_counter() # counter holding the total number of lora connections (also unsuccessful)              


# Reset lora_iterations counter if power-on reset or if RTC memory is invalid
if reset_reason == PWRON_RESET or lora_iterations is None:
    lora_iterations = 0
    print("Starting new lora_iterations counter from 0")
else:
    print(f"Proceeding lora_iterations counter from {lora_iterations}")

# set WDT related variables
if WDT_ENABLED:                          # case the WDT_ENABLED is set True
    wdt_time_ms = 60000                  # wdt is initially set to 60 secs
    wdt = WDT(timeout = wdt_time_ms)     # watchdog to wake the ESP32 if got freezing


# Load configuration
config = load_config(FILE_NAME_CONFIG)



# retrive settings from the config dictionaty

wifi_enabled =      bool(config["wifi"]["enable"])       # wifi gets eventually enabled
oled_disp_enabled = bool(config["oled_display"]["oled_enabled"]) # oled display gets eventually enabled
sytem_timeout_m =        config["system"]["timeout_m"]   # timeout, in minutes, for the portable device deepsleep
lora_interval_s =        config["system"]["lora_interval_s"] # cadence, in secs, to connect with the stationary device
utc_shift =              config["time"]["timezone"]      # Adjust UTC time by timezone and DST (Day Saving Time: 1=yes, 0=no)
dst_shift =              config["time"]["dst"]           # dst_shift = utc_shift + 1 if config["time"]["dst"] else utc_shift

duty_cycle_percent =     config["lora"]["duty_cycle_percent"]  # allowed duty-cycle in percentage (1 for 1% ; 0.1 for 0.1%)

radar_min_signal_time_s = int(config["scanners"]["radar_keep_signal_period_s"])  # period (s) idling after acknowledge 'cats at the door'

buzzer_enabled = bool(config["buzzer"]["enable"])  # enable/disable the buzzer function
buzzer_volume =  int(config["buzzer"]["volume"])   # buzzer volume
if buzzer_volume > 9:                              # case buzzer_volume is > than 9 (likely entered as range 0~100)
    buzzer_volume = buzzer_volume * 9 // 100       # buzzer volume is scaled down to range 0~9
buzzer_repeats = int(config["buzzer"]["repeats"])  # max number of times the buzzer should be activated


# cadence for LoRa connection to the stationary device is calculated in milliseconds
lora_conn_interval_ms = int(1000 * lora_interval_s)   # period (in ms) in between devices LoRa communication

# the max number of LoRa iterations is calculated 
max_lora_iterations = int(sytem_timeout_m * 60 / lora_interval_s) # max number of iterations (lora connections to stationary dev)

# after the initial settings the wdt is linked to the lora_conn_interval_ms
if WDT_ENABLED:                     # case the WDT_ENABLED is set True
    wdt_time_ms = int(2.5 * lora_conn_interval_ms)  # watchdog time (in ms) 
    wdt = WDT(timeout = wdt_time_ms)    # watchdog to wake the ESP32 if got freezing


# Load ble_tags data
my_ble_tags = load_tag_ids(FILE_NAME_TAGS)
cats_number, cats_idx, cats_names, cats_macs, no_cats_text = decode_my_ble_tags(my_ble_tags)
cats_status = no_cats_text
# print("my_ble_tags:", my_ble_tags)
# print(f"\nText for no-cats detection: {no_cats_text}")
# print(f"(info: first digit is cat_idx, second digit means absence), r means radar detected something\n")


# Initialize the SX1262 LoRa module
lora_sx, lora_tx_power, initial_lora_tx_power = init_lora(config)


# validates the lora_interval_s setting against the duty_cycle_percent and LoRa settings
lora_interval_s = validate_lora_interval_s(duty_cycle_percent, lora_interval_s, cats_number)


# datetime conversion from EPOCH to date time
converter = DateTimeConverter()


# push button interrupt
PUSH_BUTTON.irq(trigger=Pin.IRQ_FALLING, handler=btn_pressed)


# rotary encoder initial variabile
rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=3, min_val=0, max_val=10, reverse=True, range_mode=RotaryIRQ.RANGE_WRAP)
last_encoder_value = rotary.value() # read and print rotary value


# Setting up the oled display, if enabled
if not oled_disp_enabled:
    print("\nOled display is disabled")

elif oled_disp_enabled:
    print("\nOled display is enabled")
    i2c = get_i2c(config)                             # Initialize I2C
    oled_display = ssd1306.SSD1306_I2C(128, 64, i2c)  # Initialize the OLED display
    oled_backlight_pin = init_oled_backlight(config)  # Initialize backlight


# Setting up the Wi-Fi, if enabled
if not wifi_enabled:
    print("\nWi-Fi is disabled")

elif wifi_enabled:
    print("\nWi-Fi is enabled")
    try:
        wlan, wifi_connected = get_wlan(config) # Connect to WiFi
    except Exception as e:
        print(f"Error while setting the WLAN : {e}")
        print("\nWi-Fi gets disabled")
        wifi_enabled = False
        
        


# update the date and time (when WiFi connection it uses NTP server)
now = utime.time()
last_ntp_time_update = 0
last_ntp_time_update, date_time = update_time(now, last_ntp_time_update, NTP_UPDATE_INTERVAL_S, first_time=True)


# first check of the battery voltage and related level 
battery_voltage, battery_voltage_list, battery_level, last_battery_check_time = check_battery(now,
                                                                                              battery_voltage,
                                                                                              battery_voltage_list,
                                                                                              battery_level,
                                                                                              last_battery_check_time,
                                                                                              BATTERY_CHECK_INTERVAL_S)

print(f"\n\n{'#' * 60}")
print(f"{'#' * 11}    Portable device is active ...    {'#' * 12}")
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

test = 0

while True:
    
    # feed the wdt in case it is enabled
    if WDT_ENABLED:              # case the WDT_ENABLED is set True
        wdt.feed()               # reset the watchdog timer
    
    
    # current time reference
    now = utime.time()           # current epoch time in seconds
    now_ms = utime.ticks_ms()    # current time in milliseconds (not referenced to the real word time)
    
    
    # check the battery voltage and level (no action when BATTERY_CHECK_INTERVAL_S is not elapsed)
    battery_voltage, battery_voltage_list, battery_level, last_battery_check_time = check_battery(now,
                                                                                                  battery_voltage,
                                                                                                  battery_voltage_list,
                                                                                                  battery_level,
                                                                                                  last_battery_check_time,
                                                                                                  BATTERY_CHECK_INTERVAL_S)


    # check the current time and synchronize with npt (no action when NTP_UPDATE_INTERVAL_S is not elapsed)
    last_ntp_time_update, date_time = update_time(now, last_ntp_time_update, NTP_UPDATE_INTERVAL_S)
    
    
    # button status check
    # be noted the pressing time must be longer than the lighsleep time when cat_at_door_reminder == True
    if button_pressed:                         # case the button is pressed
        button_pressed = False                 # reset the button_pressed status value
        
        
        # if cat_at_door_flag, then it acts as presence aknowledgement
        if cat_at_door_flag:                   
            cat_at_door_flag = False           # reset the cat_at_door_flag status
            cats_at_the_door = []              # reset tle list of the cats names
            cats_status = no_cats_text         # reset the cats_status
            buzzer_done = buzzer_repeats       # prevents further buzzing
            print("Aknowledged the cat_at_door presence")  # feedback is printed to the terminal
            show_ack_epd(epd, wifi_connected, battery_level,
                         date_time, last_stat_mcu_temp, lora_iterations) # plot info to the epd display
        
        
        # if cat_at_door_reminder and appearing on screen with some delay, then it acts as reminder aknowledgement
        if cat_at_door_reminder and utime.time() > cat_at_door_time + radar_min_signal_time_s:
            cat_at_door_reminder = False       # reset the cat_at_door_reminder 
            buzzer_done = 0                    # reset the buzzer counter
            print("Aknowledged the cat_at_door reminder")  # feedback is printed to the terminal
#             show_ack_epd(epd, wifi_connected, battery_level,
#                          date_time, last_stat_mcu_temp, lora_iterations) # plot info to the epd display

    
    # checking if it is time connect with the stationary device
    if now_ms >= lora_last_connect_time_ms + lora_conn_interval_ms:  # case it is time to connect to the stationary device
        
        # counter tracking the device ON time, based on the lora connections
        lora_iterations += 1
        
        # saves the lora_iterations counter in the RTC memory (
        save_rtc_counter(lora_iterations)
        
        # lora_connect_status is set False at every run.
        lora_connect_status = False 
        
        # portable device (this one) sends 'ck' to the stationary device (ck means check)
        # and adds a character indicating the eventual need for LoRa power adjustement
        query = "ck" + lora_sender_pwr_action
        lora_sx.send(bytes(query, "utf-8"))
        toa_us = lora_sx.getTimeOnAir(len(query))

        # looping while waiting to the LoRa response (max 3 secs)
        t_ref = utime.ticks_ms()
        while utime.ticks_ms() - t_ref < 3000:
            if lora_connect_counter > prev_lora_connect_counter:
                prev_lora_connect_counter = lora_connect_counter
                lora_sx.sleep(retainConfig=True)  # set the LoRa chip to sleep
                lora_connect_status = True
                break
            else:
                utime.sleep_ms(50)
        
        if not lora_connect_status:           # case the lora connection failed
            lora_sx.sleep(retainConfig=True)  # set the LoRa chip to sleep (retaining config)
        
        # check if the LoRa connection failed
        if not lora_connect_status: # case no connections with the stationary device
            # lora_last_connect_time_ms is a global variable updated by the LoRa function 'cb' or forced right here
            lora_last_connect_time_ms = now_ms
            print("LoRa connection failure: Check if the stationary device is powered on")
        
        # debug purpose: force radar and cat detection for debug purpose:
#         cats_status = "r01-90"
        
        # radar detection status based on loRa info
        radar_detection = True if cats_status[0] == 'r' else False # r means radar
        
        # reset cat_at_door_flag (i.e. when the stationary dev is booted)
        if cats_status[0] == 'd':                   # 'd' means disabled
            cat_at_door_flag = False                # flag to track the presence of cats at the door is set False
            cats_status = cats_status[1:]           # initial 'd' is removed from the cats_status string
        
        # analyze the cats status (LoRa info)
        at_the_door, cats_lbe_rssi, stat_mcu_temp, lora_rssi_sender = analyze_cats_status(cats_status)
        
        # case no mcu_temp update from the stationary device
        if stat_mcu_temp != "":
            last_stat_mcu_temp = stat_mcu_temp

        # reset variable right after cat presence aknoledgement
        if utime.time() - cat_at_door_time <  radar_min_signal_time_s:  # case the cats detection has been aknowledged
            cat_at_door_flag = False                       # reset the cat_at_door_flag status
            cats_at_the_door = []                          # reset tle list of the cats names
            cats_status = no_cats_text                     # reset the cats_status
        
        # check whether the current cats situation at the door differs from status before
        if len(at_the_door) > 0:                           # case there is at least one cat at the door
            if len(at_the_door) > len(cats_at_the_door):   # case more cats at the door than before
                cats_at_the_door = at_the_door             # local cats_at_the_door list is updated
                cat_at_door_flag = True                    # flag for cats at the door (this flag gets reset)
                cat_at_door_reminder = True                # flag for cats at the door (this flag never gets reset)
                cat_at_door_time = utime.time()            # time when cats are detected

            elif len(at_the_door) == len(cats_at_the_door): # case same cats at the door as before                                         
                for cat_name in at_the_door:               # iterates over the current cats names at the door
                    if cat_name not in cats_at_the_door:   # case different cats names at the door
                        cats_at_the_door = at_the_door     # local cats_at_the_door list is updated
                        cat_at_door_flag = True            # flag for cats at the door (this flag gets reset)
                        cat_at_door_reminder = True        # flag for cats at the door (this flag never gets reset)
                        cat_at_door_time = utime.time()    # time when cats are detected
                        buzzer_done = 0                    # reset the buzzer counter
        
        # check if conditions to activate the buzzer
        if buzzer_enabled and cat_at_door_flag and buzzer_done < buzzer_repeats:
            activate_buzzer()
            buzzer_done += 1
            print("buzzer_done:", buzzer_done, "   buzzer_repeats:", buzzer_repeats)
        
        # update the epd display
        show_info_epd(epd, wifi_connected, battery_level, date_time, last_stat_mcu_temp,
                      radar_detection, cats_at_the_door, cat_at_door_reminder,
                      cat_at_door_time, cats_lbe_rssi, lora_iterations)
        
        # check and eventually adjust the loRa trasnmission power
        lora_tx_power, missed_lora_packets, lora_sender_pwr_action = adjust_lora_tx_power(lora_rssi_sender,
                                                                                          lora_tx_power,
                                                                                          missed_lora_packets)
        
        # preventing LoRa power adjustments based on old lora_rssi values
        lora_rssi = None    # set lora_rssi to None
        
        # measure the cycle time (in ms)
        cycle_time = utime.ticks_ms() - now_ms
        
        # MCU lightsleep while waiting for the next loRa connections (reduce battery consumption)
        if not cat_at_door_flag:
            lightsleep(lora_conn_interval_ms - cycle_time)
        
        # checks if devise is active longer than timeout; If so go to deepsleep
        if lora_iterations >= max_lora_iterations:   # case the lora iterations equals/exceed the max
            goto_deepsleep()                         # calls the goto_deepsleep function
        

        
    else:                 # case it is not time to communicate with the stationary device
        utime.sleep(0.05) # sleep for some little time
        
# end of code

