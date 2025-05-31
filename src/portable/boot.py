

"""
Andrea Favero, 20250603


'Door at the door' project

This file is executed on every boot
When booting with the encoder push button pressed, the boot.py acts as setting menu.
Once the menu is exit (EXIT...), then the main.py file is executed.
"""

import machine, utime

# setup the pin used to detect "setup mode"
setup_pin = machine.Pin(4, machine.Pin.IN, machine.Pin.PULL_DOWN)

if setup_pin.value() == 0:
    # Button pressed -> Config Mode
    

    #######################################################################################################
    #######################################################################################################
    # hardware setup for buzzer, used to feedback the push button can be released
    buzzer = machine.PWM(machine.Pin(5), freq=1000, duty_u16=65535)  # Start OFF (transistor inverts logic)
    
    for i in range(4):
        buzzer.duty_u16(250)         # duty cycle is set
        utime.sleep_ms(1)            # very short sleep time, while the bybber starts buzzing
        buzzer.duty_u16(65535)       # transistor OFF to keep the buzzer volume very low
        utime.sleep_ms(130)
    
    # import to manage the settings
    from config_utils import ConfigManager
    import encoder_menu as menu
    
    
    
    #######################################################################################################
    #######################################################################################################   
    # settings manipulation via a menu
    
    CONFIG_PATH = "config.json"


    def save_config_if_changed():
        global original_menu_data
        
        if hasattr(menu, 'display'):  # case the menu (imported menu_encoder) has display attribute
            menu.display.release()    # release the EPD resources
        
        if menu.menu_data != original_menu_data:  # case the menuitems have been modified
            cfg._merge_changes(cfg._unflatten(menu.menu_data)) # convert and merge changes to the config
            cfg.save()                # saves the config.json file with changes made via the UI menu
        
        menu.exit_menu()              # exits the menu loop 
       


    def _buzzer_info():
        buzzer_info = ""
        group = "buzzer."
        group_l = len(group)
        for field, value in menu.menu_data.items():
            if group in field:
                field = field[group_l:]
                buzzer_info += f"{field} = {value}; "
        buzzer_info = buzzer_info[:-2]
        return buzzer_info
    
    
    # ------------- Selected fields will be available in the menu -------------
    core_settings = (
        'buzzer.enable',
        'buzzer.volume',
        'buzzer.repeats'
    )



    # ------------- Initial config loading and flattening -------------

    # Initialize config manager
    cfg = ConfigManager(CONFIG_PATH)   # loads the Configmanager Class
    config = cfg.load()                # config is retrieved

    # Initialize menu data
    menu.menu_data = cfg.flatten_config(filter_keys=core_settings)
    original_menu_data = dict(menu.menu_data)  # For change tracking


    # ------------- Define menu items based on core settings -------------
    get_buzzer_info    = menu.info(_buzzer_info)
    set_buzzer_enable  = menu.get_integer(low_v=0, high_v=1, increment=1, caption='ENABLE', field='buzzer.enable')
    set_buzzer_volume  = menu.get_integer(low_v=0, high_v=9, increment=1, caption='VOLUME', field='buzzer.volume')
    set_buzzer_repeats = menu.get_integer(low_v=1, high_v=50, increment=1, caption='REPEATS', field='buzzer.repeats')
    
    

    # ------------- Define menu structure -------------
    root_menu = menu.wrap_menu([
        ('INFO',    get_buzzer_info),
        ('ENABLE',  set_buzzer_enable),
        ('VOLUME',  set_buzzer_volume),
        ('REPEATS', set_buzzer_repeats),
        ('EXIT...', save_config_if_changed)
#         ('BACK...', menu.back)
        ])

    
    # ------------- Run the menu system -------------
    root_menu()       # first menu getting displayed
    menu.run_menu()   # endless loop, until the menu 'EXIT...' is selected

    print("Exiting the setting menu.")


print("\n"*2)
print("Starting main.py")
print("\n"*2)     


