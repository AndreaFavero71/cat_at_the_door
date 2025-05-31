"""
Andrea Favero, 20250603 

'Door at the door' project

Code for the menu managment, largely based on https://github.com/sgall17a/encodermenu/tree/main
main difference for this implementation is:
- usage of an epd display
- INFO menu displays the real time values
- the menu quitting function

"""


from rotary_irq_esp  import RotaryIRQ
from epd2in9_V2 import EPD, Display
from writer import Writer
import helvetica22bold

from machine import Pin
import uasyncio as asyncio
import sys, utime

# Hardware setup
button = Pin(4, Pin.IN, Pin.PULL_DOWN)
encoder = RotaryIRQ(pin_num_clk=2, pin_num_dt=3, min_val=0, max_val=100, reverse=True, range_mode=RotaryIRQ.RANGE_WRAP)

_display = None # Internal display instance

exit_requested = False # Exit flag

def init_display_manager():
    global _display
    epd = EPD(width=128, height=296, orientation=90)
    epd.init()
    epd.Clear()
    epd.width = 64
    epd.height = 296
    _display = DisplayManager(epd)
    return _display

class DisplayManager:
    def __init__(self, epd):
        self.epd = epd
        self.h = epd.height
        self.w = epd.width
        self.text1 = ""
        self.text2 = ""
        self.needs_update = False

    def release(self):
        """Releases the display resource"""
        self.epd.sleep()
        self.epd = None
    
    def request(self, text1="", text2=""):
        self.text1 = text1
        self.text2 = text2
        self.needs_update = True

    async def loop(self):
        while True:
            if self.needs_update:
                self.epd.new_buffer(self.h, self.w)
                self.epd.fb.fill(0xff)
                display_epd = Display(self.h, self.w, self.epd.buffer)

                if self.text1:
                    Writer.set_textpos(display_epd, 0, 0)
                    wri = Writer(display_epd, helvetica22bold, verbose=False)
                    wri.printstring(self.text1, invert=True)

                if self.text2:
                    Writer.set_textpos(display_epd, 30, 0)
                    wri = Writer(display_epd, helvetica22bold, verbose=False)
                    wri.printstring(self.text2, invert=True)

                self.epd.display_Partial(
                    self.epd.get_buffer(), x=0, y=0, w=self.w, h=self.h
                )
                self.needs_update = False
            await asyncio.sleep_ms(50)


class Menu:
    def __init__(self, menu):
        self.menu = menu
        self.index = 0
        self.increment = 1

    def on_scroll(self, value):
        self.index = value
        _display.request('', self.menu[value][0])

    def on_click(self):
        self.menu[self.index][1]()

    def on_current(self):
        set_encoder(self.index, 0, len(self.menu) - 1)
        _display.request('', self.menu[self.index][0])


class GetInteger:
    def __init__(self, low_v=0, high_v=100, increment=10, caption='plain', field='datafield', default=0):
        self.field = field
        self.caption = caption
        self.increment = increment
        self.low_v = low_v
        self.high_v = high_v
        self.default = default
        self.value = 0
        self.get_initial_value()

    def get_initial_value(self):
        try:
            data_v = int(menu_data.get(self.field, self.default))
        except:
            data_v = 0
        data_v = max(self.low_v, min(self.high_v, data_v))
        encoder._value = data_v
        self.value = data_v

    def on_scroll(self, val):
        self.value = val
        _display.request(self.caption, str(val * self.increment))

    def on_click(self):
        menu_data[self.field] = self.value
        back()

    def on_current(self):
        self.get_initial_value()
        set_encoder(self.value, self.low_v, self.high_v)
        _display.request(self.caption, str(self.value * self.increment))


class Wizard:
    def __init__(self, menu):
        self.menu = menu
        self.index = 0
        self.increment = 1

    def on_scroll(self, value):
        self.device.on_scroll(value)

    def on_click(self):
        global current
        self.index += 1
        if self.index > len(self.menu) - 1:
            self.device.on_click()
        else:
            self.device.on_click()
            stack.append(self)
            (self.menu[self.index][1])()
            self.device = current
            current = self
            stack.pop()

    def on_current(self):
        global current
        self.index = 0
        (self.menu[0][1])()
        self.device = current
        current = self
        stack.pop()


class Info:
    def __init__(self, message):
        self.message = message

    def on_scroll(self, val):
        pass

    def on_click(self):
        back()

    def on_current(self):
        msg = self.message() if callable(self.message) else self.message
        lines = msg.split('\n')
        text1 = lines[0] if lines else ""
        text2 = lines[1] if len(lines) > 1 else ""
        _display.request(text1, text2)


class Selection:
    def __init__(self, field, choices):
        def str2tuple(x):
            return (x, x) if isinstance(x, str) else x
        self.field = field
        self.choice = [str2tuple(x) for x in choices]
        self.set_initial_value()

    def set_initial_value(self):
        self.index = 0
        for i, a in enumerate(self.choice):
            if menu_data.get(self.field, 'zzz') == a[1]:
                self.index = i
                break

    def on_scroll(self, val):
        self.index = val
        _display.request('', self.choice[self.index][0])

    def on_click(self):
        menu_data[self.field] = self.choice[self.index][1]
        back()

    def on_current(self):
        self.set_initial_value()
        set_encoder(self.index, 0, len(self.choice) - 1)
        _display.request('', self.choice[self.index][0])



# Menu system globals
stack = []
current = None
menu_data = {}
task = None
old_v = -1
old_switch = button()


def value():
    return encoder._value

def set_encoder(value, min_value, max_value):
    encoder._value = value
    encoder.set(value=value, min_val=min_value, max_val=max_value)

def set_data(key, value):
    global menu_data
    menu_data[key] = value

def set_global_exception():
    def handle_exception(loop, context):
        sys.print_exception(context["exception"])
        sys.exit()
    loop = asyncio.get_event_loop()
    loop.set_exception_handler(handle_exception)

def mainloop():
    global exit_requested
    set_global_exception()
    while not exit_requested:
        await step()

def run_async(func):
    try:
        asyncio.run(func())
    finally:
        asyncio.new_event_loop()

def run_menu():
    async def main():
        asyncio.create_task(_display.loop())
        await mainloop()
    run_async(main)

def exit_menu():
    global exit_requested
    exit_requested = True

async def step():
    global old_v, old_switch
    enc_v = value()
    if enc_v != old_v:
        current.on_scroll(enc_v)
        old_v = enc_v
    sw_v = button()
    if sw_v != old_switch:
        if sw_v:
            current.on_click()
        old_switch = sw_v
        await asyncio.sleep_ms(250)
    await asyncio.sleep_ms(100)

def back():
    if len(stack) > 1:
        stack.pop()
        set_current(stack.pop())

def set_current(obj):
    global current
    stack.append(obj)
    current = obj
    current.on_current()

def stop():
    global task
    try:
        task.cancel()
        task = None
    except:
        pass

def make_task(func):
    global task
    task = asyncio.create_task(func())


# Utility wrappers
def wrap_object(myobject):
    def mywrap():
        global current
        set_current(myobject)
    return mywrap

def wrap_menu(mymenulist):
    return wrap_object(Menu(mymenulist))

def wizard(mymenu):
    return wrap_object(Wizard(mymenu))

def info(string):
    return wrap_object(Info(string))

def selection(field, mylist):
    return wrap_object(Selection(field, mylist))

def get_integer(low_v=0, high_v=100, increment=10, caption='plain', field='datafield', default='DEF'):
    return wrap_object(GetInteger(low_v, high_v, increment, caption, field, default))

def dummy():
    pass


init_display_manager()   # display get instantiated