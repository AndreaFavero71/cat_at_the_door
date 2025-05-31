"""
Andrea Favero, 20250603

'Door at the door' project

Micropython driver for e-ink display Waveshare EPD2in9_V2

Most of this code is derived from the python Waveshare e-Paper library, available at:
https://github.com/waveshareteam/e-Paper/blob/master/RaspberryPi_JetsonNano/python/lib/waveshare_epd/epd2in9_V2.py

Licensed under the MIT License
Copyright (c) Waveshare
"""

import framebuf, utime

class Display(framebuf.FrameBuffer):
    """Class to later enable the usage of Writer, a FrameBuffer based text editor."""
    def __init__(self, width, height, buffer):
        self.width = width
        self.height = height
        self.buffer = buffer
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)


class EPD:
    def __init__(self, width=128, height=296, orientation=90):
        """
        Class for the waveshare epaper 2.9 inches; This code is largerly derived from:
        https://github.com/waveshareteam/e-Paper/blob/master/RaspberryPi_JetsonNano/python/lib/waveshare_epd/epd2in9_V2.py
        Some additions are made to use the display in landscape mode
        """
        from machine import Pin, SoftSPI
        
        
        # Display buffer
        self.width = width
        self.height = height
        self.orientation = orientation
        
        # Initialize GPIO (using your pin configuration)
        self.busy_pin = Pin(39, Pin.IN, Pin.PULL_UP)
        self.reset_pin = Pin(40, Pin.OUT)
        self.dc_pin = Pin(41, Pin.OUT)
        self.cs_pin = Pin(42, Pin.OUT)
        
        # Initialize SoftSPI
        self.spi = SoftSPI(
            baudrate=800_000,   # SoftSPI limit is 800_000
            polarity=0,
            phase=0,
            sck=Pin(45),
            mosi=Pin(46),
            miso=Pin(6)
        )
#         print("Using SoftSPI: {}".format(self.spi))
        
        self.new_buffer(h=self.height, w=self.width)

    WF_PARTIAL_2IN9 =   bytearray(b"\x00\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00"
                                  b"\x00\x00\x00\x00\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0a\x00\x00\x00"
                                  b"\x00\x00\x01\x01\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
                                  b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
                                  b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x22\x22\x22\x22\x22\x22\x00\x00\x00\x22\x17\x41\xb0\x32\x36")
    
    WS_20_30 =          bytearray(b"\x80\x66\x00\x00\x00\x00\x00\x00\x40\x00\x00\x00\x10\x66\x00\x00\x00\x00\x00\x00\x20\x00\x00\x00\x80\x66\x00\x00\x00\x00\x00\x00"
                                  b"\x40\x00\x00\x00\x10\x66\x00\x00\x00\x00\x00\x00\x20\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x08\x00\x00"
                                  b"\x00\x00\x02\x0a\x0a\x00\x0a\x0a\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
                                  b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x08\x00\x01\x00\x00\x01\x00\x00\x00\x00\x00"
                                  b"\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x44\x44\x44\x44\x44\x44\x00\x00\x00\x22\x17\x41\x00\x32\x36")

    WF_FULL =           bytearray(b"\x90\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x60\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x90\x00\x00\x00\x00\x00\x00\x00"
                                  b"\x00\x00\x00\x00\x60\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x19\x19\x00\x00"
                                  b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
                                  b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
                                  b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x24\x42\x22\x22\x23\x32\x00\x00\x00\x22\x17\x41\xae\x32\x38")
    
    
    def digital_write(self, pin, value):
        pin.value(value)


    def digital_read(self, pin):
        return pin.value()


    def reset(self):
        self.reset_pin.value(1)
        utime.sleep_ms(50)
        self.reset_pin.value(0)
        utime.sleep_ms(2)
        self.reset_pin.value(1)
        utime.sleep_ms(50)


    def send_command(self, command):
        self.dc_pin.value(0)
        self.cs_pin.value(0)
        self.spi.write(bytearray([command]))
        self.cs_pin.value(1)


    def send_data(self, data):
        self.dc_pin.value(1)
        self.cs_pin.value(0)
        if isinstance(data, int):
            self.spi.write(bytearray([data]))
        else:
            self.spi.write(bytearray(data))
        self.cs_pin.value(1)
    

    def ReadBusy(self):
        
        while(self.digital_read(self.busy_pin) == 1):  #  0: idle, 1: busy
            utime.sleep_ms(5) 
        utime.sleep_ms(10)


    def TurnOnDisplay(self):
        self.send_command(0x22) # DISPLAY_UPDATE_CONTROL_2
        self.send_data(0xc7)
        self.send_command(0x20) # MASTER_ACTIVATION
        self.ReadBusy()


    def TurnOnDisplay_Partial(self):
        self.send_command(0x22) # DISPLAY_UPDATE_CONTROL_2
        self.send_data(0x0F)
        self.send_command(0x20) # MASTER_ACTIVATION
        self.ReadBusy()


    def wait_until_idle(self, timeout=5000):
        """Wait for BUSY pin to go LOW (0=ready) with timeout"""
        start = utime.ticks_ms()
        while self.busy_pin.value() == 1:  # 1=busy
            if utime.ticks_diff(utime.ticks_ms(), start) > timeout:
                raise RuntimeError("Display busy timeout")
            utime.sleep_ms(10)


    def init(self):
        # EPD hardware init start     
        self.reset()

        self.ReadBusy()
        self.send_command(0x12)  #SWRESET
        self.ReadBusy() 

        self.send_command(0x01) #Driver output control      
        self.send_data(0x27)
        self.send_data(0x01)
        self.send_data(0x00)
    
        self.send_command(0x11) #data entry mode       
        self.send_data(0x03)

        self.SetWindow(0, 0, self.width-1, self.height-1)

        self.send_command(0x21) #  Display update control
        self.send_data(0x00)
        self.send_data(0x80)
    
        self.SetCursor(0, 0)
        self.ReadBusy()

        self.SetLut(self.WS_20_30)
        # EPD hardware init end
        return 0


    def init_Fast(self):
        # EPD hardware init start     
        self.reset()

        self.ReadBusy()
        self.send_command(0x12)  #SWRESET
        self.ReadBusy() 

        self.send_command(0x01) #Driver output control      
        self.send_data(0x27)
        self.send_data(0x01)
        self.send_data(0x00)
    
        self.send_command(0x11) #data entry mode       
        self.send_data(0x03)

        self.SetWindow(0, 0, self.width-1, self.height-1)

        self.send_command(0x3C) #BorderWavefrom
        self.send_data(0x05)

        self.send_command(0x21) #Display update control
        self.send_data(0x00)
        self.send_data(0x80)
    
        self.SetCursor(0, 0)
        self.ReadBusy()

        self.SetLut(self.WF_FULL)
        # EPD hardware init end
        return 0


    def lut(self, lut):
        self.send_command(0x32)
        for i in range(0, 153):
            self.send_data(lut[i])
        self.ReadBusy()


    def SetLut(self, lut):
        self.lut(lut)
        self.send_command(0x3f)
        self.send_data(lut[153])
        self.send_command(0x03)   # gate voltage
        self.send_data(lut[154])
        self.send_command(0x04)   # source voltage
        self.send_data(lut[155])  # VSH
        self.send_data(lut[156])  # VSH2
        self.send_data(lut[157])  # VSL
        self.send_command(0x2c)   # VCOM
        self.send_data(lut[158])


    def SetWindow(self, x_start, y_start, x_end, y_end):
        self.send_command(0x44) # SET_RAM_X_ADDRESS_START_END_POSITION
        # x point must be the multiple of 8 or the last 3 bits will be ignored
        self.send_data((x_start>>3) & 0xFF)
        self.send_data((x_end>>3) & 0xFF)
        self.send_command(0x45) # SET_RAM_Y_ADDRESS_START_END_POSITION
        self.send_data(y_start & 0xFF)
        self.send_data((y_start >> 8) & 0xFF)
        self.send_data(y_end & 0xFF)
        self.send_data((y_end >> 8) & 0xFF)


    def SetCursor(self, x, y):
        self.send_command(0x4E) # SET_RAM_X_ADDRESS_COUNTER
        # x point must be the multiple of 8 or the last 3 bits will be ignored
        self.send_data(x & 0xFF)
        
        self.send_command(0x4F) # SET_RAM_Y_ADDRESS_COUNTER
        self.send_data(y & 0xFF)
        self.send_data((y >> 8) & 0xFF)


    def display(self, buffer = None):
        if buffer == None:
            buffer = self.buffer

        self.send_command(0x24) # WRITE_RAM
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(buffer[i + j * int(self.width / 8)])
        self.TurnOnDisplay()
    

    def display_Base(self, buffer = None):
        if buffer == None:
            buffer = self.buffer   
    
        self.send_command(0x24) # WRITE_RAM
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(buffer[i + j * int(self.width / 8)])
                
        self.send_command(0x26) # WRITE_RAM
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(buffer[i + j * int(self.width / 8)])  
                
        self.TurnOnDisplay()

    
    def display_Partial(self, buffer, x=0, y=0, w=None, h=None):
        if buffer is None:
            return
        
        # Defaults: full width and height if not specified
        if w is None:
            w = self.width
        if h is None:
            h = self.height

        self.reset_pin.value(0)
        utime.sleep_ms(2)
        self.reset_pin.value(1)
        utime.sleep_ms(2)
        
        self.SetLut(self.WF_PARTIAL_2IN9)
        
        self.send_command(0x37)
        self.send_data(0x00)
        self.send_data(0x00)
        self.send_data(0x00)
        self.send_data(0x00)
        self.send_data(0x00)
        self.send_data(0x40)
        self.send_data(0x00)
        self.send_data(0x00)
        self.send_data(0x00)  
        self.send_data(0x00)

        self.send_command(0x3C) #BorderWavefrom
        self.send_data(0x80)

        self.send_command(0x22) 
        self.send_data(0xC0)
        self.send_command(0x20)
        self.ReadBusy()

        # Set window and cursor at correct position
        self.SetWindow(x, y, x + w - 1, y + h - 1)
        self.SetCursor(x, y)
        
        self.send_command(0x24) # WRITE_RAM

        # send only the rectangle (partial buffer)
        for j in range(0, h):
            for i in range(0, w // 8):
                self.send_data(buffer[i + j * (w // 8)])  
        
        self.TurnOnDisplay_Partial()
    

    
    def Clear(self, color=0xff):        
        self.send_command(0x24) # WRITE_RAM
        self.send_data([color] * self.height * int(self.width / 8))
        self.TurnOnDisplay()
        self.send_command(0x26) # WRITE_RAM
        self.send_data([color] * self.height * int(self.width / 8))
        self.TurnOnDisplay()



    def sleep(self):
        self.send_command(0x10)  # DEEP_SLEEP
        self.send_data(0x01)
        utime.sleep_ms(100)


    # Drawing methods
    def text(self, text, x, y, color=0):
        self.fb.text(text, x, y, color)


    def hline(self, x, y, l, color):
        self.fb.hline(x, y, l, color)


    def vline(self, x, y, l, color):
        self.fb.vline(x, y, l, color)


    def line(self, x0, y0, x1, y1, color):
        self.fb.line(x0, y0, x1, y1, color)


    def rect(self, x, y, w, h, color, fill=False):
        if fill:
            self.fb.fill_rect(x, y, w, h, color)
        else:
            self.fb.rect(x, y, w, h, color)


    def new_buffer(self, h=None, w=None):
        """Generates a new buffer and a new framebuffer obj."""
        if h is None:
            h = self.height
        else:
            if h % 8 != 0:
                raise ValueError("The h value must be a multiple of 8.")
            self.height = h
        
        if w is None:
            w = self.width
        else:
            if w % 8 != 0:
                raise ValueError("The w value must be a multiple of 8.")
            self.width = w
            
        self.buffer = bytearray(h * w // 8)
        self.fb = self._set_oriented_fb()

    

    def _rotate90(self):
        """
        Rotates a MONO_VLSB framebuffer by 90 degrees clockwise.
        Returns a new bytearray ready to be plot on a portrait native epaper.
        """
        w = self.width
        h = self.height
        v_buffer = self.buffer
        h_buffer = bytearray(h * w // 8)
        x=0; y=-1; n=0; R=0
        for i in range(0, w//8):
            for j in range(0, h):
                R = (n-x)+(n-y)*((w//8)-1)
                pixel = v_buffer[n]
                h_buffer[R] = pixel
                n +=1
            x = n+i+1
            y = n-1
        return h_buffer


    def _rotate180(self):
        """
        Rotate a MONO_HMSB buffer 180 degrees.
        Returns a new bytearray.
        """
        buffer = self.buffer
        rotated = bytearray(self.width * self.height // 8)
        for i in range(len(buffer)):
            rotated[-1 - i] = buffer[i]
        return rotated
    
    
    def _set_oriented_fb(self, h=None, w=None):
        """
        Returns a framebuffer oriented as per self.orientation.
        This involves different framebuf modes.
        When self.orientation differs from 0, it must rotated 'back' before 'display' it.
        """
        h = h or self.height
        w = w or self.width
        
        if self.orientation not in [0, 90, 180]:
            raise ValueError("Rotation must be one of: 0, 90, 180")
        if self.orientation == 0:
            return framebuf.FrameBuffer(self.buffer, w, h, framebuf.MONO_HLSB)
        elif self.orientation == 90:
            return framebuf.FrameBuffer(self.buffer, h, w, framebuf.MONO_VLSB)
        elif self.orientation == 180:
            return framebuf.FrameBuffer(self.buffer, w, h, framebuf.MONO_HMSB)


    def get_buffer(self):
        """ ."""
        if self.orientation not in [0, 90, 180]:
            raise ValueError("Rotation must be one of: 0, 90, 180")
        if self.orientation == 0:
            return self.buffer
        elif self.orientation == 90:
            self.buffer = self._rotate90()
            return self.buffer
        elif self.orientation == 180:
            self.buffer = self._rotate180()
            return self.buffer

