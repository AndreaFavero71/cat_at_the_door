"""
Andrea Favero 20250603

'Door at the door' project

Library modified from its original form (https://github.com/ehong-tl/micropySX126X):
- removed the FSK part
"""

from _sx126x import *
from sx126x import SX126X
from time import ticks_us

_SX126X_PA_CONFIG_SX1262 = const(0x00)

class SX1262(SX126X):
    TX_DONE = SX126X_IRQ_TX_DONE
    RX_DONE = SX126X_IRQ_RX_DONE
    STATUS = ERROR

    def __init__(self, spi_bus, clk, mosi, miso, cs, irq, rst, gpio):
        super().__init__(spi_bus, clk, mosi, miso, cs, irq, rst, gpio)
        self._callbackFunction = self._dummyFunction

    def begin(self, freq=434.0, bw=125.0, sf=9, cr=7, syncWord=SX126X_SYNC_WORD_PRIVATE,
              power=14, currentLimit=60.0, preambleLength=8, implicit=False, implicitLen=0xFF,
              crcOn=True, txIq=False, rxIq=False, tcxoVoltage=1.6, useRegulatorLDO=False,
              blocking=True):
        state = super().begin(bw, sf, cr, syncWord, currentLimit, preambleLength, tcxoVoltage, useRegulatorLDO, txIq, rxIq)
        ASSERT(state)

        if not implicit:
            state = super().explicitHeader()
        else:
            state = super().implicitHeader(implicitLen)
        ASSERT(state)

        state = super().setCRC(crcOn)
        ASSERT(state)

        state = self.setFrequency(freq)
        ASSERT(state)

        state = self.setOutputPower(power)
        ASSERT(state)

        state = super().fixPaClamping()
        ASSERT(state)

        state = self.setBlockingCallback(blocking)

        return state


    def setFrequency(self, freq, calibrate=True):
        if freq < 150.0 or freq > 960.0:
            return ERR_INVALID_FREQUENCY

        state = ERR_NONE

        if calibrate:
            data = bytearray(2)
            if freq > 900.0:
                data[0] = SX126X_CAL_IMG_902_MHZ_1
                data[1] = SX126X_CAL_IMG_902_MHZ_2
            elif freq > 850.0:
                data[0] = SX126X_CAL_IMG_863_MHZ_1
                data[1] = SX126X_CAL_IMG_863_MHZ_2
            elif freq > 770.0:
                data[0] = SX126X_CAL_IMG_779_MHZ_1
                data[1] = SX126X_CAL_IMG_779_MHZ_2
            elif freq > 460.0:
                data[0] = SX126X_CAL_IMG_470_MHZ_1
                data[1] = SX126X_CAL_IMG_470_MHZ_2
            else:
                data[0] = SX126X_CAL_IMG_430_MHZ_1
                data[1] = SX126X_CAL_IMG_430_MHZ_2
            state = super().calibrateImage(data)
            ASSERT(state)

        return super().setFrequencyRaw(freq)

    def setOutputPower(self, power):
        if not ((power >= -9) and (power <= 22)):
            return ERR_INVALID_OUTPUT_POWER

        ocp = bytearray(1)
        ocp_mv = memoryview(ocp)
        state = super().readRegister(SX126X_REG_OCP_CONFIGURATION, ocp_mv, 1)
        ASSERT(state)

        state = super().setPaConfig(0x04, _SX126X_PA_CONFIG_SX1262)
        ASSERT(state)

        state = super().setTxParams(power)
        ASSERT(state)

        return super().writeRegister(SX126X_REG_OCP_CONFIGURATION, ocp, 1)

    def setTxIq(self, txIq):
        self._txIq = txIq

    def setRxIq(self, rxIq):
        self._rxIq = rxIq
        if not self.blocking:
            ASSERT(super().startReceive())

    def setPreambleDetectorLength(self, preambleDetectorLength):
        self._preambleDetectorLength = preambleDetectorLength
        if not self.blocking:
            ASSERT(super().startReceive())

    def setBlockingCallback(self, blocking, callback=None):
        self.blocking = blocking
        if not self.blocking:
            state = super().startReceive()
            ASSERT(state)
            if callback != None:
                self._callbackFunction = callback
                super().setDio1Action(self._onIRQ)
            else:
                self._callbackFunction = self._dummyFunction
                super().clearDio1Action()
            return state
        else:
            state = super().standby()
            ASSERT(state)
            self._callbackFunction = self._dummyFunction
            super().clearDio1Action()
            return state

    def recv(self, len=0, timeout_en=False, timeout_ms=0):
        if not self.blocking:
            return self._readData(len)
        else:
            return self._receive(len, timeout_en, timeout_ms)

    def send(self, data):
        if not self.blocking:
            return self._startTransmit(data)
        else:
            
            return self._transmit(data)

    def _events(self):
        return super().getIrqStatus()

    def _receive(self, len_=0, timeout_en=False, timeout_ms=0):
        state = ERR_NONE
        
        length = len_
        
        if len_ == 0:
            length = SX126X_MAX_PACKET_LENGTH

        data = bytearray(length)
        data_mv = memoryview(data)

        try:
            state = super().receive(data_mv, length, timeout_en, timeout_ms)
        except AssertionError as e:
            state = list(ERROR.keys())[list(ERROR.values()).index(str(e))]

        if state == ERR_NONE or state == ERR_CRC_MISMATCH:
            if len_ == 0:
                length = super().getPacketLength(False)
                data = data[:length]

        else:
            return b'', state

        return  bytes(data), state

    def _transmit(self, data):
        if isinstance(data, bytes) or isinstance(data, bytearray):
            pass
        else:
            return 0, ERR_INVALID_PACKET_TYPE

        state = super().transmit(data, len(data))
        return len(data), state

    def _readData(self, len_=0):
        state = ERR_NONE

        length = super().getPacketLength()

        if len_ < length and len_ != 0:
            length = len_

        data = bytearray(length)
        data_mv = memoryview(data)

        try:
            state = super().readData(data_mv, length)
        except AssertionError as e:
            state = list(ERROR.keys())[list(ERROR.values()).index(str(e))]

        ASSERT(super().startReceive())

        if state == ERR_NONE or state == ERR_CRC_MISMATCH:
            return bytes(data), state

        else:
            return b'', state

    def _startTransmit(self, data):
        if isinstance(data, bytes) or isinstance(data, bytearray):
            pass
        else:
            return 0, ERR_INVALID_PACKET_TYPE
        state = super().startTransmit(data, len(data))
        return len(data), state

    def _dummyFunction(self, *args):
        pass

    def _onIRQ(self, callback):
        events = self._events()
        if events & SX126X_IRQ_TX_DONE:
            super().startReceive()
        self._callbackFunction(events)
