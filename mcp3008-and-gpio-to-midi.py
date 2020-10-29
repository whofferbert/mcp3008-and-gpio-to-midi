#!/usr/bin/python3
# a script to watch some GPIO pins, as well
# as an mcp3008, and spit out midi info about it

import mido
import re
import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from uresponsivevalue.uresponsivevalue import ResponsiveValue
 
import RPi.GPIO as GPIO

import threading

# the name of how it'll show up
midiName = "The_Never_MIDI"

# these callbacks should be better
def my_callback(channel):
    # 11
    if GPIO.input(channel) == GPIO.HIGH:
        send_cc(0, 11, 0)
        #print('1 ▼ ')
    else:
        send_cc(0, 11, 127)
        #print('1 ▲ ')
 
def my_callback2(channel):
    # 12
    if GPIO.input(channel) == GPIO.HIGH:
        send_cc(0, 12, 0)
        #print('2 ▼ ')
    else:
        send_cc(0, 12, 127)
        #print('2 ▲ ')

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D12)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

class AnalogInMidi(AnalogIn):
    last = 0
    midiAssignment = 0

    # this is necessary to work with ResponsiveValue
    def readVal(self):
        return self.value

# set tolerance for something reasonable for midi
adcTolerance = 65535 / 128 * 1.25

# create an analog input channel on pin 0-3 of the ADC
# and their various properties

# bottom forward
chan0 = AnalogInMidi(mcp, MCP.P0)
chan0.midiAssignment = 13
chan0.smoothed = ResponsiveValue(chan0.readVal, max_value=65535, activity_threshold=adcTolerance)
# top forward
chan1 = AnalogInMidi(mcp, MCP.P1)
chan1.midiAssignment = 7
chan1.smoothed = ResponsiveValue(chan1.readVal, max_value=65535, activity_threshold=adcTolerance)
# bottom back
chan2 = AnalogInMidi(mcp, MCP.P2)
chan2.midiAssignment = 10
chan2.smoothed = ResponsiveValue(chan2.readVal, max_value=65535, activity_threshold=adcTolerance)
# top back
chan3 = AnalogInMidi(mcp, MCP.P3)
chan3.midiAssignment = 9
chan3.smoothed = ResponsiveValue(chan3.readVal, max_value=65535, activity_threshold=adcTolerance)

# array of knobs
knobArr = [chan0, chan1, chan2, chan3]

GPIO.setmode(GPIO.BCM)


# https://raspberrypi.stackexchange.com/questions/76667/debouncing-buttons-with-rpi-gpio-too-many-events-detected
class ButtonHandler(threading.Thread):
    def __init__(self, pin, func, edge='both', bouncetime=200):
        super().__init__(daemon=True)

        self.edge = edge
        self.func = func
        self.pin = pin
        self.bouncetime = float(bouncetime)/1000

        self.lastpinval = GPIO.input(self.pin)
        self.lock = threading.Lock()

    def __call__(self, *args):
        if not self.lock.acquire(blocking=False):
            return

        t = threading.Timer(self.bouncetime, self.read, args=args)
        t.start()

    def read(self, *args):
        pinval = GPIO.input(self.pin)

        if (
                ((pinval == 0 and self.lastpinval == 1) and
                 (self.edge in ['falling', 'both'])) or
                ((pinval == 1 and self.lastpinval == 0) and
                 (self.edge in ['rising', 'both']))
        ):
            self.func(*args)

        self.lastpinval = pinval
        self.lock.release()


# set up interrupt pins for switches
# these should be debounced
# forward button
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
tmp_cb1 = ButtonHandler(6, my_callback, edge='both', bouncetime=10)
tmp_cb1.start()
GPIO.add_event_detect(6, GPIO.BOTH, callback=tmp_cb1)

# back button
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
tmp_cb2 = ButtonHandler(13, my_callback2, edge='both', bouncetime=10)
tmp_cb2.start()
GPIO.add_event_detect(13, GPIO.BOTH, callback=tmp_cb2)


def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # Convert the left range into a 0-1 range (int)
    valueScaled = int(value - left_min) / int(left_span)

    # Convert the 0-1 range into a value in the right range.
    return int(right_min + (valueScaled * right_span))


def send_cc(channel, ccnum, val):
    msg = mido.Message('control_change', channel=channel, control=ccnum, value=val)
    output = mido.open_output(midi_output_device)
    output.send(msg)


def check_for_running_midi():
    # TODO make this better
    # so shitty, not pythonic at all
    # if only I weren't a linux sysadmin
    checkGrep = 'ps -ef | grep -Po "amidithru\s*' + midiName + '" | grep -v grep >/dev/null'
    check = os.system(checkGrep)
    #print("check val is %s" % check)
    # 0 = running
    # 256/anything else = nope
    return check


def setup_midi_backend():
    # set up backend
    mido.set_backend('mido.backends.rtmidi')

    # system command to set up the midi thru port
    if check_for_running_midi():
        runCmd = "amidithru '" + midiName + "' &"
        os.system(runCmd)
        # wait a sec for amidithru to do it's thing
        time.sleep(1)

    # regex to match on rtmidi port name convention
    # TODO is it necessary to write:  "\s+(\d+)?:\d+)"  instead?
    nameRegex = "(" + midiName + ":" + midiName + "\s+\d+:\d+)"
    matcher = re.compile(nameRegex)
    newList = list(filter(matcher.match, mido.get_output_names()))
    # all to get the name of the thing we just made
    global midi_output_device
    midi_output_device = newList[0]
    #print("Using MIDI device:", midi_output_device)


def loop():
    while True:
        for knob in knobArr:
        
            # we'll assume that the pot didn't move
            trim_pot_changed = False
        
            # update the analog pin
            knob.smoothed.update()
    
            # get smoothed value
            trim_pot = knob.smoothed.responsive_value
    
            # convert to midi range
            # convert 16bit adc0 (0-65535) trim pot read into 0-127 volume level.
            # weird misnomer. the mcp3008 is 10 bit resolution, but this library 
            # defaults to 16 bits of resolution on analog inputs
            set_midi = remap_range(trim_pot, 0, 65535, 0, 127)
        
            # how much has it changed since the last read?
            pot_adjust = abs(set_midi - knob.last)
        
            if pot_adjust > 0:
                trim_pot_changed = True
        
            if trim_pot_changed:
                #print('midival = {volume} for pot {id}' .format(volume = set_midi, id = knob.midiAssignment))
                send_cc(0, knob.midiAssignment, set_midi)
        
                # save the potentiometer reading for the next loop
                knob.last = set_midi
        
        # hang out for a bit, 10ms
        time.sleep(0.01)


def run():
    # set up the midi stuff
    setup_midi_backend()
    # do initial callbacks to set button states
    my_callback(6)
    my_callback2(13)
    # then just loop
    loop()


if __name__ == "__main__":
    run()
