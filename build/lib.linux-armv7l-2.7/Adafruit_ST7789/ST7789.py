# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import numbers
import time
import numpy as np

from PIL import Image
from PIL import ImageDraw

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.SPI as SPI


# Constants for interacting with display registers.

# START - ST7789 Specific constants
ST7789_WIDTH      = 240
ST7789_HEIGHT     = 240

# START - copy functions from Adafruit_SD77xx.h
ST_CMD_DELAY      = 0x80    #// special signifier for command lists
ST77XX_NOP        = 0x00
ST77XX_SWRESET    = 0x01
ST77XX_RDDID      = 0x04
ST77XX_RDDST      = 0x09

ST77XX_SLPIN      = 0x10
ST77XX_SLPOUT     = 0x11        
ST77XX_PTLON      = 0x12
ST77XX_NORON      = 0x13

ST77XX_INVOFF     = 0x20
ST77XX_INVON      = 0x21
ST77XX_DISPOFF    = 0x28
ST77XX_DISPON     = 0x29
ST77XX_CASET      = 0x2A
ST77XX_RASET      = 0x2B
ST77XX_RAMWR      = 0x2C
ST77XX_RAMRD      = 0x2E

ST77XX_PTLAR      = 0x30
ST77XX_TEOFF      = 0x34
ST77XX_TEON       = 0x35
ST77XX_MADCTL     = 0x36
ST77XX_COLMOD     = 0x3A

ST77XX_MADCTL_MY  = 0x80
ST77XX_MADCTL_MX  = 0x40
ST77XX_MADCTL_MV  = 0x20
ST77XX_MADCTL_ML  = 0x10
ST77XX_MADCTL_RGB = 0x00

ST77XX_RDID1      = 0xDA
ST77XX_RDID2      = 0xDB
ST77XX_RDID3      = 0xDC
ST77XX_RDID4      = 0xDD

#// Some ready-made 16-bit ('565') color settings:
ST77XX_BLACK      = 0x0000
ST77XX_WHITE      = 0xFFFF
ST77XX_RED        = 0xF800
ST77XX_GREEN      = 0x07E0
ST77XX_BLUE       = 0x001F
ST77XX_CYAN       = 0x07FF
ST77XX_MAGENTA    = 0xF81F
ST77XX_YELLOW     = 0xFFE0
ST77XX_ORANGE     = 0xFC00
# END from REF_Adafruit_ST77xx.h

def color565(r, g, b):
    """Convert red, green, blue components to a 16-bit 565 RGB value. Components
    should be values 0 to 255.
    """
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

def image_to_data(image):
    """Generator function to convert a PIL image to 16-bit 565 RGB bytes."""
    #NumPy is much faster at doing this. NumPy code provided by:
    #Keith (https://www.blogger.com/profile/02555547344016007163)
    pb = np.array(image.convert('RGB')).astype('uint16')
    color = ((pb[:,:,0] & 0xF8) << 8) | ((pb[:,:,1] & 0xFC) << 3) | (pb[:,:,2] >> 3)
    return np.dstack(((color >> 8) & 0xFF, color & 0xFF)).flatten().tolist()

class ST7789(object):
    """Representation of an ST7789 TFT LCD."""

    def __init__(self, dc, spi, rst=None, gpio=None, width=ST7789_WIDTH,
        height=ST7789_HEIGHT):
        """Create an instance of the display using SPI communication.  Must
        provide the GPIO pin number for the D/C pin and the SPI driver.  Can
        optionally provide the GPIO pin number for the reset pin as the rst
        parameter.
        """
        self._dc = dc
        self._rst = rst
        self._spi = spi
        self._gpio = gpio
        self.width = width
        self.height = height
        if self._gpio is None:
            self._gpio = GPIO.get_platform_gpio()
        # Set DC as output.
        self._gpio.setup(dc, GPIO.OUT)
        # Setup reset as output (if provided).
        if rst is not None:
            self._gpio.setup(rst, GPIO.OUT)
        # Set SPI to mode 0, MSB first.
        spi.set_mode(0)
        spi.set_bit_order(SPI.MSBFIRST)
        spi.set_clock_hz(16000000)
        # Create an image buffer.
        self.buffer = Image.new('RGB', (width, height))

    def send(self, data, is_data=True, chunk_size=4096):
        """Write a byte or array of bytes to the display. Is_data parameter
        controls if byte should be interpreted as display data (True) or command
        data (False).  Chunk_size is an optional size of bytes to write in a
        single SPI transaction, with a default of 4096.
        """
        # Set DC low for command, high for data.
        self._gpio.output(self._dc, is_data)
        # Convert scalar argument to list so either can be passed as parameter.
        if isinstance(data, numbers.Number):
            data = [data & 0xFF]
        # Write data a chunk at a time.
        for start in range(0, len(data), chunk_size):
            end = min(start+chunk_size, len(data))
            self._spi.write(data[start:end])

    def command(self, data):
        """Write a byte or array of bytes to the display as command data."""
        self.send(data, False)

    def data(self, data):
        """Write a byte or array of bytes to the display as display data."""
        self.send(data, True)

    def reset(self):
        """Reset the display, if reset pin is connected."""
        if self._rst is not None:
            self._gpio.set_high(self._rst)
            time.sleep(0.005)
            self._gpio.set_low(self._rst)
            time.sleep(0.02)
            self._gpio.set_high(self._rst)
            time.sleep(0.150)

    def _init(self):
        # Initialize the display.  Broken out as a separate function so it can
        # be overridden by other displays in the future.
        
        # from Adafruit_SD77XX
        self.command(ST77XX_SWRESET)
        time.sleep(0.150)
        self.command(ST77XX_SLPOUT)
        time.sleep(0.01)
        self.command(ST77XX_COLMOD)
        self.data(0x55)
        time.sleep(0.01)
        self.command(ST77XX_MADCTL)
        self.data(0x08)
        self.command(ST77XX_CASET)
        self.data(0x00)
        self.data(0)
        self.data(0)
        self.data(240)
        self.command(ST77XX_RASET)
        self.data(0x00)
        self.data(0)
        self.data(240>>8)
        self.data(240&0xFF)
        self.command(ST77XX_INVON)
        time.sleep(0.01)
        self.command(ST77XX_NORON)
        time.sleep(0.01)
        self.command(ST77XX_DISPON)
        time.sleep(0.01)

    def begin(self):
        """Initialize the display.  Should be called once before other calls that
        interact with the display are called.
        """
        self.reset()
        self._init()

    def set_window(self, x0=0, y0=0, x1=None, y1=None):
        """Set the pixel address window for proceeding drawing commands. x0 and
        x1 should define the minimum and maximum x pixel bounds.  y0 and y1
        should define the minimum and maximum y pixel bound.  If no parameters
        are specified the default will be to update the entire display from 0,0
        to 239,319.
        """
        if x1 is None:
            x1 = self.width-1
        if y1 is None:
            y1 = self.height-1
        self.command(ST77XX_CASET)        # Column addr set
        self.data(x0 >> 8)
        self.data(x0)                    # XSTART
        self.data(x1 >> 8)
        self.data(x1)                    # XEND
        self.command(ST77XX_RASET)        # Row addr set
        self.data(y0 >> 8)
        self.data(y0)                    # YSTART
        self.data(y1 >> 8)
        self.data(y1)                    # YEND
        self.command(ST77XX_RAMWR)        # write to RAM

    def display(self, image=None):
        """Write the display buffer or provided image to the hardware.  If no
        image parameter is provided the display buffer will be written to the
        hardware.  If an image is provided, it should be RGB format and the
        same dimensions as the display hardware.
        """
        # By default write the internal buffer to the display.
        if image is None:
            image = self.buffer
        # Set address bounds to entire display.
        self.set_window()
        # Convert image to array of 16bit 565 RGB data bytes.
        # Unfortunate that this copy has to occur, but the SPI byte writing
        # function needs to take an array of bytes and PIL doesn't natively
        # store images in 16-bit 565 RGB format.
        pixelbytes = list(image_to_data(image))
        # Write data to hardware.
        self.data(pixelbytes)

    def clear(self, color=(0,0,0)):
        """Clear the image buffer to the specified RGB color (default black)."""
        width, height = self.buffer.size
        self.buffer.putdata([color]*(width*height))

    def draw(self):
        """Return a PIL ImageDraw instance for 2D drawing on the image buffer."""
        return ImageDraw.Draw(self.buffer)
