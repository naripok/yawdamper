## GPIO DRIVER ##########################

OUT     = 0
IN      = 1
HIGH    = True
LOW     = False

RISING      = 1
FALLING     = 2
BOTH        = 3

PUD_OFF  = 0
PUD_DOWN = 1
PUD_UP   = 2


class BaseGPIO(object):
    """Base class for implementing simple digital IO for a platform.
    Implementors are expected to subclass from this and provide an implementation
    of the setup, output, and input functions."""

    def setup(self, pin, mode, pull_up_down=PUD_OFF):
        """Set the input or output mode for a specified pin.  Mode should be
        either OUT or IN."""
        raise NotImplementedError

    def output(self, pin, value):
        """Set the specified pin the provided high/low value.  Value should be
        either HIGH/LOW or a boolean (true = high)."""
        raise NotImplementedError

    def input(self, pin):
        """Read the specified pin and return HIGH/true if the pin is pulled high,
        or LOW/false if pulled low."""
        raise NotImplementedError

    def set_high(self, pin):
        """Set the specified pin HIGH."""
        self.output(pin, HIGH)

    def set_low(self, pin):
        """Set the specified pin LOW."""
        self.output(pin, LOW)

    def is_high(self, pin):
        """Return true if the specified pin is pulled high."""
        return self.input(pin) == HIGH

    def is_low(self, pin):
        """Return true if the specified pin is pulled low."""
        return self.input(pin) == LOW

    # Basic implementation of multiple pin methods just loops through pins and
    # processes each one individually. This is not optimal, but derived classes can
    # provide a more optimal implementation that deals with groups of pins
    # simultaneously.
    # See MCP230xx or PCF8574 classes for examples of optimized implementations.

    def output_pins(self, pins):
        """Set multiple pins high or low at once.  Pins should be a dict of pin
        name to pin value (HIGH/True for 1, LOW/False for 0).  All provided pins
        will be set to the given values.
        """
        # General implementation just loops through pins and writes them out
        # manually.  This is not optimized, but subclasses can choose to implement
        # a more optimal batch output implementation.  See the MCP230xx class for
        # example of optimized implementation.
        for pin, value in iter(pins.items()):
            self.output(pin, value)

    def setup_pins(self, pins):
        """Setup multiple pins as inputs or outputs at once.  Pins should be a
        dict of pin name to pin type (IN or OUT).
        """
        # General implementation that can be optimized by derived classes.
        for pin, value in iter(pins.items()):
            self.setup(pin, value)

    def input_pins(self, pins):
        """Read multiple pins specified in the given list and return list of pin values
        GPIO.HIGH/True if the pin is pulled high, or GPIO.LOW/False if pulled low.
        """
        # General implementation that can be optimized by derived classes.
        return [self.input(pin) for pin in pins]

    def add_event_detect(self, pin, edge):
        """Enable edge detection events for a particular GPIO channel.  Pin
        should be type IN.  Edge must be RISING, FALLING or BOTH.
        """
        raise NotImplementedError

    def remove_event_detect(self, pin):
        """Remove edge detection for a particular GPIO channel.  Pin should be
        type IN.
        """
        raise NotImplementedError

    def add_event_callback(self, pin, callback):
        """Add a callback for an event already defined using add_event_detect().
        Pin should be type IN.
        """
        raise NotImplementedError

    def event_detected(self, pin):
        """Returns True if an edge has occured on a given GPIO.  You need to
        enable edge detection using add_event_detect() first.   Pin should be
        type IN.
        """
        raise NotImplementedError

    def wait_for_edge(self, pin, edge):
        """Wait for an edge.   Pin should be type IN.  Edge must be RISING,
        FALLING or BOTH."""
        raise NotImplementedError

    def cleanup(self, pin=None):
        """Clean up GPIO event detection for specific pin, or all pins if none
        is specified.
        """
        raise NotImplementedError

    # helper functions useful to derived classes

    def _validate_pin(self, pin):
        # Raise an exception if pin is outside the range of allowed values.
        if pin < 0 or pin >= self.NUM_GPIO:
            raise ValueError('Invalid GPIO value, must be between 0 and {0}.'.format(self.NUM_GPIO))

    def _bit2(self, src, bit, val):
        bit = 1 << bit
        return (src | bit) if val else (src & ~bit)


class PIGPIOAdapter(BaseGPIO):
    def __init__(self, pigpio, mode=None):
        self.gpio = pigpio.pi()

        self._dir_mapping = {OUT: pigpio.OUTPUT,
                             IN: pigpio.INPUT}
        self._pud_mapping = {PUD_OFF: pigpio.PUD_OFF,
                             PUD_DOWN: pigpio.PUD_DOWN,
                             PUD_UP: pigpio.PUD_UP}
        self._edge_mapping = {RISING: pigpio.RISING_EDGE,
                              FALLING: pigpio.FALLING_EDGE,
                              BOTH: pigpio.EITHER_EDGE}

    def setup(self, pin, mode, pull_up_down=PUD_OFF):
        """Set the input or output mode for a specified pin.  Mode should be
        either OUTPUT or INPUT.
        """
        self.gpio.set_mode(pin, self._dir_mapping[mode])
        self.gpio.set_pull_up_down(pin, self._pud_mapping[pull_up_down])

    def output(self, pin, value):
        """Set the specified pin the provided high/low value.  Value should be
        either HIGH/LOW or a boolean (true = high).
        """
        self.gpio.write(pin, value)

    def input(self, pin):
        """Read the specified pin and return HIGH/true if the pin is pulled high,
        or LOW/false if pulled low.
        """
        return self.gpio.read(pin)

    def input_pins(self, pins):
        """Read multiple pins specified in the given list and return list of pin values
        GPIO.HIGH/True if the pin is pulled high, or GPIO.LOW/False if pulled low.
        """
        # maybe rpi has a mass read...  it would be more efficient to use it if it exists
        return [self.gpio.read(pin) for pin in pins]

    def cleanup(self, pin=None):
        """Clean up GPIO event detection for specific pin, or all pins if none
        is specified.
        """
        if pin is None:
            self.rpi_gpio.cleanup()
        else:
            self.rpi_gpio.cleanup(pin)




class RPiGPIOAdapter(BaseGPIO):
    """GPIO implementation for the Raspberry Pi using the RPi.GPIO library."""

    def __init__(self, rpi_gpio, mode=None):
        self.rpi_gpio = rpi_gpio
        # Suppress warnings about GPIO in use.
        rpi_gpio.setwarnings(False)
        # Setup board pin mode.
        if mode == rpi_gpio.BOARD or mode == rpi_gpio.BCM:
            rpi_gpio.setmode(mode)
        elif mode is not None:
            raise ValueError('Unexpected value for mode.  Must be BOARD or BCM.')
        else:
            # Default to BCM numbering if not told otherwise.
            rpi_gpio.setmode(rpi_gpio.BCM)
        # Define mapping of Adafruit GPIO library constants to RPi.GPIO constants.
        self._dir_mapping = {OUT: rpi_gpio.OUT,
                             IN: rpi_gpio.IN}
        self._pud_mapping = {PUD_OFF: rpi_gpio.PUD_OFF,
                             PUD_DOWN: rpi_gpio.PUD_DOWN,
                             PUD_UP: rpi_gpio.PUD_UP}
        self._edge_mapping = {RISING: rpi_gpio.RISING,
                              FALLING: rpi_gpio.FALLING,
                              BOTH: rpi_gpio.BOTH}

    def setup(self, pin, mode, pull_up_down=PUD_OFF):
        """Set the input or output mode for a specified pin.  Mode should be
        either OUTPUT or INPUT.
        """
        self.rpi_gpio.setup(pin, self._dir_mapping[mode],
                            pull_up_down=self._pud_mapping[pull_up_down])

    def output(self, pin, value):
        """Set the specified pin the provided high/low value.  Value should be
        either HIGH/LOW or a boolean (true = high).
        """
        self.rpi_gpio.output(pin, value)

    def input(self, pin):
        """Read the specified pin and return HIGH/true if the pin is pulled high,
        or LOW/false if pulled low.
        """
        return self.rpi_gpio.input(pin)

    def input_pins(self, pins):
        """Read multiple pins specified in the given list and return list of pin values
        GPIO.HIGH/True if the pin is pulled high, or GPIO.LOW/False if pulled low.
        """
        # maybe rpi has a mass read...  it would be more efficient to use it if it exists
        return [self.rpi_gpio.input(pin) for pin in pins]

    def add_event_detect(self, pin, edge, callback=None, bouncetime=-1):
        """Enable edge detection events for a particular GPIO channel.  Pin
        should be type IN.  Edge must be RISING, FALLING or BOTH.  Callback is a
        function for the event.  Bouncetime is switch bounce timeout in ms for
        callback
        """
        kwargs = {}
        if callback:
            kwargs['callback'] = callback
        if bouncetime > 0:
            kwargs['bouncetime'] = bouncetime
        self.rpi_gpio.add_event_detect(pin, self._edge_mapping[edge], **kwargs)

    def remove_event_detect(self, pin):
        """Remove edge detection for a particular GPIO channel.  Pin should be
        type IN.
        """
        self.rpi_gpio.remove_event_detect(pin)

    def add_event_callback(self, pin, callback):
        """Add a callback for an event already defined using add_event_detect().
        Pin should be type IN.
        """
        self.rpi_gpio.add_event_callback(pin, callback)

    def event_detected(self, pin):
        """Returns True if an edge has occured on a given GPIO.  You need to
        enable edge detection using add_event_detect() first.   Pin should be
        type IN.
        """
        return self.rpi_gpio.event_detected(pin)

    def wait_for_edge(self, pin, edge):
        """Wait for an edge.   Pin should be type IN.  Edge must be RISING,
        FALLING or BOTH.
        """
        self.rpi_gpio.wait_for_edge(pin, self._edge_mapping[edge])

    def cleanup(self, pin=None):
        """Clean up GPIO event detection for specific pin, or all pins if none
        is specified.
        """
        if pin is None:
            self.rpi_gpio.cleanup()
        else:
            self.rpi_gpio.cleanup(pin)
