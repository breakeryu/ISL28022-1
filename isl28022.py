import smbus


class ISL28022(object):
    # Registers
    REG_CONFIG = 0x00
    REG_SHUNT_VOLTAGE = 0x01
    REG_BUS_VOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05
    REG_SHUNT_VOLTAGE_THRESH = 0x06
    REG_BUS_VOLTAGE_THRESH = 0x07
    REG_DCS_INTERRUPT_STATUS = 0x08
    REG_AUX_CONTROL = 0x09

    BRNG_MASK = 0b11 << 13
    BRNG_16 = 0b00 << 13
    BRNG_32 = 0b01 << 13
    BRNG_60 = 0b11 << 13

    PG_MASK = 0b11 << 11
    PG_40 = 0b00 << 11
    PG_80 = 0b01 << 11
    PG_160 = 0b10 << 11
    PG_320 = 0b11 << 11

    BADC_MASK = 0b1111 << 7
    BADC_12_BIT = 0b0000 << 7
    BADC_13_BIT = 0b0001 << 7
    BADC_14_BIT = 0b0010 << 7
    BADC_15_BIT = 0b0011 << 7
    BADC_2_SAMPLES = 0b1001 << 7
    BADC_4_SAMPLES = 0b1010 << 7
    BADC_8_SAMPLES = 0b1011 << 7
    BADC_16_SAMPLES = 0b1100 << 7
    BADC_32_SAMPLES = 0b1101 << 7
    BADC_64_SAMPLES = 0b1110 << 7
    BADC_128_SAMPLES = 0b1111 << 7

    SADC_MASK = 0b1111 << 3
    SADC_12_BIT = 0b0000 << 3
    SADC_13_BIT = 0b0001 << 3
    SADC_14_BIT = 0b0010 << 3
    SADC_15_BIT = 0b0011 << 3
    SADC_2_SAMPLES = 0b1001 << 3
    SADC_4_SAMPLES = 0b1010 << 3
    SADC_8_SAMPLES = 0b1011 << 3
    SADC_16_SAMPLES = 0b1100 << 3
    SADC_32_SAMPLES = 0b1101 << 3
    SADC_64_SAMPLES = 0b1110 << 3
    SADC_128_SAMPLES = 0b1111 << 3

    MODE_MASK = 0b111
    MODE_POWER_DOWN = 0b000
    MODE_SHUNT_TRIGGERED = 0b001
    MODE_BUS_TRIGGERED = 0b010
    MODE_SHUNT_BUS_TRIGGERED = 0b011
    MODE_ADC_OFF = 0b100
    MODE_SHUNT_CONTINUOUS = 0b101
    MODE_BUS_CONTINUOUS = 0b110
    MODE_SHUNT_BUS_CONTINUOUS = 0b111

    BUS_MIN_WARN = 0b0001
    BUS_MAX_WARN = 0b0010
    SHUNT_MIN_WARN = 0b0100
    SHUNT_MAX_WARN = 0b1000

    FORCE_INTERRUPT = 0x100
    INTERRUPT_ENABLE = 0x080
    EXT_CLK_ENABLE = 0x040
    EXT_CLK_DIV = 0x03F

    # 10 uV
    VSHUNT_LSB = 0.00001
    # 4 mV
    VBUS_LSB = 0.004
    # 2.56 mV
    VSHUNT_THRESH_LSB = 0.00256
    # 256 mV
    VBUS_THRESH_LSB = 0.256

    def __init__(self, address, bus=None, port=None):
        if bus:
            self.smbus = bus
        elif port:
            self.smbus = smbus.SMBus(port)
        else:
            raise ValueError("Must pass either 'bus' or 'port'")

        self.address = address
        self.config = None

        self.last_conversion = 0
        self.last_overflow = 0
        self.current_lsb = 0

    def configure(self, brng=None, pg=None, badc=None, sadc=None, mode=None):
        current = self.read_config()
        if brng is None:
            brng = current & self.BRNG_MASK
        if pg is None:
            pg = current & self.PG_MASK
        if badc is None:
            badc = current & self.BADC_MASK
        if sadc is None:
            sadc = current & self.SADC_MASK
        if mode is None:
            mode = current & self.MODE_MASK

        self.config = brng | pg | badc | sadc | mode
        self.write_config(self.config)

    def calibrate(self, shunt_ohms):
        v_shunt = 0
        if self.test_pg(self.PG_320):
            v_shunt = 0.320
        elif self.test_pg(self.PG_160):
            v_shunt = 0.160
        elif self.test_pg(self.PG_80):
            v_shunt = 0.080
        elif self.test_pg(self.PG_40):
            v_shunt = 0.040

        current_fs = v_shunt/shunt_ohms

        if self.test_sadc(self.SADC_12_BIT):
            adc_res = 2**12
        elif self.test_sadc(self.SADC_13_BIT):
            adc_res = 2**13
        elif self.test_sadc(self.SADC_14_BIT):
            adc_res = 2**14
        else:
            adc_res = 2**15

        self.current_lsb = current_fs / adc_res
        cal_reg_val = int(0.04096/(self.current_lsb * shunt_ohms))
        self.write_register(self.REG_CALIBRATION, cal_reg_val)

    @classmethod
    def byte_swap(cls, val):
        low = (val & 0xFF00) >> 8
        high = (val & 0x00FF) << 8
        return high + low

    def test_pg(self, val):
        return self.config & self.PG_MASK == val

    def test_brng(self, val):
        return self.config & self.BRNG_MASK == val

    def test_sadc(self, val):
        return self.config & self.SADC_MASK == val

    def read_register(self, register_address):
        val = self.smbus.read_word_data(self.address, register_address)
        return self.byte_swap(val)

    def write_register(self, register_address, val):
        swapped = self.byte_swap(val)
        self.smbus.write_word_data(self.address, register_address, swapped)

    def read_config(self):
        return self.read_register(self.REG_CONFIG)

    def write_config(self, val):
        self.write_register(self.REG_CONFIG, val)

    def read_shunt_voltage(self):
        val = self.read_register(self.REG_SHUNT_VOLTAGE)
        sign = 0

        if self.test_pg(self.PG_320):
            if val & (1 << 15):
                sign = -1 * 2**15
            val = (val & 0x7FFF) + sign

        if self.test_pg(self.PG_160):
            if val & (1 << 14):
                sign = -1 * 2**14
            val = (val & 0x3FFF) + sign

        if self.test_pg(self.PG_80):
            if val & (1 << 13):
                sign = -1 * 2 ** 13
            val = (val & 0x1FFF) + sign

        if self.test_pg(self.PG_40):
            if val & (1 << 12):
                sign = -1 * 2**12
            val = (val & 0x0FFF) + sign

        # LSB is 10uV, so multiply the final value with this
        return val * self.VSHUNT_LSB

    def read_bus_voltage(self):
        val = self.read_register(self.REG_BUS_VOLTAGE)

        self.last_conversion = val & 0b10
        self.last_overflow = val & 0b01

        if self.test_brng(self.BRNG_60):
            val = val >> 2
        if self.test_brng(self.BRNG_32):
            val = val >> 3
        if self.test_brng(self.BRNG_16):
            val = val >> 3

        # LSB is 4mV, so multiply the final value with this
        return val * self.VBUS_LSB

    def read_current(self):
        val = self.read_register(self.REG_CURRENT)
        sign = 0

        if val & (1 << 15):
            sign = -1 * 2**15
        val = (val & 0x7FFF) + sign

        return self.current_lsb * val

    def read_power(self):
        val = self.read_register(self.REG_POWER)
        power_lsb = self.current_lsb * self.VBUS_LSB

        val = val * power_lsb * 5000

        if self.test_brng(self.BRNG_60):
            val *= 2

        return val

    def read_shunt_voltage_thresholds(self):
        val = self.read_register(self.REG_SHUNT_VOLTAGE_THRESH)
        thresh_max = (val & 0x7F00) >> 8
        thresh_min = val & 0x007F

        if val & 0x8000:
            thresh_max += -128
        if val & 0x0080:
            thresh_min += -128

        return [v * self.VSHUNT_THRESH_LSB for v in (thresh_min, thresh_max)]

    def write_shunt_voltage_thresholds(self, thresh_min, thresh_max):
        thresh_min = int(thresh_min/self.VSHUNT_THRESH_LSB)
        if thresh_min < 0:
            # Overflow the 8bit number and set the sign bit
            thresh_min += 128
            thresh_min |= 0x80

        thresh_max = int(thresh_max/self.VSHUNT_THRESH_LSB)
        if thresh_max < 0:
            # Overflow the 8bit number and set the sign bit
            thresh_min += 128
            thresh_min |= 0x80

        val = (thresh_max << 8) + thresh_min
        self.write_register(self.REG_SHUNT_VOLTAGE_THRESH, val)

    def read_bus_voltage_thresholds(self):
        val = self.read_register(self.REG_BUS_VOLTAGE_THRESH)
        thresh_max = (val & 0xFF00) >> 8
        thresh_min = val & 0x00FF

        return [v * self.VBUS_THRESH_LSB for v in (thresh_min, thresh_max)]

    def write_bus_voltage_thresholds(self, thresh_min, thresh_max):
        thresh_min = int(thresh_min / self.VBUS_THRESH_LSB)
        thresh_max = int(thresh_max / self.VBUS_THRESH_LSB)

        val = (thresh_max << 8) + thresh_min
        self.write_register(self.REG_BUS_VOLTAGE_THRESH, val)

    def read_interrupts(self):
        return self.read_register(self.REG_DCS_INTERRUPT_STATUS)

    def write_interrupts(self, val):
        return self.write_register(self.REG_DCS_INTERRUPT_STATUS, val)

    def is_bus_min_warn(self):
        return self.read_interrupts() & self.BUS_MIN_WARN

    def is_bus_max_warn(self):
        return self.read_interrupts() & self.BUS_MAX_WARN

    def is_shunt_min_warn(self):
        return self.read_interrupts() & self.SHUNT_MIN_WARN

    def is_shunt_max_warn(self):
        return self.read_interrupts() & self.SHUNT_MAX_WARN

    def clear_bus_min_warn(self):
        self.write_interrupts(self.BUS_MIN_WARN)

    def clear_bus_max_warn(self):
        self.write_interrupts(self.BUS_MAX_WARN)

    def clear_shunt_min_warn(self):
        self.write_interrupts(self.SHUNT_MIN_WARN)

    def clear_shunt_max_warn(self):
        self.write_interrupts(self.SHUNT_MAX_WARN)

    def read_aux_control(self):
        return self.read_register(self.REG_AUX_CONTROL)

    def write_aux_control(self, val):
        self.write_register(self.REG_AUX_CONTROL, val)

    def force_interrupt(self):
        val = self.read_aux_control()
        self.write_aux_control(self.FORCE_INTERRUPT | val)

    def enable_interrupt(self):
        val = self.read_aux_control()
        self.write_aux_control(self.INTERRUPT_ENABLE | val)

    def disable_interrupt(self):
        val = self.read_aux_control()
        self.write_aux_control(~self.INTERRUPT_ENABLE & val)

    def is_interrupt_enabled(self):
        return self.read_aux_control() & self.INTERRUPT_ENABLE

    def enable_ext_clock(self):
        val = self.read_aux_control()
        self.write_aux_control(self.EXT_CLK_ENABLE | val)

    def disable_ext_clock(self):
        val = self.read_aux_control()
        self.write_aux_control(~self.EXT_CLK_ENABLE & val)

    def is_ext_clock_enabled(self):
        return self.read_aux_control() & self.EXT_CLK_ENABLE

    def read_ext_clock_div(self):
        return self.read_aux_control() & self.EXT_CLK_DIV

    def write_ext_clock_div(self, div_val):
        val = self.read_aux_control()
        self.write_aux_control(val | div_val)
