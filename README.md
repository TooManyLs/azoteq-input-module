> [!CAUTION]
> THIS MODULE HAS NOT BEEN TESTED, SO I DO NOT RECOMMEND USING IT, IF YOU
> INSTALL IT YOU DO IT AT YOUR OWN RISK !!!!

Driver for the azoteq trackpad with ZMK software
# Driver for the azoteq trackpad

# config example

```conf
CONFIG_INPUT=y
CONFIG_INPUT_AZOTEQ_IQS5XX=y
# CONFIG_ZMK_INPUT_AZOTEQ_IQS5XX_IDLE_SLEEPER=y
# CONFIG_INPUT_AZOTEQ_IQS5XX_INIT_PRIORITY=60
```

### Example Devicetree Overlay
```dts
&i2c1 {
    iqs5xx: iqs5xx@74 {
        compatible = "azoteq,iqs5xx";
        reg = <0x74>;
	// same pin like niceview
        dr-gpios = <&gpio0 1 GPIO_ACTIVE_LOW>;
        x-invert = <false>;
        y-invert = <false>;
        no-taps = <false>;
    };
};
```

5 pins are needed to configure the azoteq trackpad:

1. **Power**:
   - **3V** on the nice!nano -> **VDD** on the IQS5xx.
   - **G** (Ground) on the nice!nano -> **GND** on the IQS5xx.

2. **I2C Signals**:
   - **SDA** (Feather pin labeled "SDA") on the nice!nano -> **SDA** on the
     IQS5xx.
   - **SCL** (Feather pin labeled "SCL") on the nice!nano -> **SCL** on the
     IQS5xx.

3. **Data Ready / Interrupt Pin**:
   - The IQS5xx "DR" or "RDY" pin -> **Any available GPIO** on the nice!nano.
   - For example, you can use **D2**, **D3**, or **A0**—whichever is free in
     your design. In devicetree, you’ll reference this pin under `dr-gpios`.


[//]: # ( vim: set fdm=marker: )
