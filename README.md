This is a work in progress, experimental Zephyr bit-bang driver for the Sharp LS0XXB7 series of memory-in-pixel displays. It currently only supports the LS014B7DD01 model.

# Using the driver
This is a fork of the [nrfconnect/ncs-example-application](https://github.com/nrfconnect/ncs-example-application) template:
- [`drivers/display/sharp_ls0xxb7_bitbang`](./drivers/display/sharp_ls0xxb7_bitbang): The driver itself ([MIT license](./drivers/display/sharp_ls0xxb7_bitbang/LICENSE))
- [`app/`](./app/): An example application aimed at the nRF54L15DK dev board and the [hlord2000/SharpBreakout](https://github.com/hlord2000/SharpBreakout) shield
  - [`app/boards/shield/sharp_breakout`](./app/boards/shields/sharp_breakout): overlays from [hlord2000/SharpBitbang](https://github.com/hlord2000/SharpBitbang) ([MIT license](./app/boards/shields/sharp_breakout/LICENSE))
  - [`app/dts/bindings/display`](./dts/bindings/display): The device tree bindings for the [hlord2000/SharpBreakout](https://github.com/hlord2000/SharpBreakout) shield ([MIT license](./app/dts/bindings/display/LICENSE))

# Using the driver
Add to your `west.yml`:

```yaml
  projects:
    - name: sharp_ls0xxb7_bitbang
      path: modules/sharp_ls0xxb7_bitbang
      revision: main
      url: https://github.com/rbaron/sharp-ls0xxb7-bitbang
```
