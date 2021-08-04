# tinyQuan

tinyQuan is a CV quantizer with more than a hundred scales.

Input:
- CV-in (0-5V)

Output:
- CV-out (0-5V)
- Gate-out (PWM)

Controls:
- Encoder 1
-- Rotate to change scale
-- Push to activate/deactivate in-scale CV mode. When the mode is on the little indicator appears solid.
- Encoder 2
-- Rotate to change root note
-- Push to switch between piano and beatstep pro layout

### Comments
##### In-scale CV mode
This mode when active (default) splits each volt in as many bins as there are notes in the scale. Notes will therefore be separated by the same ΔCV in a given scale, but different ΔCV across different scales.

When deactivated tinyQuan follows the 1V/Oct convention. Semitones are always separated by a ΔCV of 83mV. Not all of them appear in a given scale and thus notes are not equally separated by the same ΔCV.

##### TODO
This is a very rough prototype. There are no overvoltage protection on any jack. The code is probably still full of bug and certainly not as efficient as it could. The CV-out DAC is 12-bit and should probably be more precise. The CV-in ADC is 16-bit and has 4 ports which is way overkill. Changing either would require complete retuning.

##### BOM
- 1x Arduino Nano
- 1x SSD1608 128x64 (OLED display)
- 1x ADS1115 (CV-in ADC)
- 1x MCP4725 (CV-out DAC)
- 2x Full-cycle rotary encoder with push-switch (mines were EC11)
- 13x 10K resistors
- 4x 10nF capacitors
- 2x 100nF capacitors
- 2x 3.5mm jacks
- 1x 3.5mm jack with switch
- LM317 MOSFET
- 1x trimpot (I use a multi-turn to precisely trim to 5.000V)
- 5V-12V power supply (LM317 will tolerate up to 37V)
