# Ball Balancer
3-RRS parallel manipulator that balances a steel ball atop a large resistive touch screen platform.

## Basic Features
- Built to balance and position a single 1.5" steel ball.
- 8.4" diagonal operational area.
- Software powered by the STM32 NUCLEO-F411RE microcontroller.
- Stable and precise 3-RRS parallel manipulator design.
- Requires 12V, 24W power supply.

## Detailed Information
- Platform motion driven by three [Adafruit](https://www.adafruit.com/) [12V stepper motors](https://www.adafruit.com/product/324).
- Stepper motors driven by three [ST](https://www.st.com/content/st_com/en.html) [X-NUCLEO-IHM03A1](https://www.st.com/en/ecosystems/x-nucleo-ihm03a1.html) high-power stepper motor driver expansion boards for the STM32 Nucleo-64 family of microcontrollers.
- Ball position measured by [VSDISPLAY](https://www.amazon.com/stores/VSDISPLAY/VSDISPLAY/page/DE75699F-7981-4B21-AC69-5A6736383F62) [8.4" Resistive Touch Screen](https://a.co/d/e97aLYa).
- 3D printed body is a custom modified version for ease of manufacture and assembly of the [3-RRS Parallel Manipulator by Aaed Musa](https://myhub.autodesk360.com/ue2bd9d7f/g/shares/SH512d4QTec90decfa6e8a1bdd403930aff7).
- Assembled with M3 and M4 metal hardware.
- Developed in VS Code and STM32CubeIDE.
