# CAT AT THE DOOR – A system to detect and notify when your cats are waiting at the door.
<br><br>
![title image](/images/cat_at_the_door.bmp)

<br><br>
## Project overview

**CAT AT THE DOOR** is a wireless system that detects when your cat is waiting at the door and notifies you via a portable device.  

It’s ideal if:  
- You cannot install a cat flap or automatic pet door.
- You like to let your cats roam outside.
- You want to be notified the moment your cat is ready to come back in.

This project uses two devices communicating via **LoRa**:
- A **stationary unit** with radar + BLE detection, placed inside near the door.
- A **portable unit** with e-paper display + buzzer, that you carry with you.


<br><br>
## Project motivation
I was perfectly happy building machines no one asked for, until my wife kindly reminded me that we also have cats… not just robots.  
Once I accepted the challenge, the project became increasingly interesting, giving me the opportunity to explore technologies I had never used before.


<br><br>
## Demo video

MACCHIA, one of our cats, comes back home after wandering in the garden for over an hour:

[![Watch the Demo](https://i.ytimg.com/vi/0kiuHv76AjQ/maxresdefault.jpg)](https://youtu.be/0kiuHv76AjQ)



<br><br>
## Key features

- Detects when one or more cats approach your door
- Uses BLE tags for individual cat recognition (supports 1–4 cats)
- Radar motion sensing, customizable via Bluetooth app
- Visual notification via e-paper display
- Audible alert via buzzer (optional)
- Portable device is battery-powered with sleep mode
- Communication via LoRa, a **Lo**ng **Ra**nge low-power wireless communication system (reliable even through walls/floors)
- Optional Wi-Fi sync for correct date/time
- Rotary encoder menu interface
- 3D printed cases included (STL & STEP)



<br><br>
## Required hardware (summary)

| Qty | Component                            | Notes |
|-----|--------------------------------------|-------|
| 2   | Heltec Wi-Fi LoRa 32 V3              | With antenna + JST battery connector |
| 1+  | Tile Sticker BLE tag (per cat)       | Use the Tile app to activate |
| 1   | HLK TD2010C radar module             | Tuned via HLKRadarTool app |
| 1   | 2.9" Waveshare e-paper display       | Black & white |
| 1   | 3.7V 2200mAh LiPo battery            | For portable device |
| 1   | Rotary encoder + push-button         | For settings menu |
| 1   | Buzzer (3.3–5V, with transistor)     | For optional alerts |
|     | USB-C panel mount sockets            | Power and recharging |
|     | 3D printed case set                  | STL and STEP files provided |
|     | Aluminum tape                        | Blocks radar from sensing inside room |
> Please read the manual for full instructions before proceeding.


<br><br>
## Quick start
1. Attach a **Tile Sticker BLE tag** to your cat’s collar and retrieve its MAC address (e.g., using *nRF Connect* app)
2. Flash **MicroPython** onto both Heltec Wi-Fi LoRa 32 V3 boards
3. Upload files via **Thonny** to both boards (portable and stationary)
4. Edit `ble_tags.json` and `config.json` with your own settings
5. Tune radar sensitivity and detection area using the **HLKRadarTool** app
6. Power up both devices and observe how it detects when your cat returns!

> This is not a plug-and-play kit. Expect some assembly, soldering, BLE scanning, and radar tuning.  
> Please read the manual for full instructions before proceeding.



<br><br>
## LoRa Power Negotiation System

This project features a **dynamic LoRa power adjustment** mechanism that minimizes transmission power while maintaining reliability.  
Here's how it works:

1. Both devices begin with the default `tx_power` value set in `config.json`.
2. After each successful data exchange, they evaluate the received signal strength (RSSI):
   - If **RSSI is too low**, the device adds a `<` character to its reply to request **more power**.
   - If **RSSI is too high**, the device adds a `>` to request **less power**.
   - If RSSI is within the configured window, no change is requested.
3. If no reply is received for 4 consecutive transmissions:
   - The sending device increases its transmission power to **22 dBm** temporarily.
4. If no reply is received after 15 failures:
   - It resets the transmission power to the default value, assuming the peer is off.

> This simple yet effective mechanism helps conserve battery life without compromising reliability—even across multiple concrete walls or floors.



<br><br>
## LoRa Duty-Cycle Compliance

LoRa frequencies and transmission limits are **regulated** in most countries.  
For example, in the EU (including the Netherlands), the max **duty cycle is 1%** (36 sec/hour).

This system:
- Calculates Time-on-Air (ToA) based on your number of cats and LoRa settings
- Automatically increases the transmission interval if the estimated duty cycle exceeds the limit
- Provides console feedback via Thonny when tuning or debugging

> All related parameters (LoRa power, frequency, spreading factor, etc.) are adjustable in `config.json`.


<br><br>
## Detailed manual

All build instructions, tuning tips, and troubleshooting steps are in the downloadable 70+ page guide:  
📄 [How_to_make_CAT_AT_THE_DOOR.pdf](doc/How_to_make_CAT_AT_THE_DOOR.pdf)

| #  | Section         | Description                            |
|----|------------------|----------------------------------------|
| 1  | Introduction     | Project goals and context              |
| 2  | Supplies         | Full list of parts                     |
| 3  | Make             | 3D printing, wiring, assembling        |
| 4  | Program          | Flashing and uploading code            |
| 5  | Tuning           | Radar and BLE tuning                   |
| 6  | Troubleshooting  | Common issues and fixes                |
| 7  | How to use it    | Practical operation                    |
| 8  | Info             | Legal, safety, and technical remarks   |



<br><br>
## ⚠️ System Limitations

While the system works reliably with proper setup, it’s important to be aware of its current limitations:

- **Radar sensitivity to movements (its own job :smile:)**  
  If the radar moves, it detects movement; The portable device should not move.  
  The mmWave radar may detect raindrops as movements.

- **Radar tuning is essential**  
  Each installation requires some trial and error. Radar positioning and threshold tuning are necessary and can take time.

- **BLE Tag compatibility**  
  Only BLE tags with fixed MAC addresses can be used. **Apple AirTags** are not compatible due to MAC address randomization.

- **Size of BLE tag**  
  The Tile Sticker is compact but may still feel bulky on very small cats.

- **Not suitable for automatic door control**  
  This project only **notifies** when your cat is outside, it does **not open** any doors or actuators.

- **Radar field of view**  
  The mmWave radar has an approximate horizontal detection angle of **60 degrees**. This is sufficient to cover a door-sized area.  
  However, if your cat tends to return from wider angles or corners not covered by this range, you may need to reposition the radar.  
  Adding a second radar module shouldn't take too much effort, and it would surely wider the coverage.

- **Intermediate DIY skills**  
  Basic soldering, 3D printing, and firmware flashing are required. Beginners can follow the guide, but the effort is non-trivial.

> Despite these limitations, with a bit of care during setup, the system proves to be a helpful, fun, and very practical solution.



<br><br>
## Where to begin

If you're interested:

1. Skim the manual once to get a general idea.
2. Let it sit for a day—yes, really!
3. Read it again more carefully.
4. You'll then know exactly what to do next.



<br><br>
_Made by Andrea Favero – for our cats Freya and Macchia_
