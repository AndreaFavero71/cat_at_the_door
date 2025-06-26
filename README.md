# CAT AT THE DOOR – A system to detect and notify when your cats are waiting at the door.<br>

<br><br>
This repository contains all the files and information needed to reproduce the full system.<br><br>
![title image](/images/cat_at_the_door.bmp)



<br><br><br>
## Short introduction:
This system is based on two devices: a **Stationary** unit and a **Portable** unit.
- The **Stationary device** is fixed inside your house, pointing ouside the door or area where your cats usually return.
- The **Portable device** is battery-powered and can be kept wherever you are—working, relaxing, or just not near the door.<br>

Here's how it works:<br>
- The Stationary device includes a radar sensor that detects motion.<br>
- When motion is detected, it scans for **BLE tags** worn by your cat(s).<br>
- If a registered tag is found, the Portable device provides **visual** and **audible** notifications.

The devices communicate via **LoRa technology**, which ensures a strong and reliable connection—even across walls or floors.


<br><br><br>
## Demo video:
MACCHIA, one of our cats, comes back home after wandering in the garden for over an hour:<br>

 
   
https://youtu.be/Dh-xW871_UM
[![Watch the Demo](https://i.ytimg.com/vi/0kiuHv76AjQ/maxresdefault.jpg)](https://youtu.be/0kiuHv76AjQ)


<br><br><br>
## The devices:
![title image](/images/devices.jpg)<br>
The stationary device is wrapped with aluminium tape, on most of the faces, to prevent the radar from sensing inside the room.<br>


<br><br><br>
## System architectures:
This system combines different technologies:
- Proximity sensor via mmWave radar.
- BLE scanner, for the tags on cat's collar.
- LoRa for the communication between the two devices.
- Wi-Fi for NTP time synchronization.
- EPD display for the visual feedback.
- Buzzer for audible feedback.
- Rotation encoder with push button, for simple UI interaction.
- Battery management.

![title image](/images/System_achitecture.jpg)<br>


<br><br><br>
## LoRa power negotiation system:
In this system, I’ve implemented a **dynamic power adjustment** for LoRa transmission, based on a single character.<br>
Perhaps this might be interesting for somebody ...<br>

**Here’s how it works:**<br>
Both devices start communicating using the LoRa power level defined in the `config.json` settings file.<br>
After each data reception, each device checks the LoRa signal strength:
- If it’s **below** the predefined window, it appends the character `<` to the outgoing string to inform the other device to **increase** its transmission power.
- If it’s **above** the predefined window, it appends the character `>` to inform the other device to **decrease** its power (to save energy).
- If the signal is **within** the predefined window, no character is added.
- If a reply is **completely missed** four times in a row, the device temporarily increases its transmission power to the **maximum (22 dB)**. If no response is received after **15 consecutive failures**, the device resets the LoRa power to the **default** value, assuming the other device has been turned off or forgotten.<br>

The idea behind this mechanism is to let the devices **optimize transmission power** to the minimum necessary level. Because of this, it’s not necessary to set the default LoRa power to the high levels required for edge cases.<br>

This clearly is a rather basic system, yet it delivers on **power management** without impacting LoRa Time-on-Air.<br>
Although I haven’t yet measured the actual benefit (e.g., battery life improvement), I think this feature is pretty COOL :smile:<br>


<br><br><br>
## LoRa Time-on-Air:<br>
LoRa frequencies and transmission durations (duty cycles) are regulated by national authorities and vary from country to country.<br>

This topic is covered more extensively in the project manual, but here’s a brief summary:<br>
Relevant parameters, such as LoRa frequency and duty-cycle limits, are defined in the `config.json` file and **must comply with the regulations of your region**.<br>
The actual duty cycle depends heavily on the **Time-on-Air**, which in turn is influenced by LoRa settings (e.g., spreading factor, bandwidth), as well as application-specific parameters like:<br>
- the number of cats being tracked,
- the number of updates per minute, etc.

At startup, the **portable device** reads these settings and calculates its expected duty cycle.<br>
If the calculated value exceeds the allowed limit (as defined in `config.json`), the device will automatically **increase the time interval between transmissions** to stay within legal constraints.<br>
If the portable device is connected to a PC (e.g., via Thonny), relevant feedback will be printed to the shell for better understanding.<br>
Please refer to the manual for more detailed information.<br>


<br><br><br>
## The instruction manual:
A detailed 70-page instruction manual is available here: [How_to_make_CAT_AT_THE_DOOR.pdf](doc/How_to_make_CAT_AT_THE_DOOR.pdf) <br>
The manual is organized in several chapters, divided into 8 main sections:<br>
| #  | Section           | Description |
|----|-------------------|-------------|
| 1) | Introduction      | Project presentation |
| 2) | Supplies          | List of necessary parts |
| 3) | Make              | 3D prints, wiring, assembling, etc |
| 4) | Program           | Programming the boards |
| 5) | Tuning            | Adjusting the settings to your case |
| 6) | Troubleshooting   | An initial guide on what could go wrong, and how to move on |
| 7) | How to use it     | Explain how to use this system |
| 8) | Info              | Side information |

One of the chapters is the **To Do list** :smile:


<br><br><br>
## Where to start:
If you're interested, I'd suggest:

1. Skim the manual once to get a general idea.
2. Let it sit for a day—yes, really!
3. Read it again more carefully.
4. You'll then know exactly what to do next.


