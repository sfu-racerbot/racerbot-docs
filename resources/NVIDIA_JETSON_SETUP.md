# How to Setup The NVIDIA Jetson Orin Nano

## Prerequisites
You need to be either running `Windows 10/11` or `Ubuntu 22.04 LTS` (Cannot be Ubuntu derivatives such as Linux Mint, has to be Ubuntu 22.04)

You need an NVIDIA account (it's free) or get the club NVIDIA account from an executive

You need to have the NVIDIA SDK Manager Installed, you can download it [here](https://developer.nvidia.com/sdk-manager) (This does require a NVIDIA account)

## Step 1: Put Jetson into recovery mode
First, make sure the Jetson is powered off and unplugged

Plug in a USB-C cable into your computer (do not use hubs, connect it directly to your computer)

Put a jumper on pins 9 and 10 of the button header. The image below (Figure 1-4) shows you where the button header is and what pins to jump

![Figure 1](/assets/resources/figure-1.png)

[Image Source](https://developer.download.nvidia.com/assets/embedded/secure/jetson/orin_nano/docs/Jetson-Orin-Nano-DevKit-Carrier-Board-Specification_SP-11324-001_v1.3.pdf?__token__=exp=1775187958~hmac=88cb04badcc35cb9edfc9d8d1b21fc1172a048684a0c192f0dcef37fa13782f4&t=eyJscyI6IndlYnNpdGUiLCJsc2QiOiJkZXZlbG9wZXIubnZpZGlhLmNvbS8/ZmlsZW5hbWU9NDAzLmh0bWwifQ==)

Next, plug the power cable into the Jetson

Wait three seconds and then remove the jumper

To verify that the Jetson is in recovery mode (if you are on Linux) open a terminal and run the command `lsusb`, if there is an entry that contains something like `NVIDIA AGX`, the Jetson is in recovery mode.

## Step 2: Flash the Jetson
Open the NVIDIA SDK Manager and login to your own NVIDIA account or the club account

There should be a popup saying that the board was detected and two options are given, choose the one that has `Developer Kit` in the name.

The software should look like this, except yours should say that the board was detected
![Figure 2](/assets/resources/figure-2.png)

Click the continue button, you will now be on a screen that looks like this (If it says `Checking Status` next to each component that is ok):
![Figure 3](/assets/resources/figure-3.png)

Accept the terms and conditions, then click the big green continue button.

It may prompt you during the installation to set a username and password, please consult an executive before doing so.

Once the installation is finished, it is safe to close the SDK manager and unplug the Jetson.

## How to remotely access the Jetson

## SSH
Connect the Jetson to the same network as your computer, and run `ip addr`. The number that looks like `xxx.xxx.xxx.xx` is the IP address.

On your computer, open a terminal and run `ssh username@ip_address`. At the first prompt type `yes` and then enter the password for the user. Now you should have a remote shell
into the Jetson

## Remote Desktop
Connect the Jetson to the same network as your computer, and run `ip addr`. The number that looks like `xxx.xxx.xxx.xx` is the IP address.

With the Jetson hooked up to a monitor/mouse/keyboard, press the Windows key, go to the search bar and type `settings`. Then open settings. Go to the sharing section and click
`Remote Desktop`, set remote desktop to `on` and look at the username and password for it. Also toggle on `Allow Screen Control`

### Windows
Open the start menu and search for `Remote Desktop Connection`, open it up and in the `Computer` field type the IP address of the Jetson. It will then prompt you for a username
and password, enter the username and password you saw before (not the one to login, for some reason it's different).

You should now be remotely connected to the Jetson and can safely disconnect the monitor/keyboard/mouse.

### Linux
I highly recommend looking into installing [Remmina](https://remmina.org/)

## What to do on SFU Wi-Fi
SFU's internet doesn't allow for easily connecting to local devices, so for now the best thing to do is create a hotspot from your computer. On Windows 11, go to Settings, go
to Network, go to Mobile Hotspot and turn it on. Look at the SSID and password then connect the Jetson to that network. In the table below the QR code it will show you the IP
address of the Jetson. Use that to Remote Desktop or SSH into the Jetson. 
