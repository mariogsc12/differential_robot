#!/bin/bash

nmcli connection up Hotspot
nmcli con mod Hotspot connection.autoconnect true
