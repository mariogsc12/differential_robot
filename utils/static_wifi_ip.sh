#!/bin/bash

# Default values
DEFAULT_IP="192.168.1.100"
DEFAULT_NETMASK="24"
DEFAULT_GATEWAY="192.168.1.1"
DEFAULT_WIFI="MiWifi"

print_help() {
    echo ""
    echo "Bash script to configure static WiFi."
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --help          Show this help"
    echo "  --ip            IP to assign (default: $DEFAULT_IP)"
    echo "  --netmask       Netmask in CIDR notation (default: $DEFAULT_NETMASK)"
    echo "  --gateway       Gateway IP (default: $DEFAULT_GATEWAY)"
    echo "  --wifi          WiFi connection name (default: $DEFAULT_WIFI)"
    echo ""
}

# Default variables (will be overwritten by args)
IP="$DEFAULT_IP"
NETMASK="$DEFAULT_NETMASK"
GATEWAY="$DEFAULT_GATEWAY"
WIFI="$DEFAULT_WIFI"

# Convert netmask from 255.255.255.0 format to CIDR number
netmask_to_cidr() {
    local IFS=.
    read -r i1 i2 i3 i4 <<< "$1"
    local binmask=$(printf "%08d%08d%08d%08d\n" \
        "$(bc <<< "obase=2;$i1")" \
        "$(bc <<< "obase=2;$i2")" \
        "$(bc <<< "obase=2;$i3")" \
        "$(bc <<< "obase=2;$i4")")
    # Count number of 1s
    echo "$binmask" | grep -o "1" | wc -l
}

parse_command_line() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --help)
                print_help
                exit 0
                ;;
            --ip)
                IP="$2"
                shift 2
                ;;
            --netmask)
                NETMASK="$2"
                # If format is decimal like 255.255.255.0, convert to CIDR
                if [[ "$NETMASK" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                    NETMASK=$(netmask_to_cidr "$NETMASK")
                fi
                shift 2
                ;;
            --gateway)
                GATEWAY="$2"
                shift 2
                ;;
            --wifi)
                WIFI="$2"
                shift 2
                ;;
            *)
                echo "Unknown option: $1"
                print_help
                exit 1
                ;;
        esac
    done
}

change_ip() {
    echo "Configuring static IP on WiFi connection '$WIFI'..."
    nmcli con mod "$WIFI" ipv4.addresses "$IP/$NETMASK"
    nmcli con mod "$WIFI" ipv4.gateway "$GATEWAY"
    nmcli con mod "$WIFI" ipv4.method manual
    nmcli con mod "$WIFI" ipv4.dns "8.8.8.8 1.1.1.1"

    echo "Restarting connection..."
    nmcli con down "$WIFI"
    nmcli con up "$WIFI"

    echo "Static IP configuration applied:"
    echo "  IP: $IP/$NETMASK"
    echo "  Gateway: $GATEWAY"
    echo "  WiFi: $WIFI"
}

main() {
    parse_command_line "$@"
    change_ip
}

main "$@"
