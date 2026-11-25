#!/usr/bin/env bash
# Read-only inspection of the EtherCAT bus using IGH's 'ethercat' CLI
#
# ./bringup.sh       -> inspect every slave
# ./bringup.sh 0     -> inspect slave at position 0
#
# Optional:
# ETHERCAT=/opt/etherlab/bin/ethercat ./bringup.sh

set -u

E=${ETHERCAT:-ethercat}

# Check EtherCAT CLI availability
if ! command -v "$E" >/dev/null 2>&1; then
    echo "ERROR: ethercat tool not in PATH."
    echo "Try: ETHERCAT=/opt/etherlab/bin/ethercat"
    exit 1
fi

echo "========= MASTER =========="
$E master | head -30

echo
echo "========= SLAVES =========="
$E slaves -v

# Determine slave positions
if [ $# -ge 1 ]; then
    POS="$1"
else
    POS=$($E slaves | awk '{print $1}' | sed 's/:.*//')
fi

if [ -z "$POS" ]; then
    echo "No EtherCAT slaves detected."
    exit 0
fi

# Inspect each requested slave
for p in $POS; do

    echo
    echo "########## SLAVE $p ##########"

    echo "---- Identity Object (0x1018)"
    echo "Vendor / Product / Revision / Serial"

    for s in 1 2 3 4; do
        echo "Subindex $s:"
        $E upload -p "$p" 0x1018 "$s" --type uint32 2>&1
    done

    echo
    echo "---- PDOs currently assigned/mapped"
    $E pdos -p "$p" 2>&1

    echo
    echo "---- Key objects"

    echo "Statusword (0x6041)"
    $E upload -p "$p" 0x6041 0 --type uint16 2>&1

    echo "Mode display (0x6061)"
    $E upload -p "$p" 0x6061 0 --type int8 2>&1

    echo "Actual position (0x6064)"
    $E upload -p "$p" 0x6064 0 --type int32 2>&1

    echo
    echo "---- Optional: Max torque (0x6072)"
    $E upload -p "$p" 0x6072 0 --type uint16 2>&1 \
        || echo "Not readable -> do not use --max-tq-sdo"

    echo
    echo "---- Full dictionary dump"

    $E sdos -p "$p" > "sdos_${p}.txt" 2>&1

    if [ $? -eq 0 ]; then
        echo "Saved: sdos_${p}.txt"
    else
        echo "SDO dictionary dump failed."
    fi

    echo
    echo "---- PDO mapping objects in dictionary"

    if [ -f "sdos_${p}.txt" ]; then
        grep -E "0x1600|0x1A00|0x1C12|0x1C13" \
            "sdos_${p}.txt" | head || true
    fi

done

echo
echo "========= INSPECTION COMPLETE =========="
echo "Next: ./myactl_ctl --monitor"
