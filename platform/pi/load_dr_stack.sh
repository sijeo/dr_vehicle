#!/bin/bash
#-----------------------------------------------------------------------
# Dead Reckoning System Loader Script
# Loads kernel modules and Launches the DR application
# Usage: 
#   chmod +x load_dr_stack.sh
#   sudo ./load_dr_stack.sh
#-----------------------------------------------------------------------


LOGTAG="[DR-LOADER]"
set -e #Exit immediately on unhandled errors

#Absolute paths of modules and app
MPU6050_MOD="/home/sijeo/dr_vehicle/mpu6050_char.ko"
NEO6M_MOD="/home/sijeo/dr_vehicle/neo6m_gnss_serdev.ko"
APP_BIN="/home/sijeo/dr_vehicle/dr_app_1"

#Track successfully loaded modules for rollback
LOADED_MODULES=()

#Error handler: print message, unload any loaded modules, and exit
fail() {
	echo "$LOGTAG ERROR: $1" >&2
	if [ ${#LOADED_MODULES[@]} -gt 0 ]; then
		echo "$LOGTAG Rolling back previously loaded modules..." >&2
		for (( idx=${#LOADED_MODULES[@]}-1; idx>=0; idx--)); do
			mod="${#LOADED_MODULES[idx]}"
			echo "$LOGTAG Removing module: $mod" >&2
			/sbin/rmmod "$mod" 2>/dev/null || echo "$LOGTAG WARN: could not remove $mod" >&2
		done
	fi
	exit 1
}

echo "$LOGTAG Starting Dead Reckoning load sequence..."

# 1. Load regmap_i2c
if modprobe regmap_i2c 2>/dev/null; then
	LOADED_MODULES+=("regmap_i2c")
	echo "$LOGTAG regmap_i2c loaded."
else
	fail "Failed to load regmap_i2c (module missing?)."
fi

#2. Insert MPU6050 character driver
if insmod "$MPU6050_MOD" 2>/dev/null; then
	LOADED_MODULES+=("mpu6050_char")
	echo "$LOGTAG MPU6050 driver loaded: $MPU6050_MOD"
else
	fail "Failed to insert MPU6050 driver. Check dmesg for I2C issues."
fi

# 3. Insert NEO6M GNSS driver
if insmod "$NEO6M_MOD" 2>/dev/null; then
	LOAD_MODULES+=("neo6m_gnss_serdev")
	echo "$LOGTAG NEO6M GNSS driver loaded: $NEO6M_MOD"
else
	fail "Failed to insert NEO6M GNSS Driver. Verify UART overlay and connections."
fi

# 4. Launch the Dead Reckoning application
if [[ ! -x "$APP_BIN" ]]; then
	fail "Executable $APP_BIN not found or not executable."
fi

echo "$LOGTAG Launching $APP_BIN ..."
"$APP_BIN" || fail "Application exited with error."

echo "$LOGTAG All components loaded and running successfully."
exit 0

