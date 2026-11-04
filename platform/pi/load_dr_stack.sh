#!/bin/bash
#-----------------------------------------------------------------------
# Dead Reckoning stack loader: kernel modules, button pull-up, DR app.
#
#   sudo ./load_dr_stack.sh                 # normal start
#   sudo ./load_dr_stack.sh --outage-test 180,30   # extra args go to the app
#
# Used by dr_app.service at boot (see DR_APP_README.md). The app replaces
# this shell (exec) so systemd sees its exit status and restarts it.
#-----------------------------------------------------------------------
LOGTAG="[DR-LOADER]"
BASE=/home/sijeo/dr_vehicle
IMU_MOD="$BASE/mpu6050_char.ko"
GNSS_MOD="$BASE/neo6m_gnss_serdev.ko"
APP_BIN="$BASE/dr_dead_reckoning_app"
BUTTON_GPIO=26

log() { echo "$LOGTAG $*"; logger -t dr-loader "$*" 2>/dev/null || true; }

load_mod() {   # name file
	if lsmod | grep -q "^$1 "; then
		log "$1 already loaded"
	elif insmod "$2"; then
		log "$1 loaded"
	else
		log "ERROR: insmod $2 failed (see dmesg)"; return 1
	fi
}

modprobe regmap_i2c 2>/dev/null || log "WARN: regmap_i2c not loaded (may be built in)"
load_mod mpu6050_char "$IMU_MOD" || exit 1
load_mod neo6m_gnss_serdev "$GNSS_MOD" || log "WARN: no GNSS driver - DR will start from the saved state only"

# wait for the device nodes (udev)
for i in $(seq 1 50); do
	[ -e /dev/mpu6050-0 ] && break
	sleep 0.1
done
[ -e /dev/mpu6050-0 ] || { log "ERROR: /dev/mpu6050-0 missing"; exit 1; }

# button input pull-up (also settable permanently with gpio=26=ip,pu in config.txt)
if command -v pinctrl >/dev/null 2>&1; then
	pinctrl set $BUTTON_GPIO ip pu
elif command -v raspi-gpio >/dev/null 2>&1; then
	raspi-gpio set $BUTTON_GPIO ip pu
fi

[ -x "$APP_BIN" ] || { log "ERROR: $APP_BIN not found or not executable"; exit 1; }
log "starting $APP_BIN $*"
exec "$APP_BIN" --button-gpio $BUTTON_GPIO "$@"
