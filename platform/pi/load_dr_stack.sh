#!/bin/bash
#-----------------------------------------------------------------------
# Dead Reckoning System Loader Script
# Loads kernel modules and Launches the DR application
#-----------------------------------------------------------------------

set -e # Exit immediately on unhandled error
LOGTAG="[DR-LOADER]"

# Helper: print error and exit
fail() {
    echo "${LOGTAG} ERROR: $1" >&2
    exit 1
}

echo "${LOGTAG} Starting Dead Reckoning System Loader..."

# 1. Load regmap_i2c (build in some kernels; modprobe is safe either way)
if ! modprobe regmap_i2c 2>/dev/null; then
    fail "Failed to load regmap_i2c module."
fi
echo "${LOGTAG} Loaded regmap_i2c module."

# 2. Insert MPU6050 character driver module
if ! insmod ./mpu6050_char.ko; then
    fail "Failed to insert mpu6050_char module."
fi
echo "${LOGTAG} Inserted mpu6050_char module."

# 3. Insert NEO-6M GPS character driver module
if ! insmod ./neo6m_gnss_serdev.ko; then
    fail "Failed to insert neo6m_gnss_serdev module."
fi
echo "${LOGTAG} Inserted neo6m_gnss_serdev module."

# 4. Launch Dead Reckoning application
if [[ ! -x ./dr_app_1 ]]; then
    fail "Executable dr_app_1 not found or not executable."
fi

echo "${LOGTAG} Launching Dead Reckoning application..."
./dr_app_1 || fail "Dead Reckoning application terminated with an error."

echo "${LOGTAG} All Components Loaded Successfully."
exit 0

