adb wait-for-device

adb root
adb wait-for-device
adb remount
adb shell setenforce 0

echo "start install..."

adb push ../linux-arm64 /data/
adb push traced /usr/sbin
adb push traced_probes /usr/sbin
adb push perfetto /usr/bin
adb push tracebox /usr/bin

adb push traced.service /lib/systemd/system
adb push traced-probes.service /lib/systemd/system

adb shell "chmod 755 /usr/sbin/traced"
adb shell "chmod 755 /usr/sbin/traced_probes"
adb shell "chmod 755 /usr/bin/perfetto"
adb shell "chmod 755 /usr/bin/tracebox"

adb shell "chmod 755 /lib/systemd/system/traced.service"
adb shell "chmod 755 /lib/systemd/system/traced-probes.service"

adb shell "systemctl enable traced.service"
adb shell "systemctl enable traced-probes.service"

echo "install finish, please reboot device now..."

pause