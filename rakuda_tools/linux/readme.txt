Copiare in /etc/udev/rules.d/ e Ricarica udev

sudo udevadm control --reload-rules && sudo udevadm trigger

### respeaker

sudo bash install_respeaker.sh

Lo script fa tutto in sequenza:

pip install pyusb pixel-ring con --break-system-packages
Scrive la udev rule in /etc/udev/rules.d/60-respeaker.rules
Crea /opt/rakuda/led_off.py con retry loop (10 tentativi, 1s ciascuno)
Crea e abilita il servizio systemd respeaker-led-off
Ricarica udev e systemd, avvia subito il servizio