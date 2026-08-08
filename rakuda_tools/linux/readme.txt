Copiare in /etc/udev/rules.d/ e Ricarica udev

sudo udevadm control --reload-rules && sudo udevadm trigger

### respeaker

sudo bash install_respeaker.sh

Lo script fa tutto da solo. Ma basandosi sul respeaker corrente, magari possono uscire updates che cambiano il codice
