Root folder
============

Ce dossier contient le patch brut à appliquer au système.

USB Symlink
------------

Nous utilisons plusieurs devices USB. Ceci pose problème pour savoir quel port correspond à qui. C'est pourquoi, nous avons ajouté un lien symbolique basé sur l'id du vendeur et du produit du composant USB.

Ceci est réalisé dans le fichier `/etc/udev/rules.d/99-ttyNames.custom.rules`

    SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="2341", ATTR{idProduct}=="0042", SYMLINK+="ttyLasers"
    SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="1a86", ATTR{idProduct}=="7523", SYMLINK+="ttyPixHawk"

Si l'on change d'Arduino, il faut donc changler le vendeur et le produit. Pour ce faire, branchez l'Arduno, et utilisez la commande `lsusb` afin d'obtenir les ID. 

    Bus 001 Device 008: ID vvvv:pppp Arduino SA Mega 2560 R3 (CDC ACM)

Dans ce cas-ci, vvvv est l'idVendor et pppp l'idProduct. De plus, le symlink doit avoir le nom : ttyCustomName, afin de savoir que c'est une connexion série.