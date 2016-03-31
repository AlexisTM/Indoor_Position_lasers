Root folder
============

Ce dossier contient le patch brut à appliquer au système.

Installation de ROS 
----------------
http://wiki.ros.org/indigo/Installation/Ubuntu

Installez Kubuntu 14.04-04 via une clé USB bootable (Linux Live USB Creator)

Redémarrez

Ajoutez les sources de packages ROS

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116

Mettez la liste des fichiers à jour

	sudo apt-get update

Installez ROS 
	
	sudo apt-get install ros-indigo-desktop-full ros-indigo-mavros ros-indigo-mavros-extras python-rosinstall ros-indigo-rosserial ros-indigo-rosserial-arduino python-catkin-tools

Cherchez éventuellement des packages additionnels

	apt-cache search ros-indigo

Initialisez ROSDEP (dépendances des packages ROS)

	sudo rosdep init
	rosdep update

Lancez l'environnement ROS

	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
	source ~/.bashrc

Redémarrez, lancez roscore et vous avez un PC prêt à l'emploi

USB Symlink
------------

Nous utilisons plusieurs devices USB. Ceci pose problème pour savoir quel port correspond à qui. C'est pourquoi, nous avons ajouté un lien symbolique basé sur l'id du vendeur et du produit du composant USB.

Ceci est réalisé dans le fichier `/etc/udev/rules.d/99-ttyNames.custom.rules`

    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyLasers"
    SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixHawk"

Si l'on change d'Arduino, il faut donc changler le vendeur et le produit. Pour ce faire, branchez l'Arduno, et utilisez la commande `lsusb` afin d'obtenir les ID. 

    Bus 001 Device 008: ID vvvv:pppp Arduino SA Mega 2560 R3 (CDC ACM)

Dans ce cas-ci, vvvv est l'idVendor et pppp l'idProduct. De plus, le symlink doit avoir le nom : ttyCustomName, afin de savoir que c'est une connexion série.

TTY group rights
----------------

Afin de ne pas utiliser le superutilisateur dès que l'on utilise les ports séries, il faut ajouter l'utilisateur linux dans le groupe autorisé.

    usermod -a -G tty ros-user
    usermod -a -G dialout ros-user