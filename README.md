# Projet-STM32
Projet sur STM32L152RE dev-board et carte ISEN
le but est de créer une centrale météo

## Cahier des charges
1.	Récupérer la température avec un capteur I2C.
2.	Afficher la température sur un afficheur 7 segments qui communique via le SPI
3.	A partir d’une certaine température, il faudra déclencher le moteur (PWM) pour refroidir.
4.	Si la température est critique, il faudra faire sonner l’alarme (buzzer en PWM)
5.	Une gestion avec les 4 boutons contrôlera la consigne en température et le volume du buzzer.
6.	Les résultats pourrons être communiqué par UART vers le PC pour le retour d’information.
7.	Un bouton URGENCE forcera le démarrage du moteur.

## Interfaces
- [x] 1 I2C
- [x] 1 SPI
- [x] 1 timer avec 2 channels pour la génération de 2 PWM (avec IT???)
	* PWM pour le buzzer
	* PWM pour le moteur
- [x] 1 UART
- [x] (4 + 1 GPIO IT) + 1 GPIO pour le NSS du SPI = 6 GPIOs
## Pinout STM32-NUCLEOL152RE

![50% center](Documentation/Doc\ ST/nucleo64_revc_l152re_pinout.png)

## NOTES
Attention aux niveaux de priorités !!! 


