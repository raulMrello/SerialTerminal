# SerialTerminal

Orientado a su ejecución en MBED.

Clase que implementa un terminal serie para recibir y enviar comandos remotos . Proporciona callbacks para notificar fin de transmisión, fin de recepción, el fallo por timeout y el fallo por desborde de buffer de recepción. Utiliza un caracter para detectar el fin de trama recibido.

También permite finalizar la recepción tras un BREAK o IDLE en la línea de recepción.