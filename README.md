# SerialTerminal

Preparado para MBED-OS, proporciona el control de un puerto serie avanzado, con las siguientes clases:

* ISerial: es un interfaz que proporciona diferentes flags de estado, así como varios modos de funcionamiento, como:
	- Detección de fín de trama en la detección del flag IDLE
	- Detección de fín de trama mediante un temporizador Ticker
	- Sin detección de fín de trama, utilizando un analizador de tramas (StreamAnalyzer) para tramas con formato |header|size|data...|footer|

* SerialMon: es una implementación para MBED-OS del interfaz ISerial por defecto sin detección de fín de trama. Proporciona lo siguiente:
	- Thread: hilo de control propio para notificar tramas enviadas y recibidas, así como otros errores (buffers llenos, vacíos, etc...)
	- Callbacks: permite registrar callbacks para notificar los eventos procesados en el thread propio.
	- Buffers TX,RX: integra buffers de envío y recepción