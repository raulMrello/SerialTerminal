/*
    Copyright (c) 2016 raulMrello
 
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
 
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
 
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
    
    @file          SerialMon.h 
    @purpose       Monitor Serie tx/rx con buffer
    @version       see ChangeLog.c
    @date          Nov 2016
    @author        raulMrello
*/

#ifndef SERIALMON_H
#define SERIALMON_H


#include "mbed.h"



/** \class SerialMon
 *  \brief Este componente permite gestionar comunicaciones serie, proporcionando buffers
 *         de envío y recepción. Se ejecuta en su propio hilo, utiliza callbacks de notificación
 */

class SerialMon {
public:

    /**--------------------------------------------------------------------------------------
     * Constructor
     * @param tx PinName TX pin.
     * @param rx PinName RX pin.
     * @param txBufferSize Tamaño buffer envío
     * @param rxBufferSize Tamaño buffer recepción
     * @param baud Baudrate
     * @param name Nombre del objeto
     */    
    SerialMon(PinName tx, PinName rx, int txBufferSize, int rxBufferSize, int baud, const char* name = (const char*)"NO NAME");
 
    
    /**--------------------------------------------------------------------------------------
     * Destructor
     */
    virtual ~SerialMon();


	/**--------------------------------------------------------------------------------------
	 * Inicia la ejecución de su hilo de control
	 * @param priority Prioridad del thread
	 * @param stack_size Tamaño de pila asociada
     */
    void start(osPriority priority=osPriorityNormal, uint32_t stack_size = OS_STACK_SIZE);


    /**--------------------------------------------------------------------------------------
     * Flags de señalización de estados
     */
    enum Flags {
        FLAG_EOT = 		(1<<0), ///< Flag de fin de transmisión
		FLAG_EOR = 		(1<<1),	///< Flag de fin de recepción
		FLAG_RTIMED =   (1<<2),	///< Flag para indicar que ha ocurrido timeout en recepción
		FLAG_SENDING = 	(1<<3),	///< Flag para indicar que hay un proceso de envío en marcha
		FLAG_RXFULL =	(1<<4),	///< Flag para indicar que el buffer de recepción está lleno
		FLAG_RXFULL =  	(1<<5),	///< Flag para indicar que el buffer de transmisión está lleno
		FLAG_STARTED = 	(1<<6),	///< Flag para indicar que el objeto está iniciado
    };

    /**--------------------------------------------------------------------------------------
     * Tipos de callback instalables
     */
    enum CallbackType{
    	CbRx,		//!< Callback para procesar eventos de recepción
		CbTx,		//!< Callbacks para procesar eventos de transmisión
    };


    /**--------------------------------------------------------------------------------------
	 * Método para registrar diferentes callbacks relativas a la comunicación
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    void attachRxCallback(Callback<void(uint8_t* , int , Flags)>& cb);


    /**--------------------------------------------------------------------------------------
	 * Método para registrar diferentes callbacks relativas a la comunicación
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    void attachTxCallback(Callback<void(Flags)>& cb);

	
    /**--------------------------------------------------------------------------------------
     * Método para enviar un buffer de datos con un tamaño concreto
     * @param data buffer a enviar
	 * @param size tamaño del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    int send(uint8_t *data, int size);


    /**--------------------------------------------------------------------------------------
     * Idem que el anterior, pero bloqueante hasta que no se detecta el flag EOT
     * @param data buffer a enviar
	 * @param size tamaño del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    int sendComplete(uint8_t *data, int size);

    
    /**--------------------------------------------------------------------------------------
     * Estructura de datos por defecto para los topics aceptados por este componente en el
	 * mecanismo pub-sub
     */
    typedef struct {
        uint8_t * data;
		int size;
    }topic_t;


protected:
    
    /**--------------------------------------------------------------------------------------
     * Estructura de datos del buffer utilizado en transmisión y recepción
     */
     typedef struct {
        char* mem;			///< Puntero a la memoria reservada al buffer
        char* limit;		///< Ultima posición del buffer
        char* in;			///< Posición de escritura en el buffer
        char* ou;			///< Posición de lectura del buffer
        int sz;				///< Tamaño del buffer
    }buffer_t;


    /** Puerto serie asociado */
    RawSerial *_serial;

    /** Timer y variable asociada para notificación de eventos de trama completa */
    Ticker _rx_tick;
    uint32_t _us_timeout;

    /** Mutex para el control de acceso a la transmisión */
    Mutex _mtx;

    /** Semáforo para operaciones bloqueantes */
    Semaphore* _sem;

    /** Controlador del hilo de ejecución asociado */
    Thread* _th;

    /** Flag de estado */
	int _stat;

	/** Nombre asociado al thread */
    char * _name;

    /** Buffers de transmisión y recepción */
    buffer_t _txbuf;
    buffer_t _rxbuf;

    /** Callbacks para notificación de eventos */
    Callback<void(uint8_t* data, int size, Flags flags)> _cb_rx;
    Callback<void(Flags flags)> _cb_tx;


    /**--------------------------------------------------------------------------------------
     *	Hilo de ejecución
     */
	void _task();


    /**--------------------------------------------------------------------------------------
     * Rutina para determinar si hay datos pendientes por leer en el buffer de recepción
     * @return true si hay datos, false en caso contrario
     */
    bool _readable() { return (((_rxbuf.in != _rxbuf.ou) || (_rxbuf.in == _rxbuf.ou && (_stat & FLAG_RXFULL)!=0))? true : false); }


    /**--------------------------------------------------------------------------------------
     * Método para leer los datos recibidos del buffer de recepción
	 * @param len Recibe el tamaño de los datos
     * @return Puntero al buffer creado. Se debe liberar con delete(buffer);
     */
    uint8_t* _read(int& len);


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de recepción del puerto serie
     */
    void _rxCallback();


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de transmisión del puerto serie
     */
    void _txCallback();


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de timeout en recepción
     */
    void _timeoutCallback();

};


#endif
