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
#include "ISerial.h"
#include "mdf_api_cortex.h"
#include "cpp_utils.h"



/** \class SerialMon
 *  \brief Este componente permite gestionar comunicaciones serie, proporcionando buffers
 *         de envío y recepción. Se ejecuta en su propio hilo, utiliza callbacks de notificación
 */

class SerialMon : public ISerial {
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
     * Actualiza el nivel de visualización de las tramas de depuración
     */
    void setLoggingLevel(esp_log_level_t level);


	/**--------------------------------------------------------------------------------------
	 * Inicia la ejecución de su hilo de control
	 * @param max_msg_size Tamaño máximo de la trama de recepción
	 * @param priority Prioridad del thread
	 * @param stack_size Tamaño de pila asociada
     */
    void start(uint32_t max_msg_size, osPriority priority=osPriorityNormal, uint32_t stack_size = OS_STACK_SIZE);


    /**--------------------------------------------------------------------------------------
	 * [ISerial] Método para registrar diferentes callbacks relativas a la comunicación
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    virtual void attachRxCallback(Callback<void(uint8_t* , int , Flags)> cb);


    /**--------------------------------------------------------------------------------------
	 * [ISerial] Método para registrar diferentes callbacks relativas a la comunicación
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    virtual void attachTxCallback(Callback<void(Flags)> cb);

	
    /**--------------------------------------------------------------------------------------
     * [ISerial] Método para enviar un buffer de datos con un tamaño concreto
     * @param data buffer a enviar
	 * @param size tamaño del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    virtual int send(uint8_t *data, int size);


    /**--------------------------------------------------------------------------------------
     * [ISerial] Idem que el anterior, pero bloqueante hasta que no se detecta el flag EOT
     * @param data buffer a enviar
	 * @param size tamaño del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    virtual int sendComplete(uint8_t *data, int size);


protected:
    
    /**--------------------------------------------------------------------------------------
     * Estructura de datos del buffer utilizado en transmisión y recepción
     */
     typedef struct {
    	 uint8_t* mem;			///< Puntero a la memoria reservada al buffer
    	 uint8_t* limit;		///< Ultima posición del buffer
    	 uint8_t* in;			///< Posición de escritura en el buffer
    	 uint8_t* ou;			///< Posición de lectura del buffer
    	 uint32_t sz;			///< Tamaño del buffer
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
    const char * _name;

    /** Buffers de transmisión y recepción */
    buffer_t _txbuf;
    buffer_t _rxbuf;

    /** Callbacks para notificación de eventos */
    Callback<void(uint8_t* data, int size, Flags flags)> _cb_rx;
    Callback<void(Flags flags)> _cb_tx;

    /** Controladores del análisis de datos en isr y en tarea */
    ISerial::AnalysisCtrl ac_isr;
    ISerial::AnalysisCtrl ac_task;

    /** Variables de control del procesado de la trama en curso */
    uint32_t _max_msg_size;
    uint8_t* _curr_rx_msg;
    uint8_t* _curr_rx_msg_start;


    /**--------------------------------------------------------------------------------------
     *	Hilo de ejecución
     */
	void _task();


    /**--------------------------------------------------------------------------------------
     * Lee el buffer de recepción, analizándolo y notificando a la callback _cb_rx en caso de
     * encontrar una trama válida. Termina cuando no quedan más bytes que procesar.
     */
    void _read();


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de recepción del puerto serie
     */
    void _rxCallback();


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones IDLE del puerto serie
     */
    void _rxIdleCallback();


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de transmisión del puerto serie
     */
    void _txCallback();


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de timeout en recepción
     */
    void _timeoutCallback();


    /**--------------------------------------------------------------------------------------
     * Manejadores por defecto de las callbacks no instaladas en la notificación de eventos
     */
    void _defaultRxUnhandledCallback(uint8_t* data, int size, Flags flag){}
    void _defaultTxUnhandledCallback(Flags flag){}
};


#endif
