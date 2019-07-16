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



/** \class SerialMon
 *  \brief Este componente permite gestionar comunicaciones serie, proporcionando buffers
 *         de env�o y recepci�n. Se ejecuta en su propio hilo, utiliza callbacks de notificaci�n
 */

class SerialMon : public ISerial {
public:

    /**--------------------------------------------------------------------------------------
     * Constructor
     * @param tx PinName TX pin.
     * @param rx PinName RX pin.
     * @param txBufferSize Tama�o buffer env�o
     * @param rxBufferSize Tama�o buffer recepci�n
     * @param baud Baudrate
     * @param name Nombre del objeto
     */    
    SerialMon(PinName tx, PinName rx, int txBufferSize, int rxBufferSize, int baud, const char* name = (const char*)"NO NAME");
 
    
    /**--------------------------------------------------------------------------------------
     * Destructor
     */
    virtual ~SerialMon();


    /**--------------------------------------------------------------------------------------
     * Actualiza el nivel de visualizaci�n de las tramas de depuraci�n
     */
    void setLoggingLevel(esp_log_level_t level);


	/**--------------------------------------------------------------------------------------
	 * Inicia la ejecuci�n de su hilo de control
	 * @param priority Prioridad del thread
	 * @param stack_size Tama�o de pila asociada
     */
    void start(osPriority priority=osPriorityNormal, uint32_t stack_size = OS_STACK_SIZE);


    /**--------------------------------------------------------------------------------------
	 * [ISerial] M�todo para registrar diferentes callbacks relativas a la comunicaci�n
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    virtual void attachRxCallback(Callback<void(uint8_t* , int , Flags)> cb);


    /**--------------------------------------------------------------------------------------
	 * [ISerial] M�todo para registrar diferentes callbacks relativas a la comunicaci�n
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    virtual void attachTxCallback(Callback<void(Flags)> cb);

	
    /**--------------------------------------------------------------------------------------
     * [ISerial] M�todo para enviar un buffer de datos con un tama�o concreto
     * @param data buffer a enviar
	 * @param size tama�o del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    virtual int send(uint8_t *data, int size);


    /**--------------------------------------------------------------------------------------
     * [ISerial] Idem que el anterior, pero bloqueante hasta que no se detecta el flag EOT
     * @param data buffer a enviar
	 * @param size tama�o del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    virtual int sendComplete(uint8_t *data, int size);


protected:
    
    /**--------------------------------------------------------------------------------------
     * Estructura de datos del buffer utilizado en transmisi�n y recepci�n
     */
     typedef struct {
        char* mem;			///< Puntero a la memoria reservada al buffer
        char* limit;		///< Ultima posici�n del buffer
        char* in;			///< Posici�n de escritura en el buffer
        char* ou;			///< Posici�n de lectura del buffer
        int sz;				///< Tama�o del buffer
    }buffer_t;


    /** Puerto serie asociado */
    RawSerial *_serial;

    /** Timer y variable asociada para notificaci�n de eventos de trama completa */
    Ticker _rx_tick;
    uint32_t _us_timeout;

    /** Mutex para el control de acceso a la transmisi�n */
    Mutex _mtx;

    /** Sem�foro para operaciones bloqueantes */
    Semaphore* _sem;

    /** Controlador del hilo de ejecuci�n asociado */
    Thread* _th;

    /** Flag de estado */
	int _stat;

	/** Nombre asociado al thread */
    const char * _name;

    /** Buffers de transmisi�n y recepci�n */
    buffer_t _txbuf;
    buffer_t _rxbuf;

    /** Callbacks para notificaci�n de eventos */
    Callback<void(uint8_t* data, int size, Flags flags)> _cb_rx;
    Callback<void(Flags flags)> _cb_tx;


    /**--------------------------------------------------------------------------------------
     *	Hilo de ejecuci�n
     */
	void _task();


    /**--------------------------------------------------------------------------------------
     * Rutina para determinar si hay datos pendientes por leer en el buffer de recepci�n
     * @return true si hay datos, false en caso contrario
     */
    bool _readable() { return (((_rxbuf.in != _rxbuf.ou) || (_rxbuf.in == _rxbuf.ou && (_stat & FLAG_RXFULL)!=0))? true : false); }


    /**--------------------------------------------------------------------------------------
     * M�todo para leer los datos recibidos del buffer de recepci�n
	 * @param len Recibe el tama�o de los datos
     * @return Puntero al buffer creado. Se debe liberar con delete(buffer);
     */
    uint8_t* _read(int& len);


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de recepci�n del puerto serie
     */
    void _rxCallback();


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de transmisi�n del puerto serie
     */
    void _txCallback();


    /**--------------------------------------------------------------------------------------
     * Callback propia para manejar las interrupciones de timeout en recepci�n
     */
    void _timeoutCallback();

};


#endif
