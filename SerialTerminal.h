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
    
    @file          SerialTerminal.h 
    @purpose       Clase que implementa un terminal serie para recibir y enviar
                   comandos remotos . Proporciona callbacks para notificar fin de
                   transmisi�n, fin de recepci�n, el fallo por timeout y el 
                   fallo por desborde de buffer de recepci�n. Utiliza un caracter 
                   para detectar el fin de trama recibido.
    @date          Jul 2017
    @author        raulMrello
    @version       1.0.0-30.06.2017
*/

#ifndef SERIALTERMINAL_H
#define SERIALTERMINAL_H

/** Archivos de cabecera */
#include "mbed.h"


class SerialTerminal : public RawSerial {

public:
    /** Receiver_mode
     *  Enumeraci�n para listar los diferentes modos de operaci�n del receptor:
     *  ReceiveWithEofCharacter - Utiliza un caracter concreto como detecci�n de fin de trama
     *  ReceiveWithDedicatedHandling - Utiliza una callback bool(uint8_t*,uint_16_t) para procesar
     *      cada byte recibido. Cuando la trama se haya completado, devolver� (true).
     */
    enum Receiver_mode{
        ReceiveWithEofCharacter,
        ReceiveWithDedicatedHandling,
        ReceiveAfterBreakTime,
    };
    
    /** SerialTerminal()
     *  Crea el objeto asignando un puerto serie para la interfaz con el equipo digital, un tama�o
     *  de buffer para recibir las tramas y la velocidad de transmisi�n
     *  @param tx L�nea de transmisi�n
     *  @param rx L�nea de recepci�n
     *  @param maxbufsize Tama�o del buffer de recepci�n (no se podr�n recibir tramas de mayor tama�o)
     *  @param baud Velocidad del puerto serie
     *  @param mode Modo de detecci�n de fin de trama (por defecto eof \0)
     */
    SerialTerminal(PinName tx, PinName rx, uint16_t maxbufsize = 256, int baud = MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE, Receiver_mode mode = ReceiveWithEofCharacter);

    /** config()
     *  Configura las callbacks, el timeout y el caracter de fin de trama
     *  @param rx_done Callback a invocar tras la recepci�n completa
     *  @param rx_timeout Callback a invocar tras un fallo por timeout
     *  @param rx_ovf Callback a invocar tras un fallo por overflow en el buffer de recepci�n
     *  @param us_timeout Tiempo en us para recibir la trama antes de notificar un error por timeout o por fin de trama
     *  @param eof Caracter de fin de trama (end_of_file)
     *  @return Flag para indicar si la configuraci�n es correcta (True) o incorrecta (False)
     */
    bool config(Callback<void()> rx_done, Callback <void()> rx_timeout, Callback <void()> rx_ovf, uint32_t us_timeout, char eof = 0);

    /** dedicatedHandling()
     *  Configura la callbacks de procesamiento dedicado
     *  @param cb_proc Callback a invocar tras la recepci�n de cada byte, para su procesado
     */
    void dedicatedHandling(Callback<bool(uint8_t*,uint16_t)> cb_proc) {_cb_proc = cb_proc;}

    /** startReceiver()
     *  Habilita el receptor en modo isr-managed y por lo tanto lo deja listo para recibir
     *  datos en modo interrupci�n
     */
    void startReceiver(){ startManaged(false, true); }

    /** stopReceiver()
     *  Deshabilita el receptor en modo isr-managed y por lo tanto deja de recibir
     *  datos en modo interrupci�n
     */
    void stopReceiver(){ stopManaged(false, true); }

    /** startTransmitter()
     *  Habilita el transmisor en modo isr-managed y por lo tanto lo deja listo para transmitir
     *  datos en modo interrupci�n
     */
    void startTransmitter(){ startManaged(true, false); }

    /** stopReceiver()
     *  Deshabilita el transmisor en modo isr-managed y por lo tanto deja de enviar
     *  datos en modo interrupci�n
     */
    void stopTransmitter(){ stopManaged(true, false); }    

    /** busy()
     *  Informa si el transmisor est� ocupado o no
     *  @return True: ocupado, False: listo para enviar
     */
    bool busy(){ return((_tosend > 0)? true : false);}    
    
    
    /** send()
     *  Prepara para una nueva transimisi�n gestionada por interrupciones. El final de transmisi�n 
     *  se notifica invocando la callback
     *  @param data Buffer de datos de origen
     *  @param size Tama�o del buffer a enviar
     *  @param cb_data_sent Callback a invocar al finalizar el env�o
     *  @return Indica si la transferencia se ha iniciado (true) o no (false)
     */
    bool send(void* data, uint16_t size, Callback<void()> tx_done);    

    /** recv()
     *  Lee el contenido del buffer de recepci�n hasta un m�ximo de maxsize bytes
     *  @param buf Buffer de destino en el que copiar la trama recibida
     *  @param maxsize Tama�o del buffer de destino
     *  @param enable_receiver Flag para activar las interrupciones de recepci�n despu�s
     *         de devolver los datos le�dos. Por defecto, est� activado.
     *  @return N�mero de bytes copiados
     */
    uint16_t recv(void* buf, uint16_t maxsize, bool enable_receiver = true);

    /** isTxManaged()
     *  Comprueba si el transmisor con gesti�n de interrupciones est� habilitado
     *  @return estado del transmisor(true=habilitado)
     */
    bool isTxManaged();

    /** isRxManaged()
     *  Comprueba si el receptor con gesti�n de interrupciones est� habilitado
     *  @return estado del receptor(true=habilitado)
     */
    bool isRxManaged();
    
protected:

    /** startManaged()
     *  Prepara el terminal para su funcionamiento, pudiendo preparar de forma independiente el transmisor
     *  y el receptor.
     *  @param transmitter Prepara el transmisor, habilitando las interrupciones de env�o
     *  @param receiver Prepara el receptor, habilitando las interrupciones de recepci�n
     */
    void startManaged(bool transmitter, bool receiver);

    /** stopManaged()
     *  Desactiva el terminal, pudiendo desactivar de forma independiente el transmisor
     *  y el receptor.
     *  @param transmitter Desactiva el transmisor, deshabilitando las interrupciones de env�o
     *  @param receiver Desactiva el receptor, deshabilitando las interrupciones de recepci�n
     */
    void stopManaged(bool transmitter, bool receiver);

    /** onTxData()
     *  Manejador ISR de datos enviados v�a serie
     */
    void onTxData();

    /** onRxData()
     *  Manejador ISR de datos recibidos v�a serie
     */
    void onRxData();

    /** onRxTimeout()
     *  Manejador ISR de timeout en la recepci�n serie
     */
    void onRxTimeout();

    /** Acquire exclusive access to this serial port
     */
    virtual void lock(void){
        _mtx.lock();
    }

    /** Release exclusive access to this serial port
     */
    virtual void unlock(void){
        _mtx.unlock();
    }
    
    Mutex _mtx;
    Ticker _tmr;                    ///!< Objeto Timer
    uint32_t _us_timeout;           ///!< Timeout en microsegundos
    uint16_t _recv;                 ///!< Posici�n de escritura en el buffer de recepci�n
    uint16_t _sent;                 ///!< Posici�n de lectura en el buffer de transmisi�n
    uint16_t _tosend;               ///!< Tama�o de la trama a enviar
    uint16_t _bufsize;              ///!< Tama�o del buffer de recepci�n
    char _eof;                      ///!< Caracter de fin de trama
    char* _databuf;                 ///!< Buffer de recepci�n (puntero)
    uint8_t* _tbuf;                 ///!< Puntero al buffer de transmisi�n
    Callback <void()> _cb_tx;       ///!< Callback para notificar trama enviada
    Callback <void()> _cb_rx;       ///!< Callback para notificar trama recibida
    Callback <void()> _cb_rx_tmr;   ///!< Callback para notificar error por timeout
    Callback <void()> _cb_rx_ovf;   ///!< Callback para notificar error por debordamiento de buffer
    Callback <bool(uint8_t*, uint16_t)> _cb_proc;   ///!< Callback para procesar los bytes recibidos
    bool tx_managed;                ///!< Flag de estado del transmisor en modo isr-managed
    bool rx_managed;                ///!< Flag de estado del receptor en modo isr-managed
    Receiver_mode _mode;            ///!< Modo de operaci�n del receptor
};


#endif
