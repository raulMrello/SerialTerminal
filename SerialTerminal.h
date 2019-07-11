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
                   transmisión, fin de recepción, el fallo por timeout y el 
                   fallo por desborde de buffer de recepción. Utiliza un caracter 
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
     *  Enumeración para listar los diferentes modos de operación del receptor:
     *  ReceiveWithEofCharacter - Utiliza un caracter concreto como detección de fin de trama
     *  ReceiveWithDedicatedHandling - Utiliza una callback bool(uint8_t*,uint_16_t) para procesar
     *      cada byte recibido. Cuando la trama se haya completado, devolverá (true).
     */
    enum Receiver_mode{
        ReceiveWithEofCharacter,
        ReceiveWithDedicatedHandling,
        ReceiveAfterBreakTime,
    };
    
    /** SerialTerminal()
     *  Crea el objeto asignando un puerto serie para la interfaz con el equipo digital, un tamaño
     *  de buffer para recibir las tramas y la velocidad de transmisión
     *  @param tx Línea de transmisión
     *  @param rx Línea de recepción
     *  @param maxbufsize Tamaño del buffer de recepción (no se podrán recibir tramas de mayor tamaño)
     *  @param baud Velocidad del puerto serie
     *  @param mode Modo de detección de fin de trama (por defecto eof \0)
     */
    SerialTerminal(PinName tx, PinName rx, uint16_t maxbufsize = 256, int baud = MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE, Receiver_mode mode = ReceiveWithEofCharacter);

    /** config()
     *  Configura las callbacks, el timeout y el caracter de fin de trama
     *  @param rx_done Callback a invocar tras la recepción completa
     *  @param rx_timeout Callback a invocar tras un fallo por timeout
     *  @param rx_ovf Callback a invocar tras un fallo por overflow en el buffer de recepción
     *  @param us_timeout Tiempo en us para recibir la trama antes de notificar un error por timeout o por fin de trama
     *  @param eof Caracter de fin de trama (end_of_file)
     *  @return Flag para indicar si la configuración es correcta (True) o incorrecta (False)
     */
    bool config(Callback<void()> rx_done, Callback <void()> rx_timeout, Callback <void()> rx_ovf, uint32_t us_timeout, char eof = 0);

    /** dedicatedHandling()
     *  Configura la callbacks de procesamiento dedicado
     *  @param cb_proc Callback a invocar tras la recepción de cada byte, para su procesado
     */
    void dedicatedHandling(Callback<bool(uint8_t*,uint16_t)> cb_proc) {_cb_proc = cb_proc;}

    /** startReceiver()
     *  Habilita el receptor en modo isr-managed y por lo tanto lo deja listo para recibir
     *  datos en modo interrupción
     */
    void startReceiver(){ startManaged(false, true); }

    /** stopReceiver()
     *  Deshabilita el receptor en modo isr-managed y por lo tanto deja de recibir
     *  datos en modo interrupción
     */
    void stopReceiver(){ stopManaged(false, true); }

    /** startTransmitter()
     *  Habilita el transmisor en modo isr-managed y por lo tanto lo deja listo para transmitir
     *  datos en modo interrupción
     */
    void startTransmitter(){ startManaged(true, false); }

    /** stopReceiver()
     *  Deshabilita el transmisor en modo isr-managed y por lo tanto deja de enviar
     *  datos en modo interrupción
     */
    void stopTransmitter(){ stopManaged(true, false); }    

    /** busy()
     *  Informa si el transmisor está ocupado o no
     *  @return True: ocupado, False: listo para enviar
     */
    bool busy(){ return((_tosend > 0)? true : false);}    
    
    
    /** send()
     *  Prepara para una nueva transimisión gestionada por interrupciones. El final de transmisión 
     *  se notifica invocando la callback
     *  @param data Buffer de datos de origen
     *  @param size Tamaño del buffer a enviar
     *  @param cb_data_sent Callback a invocar al finalizar el envío
     *  @return Indica si la transferencia se ha iniciado (true) o no (false)
     */
    bool send(void* data, uint16_t size, Callback<void()> tx_done);    

    /** recv()
     *  Lee el contenido del buffer de recepción hasta un máximo de maxsize bytes
     *  @param buf Buffer de destino en el que copiar la trama recibida
     *  @param maxsize Tamaño del buffer de destino
     *  @param enable_receiver Flag para activar las interrupciones de recepción después
     *         de devolver los datos leídos. Por defecto, está activado.
     *  @return Número de bytes copiados
     */
    uint16_t recv(void* buf, uint16_t maxsize, bool enable_receiver = true);

    /** isTxManaged()
     *  Comprueba si el transmisor con gestión de interrupciones está habilitado
     *  @return estado del transmisor(true=habilitado)
     */
    bool isTxManaged();

    /** isRxManaged()
     *  Comprueba si el receptor con gestión de interrupciones está habilitado
     *  @return estado del receptor(true=habilitado)
     */
    bool isRxManaged();
    
protected:

    /** startManaged()
     *  Prepara el terminal para su funcionamiento, pudiendo preparar de forma independiente el transmisor
     *  y el receptor.
     *  @param transmitter Prepara el transmisor, habilitando las interrupciones de envío
     *  @param receiver Prepara el receptor, habilitando las interrupciones de recepción
     */
    void startManaged(bool transmitter, bool receiver);

    /** stopManaged()
     *  Desactiva el terminal, pudiendo desactivar de forma independiente el transmisor
     *  y el receptor.
     *  @param transmitter Desactiva el transmisor, deshabilitando las interrupciones de envío
     *  @param receiver Desactiva el receptor, deshabilitando las interrupciones de recepción
     */
    void stopManaged(bool transmitter, bool receiver);

    /** onTxData()
     *  Manejador ISR de datos enviados vía serie
     */
    void onTxData();

    /** onRxData()
     *  Manejador ISR de datos recibidos vía serie
     */
    void onRxData();

    /** onRxTimeout()
     *  Manejador ISR de timeout en la recepción serie
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
    uint16_t _recv;                 ///!< Posición de escritura en el buffer de recepción
    uint16_t _sent;                 ///!< Posición de lectura en el buffer de transmisión
    uint16_t _tosend;               ///!< Tamaño de la trama a enviar
    uint16_t _bufsize;              ///!< Tamaño del buffer de recepción
    char _eof;                      ///!< Caracter de fin de trama
    char* _databuf;                 ///!< Buffer de recepción (puntero)
    uint8_t* _tbuf;                 ///!< Puntero al buffer de transmisión
    Callback <void()> _cb_tx;       ///!< Callback para notificar trama enviada
    Callback <void()> _cb_rx;       ///!< Callback para notificar trama recibida
    Callback <void()> _cb_rx_tmr;   ///!< Callback para notificar error por timeout
    Callback <void()> _cb_rx_ovf;   ///!< Callback para notificar error por debordamiento de buffer
    Callback <bool(uint8_t*, uint16_t)> _cb_proc;   ///!< Callback para procesar los bytes recibidos
    bool tx_managed;                ///!< Flag de estado del transmisor en modo isr-managed
    bool rx_managed;                ///!< Flag de estado del receptor en modo isr-managed
    Receiver_mode _mode;            ///!< Modo de operación del receptor
};


#endif
