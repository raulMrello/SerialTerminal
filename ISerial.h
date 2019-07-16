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
    
    @file          ISerial.h
    @purpose       Interfaz serie
    @version       see ChangeLog.c
    @date          Jul 19
    @author        raulMrello
*/

#ifndef ISerial_H
#define ISerial_H


#include "mbed.h"



/** \interface ISerial
 *  \brief Interfaz que proporciona servicios básicos para controlar las comunicaciones de un puerto
 *  serie.
 */

class ISerial {
public:

    /**--------------------------------------------------------------------------------------
     * Flags de señalización de estados
     */
    enum Flags {
        FLAG_EOT = 		(1<<0), ///< Flag de fin de transmisión
		FLAG_EOR = 		(1<<1),	///< Flag de fin de recepción
		FLAG_RTIMED =   (1<<2),	///< Flag para indicar que ha ocurrido timeout en recepción
		FLAG_SENDING = 	(1<<3),	///< Flag para indicar que hay un proceso de envío en marcha
		FLAG_RXFULL =	(1<<4),	///< Flag para indicar que el buffer de recepción está lleno
		FLAG_TXFULL =  	(1<<5),	///< Flag para indicar que el buffer de transmisión está lleno
		FLAG_STARTED = 	(1<<6),	///< Flag para indicar que el objeto está iniciado
    };


    /**--------------------------------------------------------------------------------------
	 * Destructor
     */
    virtual ~ISerial(){}


    /**--------------------------------------------------------------------------------------
	 * Método para registrar diferentes callbacks relativas a la comunicación
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    virtual void attachRxCallback(Callback<void(uint8_t* , int , Flags)> cb) = 0;


    /**--------------------------------------------------------------------------------------
	 * Método para registrar diferentes callbacks relativas a la comunicación
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    virtual void attachTxCallback(Callback<void(Flags)> cb) = 0;

	
    /**--------------------------------------------------------------------------------------
     * Método para enviar un buffer de datos con un tamaño concreto
     * @param data buffer a enviar
	 * @param size tamaño del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    virtual int send(uint8_t *data, int size) = 0;


    /**--------------------------------------------------------------------------------------
     * Idem que el anterior, pero bloqueante hasta que no se detecta el flag EOT
     * @param data buffer a enviar
	 * @param size tamaño del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    virtual int sendComplete(uint8_t *data, int size) = 0;

};


#endif
