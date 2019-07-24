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
		FLAG_RXERROR = 	(1<<6),	///< Flag para indicar un error en la recepción
    };

	/**--------------------------------------------------------------------------------------
	 * Tipos de detección de fin de trama
	 */
	enum EORMode{
		EORByIdleTime,	//!< Tras una temporización entre bytes recibidos (2.5 veces t_byte)
		EORByFlagSize,	//!< Tras cumplirse una condición de flags (HEAD, FOOT) y size
	};

	/**
	 * Estructura de control para la detección de fin de trama mediante flags y size. Por ejemplo
	 * para una trama que inicia con el flag 0x5A, finaliza con el flag 0xA5, el campo de tamaño está
	 * en la posición 3 y es tipo uint32_t (4 bytes) en little-endian, la estructura se rellenaría con los valores:
	 * EORFlagSize_t fs = {0x5a, 0xa5, 3, sizeof(uint32_t), true}
	 */
	struct EORFlagSize_t{
		uint8_t header;
		uint8_t footer;
		uint16_t pos_size;
		uint8_t len_size;
		bool little_endian;
		uint32_t max_size;
	};


    /**--------------------------------------------------------------------------------------
	 * Destructor
     */
    ISerial(){
    	_eor_mode = EORByIdleTime;
    	_total_size = 0;
    	_count = 0;
    	_size[0]=0;_size[1]=0;_size[2]=0;_size[3]=0;
    }


    /**--------------------------------------------------------------------------------------
	 * Destructor
     */
    virtual ~ISerial(){}


    /**--------------------------------------------------------------------------------------
	 * Método para configurar el modo EORByFlagSize
     * @param header Flag de inicio de trama
     * @param footer Flag de fin de trama
	 * @param pos_size Posición del tamaño
	 * @param len_size Tamaño de la variable que especifica el tamaño
	 * @param little_endian Endian del tamaño de trama
	 * @param max_size Máximo tamaño de la trama permitido
     */
    virtual void setModeEOR(uint8_t header, uint8_t footer, uint16_t pos_size, uint8_t len_size, bool little_endian, uint32_t max_size){
    	_eor_mode = EORByFlagSize;
    	_eor_fs_cfg.header = header;
    	_eor_fs_cfg.footer = footer;
    	_eor_fs_cfg.pos_size = pos_size;
    	_eor_fs_cfg.len_size = len_size;
    	_eor_fs_cfg.little_endian = little_endian;
    	_eor_fs_cfg.max_size = max_size;
    	_count = 0;
    }


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

protected:
    EORMode _eor_mode;
    EORFlagSize_t _eor_fs_cfg;
    uint32_t _count;
    uint8_t _size[4];
    uint32_t _total_size;

    /**
     * Posibles estados del análisis del detector EORFlagSize
     */
    enum AnalysisResult{
    	Wrong,
		Analyzing,
		Valid,
    };

    /**
     * Analiza el siguiente byte recibido, ejecutando la máquina de control del detector EORFlagSize
     * @param data Siguiente byte a analizar
     * @return Estado del análisis
     */
    virtual AnalysisResult _analyzeByte(uint8_t data){
    	if(_count == 0 && data == _eor_fs_cfg.header){
    		_count++;
    		_size[0]=0;_size[1]=0;_size[2]=0;_size[3]=0;
    		_total_size=0;
    		return Analyzing;
    	}
    	if(_count > 0){
    		if(_count >= _eor_fs_cfg.pos_size && _count < _eor_fs_cfg.pos_size + _eor_fs_cfg.len_size){
    			_size[_count - _eor_fs_cfg.pos_size]=data;
    			_count++;
    			if(_count == _eor_fs_cfg.pos_size + _eor_fs_cfg.len_size){
    				if(_eor_fs_cfg.little_endian){
    					switch(_eor_fs_cfg.len_size){
							case 1:
								_total_size = _size[0];
								break;
							case 2:
								_total_size = (((uint32_t)_size[0])<<8) + ((uint32_t)_size[1]);
								break;
							case 4:
								_total_size = (((uint32_t)_size[0])<<24) + (((uint32_t)_size[1])<<16) + (((uint32_t)_size[2])<<8) + ((uint32_t)_size[3]);
								break;
    					}
    				}
    				else{
    					switch(_eor_fs_cfg.len_size){
							case 1:
								_total_size = _size[0];
								break;
							case 2:
								_total_size = (((uint32_t)_size[1])<<8) + ((uint32_t)_size[0]);
								break;
							case 4:
								_total_size = (((uint32_t)_size[3])<<24) + (((uint32_t)_size[2])<<16) + (((uint32_t)_size[1])<<8) + ((uint32_t)_size[0]);
								break;
    					}
					}
    				if(_total_size > _eor_fs_cfg.max_size){
    					_count = 0;
    					return Wrong;
    				}
    			}
    			return Analyzing;
    		}
    		if(_count < _total_size){
    			_count++;
    			if(_count == _total_size){
    				_count = 0; _total_size = 0;
					if(data == _eor_fs_cfg.footer){
						return Valid;
					}
					return Wrong;
				}
    			return Analyzing;
    		}
    	}
    	_count = 0; _total_size = 0;
    	return Wrong;
    }

};


#endif
