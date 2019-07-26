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
 *  \brief Interfaz que proporciona servicios b�sicos para controlar las comunicaciones de un puerto
 *  serie.
 */

class ISerial {
public:

    /**--------------------------------------------------------------------------------------
     * Flags de se�alizaci�n de estados
     */
    enum Flags {
        FLAG_EOT = 			(1<<0), ///< Flag de fin de transmisi�n
		FLAG_EOR = 			(1<<1),	///< Flag de fin de recepci�n
		FLAG_RECV =			(1<<2),	///< Flag de byte recibido
		FLAG_RTIMED =   	(1<<3),	///< Flag para indicar que ha ocurrido timeout en recepci�n
		FLAG_SENDING = 		(1<<4),	///< Flag para indicar que hay un proceso de env�o en marcha
		FLAG_RXSTARTED = 	(1<<5),	///< Flag para indicar que hay un proceso de recepci�n en marcha
		FLAG_RXFULL =		(1<<6),	///< Flag para indicar que el buffer de recepci�n est� lleno
		FLAG_TXFULL =  		(1<<7),	///< Flag para indicar que el buffer de transmisi�n est� lleno
		FLAG_RXERROR = 		(1<<8),	///< Flag para indicar un error en la recepci�n
    };

	/**--------------------------------------------------------------------------------------
	 * Tipos de detecci�n de fin de trama
	 */
	enum EORMode{
		EORByIdleTime,	//!< Tras una detecci�n de IDLE
		EORByTicker,	//!< Tras una temporizaci�n del ticker
		EORByFlagSize,	//!< Tras cumplirse una condici�n de flags (HEAD, FOOT) y size
		EORContinuous,	//!< No hay detecci�n de fin de trama ya que �stas se pueden concatenar
	};

	/**
	 * Estructura de control para la detecci�n de fin de trama mediante flags y size. Por ejemplo
	 * para una trama que inicia con el flag 0x5A, finaliza con el flag 0xA5, el campo de tama�o est�
	 * en la posici�n 3 y es tipo uint32_t (4 bytes) en little-endian, la estructura se rellenar�a con los valores:
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
	 * Constructor
	 * @param t_bytes Tiempo del n� de bytes para notificar un fin de recepci�n
     */
    ISerial(){
    	_t_bytes = 0;
    	_eor_mode = EORContinuous;
    	_is_ready = false;
    }


    /**--------------------------------------------------------------------------------------
	 * Destructor
     */
    virtual ~ISerial(){}


    /**--------------------------------------------------------------------------------------
	 * M�todo para configurar el modo EORByTicker
     * @param t_bytes Tiempo del n� de bytes para notificar un fin de recepci�n
     */
    virtual void setEORByTicker(float t_bytes){
    	_eor_mode = EORByTicker;
    	_t_bytes = t_bytes;
    }


    /**--------------------------------------------------------------------------------------
	 * M�todo para configurar el modo de an�lisis de la trama recibida
     * @param header Flag de inicio de trama
     * @param footer Flag de fin de trama
	 * @param pos_size Posici�n del tama�o
	 * @param len_size Tama�o de la variable que especifica el tama�o
	 * @param little_endian Endian del tama�o de trama
	 * @param max_size M�ximo tama�o de la trama permitido
     */
    virtual void cfgStreamAnalyzer(uint8_t header, uint8_t footer, uint16_t pos_size, uint8_t len_size, bool little_endian, uint32_t max_size){
    	_eor_fs_cfg.header = header;
    	_eor_fs_cfg.footer = footer;
    	_eor_fs_cfg.pos_size = pos_size;
    	_eor_fs_cfg.len_size = len_size;
    	_eor_fs_cfg.little_endian = little_endian;
    	_eor_fs_cfg.max_size = max_size;
    }


    /**--------------------------------------------------------------------------------------
	 * M�todo para registrar diferentes callbacks relativas a la comunicaci�n
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    virtual void attachRxCallback(Callback<void(uint8_t* , int , Flags)> cb) = 0;


    /**--------------------------------------------------------------------------------------
	 * M�todo para registrar diferentes callbacks relativas a la comunicaci�n
     * @param type Tipo de callback a registrar
	 * @param cb Callback
     */
    virtual void attachTxCallback(Callback<void(Flags)> cb) = 0;

	
    /**--------------------------------------------------------------------------------------
     * M�todo para enviar un buffer de datos con un tama�o concreto
     * @param data buffer a enviar
	 * @param size tama�o del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    virtual int send(uint8_t *data, int size) = 0;


    /**--------------------------------------------------------------------------------------
     * Idem que el anterior, pero bloqueante hasta que no se detecta el flag EOT
     * @param data buffer a enviar
	 * @param size tama�o del buffer a enviar
	 * @return Datos enviados (en caso de error <0)
     */
    virtual int sendComplete(uint8_t *data, int size) = 0;


    /**--------------------------------------------------------------------------------------
     * Chequea si est� listo
	 * @return True o False
     */
    virtual bool isReady() {return _is_ready;}

protected:
    EORMode _eor_mode;
    EORFlagSize_t _eor_fs_cfg;
    float _t_bytes;
    bool _is_ready;

    /**
     * Estructura de control para el an�lisis EORFlagSize
     */
    struct AnalysisCtrl{
    	uint32_t count;
    	uint8_t size[4];
    	uint32_t total_size;
    };

    /**
     * Posibles estados del an�lisis del detector EORFlagSize
     */
    enum AnalysisResult{
    	Discarded,
    	Start,
    	Wrong,
		Analyzing,
		Valid,
    };

    /**
     * Analiza el siguiente byte recibido, ejecutando la m�quina de control del detector EORFlagSize
     * @param data Byte a analizar
     * @return Estado del an�lisis
     */
    virtual AnalysisResult _analyzeByte(uint8_t data, AnalysisCtrl &ac){

    	if(ac.count == 0 && data == _eor_fs_cfg.header){
    		ac.count++;
    		ac.size[0]=0;ac.size[1]=0;ac.size[2]=0;ac.size[3]=0;
    		ac.total_size=0;
    		return Start;
    	}
    	if(ac.count > 0){
    		if(ac.count >= _eor_fs_cfg.pos_size && ac.count < _eor_fs_cfg.pos_size + _eor_fs_cfg.len_size){
    			ac.size[ac.count - _eor_fs_cfg.pos_size]=data;
    			ac.count++;
    			if(ac.count == _eor_fs_cfg.pos_size + _eor_fs_cfg.len_size){
    				if(_eor_fs_cfg.little_endian){
    					switch(_eor_fs_cfg.len_size){
							case 1:
								ac.total_size = ac.size[0];
								break;
							case 2:
								ac.total_size = (((uint32_t)ac.size[0])<<8) + ((uint32_t)ac.size[1]);
								break;
							case 4:
								ac.total_size = (((uint32_t)ac.size[0])<<24) + (((uint32_t)ac.size[1])<<16) + (((uint32_t)ac.size[2])<<8) + ((uint32_t)ac.size[3]);
								break;
    					}
    				}
    				else{
    					switch(_eor_fs_cfg.len_size){
							case 1:
								ac.total_size = ac.size[0];
								break;
							case 2:
								ac.total_size = (((uint32_t)ac.size[1])<<8) + ((uint32_t)ac.size[0]);
								break;
							case 4:
								ac.total_size = (((uint32_t)ac.size[3])<<24) + (((uint32_t)ac.size[2])<<16) + (((uint32_t)ac.size[1])<<8) + ((uint32_t)ac.size[0]);
								break;
    					}
					}
    				if(ac.total_size > _eor_fs_cfg.max_size){
    					ac.count = 0;
    					return Wrong;
    				}
    			}
    			return Analyzing;
    		}
    		if(ac.count < ac.total_size){
    			ac.count++;
    			if(ac.count == ac.total_size){
    				ac.count = 0; ac.total_size = 0;
					if(data == _eor_fs_cfg.footer){
						return Valid;
					}
					return Wrong;
				}
    			return Analyzing;
    		}
    	}
    	ac.count = 0; ac.total_size = 0;
    	return Discarded;
    }

};


#endif
