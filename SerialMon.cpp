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
    @purpose       Extends Serial to provide fully buffered IO
    @version       see ChangeLog.c
    @date          Nov 2016
    @author        raulMrello
*/

#include "SerialMon.h"

static const char* _MODULE_ = "[SerialMon].....";
#define _EXPR_	(!IS_ISR())


//--------------------------------------------------------------------------------------------------------------
//-- PUBLIC METHODS --------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------
SerialMon::SerialMon(PinName tx, PinName rx, int txBufferSize, int rxBufferSize, int baud, const char* name) : _name(name), ISerial() {
	
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Iniciando SerialMon: %s", name);

	// inicializa buffers
    _txbuf.mem = new uint8_t[txBufferSize]();
    MBED_ASSERT(_txbuf.mem);
	_txbuf.limit = (uint8_t*)&_txbuf.mem[txBufferSize];
	_txbuf.in = _txbuf.mem;
	_txbuf.ou = _txbuf.in;
	_txbuf.sz = txBufferSize;
	memset(_txbuf.mem, 0, txBufferSize);

    _rxbuf.mem = new uint8_t[rxBufferSize]();
    MBED_ASSERT(_rxbuf.mem);
	_rxbuf.limit = (uint8_t*)&_rxbuf.mem[rxBufferSize];
	_rxbuf.in = _rxbuf.mem;
	_rxbuf.ou = _rxbuf.in;
	_rxbuf.sz = rxBufferSize;
	memset(_rxbuf.mem, 0, rxBufferSize);

	/** Inicializa callbacks */
    _cb_rx = callback(this, &SerialMon::_defaultRxUnhandledCallback);
    _cb_tx = callback(this, &SerialMon::_defaultTxUnhandledCallback);

    /** Inicializa controladores de análisis de trama */
	ac_isr = {0};
	ac_task = {0};
	_max_msg_size = 0;
	_curr_rx_msg = NULL;
	_curr_rx_msg_start = NULL;
	
	// inicia el dispositivo serie,
    _serial = new RawSerial(tx, rx, baud);
    MBED_ASSERT(_serial);

    // desconecta las interrupciones serie
    _serial->attach(0, (SerialBase::IrqType)TxIrq);
    _serial->attach(0, (SerialBase::IrqType)RxIrq);

    // establece el timeout para detección de fin de trama (IDLE por timeout software x byte_time)
    _us_timeout = 0;

    // inicializa el thread
    _th = NULL;

    // inicializa el estado y borra el semáforo de operación bloqueante
    _stat = 0;
    _sem = NULL;
}


//---------------------------------------------------------------------------------
SerialMon::~SerialMon(){
	// desactiva interrupciones serie
    _serial->attach(0, (SerialBase::IrqType)TxIrq);
    _serial->attach(0, (SerialBase::IrqType)RxIrq);

    // desconecta el puerto serie
    delete(_serial);

    // libera buffers
    delete(_txbuf.mem);
    delete(_rxbuf.mem);

    // finaliza el hilo
	_th->join();
    _th->terminate();
    delete(_th);
}

//---------------------------------------------------------------------------------
void SerialMon::setLoggingLevel(esp_log_level_t level){
	esp_log_level_set(_MODULE_, level);
}

//---------------------------------------------------------------------------------
void SerialMon::start(uint32_t max_msg_size, osPriority priority, uint32_t stack_size){
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Arrancando thread SerialMon: %s", _name);

	_max_msg_size = max_msg_size;
	_curr_rx_msg = new uint8_t[_max_msg_size]();
	MBED_ASSERT(_curr_rx_msg);

	// si el no existe, lo crea
	if(_th == NULL){
		_th = new Thread(priority, stack_size, NULL, _name);
		MBED_ASSERT(_th);
	}

	// si no está iniciado lo hace
	if(_th->get_state() == Thread::Deleted){
		_th->start(callback(this, &SerialMon::_task));
	}
}


//---------------------------------------------------------------------------------
void SerialMon::attachRxCallback(Callback<void(uint8_t*, int, Flags)> cb){
	if(cb != (Callback<void(uint8_t*, int, Flags)>) NULL){
		_cb_rx = cb;
	}
	else{
		_cb_rx = callback(this, &SerialMon::_defaultRxUnhandledCallback);
	}
}


//---------------------------------------------------------------------------------
void SerialMon::attachTxCallback(Callback<void(Flags)> cb){
	if(cb != (Callback<void(Flags)>) NULL){
		_cb_tx = cb;
	}
	else{
	    _cb_tx = callback(this, &SerialMon::_defaultTxUnhandledCallback);
	}
}


//---------------------------------------------------------------------------------
int SerialMon::send(uint8_t* data, int size){
    _mtx.lock();

	int free_size = (_txbuf.ou <= _txbuf.in)? ((_txbuf.limit -_txbuf.in) + (_txbuf.ou - _txbuf.mem)) : ((_txbuf.ou-1) - _txbuf.in);
	if(free_size < size){
		_mtx.unlock();
		DEBUG_TRACE_E(_EXPR_, _MODULE_, "No hay espacio suficiente para el envio %d < %d", size, free_size);
		return 0;
	}

    // inserta los datos en el buffer
    for(int i=0;i<size;i++){
        *_txbuf.in++ = data[i];
        _txbuf.in = (_txbuf.in >= _txbuf.limit)? _txbuf.mem : _txbuf.in;
    }

    DEBUG_TRACE_I(_EXPR_, _MODULE_, "Se han encolado %d nuevos bytes", size);
    // si no hay ningún envío en marcha, lo inicia asociando el manejador de interrupciones
    if((_stat & FLAG_SENDING)==0){
        _stat |= FLAG_SENDING;
        _serial->attach(0, (SerialBase::IrqType)TxIrq);
        _txCallback();
        _serial->attach(callback(this, &SerialMon::_txCallback), (SerialBase::IrqType)TxIrq);
    }

    _mtx.unlock();
    return size;
}


//---------------------------------------------------------------------------------
int SerialMon::sendComplete(uint8_t* data, int size){
	// si hay una operación bloqueante en curso, no permite otros envíos
	if(_sem != NULL){
		return 0;
	}

	_mtx.lock();
	_sem = new Semaphore();
	MBED_ASSERT(_sem);

	// si se pueden enviar datos, espera a que se libere el semáforo para devolver el control
	int sent;
	if((sent = send(data, size)) > 0){
		_sem->wait();
	}

	// destruye el semáforo y termina
	delete(_sem);
	_sem = NULL;
	_mtx.unlock();
	return sent;
}



//-----------------------------------------------------------------------------------
//-- PROTECTED ----------------------------------------------------------------------
//-----------------------------------------------------------------------------------


//---------------------------------------------------------------------------------
void SerialMon::_task(){

    // se conecta al módulo de recepción
    _serial->attach(callback(this, &SerialMon::_rxCallback), (SerialBase::IrqType)RxIrq);

    DEBUG_TRACE_D(_EXPR_, _MODULE_, "Thread iniciado SerialMon: %s. Esperando eventos", _name);
    _is_ready = true;

    // espera eventos hardware forever and ever...
    for(;;){
		osEvent oe = _th->signal_wait(osFlagsWaitAny, osWaitForever);

		// si es un flag de fin de trama o de nuevo byte recibido...
        if(oe.status == osEventSignal &&  (oe.value.signals & (FLAG_RECV|FLAG_EOR)) != 0){
        	// lee el buffer, lo procesa y notifica en caso de encontrar una trama válida
        	_read();
        }

        // si es un flag de timeout, lo notifica
        if(oe.status == osEventSignal &&  (oe.value.signals & FLAG_RTIMED) != 0){
        	DEBUG_TRACE_W(_EXPR_, _MODULE_, "Notificando FLAG_RTIMED");
        	_cb_rx(0, 0, FLAG_RTIMED);
        }

        // si es un flag de error en recepción por buffer lleno, lo notifica
        if(oe.status == osEventSignal &&  (oe.value.signals & FLAG_RXFULL) != 0){
        	DEBUG_TRACE_W(_EXPR_, _MODULE_, "Notificando FLAG_RXFULL");
        	_cb_rx(0, 0, FLAG_RXFULL);
        }

        // si es un flag de fin de transmisión, lo notifica
        if(oe.status == osEventSignal &&  (oe.value.signals & FLAG_EOT) != 0){
        	// si ha sido un envío bloqueante por semáforo, lo libera
        	if(_sem != NULL){
        		_sem->release();
        	}
        	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Notificando trama enviada");
        	_cb_tx(FLAG_EOT);
        }
    }
}


//---------------------------------------------------------------------------------
void SerialMon::_read() {
	uint8_t* ptr_end = _rxbuf.in;
	if(_curr_rx_msg_start == NULL){
		_curr_rx_msg = _rxbuf.ou;
		ac_task = {0};
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Reiniciado puntero de busqueda");
	}
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Procesando buffer %s", cpp_utils::bin2str(_curr_rx_msg, ptr_end - _curr_rx_msg));
	while(_curr_rx_msg != ptr_end){
		uint8_t* curr_ptr = _curr_rx_msg;
		uint8_t d = (uint8_t)*_curr_rx_msg++;
		if(_curr_rx_msg >= _rxbuf.limit){
			_curr_rx_msg = _rxbuf.mem;
		}

		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Procesando byte %x", d);
		ISerial::AnalysisResult res = ISerial::_analyzeByte(d, ac_task);
		if(res == ISerial::Wrong || res == ISerial::Discarded){
			DEBUG_TRACE_D(_EXPR_, _MODULE_, "Byte descartado. Reajusto _rxbuf.ou");
			_rxbuf.ou = _curr_rx_msg;
			_curr_rx_msg_start = NULL;
			_stat &= ~FLAG_RXFULL;
		}
		else if(res == ISerial::Start){
			_curr_rx_msg_start = curr_ptr;
			DEBUG_TRACE_D(_EXPR_, _MODULE_, "Identificado inicio de trama en %x", _curr_rx_msg_start);
		}
		else if(res == ISerial::Valid){
			DEBUG_TRACE_D(_EXPR_, _MODULE_, "Identificado fin de trama en %x", _curr_rx_msg);
			int size = (_curr_rx_msg_start <= _curr_rx_msg)? (_curr_rx_msg - _curr_rx_msg_start) : (_rxbuf.limit - _curr_rx_msg_start) + (_curr_rx_msg - _rxbuf.mem);
			DEBUG_TRACE_D(_EXPR_, _MODULE_, "Tamaño de trama = %d", size);
			if(size > 0){
				uint8_t* data = new uint8_t[size]();
				MBED_ASSERT(data);
				int p=0;
				DEBUG_TRACE_D(_EXPR_, _MODULE_, "Copiando datos", size);
				while(_curr_rx_msg_start != _curr_rx_msg){
					data[p++] = *_curr_rx_msg_start++;
					if(_curr_rx_msg_start >= _rxbuf.limit){
						_curr_rx_msg_start = _rxbuf.mem;
					}
				}
				DEBUG_TRACE_D(_EXPR_, _MODULE_, "Reajuste de buffer de recepcion");
				_rxbuf.ou = _curr_rx_msg;
				_stat &= ~FLAG_RXFULL;
				_curr_rx_msg_start = NULL;
				DEBUG_TRACE_D(_EXPR_, _MODULE_, "Notificando trama recibida size=%d", size);
				_cb_rx(data, size, FLAG_EOR);
				delete(data);
			}
			else{
				DEBUG_TRACE_W(_EXPR_, _MODULE_, "Tamaño=0, ERROR. Reajuste de buffer de recepcion");
				_rxbuf.ou = _curr_rx_msg;
				_stat &= ~FLAG_RXFULL;
				_curr_rx_msg_start = NULL;
			}
		}
	}
}


//---------------------------------------------------------------------------------
void SerialMon::_txCallback(){
	// envía el siguiente dato pendiente de envío
    if((_serial->writeable()) && (_txbuf.in != _txbuf.ou)) {
    	uint8_t c = *_txbuf.ou;
        _serial->putc(c);
        _txbuf.ou++;
        _txbuf.ou = (_txbuf.ou >= _txbuf.limit)? _txbuf.mem : _txbuf.ou;
        return;
    }

	// si ha terminado, borra el flag de envío y activa el flag EOT
    _stat &= ~FLAG_SENDING;

	// se desconecta del dispositivo de transmisión
    _serial->attach(0, (SerialBase::IrqType)TxIrq);

    // notifica flag a la tarea
    _th->signal_set(FLAG_EOT);
}


//---------------------------------------------------------------------------------
void SerialMon::_rxCallback(){
	// mientras haya datos pendientes, los almacena en el buffer
    while(_serial->readable()){
    	// lee byte a byte
    	uint8_t c = _serial->getc();

		// si el buffer está lleno, notifica error
        if((_stat & FLAG_RXFULL)!=0){
        	// notifica flag a la tarea
        	_th->signal_set(FLAG_RXFULL);
            return;
        }

        // si hay espacio, lo almacena
        *_rxbuf.in++ = c;
        _rxbuf.in = (_rxbuf.in >= _rxbuf.limit)? _rxbuf.mem : _rxbuf.in;
		// si se completa el buffer, marca estado
        if(_rxbuf.in == _rxbuf.ou){
            _stat |= FLAG_RXFULL;
        }
        // en modo contínuo, notifica la llegada de un nuevo byte
        if(ISerial::_eor_mode == ISerial::EORContinuous){
        	_stat |= FLAG_RXSTARTED;
        	_th->signal_set(FLAG_RECV);
        	return;
        }
        // en modo detección de fin de trama por ticker y en cada byte recibido
        if (ISerial::_eor_mode == ISerial::EORByTicker){
        	// marca el flag y activa la callback
        	_stat |= FLAG_RXSTARTED;
        	// reajusta el timer de detección de fin de trama
        	_rx_tick.attach_us(callback(this, &SerialMon::_timeoutCallback), _us_timeout);
        	return;
        }
        // en modo detección de trama por IDLE, si es el primer byte
        if (ISerial::_eor_mode == ISerial::EORByIdleTime && (_stat & FLAG_RXSTARTED)==0){
        	// actualiza el flag de estado y habilita la callback
        	_stat |= FLAG_RXSTARTED;
        	_serial->attach(callback(this, &SerialMon::_rxIdleCallback), (SerialBase::IrqType)IdleIrq);
        	return;
        }
    }
}


//---------------------------------------------------------------------------------
void SerialMon::_rxIdleCallback(){
	_stat &= ~FLAG_RXSTARTED;
	_serial->attach(0, (SerialBase::IrqType)IdleIrq);
	_th->signal_set(FLAG_EOR);
}


//---------------------------------------------------------------------------------
void SerialMon::_timeoutCallback(){
	_stat &= ~FLAG_RXSTARTED;
	_rx_tick.detach();
	_th->signal_set(FLAG_EOR);
}

