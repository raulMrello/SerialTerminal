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
SerialMon::SerialMon(PinName tx, PinName rx, int txBufferSize, int rxBufferSize, int baud, const char* name) : _name(name) {
	
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Iniciando SerialMon: %s", name);

	// inicializa buffers
    _txbuf.mem = new char[txBufferSize]();
    MBED_ASSERT(_txbuf.mem);
	_txbuf.limit = (char*)&_txbuf.mem[txBufferSize];
	_txbuf.in = _txbuf.mem;
	_txbuf.ou = _txbuf.in;
	_txbuf.sz = txBufferSize;
	memset(_txbuf.mem, 0, txBufferSize);

    _rxbuf.mem = new char[rxBufferSize]();
    MBED_ASSERT(_rxbuf.mem);
	_rxbuf.limit = (char*)&_rxbuf.mem[rxBufferSize];
	_rxbuf.in = _rxbuf.mem;
	_rxbuf.ou = _rxbuf.in;
	_rxbuf.sz = rxBufferSize;
	memset(_rxbuf.mem, 0, rxBufferSize);

	/** Inicializa callbacks */
    _cb_rx = (Callback< void(uint8_t*, int, Flags)>) NULL;
    _cb_tx = (Callback< void(Flags)>) NULL;

	
	// inicia el dispositivo serie,
    _serial = new RawSerial(tx, rx, baud);
    MBED_ASSERT(_serial);

    // desconecta las interrupciones serie
    _serial->attach(0, (SerialBase::IrqType)TxIrq);
    _serial->attach(0, (SerialBase::IrqType)RxIrq);

    // establece el timeout para detección de fin de trama (IDLE por timeout software 2.5 byte_time)
    _us_timeout = (uint32_t) (1 + (2.5 * 10 * 1000000)/baud);

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
void SerialMon::start(osPriority priority, uint32_t stack_size){
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Arrancando thread SerialMon: %s", _name);

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
void SerialMon::attachRxCallback(Callback<void(uint8_t*, int, SerialMon::Flags)> cb){
	_cb_rx = cb;
}


//---------------------------------------------------------------------------------
void SerialMon::attachTxCallback(Callback<void(SerialMon::Flags)> cb){
	_cb_tx = cb;
}


//---------------------------------------------------------------------------------
int SerialMon::send(uint8_t* data, int size){
    _mtx.lock();

    // chequea el tamaño que queda disponible del buffer de transmisión para no sobrepasar su capacidad
    int remaining = (_txbuf.in >= _txbuf.ou)? ((_txbuf.limit-_txbuf.in)+(_txbuf.ou-_txbuf.mem)) : (_txbuf.ou-_txbuf.in);
    if(remaining == 0){
    	_mtx.unlock();
    	return 0;
    }
    if(size > remaining){
		size = remaining;
    }

    // inserta los datos en el buffer
    for(int i=0;i<size;i++){
        *_txbuf.in++ = data[i];
        _txbuf.in = (_txbuf.in >= _txbuf.limit)? _txbuf.mem : _txbuf.in;
    }

    // si no hay ningún envío en marcha, lo inicia asociando el manejador de interrupciones
    if((_stat & FLAG_SENDING)==0){
        _stat |= FLAG_SENDING;
        if((_stat & FLAG_STARTED)!=0){
            _serial->attach(0, (SerialBase::IrqType)TxIrq);
            _txCallback();
        }
        _stat |= FLAG_STARTED;
        _serial->attach(callback(this, &SerialMon::_txCallback), (SerialBase::IrqType)TxIrq);
    }

    _mtx.unlock();
    return size;
}


//---------------------------------------------------------------------------------
int SerialMon::sendComplete(uint8_t* data, int size){
	// si hay una operación bloqueante en curso, no permite otros envíos
	if(_sem != NULL){
		return -1;
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

    // espera eventos hardware forever and ever...
    for(;;){
        osEvent oe = _th->signal_wait(osFlagsWaitAny, osWaitForever);

        // si es un flag de fin de trama
        if(oe.status == osEventSignal && (oe.value.signals & FLAG_EOR) != 0){
			// despacha las trama pendiente
        	int len=0;
        	uint8_t* data = _read(len);
        	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Notificando trama recibida size=%d", len);
			_cb_rx(data, len, FLAG_EOR);
			if(data){
				delete(data);
			}
        	// borra el flag
			_th->signal_clr(FLAG_EOR);
        }

        // si es un flag de timeout, lo notifica
        if(oe.status == osEventSignal && (oe.value.signals & FLAG_RTIMED) != 0){
        	DEBUG_TRACE_W(_EXPR_, _MODULE_, "Notificando FLAG_RTIMED");
        	_cb_rx(0, 0, FLAG_RTIMED);
        	_th->signal_clr(FLAG_RTIMED);
        }

        // si es un flag de error en recepción por buffer lleno, lo notifica
        if(oe.status == osEventSignal && (oe.value.signals & FLAG_RXFULL) != 0){
        	DEBUG_TRACE_W(_EXPR_, _MODULE_, "Notificando FLAG_RXFULL");
        	_cb_rx(0, 0, FLAG_RXFULL);
        	_th->signal_clr(FLAG_RXFULL);
        }

        // si es un flag de fin de transmisión, lo notifica
        if(oe.status == osEventSignal && (oe.value.signals & FLAG_EOT) != 0){
        	// si ha sido un envío bloqueante por semáforo, lo libera
        	if(_sem != NULL){
        		_sem->release();
        	}
        	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Notificando trama enviada");
        	_cb_tx(FLAG_EOT);
        	_th->signal_clr(FLAG_EOT);
        }
    }
}


//---------------------------------------------------------------------------------
uint8_t* SerialMon::_read(int& size) {
	size = (_rxbuf.ou <= _rxbuf.in)? (_rxbuf.in - _rxbuf.ou) : (_rxbuf.limit - _rxbuf.in) + (_rxbuf.ou - _rxbuf.mem);
	if(size <= 0){
		return NULL;
	}
	uint8_t* data = new uint8_t[size]();
	MBED_ASSERT(data);
	for(int s=0; s<size; s++) {
        data[s] = (uint8_t)*_rxbuf.ou++;
        _rxbuf.ou = (_rxbuf.ou >= _rxbuf.limit)? _rxbuf.mem : _rxbuf.ou;
        if(_rxbuf.in != _rxbuf.ou){
			_stat &= ~FLAG_RXFULL;
        }
    }
    return data;
}


//---------------------------------------------------------------------------------
void SerialMon::_txCallback(){
	// envía el siguiente dato pendiente de envío
    if ((_serial->writeable()) && (_txbuf.in != _txbuf.ou)) {
        char c = *_txbuf.ou;
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
        char c = _serial->getc();

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

		// reajusta el timer de detección de línea IDLE
        _rx_tick.attach_us(callback(this, &SerialMon::_timeoutCallback), _us_timeout);
    }
}


//---------------------------------------------------------------------------------
void SerialMon::_timeoutCallback(){
	_rx_tick.detach();
    _th->signal_set(FLAG_EOR);
}

