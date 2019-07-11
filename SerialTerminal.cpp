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
    @purpose       Clase que implementa un terminal serie para recibir comandos
                   remotos. Utiliza únicamente el pin Rx. Proporciona callbacks
                   para notificar el fin de recepción, el fallo por timeout y el 
                   fallo por desborde de buffer. Utiliza un caracter para detectar
                   el fin de trama.
    @date          Jul 2017
    @author        raulMrello
*/

#include "SerialTerminal.h"

//---------------------------------------------------------------------------------
//- PRIVATE -----------------------------------------------------------------------
//---------------------------------------------------------------------------------

static void unhandled_callback(){}
static bool unhandled_callback_2(uint8_t* data, uint16_t size){return false;}

//---------------------------------------------------------------------------------
//- ISR ---------------------------------------------------------------------------
//---------------------------------------------------------------------------------


//---------------------------------------------------------------------------------
void SerialTerminal::onTxData(){
    if(writeable()){
        if(_sent < _tosend){
            putc(_tbuf[_sent++]);
        }
        else if(_sent){
            stopTransmitter();
            _cb_tx.call();
            _cb_tx = callback(unhandled_callback);
        }
    }
}

//---------------------------------------------------------------------------------
void SerialTerminal::onRxData(){
    while(readable()){
        // lee un byte
        char d = (char)getc();
        // si excede el buffer, termina y notifica error
        if(_recv >= _bufsize){
            _tmr.detach();
            attach(0, (SerialBase::IrqType)RxIrq);
            _cb_rx_ovf.call();
            return;
        }
        // almacena el dato en el buffer de recepción
        _databuf[_recv++] = d;        
        // en modo eof, si coincide con él, termina y notifica
        if(_mode == ReceiveWithEofCharacter && d == _eof){
            _tmr.detach();
            attach(0, (SerialBase::IrqType)RxIrq);
            _cb_rx.call();
        }
        // en modo proc, si el proceso cumple la condición de finalización, termina y notifica
        else if(_mode == ReceiveWithDedicatedHandling && _cb_proc.call((uint8_t*)_databuf, _recv)){
            _tmr.detach();
            attach(0, (SerialBase::IrqType)RxIrq);
            _cb_rx.call();
        }
        // si es el primer byte en modo distinto de breaktime, o en cada byte en modo break_time, inicia el timer
        else if(_mode == ReceiveAfterBreakTime || (_mode != ReceiveAfterBreakTime && _recv == 1)){
            if(_us_timeout > 0){
                _tmr.attach_us(callback(this, &SerialTerminal::onRxTimeout), _us_timeout);
            }
        }
    }
}

//---------------------------------------------------------------------------------
void SerialTerminal::onRxTimeout(){
    _tmr.detach();
    attach(0, (SerialBase::IrqType)RxIrq);
    // si es modo break_time, notifica fin de trama
    if(_mode == ReceiveAfterBreakTime){
        _cb_rx.call();
        return;
    }
    // en caso contrario, notifica error de timeout
    _cb_rx_tmr.call();
}



//---------------------------------------------------------------------------------
//- IMPL. -------------------------------------------------------------------------
//---------------------------------------------------------------------------------

SerialTerminal::SerialTerminal(PinName tx, PinName rx, uint16_t maxbufsize, int baud, Receiver_mode mode) : RawSerial(tx, rx, baud){
    attach(0, (SerialBase::IrqType)RxIrq);
    attach(0, (SerialBase::IrqType)TxIrq);
    _us_timeout = 0;
    _mode = mode;
    _recv = 0;
    _eof = 0;
    _bufsize = maxbufsize;
    _databuf = (char*)malloc(maxbufsize);
    _sent = 0;
    _tosend = 0;
    _tbuf = 0;
    _cb_tx = callback(unhandled_callback);
    _cb_rx = callback(unhandled_callback);
    _cb_rx_tmr = callback(unhandled_callback);
    _cb_rx_ovf = callback(unhandled_callback);
    _cb_proc = callback(unhandled_callback_2);
    tx_managed = false;
    rx_managed = false;
}

//---------------------------------------------------------------------------------
bool SerialTerminal::config(Callback<void()> rx_done, Callback <void()> rx_timeout, Callback <void()> rx_ovf, uint32_t us_timeout, char eof){
    _cb_rx = rx_done;
    _cb_rx_tmr = rx_timeout;
    _cb_rx_ovf = rx_ovf;
    _eof = eof;
    _us_timeout = us_timeout;
    return (_databuf && _bufsize)? true : false;
}

//---------------------------------------------------------------------------------
void SerialTerminal::startManaged(bool transmitter, bool receiver){
    if(transmitter){
        tx_managed = true;
        _sent = 0;
        attach(callback(this, &SerialTerminal::onTxData), (SerialBase::IrqType)TxIrq);
    }
    if(receiver){
        rx_managed = true;
        _recv = 0;
        attach(callback(this, &SerialTerminal::onRxData), (SerialBase::IrqType)RxIrq);
    }
}

//---------------------------------------------------------------------------------
void SerialTerminal::stopManaged(bool transmitter, bool receiver){
    if(transmitter){
        attach(0, (SerialBase::IrqType)TxIrq);
        tx_managed = false;
        _tosend = 0;
    }
    if(receiver){
        attach(0, (SerialBase::IrqType)RxIrq);
        rx_managed = false;
        _recv = 0;
    }
}

//---------------------------------------------------------------------------------
bool SerialTerminal::send(void* data, uint16_t size, Callback<void()> tx_done){  
    
    if(_tosend == 0 && (uint8_t*)data && size){
        _tosend = size;
        _cb_tx = tx_done;
        _tbuf = (uint8_t*)data;
        startTransmitter();
        return true;
    }
    return false;
}

//---------------------------------------------------------------------------------
uint16_t SerialTerminal::recv(void* buf, uint16_t maxsize, bool enable_receiver){
    uint16_t nb = (_recv > maxsize)? maxsize : _recv;
    if(nb && (char*)buf){
        memcpy((char*)buf, _databuf, nb);
    }
    if(enable_receiver){
        startReceiver();
    }
    return nb;
}

//---------------------------------------------------------------------------------
bool SerialTerminal::isTxManaged(){
    return tx_managed;
}

//---------------------------------------------------------------------------------
bool SerialTerminal::isRxManaged(){
    return rx_managed;
}
