/* test_SerialMon

   Unit test of MBED-API-uSerial
*/

#include "mbed.h"
#include "test_SerialMon.h"


//------------------------------------------------------------------------------------
//-- TEST DEFINITIONS ----------------------------------------------------------------
//------------------------------------------------------------------------------------

/** Requerido para test unitarios ESP-MDF */
#if ESP_PLATFORM == 1
#include "FSManager.h"
#include "SerialMon.h"
#include "MQLib.h"
#include "SerialMon.h"
void (*syslog_print)(const char*level, const char* tag, const char* format, ...) = NULL;
static Serial* g_serial = NULL;
#define PinName_TX	GPIO_NUM_13
#define PinName_RX	GPIO_NUM_12

/** Requerido para test unitarios STM32 */
#elif __MBED__ == 1 && defined(ENABLE_TEST_DEBUGGING) && defined(ENABLE_TEST_SerialMon)
#include "Heap.h"
#include "cpp_utils.h"
#include "SerialMon.h"
#include "unity.h"
#include "unity_test_runner.h"
static SerialMon* g_serial = NULL;
#define PinName_TX	PC_10
#define PinName_RX	PC_11

#endif

/** Requerido en ambos casos */
#if ESP_PLATFORM == 1 || (__MBED__ == 1 && defined(ENABLE_TEST_DEBUGGING) && defined(ENABLE_TEST_SerialMon))

static const char* _MODULE_ = "[TEST]..........";
#define _EXPR_	(true)


//------------------------------------------------------------------------------------
//--- MBED_API_uSerial ---------------------------------------------------------------
//------------------------------------------------------------------------------------


/** Tamaño máximo del mensaje del protocolo userial */
#define MAX_USERIAL_MSG_SIZE	128
#define TX_BUFFER_SIZE			8 * MAX_USERIAL_MSG_SIZE
#define RX_BUFFER_SIZE			8 * MAX_USERIAL_MSG_SIZE
#define SERIAL_BAUDRATE			115200

/** Identificación de trama */
#define HEADER_FLAG 	0xa5
#define FOOTER_FLAG		0x5a

/** Estructura de datos a verificar */
struct HSF{
	uint8_t header;
	uint16_t size;
	uint8_t* data;
	uint8_t footer;
	/** Crea el objeto a partir de un buffer de datos en bruto */
	HSF(uint8_t* pdata, uint16_t xsize){
		header = HEADER_FLAG;
		size = xsize;
		data = new uint8_t[xsize]();
		MBED_ASSERT(data);
		memcpy(data, pdata, xsize);
		footer = FOOTER_FLAG;
	}
	/** Crea el objeto a partir de una serialización */
	HSF(uint8_t* pdata){
		int len = 0;
		len += cpp_utils::buffer2object(header, &pdata[len]);
		len += cpp_utils::buffer2object(size, &pdata[len]);
		int data_len = size - sizeof(uint8_t) - sizeof(uint16_t) - sizeof(uint8_t);
		data = new uint8_t[data_len]();
		MBED_ASSERT(data);
		memcpy(data, &pdata[len], data_len);
		len += data_len;
		len += cpp_utils::buffer2object(footer, &pdata[len]);
	}

	~HSF(){
		delete(data);
	}
	/** Serializa una trama */
	uint8_t* serialize(uint16_t& len){
		len = 0;
		uint8_t* pdata = new uint8_t[size]();
		MBED_ASSERT(pdata);
		len += cpp_utils::object2buffer(&pdata[len], header);
		len += cpp_utils::object2buffer(&pdata[len], size);
		int data_len = size - sizeof(uint8_t) - sizeof(uint16_t) - sizeof(uint8_t);
		memcpy(&pdata[len], data, data_len);
		len += data_len;
		len += cpp_utils::object2buffer(&pdata[len], footer);
		return pdata;
	}
};

/** Semáforo para controlar la ejecución del test */
static Semaphore sem{};
static uint8_t buffer[2*MAX_USERIAL_MSG_SIZE];
static int expected_callbacks=0;



//------------------------------------------------------------------------------------
//-- TEST FUNCTIONS ------------------------------------------------------------------
//------------------------------------------------------------------------------------

static void rxCallback(uint8_t* data, int size, ISerial::Flags flag){
	if(flag == ISerial::FLAG_EOR){
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Se han recibido %d bytes, Recompone objeto", size);
		HSF* hsf = new HSF(data);
		TEST_ASSERT_NOT_NULL(hsf);
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Serializando objeto recompuesto");
		uint16_t len;
		uint8_t* pdata = hsf->serialize(len);
		TEST_ASSERT_EQUAL(size,len);
		TEST_ASSERT_EQUAL(memcmp(data, pdata, size), 0);
		DEBUG_TRACE_I(_EXPR_, _MODULE_, "Envio completado con exito!!!");
	}
	else{
		DEBUG_TRACE_W(_EXPR_, _MODULE_, "Se han recibido el flag %x con data=%x, size=%d", flag, data, size);
	}
	if(--expected_callbacks <= 0){
		sem.release();
	}
}

//------------------------------------------------------------------------------------
void test_SerialMon_initialize(){
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Inicializando Puerto serie");
	#if ESP_PLATFORM==1
	g_serial = new Serial(PinName_TX, PinName_RX, RX_BUFFER_SIZE, SERIAL_BAUDRATE, 0, UART_NUM_1, false, UART_HW_FLOWCTRL_DISABLE, NC, NC, UART_DATA_8_BITS, UART_PARITY_DISABLE, UART_STOP_BITS_1);
	TEST_ASSERT_NOT_NULL(g_serial);
	#elif __MBED__ == 1
	g_serial = new SerialMon(PinName_TX, PinName_RX, TX_BUFFER_SIZE, RX_BUFFER_SIZE, SERIAL_BAUDRATE, "uSerial");
	#endif
	TEST_ASSERT_NOT_NULL(g_serial);
	DEBUG_TRACE_I(_EXPR_, _MODULE_, "SerialMon creado!");
}


//------------------------------------------------------------------------------------
void test_SerialMon_start(){
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Configurando deteccion de trama");
	g_serial->cfgStreamAnalyzer(HEADER_FLAG,
								FOOTER_FLAG,
								1,
								sizeof(uint16_t),
								true,
								RX_BUFFER_SIZE);

	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Arrancando SerialMon");
	g_serial->start(MAX_USERIAL_MSG_SIZE, osPriorityNormal, OS_STACK_SIZE);



	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Esperando a que SerialMon esté operativa");
	while(!g_serial->isReady()){
		Thread::wait(1000);
	}
	DEBUG_TRACE_I(_EXPR_, _MODULE_, "SerialMon arrancado!!");

	g_serial->attachRxCallback(callback(&rxCallback));

}


//------------------------------------------------------------------------------------
void test_SerialMon_loop_div2(){
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Creando trama de tamaño %d", MAX_USERIAL_MSG_SIZE/2);

	for(int i=0;i<MAX_USERIAL_MSG_SIZE/2;i++){
		buffer[i] = i;
	}
	HSF* hsf = new HSF(buffer, MAX_USERIAL_MSG_SIZE/2);
	TEST_ASSERT_NOT_NULL(hsf);

	uint16_t size = 0;
	uint8_t* pdata = hsf->serialize(size);
	expected_callbacks = 1;
	TEST_ASSERT_NOT_EQUAL(g_serial->send(pdata, size), 0);
	sem.wait(osWaitForever);
}


//------------------------------------------------------------------------------------
void test_SerialMon_loop_div1(){
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Creando trama de tamaño %d", MAX_USERIAL_MSG_SIZE);

	for(int i=0;i<MAX_USERIAL_MSG_SIZE;i++){
		buffer[i] = i;
	}
	HSF* hsf = new HSF(buffer, MAX_USERIAL_MSG_SIZE);
	TEST_ASSERT_NOT_NULL(hsf);

	uint16_t size = 0;
	uint8_t* pdata = hsf->serialize(size);
	expected_callbacks = 1;
	TEST_ASSERT_NOT_EQUAL(g_serial->send(pdata, size), 0);
	sem.wait(osWaitForever);
}


//------------------------------------------------------------------------------------
void test_SerialMon_loop_x2(){
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Creando trama de tamaño %d", 2*MAX_USERIAL_MSG_SIZE);

	for(int i=0;i<2*MAX_USERIAL_MSG_SIZE;i++){
		buffer[i] = i;
	}
	HSF* hsf = new HSF(buffer, 2*MAX_USERIAL_MSG_SIZE);
	TEST_ASSERT_NOT_NULL(hsf);

	uint16_t size = 0;
	uint8_t* pdata = hsf->serialize(size);
	expected_callbacks = 1;
	TEST_ASSERT_NOT_EQUAL(g_serial->send(pdata, size), 0);
	sem.wait(osWaitForever);
}


//------------------------------------------------------------------------------------
void test_SerialMon_accumulated(){
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Creando trama de tamaño %d", 2*MAX_USERIAL_MSG_SIZE);

	for(int i=0;i<2*MAX_USERIAL_MSG_SIZE;i++){
		buffer[i] = i;
	}
	HSF* hsf = new HSF(buffer, 2*MAX_USERIAL_MSG_SIZE);
	TEST_ASSERT_NOT_NULL(hsf);

	uint16_t size = 0;
	uint8_t* pdata = hsf->serialize(size);
	expected_callbacks = 3;
	TEST_ASSERT_NOT_EQUAL(g_serial->send(pdata, size), 0);
	wait_us(200);
	TEST_ASSERT_NOT_EQUAL(g_serial->send(pdata, size), 0);
	wait_us(200);
	TEST_ASSERT_NOT_EQUAL(g_serial->send(pdata, size), 0);
	sem.wait(osWaitForever);
}

//------------------------------------------------------------------------------------
//-- TEST CASES ----------------------------------------------------------------------
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
TEST_CASE("Inicializacion", "[SerialMon]") {
	test_SerialMon_initialize();
}


//------------------------------------------------------------------------------------
TEST_CASE("Arranque de la tarea", "[SerialMon]") {
	test_SerialMon_start();
}



//------------------------------------------------------------------------------------
TEST_CASE("Prueba en modo loop MSG_SIZE/2", "[SerialMon]") {
	test_SerialMon_loop_div2();
}



//------------------------------------------------------------------------------------
TEST_CASE("Prueba en modo loop MSG_SIZE", "[SerialMon]") {
	test_SerialMon_loop_div1();
}


//------------------------------------------------------------------------------------
TEST_CASE("Prueba en modo loop 2*MSG_SIZE", "[SerialMon]") {
	test_SerialMon_loop_x2();
}


//------------------------------------------------------------------------------------
TEST_CASE("Prueba 3 tramas encadenadas", "[SerialMon]") {
	test_SerialMon_accumulated();
}

//------------------------------------------------------------------------------------
TEST_CASE("Establece DEBUG level", "[SerialMon]") {
	g_serial->setLoggingLevel(ESP_LOG_DEBUG);
}

//------------------------------------------------------------------------------------
TEST_CASE("Establece INFO level", "[SerialMon]") {
	g_serial->setLoggingLevel(ESP_LOG_INFO);
}

//------------------------------------------------------------------------------------
TEST_CASE("Establece WARN level", "[SerialMon]") {
	g_serial->setLoggingLevel(ESP_LOG_WARN);
}

//------------------------------------------------------------------------------------
//-- TEST ENRY POINT -----------------------------------------------------------------
//------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------
void firmwareStart(){
	DEBUG_TRACE_I(_EXPR_, _MODULE_, "Inicio del programa");
	Heap::setDebugLevel(ESP_LOG_DEBUG);
	unity_run_menu();
}


#endif




