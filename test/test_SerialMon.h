/* test_SerialMon

   Incluye definiciones comunes para realizar los diferentes tipos de test de la librer�a
*/

#ifndef TEST_SerialMon_H
#define TEST_SerialMon_H

#include "mbed.h"
#include "AppConfig.h"

/** Claves para depurar tests unitarios (paso a paso). DESACTIVAR PARA EJECUTAR APLICACI�N DE PRODUCCI�N */
#if defined(ENABLE_TEST_DEBUGGING)

//#### COMENTAR PARA DESACTIVAR ####
//#define ENABLE_TEST_SerialMon		//!< Habilita los tests del m�dulo 'SerialMon'




/**---------------------------------------------------------------------------
 * Test para verificar la inicializaci�n de la librer�a
 */
void test_SerialMon_initialize();



#endif
#endif


