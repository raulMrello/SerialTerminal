/* test_SerialMon

   Incluye definiciones comunes para realizar los diferentes tipos de test de la librería
*/

#ifndef TEST_SerialMon_H
#define TEST_SerialMon_H

#include "mbed.h"
#include "AppConfig.h"

/** Claves para depurar tests unitarios (paso a paso). DESACTIVAR PARA EJECUTAR APLICACIÓN DE PRODUCCIÓN */
#if defined(ENABLE_TEST_DEBUGGING)

//#### COMENTAR PARA DESACTIVAR ####
//#define ENABLE_TEST_SerialMon		//!< Habilita los tests del módulo 'SerialMon'




/**---------------------------------------------------------------------------
 * Test para verificar la inicialización de la librería
 */
void test_SerialMon_initialize();



#endif
#endif


