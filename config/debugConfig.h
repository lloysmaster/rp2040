#ifndef CONFIG_DEBUGCONFIG_H
#define CONFIG_DEBUGCONFIG_H

// --- DEPURACION EN EL BUCLE DE CONTROL ---
// El menu de giroscopio integra deriva y convierte a float en cada muestra:
// cuesta unas 2300 instrucciones por ciclo (~16 % del coste del bucle) y sus
// impresiones periodicas pueden bloquear el USB varios cientos de us. Poner a 0
// para dejarlo fuera del camino critico cuando se busque la maxima cadencia.
#ifndef GYRO_DEBUG_ENABLED
#define GYRO_DEBUG_ENABLED 1
#endif

#endif // CONFIG_DEBUGCONFIG_H
