#ifndef CONFIG_RCMAP_H
#define CONFIG_RCMAP_H

// --- MAPEO DE LOS CANALES DEL CONTROL ---
// Indice dentro de crsf_data_t.channels que ocupa cada eje. El emisor manda
// AETR: alerones (roll), elevador (pitch), acelerador y timon (yaw). Si en el
// control las palancas estan cruzadas (mover pitch mueve roll), se intercambian
// aqui los indices de RC_CHANNEL_ROLL y RC_CHANNEL_PITCH.
#define RC_CHANNEL_ROLL     0
#define RC_CHANNEL_PITCH    1
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_YAW      3
#define RC_CHANNEL_ARM      4

// Sentido de cada eje: +1 deja el canal tal como llega y -1 lo invierte
// respecto del centro. Depende de como esten cableados los potenciometros de
// las palancas, asi que se ajusta con el dron sujeto: si al pedir alabeo a la
// derecha el dron se va a la izquierda (o al empujar el morro abajo sube), se
// cambia el signo del eje correspondiente.
#define RC_SIGN_ROLL  (-1)
#define RC_SIGN_PITCH (+1)
#define RC_SIGN_YAW   (+1)

#endif // CONFIG_RCMAP_H
