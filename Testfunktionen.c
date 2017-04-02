/*#include "Mixer.h"
// Implementierung yaw-Regelung

// Kn�ppelstellung repr�sentiert Geschwindigkeit, anstatt Winkel (Neutralstellung: v=0)
// Sensorwert repr�sentiert Winkelgeschwindigkeit (Eulerwinkel-Rate aus UM7)
// Kn�ppel 0% und 100% repr�sentieren beide maximale Drehgeschwindigkeit f�r den Quadrokopter
// Zeitkonstante f�r Sollwert: 22ms (Periode Empf�nger)
// Zeitkonstante f�r Istwert: 10ms  (Periode Sensor)

// f�r die wirkliche yaw-Rate Funke  (in �/sec): IC-Wert *0.0225
// f�r die wirkliche yaw-Rate Sensor (in �/sec): Sensorwert/16, bzw *0.0625

// maximale Rate: 45�/sec = 2000 (Differenz Input Capture)

// Initialisierung der Arrays f�r die Mischer
int thro [4] = {0, 0, 0, 0};
int aile [4] = {0, 0, 0, 0};
int elev [4] = {0, 0, 0, 0};
int rudd [4] = {0, 0, 0, 0};

// Initialisierung der Variablen zum Anzeigen eines erfolgten Mischens aktueller Werte
int thr_mix = 0;
int ail_mix = 0;
int ele_mix = 0;
int rud_mix = 0;

int gather = 0;

// Initialisierungen Strukturen
struct mix MIX;
struct mix *ptr_MIX = &MIX;

struct manualMix PWM;
struct manualMix *ptr_PWM = &PWM;

// Sollwerte (aus Input Capture)
void setValue (struct pw *ptr_PW)
{
    // Differenz als Sollwert (Abweichung von aktueller Kn�ppelstellung zur Neutralstellung) in Q16-Format

    ptr_FIX->sollwert_winkel_aile = ((ptr_PW->aile - neutral) * _Q16ftoi(0.85));
    ptr_FIX->sollwert_winkel_elev = ((ptr_PW->elev - neutral) * _Q16ftoi(0.85));

    ptr_FIX->sollwert_rate_rudd = (ptr_PW->rudd - neutral) * _Q16ftoi(0.0225);                  // Faktor, um bei Maximalwert von 2000 (IC) eine Rate von 45�/sec zu erhalten

    ptr_FIX->sollwert_winkel_aile = ptr_FIX->sollwert_winkel_aile >> 7;
    ptr_FIX->sollwert_winkel_elev = ptr_FIX->sollwert_winkel_elev >> 7;

    ptr_FIX->sollwert_rate_rudd   = ptr_FIX->sollwert_rate_rudd   >> 7;
}

// Istwerte (aus UM6)
void actualValue (UM7DataSensor *ptr_eulerAngle)
{
    // Tats�chlicher Winkel als Istwert in Q16-Format
    ptr_FIX->istwert_winkel_aile = (ptr_eulerAngle->eulerRollAngle  * _Q16ftoi(0.733));
    ptr_FIX->istwert_winkel_elev = (ptr_eulerAngle->eulerPitchAngle * _Q16ftoi(0.733));
    
    ptr_FIX->istwert_rate_rudd   = (ptr_eulerAngle->eulerYawRate    * _Q16ftoi(0.0625));        // Faktor, um tats�chliche Rate in �/sec zu erhalten

    ptr_FIX->istwert_winkel_aile = ptr_FIX->istwert_winkel_aile >> 7;
    ptr_FIX->istwert_winkel_elev = ptr_FIX->istwert_winkel_elev >> 7;

    ptr_FIX->istwert_rate_rudd   = ptr_FIX->istwert_rate_rudd   >> 7;
}
 * */