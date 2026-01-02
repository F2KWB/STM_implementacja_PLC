#ifndef INC_PLC_BLOCKS_H_
#define INC_PLC_BLOCKS_H_

#include "main.h"
#include <stdbool.h>

// Definicja typu TIME
typedef uint32_t TIME;

/* --- Struktura bloku TON --- */
typedef struct {
    bool IN;       // wejście
    bool Q;        // wyjście
    TIME PT;       // preset time (ms)
    TIME ET;       // elapsed time (ms)
    uint32_t startTime; // Czas startu wg HAL_GetTick()
    bool running;
} TON_Block;

/* --- Struktura bloku TOF --- */
typedef struct {
    bool IN;       // wejście
    bool Q;        // wyjście
    TIME PT;       // preset time (ms)
    TIME ET;       // elapsed time (ms)
    uint32_t startTime; // Czas startu wg HAL_GetTick()
    bool running;
} TOF_Block;

/* --- Struktura bloku CTU (Counter Up) --- */
typedef struct {
    // Wejścia
    bool CU;
    bool R;        // Reset (programowy)
    uint16_t PV;   // Preset Value (wartość zadana)

    // Wyjścia
    bool Q;        // Wyjście (osiągnięto PV)
    uint16_t CV;   // Current Value (wartość bieżąca)

    // Pamięć wewnętrzna
    bool prev_CU_state; // Flaga do wykrywania zbocza
} CTU_Block;

/* --- Struktura bloku CTD (Counter Down) --- */
typedef struct {
    // Wejścia
    bool CD;       // Count Down (zbocze narastające)
    bool LD;       // Load (ustawia CV = PV)
    uint16_t PV;   // Preset Value (wartość do załadowania)

    // Wyjścia
    bool Q;        // Wyjście (osiągnięto 0)
    uint16_t CV;   // Current Value (wartość bieżąca)

    // Pamięć wewnętrzna
    bool prev_CD_state; // Flaga do wykrywania zbocza CD
} CTD_Block;

/* --- Struktura bloku R_TRIG (Rising Edge) --- */
typedef struct {
    // Wejście
    bool CLK; // Wejście zegarowe / sygnałowe

    // Wyjście
    bool Q;   // Wyjście (impuls przez 1 cykl)

    // Pamięć wewnętrzna
    bool prev_state; // Poprzedni stan wejścia CLK
} R_TRIG_Block;

/* --- Prototypy Funkcji --- */
/*
 * @parametr t Wskaźnik do instancji bloku TON i TOF.
 * @parametr c wskaźnik do instancji bloku CTU I CTD.
 */
void TON_Update(TON_Block *t);
void TOF_Update(TOF_Block *t);
void CTU_Update(CTU_Block *c);
void CTD_Update(CTD_Block *c);
void R_TRIG_Update(R_TRIG_Block *t);

#endif /* INC_PLC_BLOCKS_H_ */
