#include "plc_blocks.h"
// Potrzebujemy HAL_GetTick()
#include "main.h"

extern TIM_HandleTypeDef htim4;
/**
 * @brief Implementacja logiki timera TON.
 */
void TON_Update(TON_Block *t) {
    // Ta funkcja jest skopiowana 1:1 z Twojego main.c
    // Używa wskaźnika 't', aby modyfikować ORYGINAŁ

    uint32_t currentTick = HAL_GetTick();

    if (t->IN) {
        if (!t->running) {
            // Start timera
            t->running = true;
            t->startTime = currentTick;
            t->ET = 0;
            t->Q = false;
        } else {
            // Timer działa - oblicz czas, który upłynął
            t->ET = currentTick - t->startTime;

            if (t->ET >= t->PT) {
                t->Q = true;  // Czas upłynął
                t->ET = t->PT; // Ogranicz ET do PT
            } else {
                t->Q = false; // Czas jeszcze nie upłynął
            }
        }
    } else {
        // Wejście nieaktywne - reset timera
        t->running = false;
        t->Q = false;
        t->ET = 0;
    }
}

void TOF_Update(TOF_Block *t) {
    uint32_t currentTick = HAL_GetTick();

    if (t->IN) {
        // Wejście aktywne - resetuj wszystko i włącz wyjście
        t->running = false;
        t->Q = true;
        t->ET = 0;
    } else {
        // Wejście nieaktywne - czas na opóźnione wyłączenie

        // Sprawdź, czy to pierwszy cykl po zmianie IN (start timera)
        if (t->Q) { // Jeśli Q jest nadal true (bo było w bloku 'if')
            if (!t->running) {
                // Start timera
                t->running = true;
                t->startTime = currentTick;
                t->ET = 0;
            } else {
                // Timer działa - oblicz czas, który upłynął
                t->ET = currentTick - t->startTime;

                if (t->ET >= t->PT) {
                    t->Q = false; // Czas upłynął - wyłącz wyjście
                    t->ET = t->PT;
                }
                // Jeśli czas nie upłynął, Q pozostaje true
            }
        }
        // Jeśli t->Q jest już false, to nic nie rób (timer skończył)
    }
}

void CTU_Update(CTU_Block *c) {
    // 1. Sprawdzenie Resetu (ma najwyższy priorytet)
    if (c->R) {
        c->CV = 0;
    }
    // 2. Sprawdzenie zbocza narastającego (tylko jeśli nie ma resetu)
    else if (c->CU && !c->prev_CU_state) { // <-- POPRAWKA: c->CU zamiast c->CV
        if (c->CV < 65535) { // Zapobieganie przepełnieniu
            c->CV++;
        }
    }

    // 3. Aktualizacja wyjścia Q
    c->Q = (c->CV >= c->PV);

    // 4. ZAPISANIE STANU NA POTRZEBY NASTĘPNEGO CYKLU
    c->prev_CU_state = c->CU;
}

void CTD_Update(CTD_Block *c) {
    // 1. Sprawdzenie Load (ma najwyższy priorytet)
    if (c->LD) {
        c->CV = c->PV;
    }
    // 2. Sprawdzenie zbocza narastającego na CD (tylko jeśli nie ma Load)
    else if (c->CD && !c->prev_CD_state) {
        if (c->CV > 0) { // Zapobieganie zejściu poniżej 0
            c->CV--;
        }
    }

    // 3. Aktualizacja wyjścia Q
    //    Wyjście Q jest aktywne, gdy CV jest równe 0
    c->Q = (c->CV == 0);

    // 4. ZAPISANIE STANU NA POTRZEBY NASTĘPNEGO CYKLU
    c->prev_CD_state = c->CD;
}

void R_TRIG_Update(R_TRIG_Block *t) {
    // 1. Sprawdź, czy jest zbocze narastające
    //    (CLK jest true ORAZ poprzednio było false)
    if (t->CLK && !t->prev_state) {
        t->Q = true; // Ustaw wyjście Q tylko na ten jeden cykl
    } else {
        t->Q = false;
    }

    // 2. ZAPISANIE STANU NA POTRZEBY NASTĘPNEGO CYKLU
    t->prev_state = t->CLK;
}
