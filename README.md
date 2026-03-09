# STM32 PLC Logic Core (Symulator Sterownika PLC)

Projekt edukacyjny / inżynierski polegający na implementacji modularnego rdzenia logicznego sterownika PLC na mikrokontrolerze z rodziny STM32. 

Celem projektu było przeniesienie koncepcji znanych z przemysłowych sterowników PLC (cykl skanu, standardowe bloki funkcyjne, obsługa I/O, determinizm czasowy) na platformę embedded programowaną w języku C.

## 🗂️ Struktura repozytorium

Większość plików konfiguracyjnych została wygenerowana przez STM32CubeMX. Nasz autorski kod i logika systemu znajdują się w następujących lokalizacjach:

* **[`Core/Src/plc_blocks.c`](Core/Src/plc_blocks.c)** oraz **[`Core/Inc/plc_blocks.h`](Core/Inc/plc_blocks.h)** * *Biblioteka implementująca standardowe bloki funkcyjne zgodne z normą IEC 61131-3.*
  * *Zawiera logikę dla: Timerów (TON, TOF), Liczników (CTU, CTD) oraz detekcji zboczy (R_TRIG).*
* **[`Core/Src/main.c`](Core/Src/main.c)** * *Główna pętla (Task Scheduler) z zachowanym stałym czasem cyklu (50 ms).*
  * *Implementacja logiki sterowania trzema przykładowymi procesami przemysłowymi: Zautomatyzowanym Parkingiem, Zbiornikiem z pompą oraz Mieszalnikiem.*
  * *Procedury bezpieczeństwa (Safety Check) i sprzężenia z peryferiami (ADC, RTC, UART).*

## ⚙️ Główne założenia i funkcje systemu

1. **Deterministyczny cykl pracy:** Program posiada zaimplementowany mechanizm stałego czasu skanu (50 ms), typowy dla sterowników PLC.
2. **Niezawodność (Watchdog):** Wdrożono sprzętowy IWDG. W przypadku błędu krytycznego (np. jednoczesny sygnał wjazdu i wyjazdu z parkingu), system odcina wyjścia i oczekuje na twardy reset.
3. **Pamięć trwała (Retentive Memory):** Zmienne takie jak stan liczników (ilość aut na parkingu, cykle serwisowe mieszalnika) są zapisywane w rejestrach Backup RTC, co pozwala na ich zachowanie po zaniku zasilania.
4. **Wizualizacja (HMI/SCADA):** System wysyła dynamiczny "Dashboard" przez port UART (widoczny np. w programie PuTTY), na którym na żywo rysowane są paski postępu, statusy timerów i stany wyjść.

## 🛠️ Wykorzystane technologie i peryferia

* **Mikrokontroler:** STM32 (konfiguracja za pomocą bibliotek HAL)
* **Język:** C
* **Peryferia STM32:** * `ADC` (odczyt sygnałów analogowych np. symulacja poziomu wody)
  * `UART` (komunikacja i wizualizacja w terminalu)
  * `RTC` (rejestry podtrzymujące dane)
  * `IWDG` (Watchdog)
  * `GPIO` (obsługa zewnętrznego czujnika odległości HC-SR04 oraz przycisków/diod)

"Fizyczny prototyp systemu na zestawie STM32 Nucleo. Widoczne podłączenie czujnika odległości HC-SR04 (symulacja bramy wjazdowej/poziomu w zbiorniku), potencjometru (ADC) oraz fizycznych wejść/wyjść (przyciski, diody statusowe)."

<img width="1152" height="2048" alt="image" src="https://github.com/user-attachments/assets/94d286d9-6038-4d56-a408-ce3aaae03dfa" />

