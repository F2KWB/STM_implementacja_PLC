# STM32 Symulator PLC 

Projekt symulatora sterownika PLC napisanego na mikrokontroler STM32 w języku C. Celem było odtworzenie mechanizmów znanych z prawdziwych sterowników przemysłowych (stały czas cyklu, standardowe bloki funkcyjne, obsługa błędów) w środowisku embedded.

Projekt bazuje na konfiguracji wygenerowanej przez STM32CubeMX. Właściwy kod znajduje się w:

* `Core/Src/plc_blocks.c` / `Core/Inc/plc_blocks.h` - Biblioteka z implementacją standardowych bloków IEC 61131-3: timery (TON, TOF), liczniki (CTU, CTD) oraz detekcja zbocza (R_TRIG).
* `Core/Src/main.c` - Główna pętla z rygorystycznym czasem cyklu (50 ms) oraz przykładowe logiki sterowania procesami (parking, napełnianie zbiornika, mieszalnik). 

## Główne założenia

* **Cykl skanu PLC:** Program działa w pętli ze stałym czasem cyklu 50 ms.
* **Watchdog (IWDG):** W przypadku wykrycia stanu krytycznego (np. jednoczesny sygnał wjazdu i wyjazdu) system wchodzi w pętlę nieskończoną, co wyzwala sprzętowy reset.
* **Pamięć trwała:** Najważniejsze dane, takie jak stan liczników serwisowych czy ilość pojazdów na parkingu, są trzymane w rejestrach Backup RTC, aby przetrwały zanik zasilania.
* **Diagnostyka UART:** System wysyła "interfejs HMI" po porcie szeregowym. W terminalu (np. PuTTY) można na żywo podglądać paski postępu, statusy timerów i flagi.

## Sprzęt i peryferia

Projekt został uruchomiony na zestawie z rodziny Nucleo. Użyte technologie:
* **Język:** C (z wykorzystaniem bibliotek HAL)
* **Peryferia STM32:**
  * `GPIO` - obsługa przycisków, diod statusowych oraz triggera dla czujnika HC-SR04
  * `ADC` - odczyt sygnałów analogowych (potencjometr symulujący poziom w zbiorniku)
  * `UART` - terminal
  * `RTC` & `IWDG`

## Stanowisko testowe

<img width="1152" height="2048" alt="image" src="https://github.com/user-attachments/assets/94d286d9-6038-4d56-a408-ce3aaae03dfa" />

## Interfejs

<img width="377" height="298" alt="image" src="https://github.com/user-attachments/assets/efb781d2-30be-4b83-972d-788c9fa757d3" />
<img width="380" height="284" alt="image" src="https://github.com/user-attachments/assets/2c1e9d8c-3f2e-4e56-80f9-cdfc07b77cf2" />
