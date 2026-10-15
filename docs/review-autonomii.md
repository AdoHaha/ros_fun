# Review ćwiczeń dotyczących autonomii

Data: 2026-10-15. Zakres: notebooki 6–14 w `jupyter_notebooks/exercises/`,
ich zależności i wejście ze spisu treści. Review dotyczy możliwości przejścia
materiału po kolei, poprawności przykładów oraz kompletności poleceń.

Ustalenie po review: użytkownik zlecił poprawki z wyjątkiem tłumaczenia.
Ćwiczenia 12–14 pozostają po angielsku; pozostałe zachowują dotychczasowy język.
Nie należy tłumaczyć nazw API ani identyfikatorów.

## Ustalenia i zakres poprawek

| Punkt | Priorytet | Problem przed poprawką | Oczekiwana poprawka |
| --- | --- | --- | --- |
| 1 | P1 | Notebook 7, komórka 9: niezdefiniowane `self`, `goal_msg` i `_feedbackCallback`; wykonanie kończy się `NameError`. Odczyt uchwytu celu nie czeka jawnie na zakończenie future. | Kompletny klient asynchroniczny z oczekiwaniem na serwer i przyjęcie celu, obsługą feedbacku, wyniku, anulowania i sprzątania. Jawne wymaganie działającego środowiska z notebooka 6. |
| 2 | P1 | Notebook 6, komórka 19: kwaternion `(0, 0, 0, 2)` oraz kwaternion zerowy nie opisują poprawnego obrotu. | Wyjaśnienie orientacji, przeliczenie yaw na kwaternion przed pierwszym użyciem i poprawne orientacje celów. |
| 3 | P2 | Zadania w 9–10 odsyłają do plików `.py`, przebudowy i wcześniejszych komórek. Brakuje kompletnej sekwencji pod poleceniem. Ponowne wywołanie przygotowania nie przebudowuje już zainstalowanej paczki. Restart tutoriala zatrzymuje także watcher. | Bezpośrednio pod zadaniami umieścić miejsce pracy lub precyzyjnie wskazany plik oraz kolejne komórki przygotowania, restartu i obserwacji. Zapewnić jawne wymuszenie przebudowy i ponowne uruchomienie obserwatora. |
| 4 | P2 | Notebook 9, ćwiczenie 1: `Send warning` dodane za stale działającym `Flash LEDs` nigdy nie jest wykonywane. Sprawdzenie samej obecności w wydruku tego nie wykrywa. | Zmiana polecenia/struktury oraz sprawdzenie wykonania ostrzeżenia i zachowania gałęzi awaryjnej. |
| 5 | P2 | Notebook 12: zadanie `audit_plan` poprzedza brak wprowadzenia do pól `State`, `Action`, `goal` i formatu wyniku `successors`. Student musi szukać informacji poza notebookiem. | Przed zadaniem dodać angielskie objaśnienie modelu i małe wykonywalne przykłady legalnych przejść, zmian energii i naliczania kosztu, bez rozwiązywania zadania. |
| 6 | P2 | Materiały 9–11 są głównie polskie, a 12–14 angielskie; w starszych ćwiczeniach występuje mieszanie języków. | **Wyłączone ze zmian na polecenie użytkownika.** Ćwiczenia 12–14 pozostają angielskie. |
| 7 | P2 | Spis treści w notebooku 1 kończy się na 8. Brakuje ścieżki do 9–14 i wskazania wymagań wstępnych. | Uzupełnić spis, wymagania i linki prowadzące kolejno do końca ścieżki. |

Numery komórek powyżej odnoszą się do wersji sprzed poprawek.

## Weryfikacja przed poprawkami

W lokalnym obrazie `adohaha/fun_ros:jazzy`, bez pobierania nowego obrazu:

- Odtworzono `NameError` z notebooka 7.
- Sprawdzono orientacje w notebooku 6: kwadraty norm wynosiły 4 i 0 zamiast 1.
- Po zmianach wymaganych przez pierwotne ćwiczenie 1 w notebooku 9 zachowanie
  `Send warning` pozostawało `INVALID` po 13 tickach.
- Dla istniejącego znacznika instalacji `ensure_ros_fun_built()` nie wywoływało
  procesu budowania. Przy instalacji symlinkowej nie każda zmiana źródła wymaga
  przebudowy, lecz instrukcja nie wyjaśniała tej różnicy.
- Pięć testów obecnego modelu restauracji przeszło.
- Startery 12–14 wykonały się od początku do końca, poprawnie zgłaszając
  niewypełnione zadania jako TODO. To nie oznacza zaliczenia zadań studenckich.

## Weryfikacja po poprawkach

Punkty 1–5 i 7 zostały poprawione. Punkt 6 pozostaje wyłączony zgodnie
z decyzją użytkownika. Zmiany przeszły przegląd agenta głównego; niezależne
przejście studenckie opisano poniżej.

- Notebook 6: wszystkie aktywne przykłady tworzą kwaterniony jednostkowe;
  helper yaw sprawdzono także dla dodatnich i ujemnych kątów.
  Kamera ma osobny executor. Trzy rzeczywiste próby callbacków obrazu równolegle
  z klientem `BasicNavigator` zakończyły się sukcesem celu, odbieraniem obrazów
  i czystym zamknięciem wątku oraz subskrypcji. Użyto serwera akcji testowej;
  dowody ruchu w Gazebo pochodzą z osobnej próby studenckiej.
- Notebook 7: w izolowanym kontenerze prawdziwy klient i serwer akcji ROS
  przeszły sześć scenariuszy: sukces, przerwanie przez serwer, odrzucenie celu,
  zaakceptowane anulowanie, odrzucone anulowanie z późniejszym sukcesem
  oraz brak serwera. Sprawdzono feedback, status i zakończenie wątku klienta.
  To test komunikacji akcji, nie dowód jazdy robota w symulacji.
- Notebooki 9–10: sprawdzono składnię, konstrukcję czterech drzew ROS,
  wymuszenie przebudowy przy istniejącym znaczniku instalacji oraz pozytywny
  i negatywny wariant ćwiczenia z ostrzeżeniem. Zadania wymagające edycji
  `.py` zachowują tę formę pracy, ale mają bezpośredni link do pliku i pełną
  sekwencję komórek pod poleceniem. Komórki nie nadpisują rozwiązań studenta.
- Notebooki 12–14: trzy niewypełnione startery przechodzą wykonanie, pozostając
  oznaczone TODO. Trzy kopie uzupełnione rozwiązaniami instruktorskimi przechodzą
  własne sprawdzenia. Nowe wprowadzenie i pozostała treść pozostają angielskie.
- Testy obecnego checkoutu i dodatkowe testy z gałęzi `ag2` (`e0a0823`):
  **16/16 przechodzi** w obrazie `adohaha/fun_ros:jazzy`. Obejmują wszystkie
  początkowe poziomy baterii i stany urządzenia, osiągalne przejścia,
  nielegalne działania, replanning, scenę i efekty adaptera Gazebo.
  Uruchomiono je w osobnej kopii, bez dodawania odpowiedzi do materiałów studentów.
- Walidacja schematów notebooków numerowanych i lokalnych linków do notebooków
  przechodzi. `git diff --check` nie zgłasza błędów.

Instruktorski `verify_notebooks.py` z `ag2` podmienia odpowiedź przez stały
indeks komórki. Po dodaniu wprowadzenia w 12 indeks się zmienia. W powyższej
weryfikacji kopii uzupełnionych wyszukiwano komórki po nazwach funkcji
`audit_plan`, `energy_report`, `recover`, zamiast opierać się na indeksach.
Nie zmieniano gałęzi instruktorskiej.

## Dodatkowe blokady wykryte w próbie studenckiej

Podczas rzeczywistego przejścia notebooka 6 agent natrafił na konflikt ABI:
obraz ładował NumPy 2.4.4 z `/usr/local`, a OpenCV i `cv_bridge` wymagały
zgodnego z nimi NumPy z pakietów Ubuntu. W obrazie jest już NumPy 1.26.4
w `/usr/lib/python3/dist-packages`.

Do `docker-compose.yml` oraz `Dockerfile` dodano `PYTHONPATH` preferujący
systemowe pakiety. Dzięki zmianie Compose poprawka działa także z dotychczasowym
opublikowanym obrazem, po odtworzeniu kontenera przez `docker compose up -d`.
Nie wymaga instalacji pakietów przez studentów. Sprawdzono importy i konwersję
obrazu RGB przez `CvBridge` w obie strony na NumPy 1.26.4 / OpenCV 4.6.0.

W notebooku 8 `ros2 param list` wywoływało usługę nowo utworzonego węzła,
zanim uruchomiono spinner. Agent niezależnie odtworzył oczekiwanie ponad 60 s
i potwierdził, że wcześniejsze uruchomienie spinnera odblokowuje listowanie
i ustawianie parametrów. Poprawiono kolejność: jawne wyjaśnienie i executor
w tle są teraz przed pierwszym wywołaniem CLI. Na końcu dodano zatrzymanie
executora, dołączenie wątku i usunięcie węzła.
Pełne wykonanie poprawionego notebooka w osobnym kontenerze przeszło,
łącznie z `ros2 param list`, ustawieniem parametru z CLI i sprzątaniem.
Lista parametrów dotyczy teraz konkretnego węzła `/test_params_rclpy`,
aby nie czekać na niespinujące węzły z innych otwartych notebooków.

W 9 szybkie wykonanie komórki po starcie tutoriala mogło dawać `Node not found`.
W 10 pojedynczy odczyt parametru raz zawisł mimo poprawnego działania kontekstu
skanowania. Dodano wspólne uruchamianie z barierą gotowości usług `/tree`
i `/battery` oraz ograniczenie czasu poleceń ROS CLI. Limit obejmuje także
procesy potomne; budowanie paczki nie ma tego krótkiego limitu. Linki do
plików `.py` otwierają teraz bezpośrednio edytor klasycznego Jupytera.
Rzeczywiste starty tutoriali 2, 5, 6 i 7 z natychmiastowymi poleceniami ROS
przeszły w osobnym kontenerze. Gotowość zajmowała od około 11 do 23 s.
Przeszły również wymuszona przebudowa, nieudany start z czytelnym timeoutem
oraz test przerwania powłoki i jej procesu potomnego bez pozostawienia sierot.
Logi i skrypty tej kontroli są w `/tmp/bt_readiness_logs/`.

## Niezależne przejście ćwiczeń

Na prośbę użytkownika osobny agent został uruchomiony bez historii rozmowy.
Otrzymał kopię materiałów studenckich bez `.git`, dokumentów review i rozwiązań
instruktorskich, w `/tmp/ros-fun-student-t2xp_nde`. Pracował we własnym kontenerze
z osobną domeną ROS. Wykonywał komórki przez IPython, pisał własne rozwiązania
i zapisywał logi. Nie oceniano klikania interfejsu w przeglądarce.

| Notebook | Wynik niezależnej próby |
| --- | --- |
| 6 | Pierwszy cel i cel za zamkiem zakończone sukcesem. Po uwadze prowadzącego pełny zamknięty patrol: **5/5 celów `SUCCEEDED`**, około **11,87 m**, **1193 obrazy** i **5 detekcji zielonej ściany**. |
| 7 | Rzeczywisty Nav2 przyjął cel, wysłał feedback i potwierdził anulowanie wynikiem `CANCELED`; klient zakończył pracę. |
| 8 | Odtworzono blokadę kolejności; samodzielne obejście uruchomienia spinnera przed CLI działa. Poprawioną wersję sprawdzono następnie osobno od początku do końca. |
| 9 | Rozwiązano zadanie z ostrzeżeniem; rzeczywiste tematy mocków potwierdziły nowy próg i kolor oraz wyłączenie LED. Wykryty wyścig startu został poprawiony i przetestowany osobno. |
| 10 | Zmieniony kolor LED i kontekst potwierdzone przez tematy, parametry i snapshoty. Bazowe anulowanie zbadano podczas odjazdu, obrotu i dokowania. Projekt rozszerzenia opisano zgodnie z dopuszczoną formą zadania; nie przypisuje mu się wyników wykonawczych. |
| 11 | Rzeczywisty patrol → przerwanie przez cel → sukces celu → powrót do patrolu → Stop. Odometria: około 2,70 m, 883 próbki; 1225 niezerowych komend prędkości; końcowa komenda zerowa, brak błędu runnera. |
| 12–14 | Trzy samodzielnie wypełnione notebooki dały `PASS`; odpowiedzi sprawdził następnie prowadzący dodatkowymi testami. |

### Kontrola pracy studenta przez prowadzącego

Przejrzano samodzielny kod `audit_plan`, `energy_report`, `recover`.
Rozwiązania nie wymagają ROS i nie importują odpowiedzi instruktorskich.
Po podstawieniu ich do zestawu z `ag2` i obecnego checkoutu **16/16 testów
przechodzi**. Dodatkowa kontrola potwierdziła legalne plany częściowe i puste,
brak modyfikowania wejściowego planu, poprawne koszty i liczbę ładowań
dla 18 konfiguracji początkowych oraz obsługę stanu już zakończonego
i stanów nieosiągalnych poza dokiem.

W logach nieudanego wznowienia patrolu 6 prowadzący znalazł ponowną publikację
domyślnej pozycji AMCL `(0, 0)`, gdy robot był już na trasie. Kod studenta tworzył
nowy `BasicNavigator` i ponownie wykonywał `waitUntilNav2Active()`. To istotna
przesłanka błędu wznowienia rozwiązania, a nie dowód wadliwych punktów trasy.
Agent otrzymał tę uwagę dopiero po pierwotnej próbie; ponowienie jest osobnym
etapem po feedbacku. W tej próbie jeden navigator był zachowany przez całą
trasę, od świeżej symulacji. Wszystkie pięć celów zamykających pętlę zakończyło
się sukcesem, w tym punkt, który wcześniej zwracał błąd. Prowadzący sprawdził
logi wyników, telemetrię, obraz kamery i kod rozwiązań. W notebooku 6 dopisano
wskazówkę o używaniu istniejącego
navigatora i zachowaniu poprawnej lokalizacji.

Próba obejmowała pierwszą turę zmian; dodatkowe blokady znalezione w jej trakcie
poprawiano i sprawdzano oddzielnymi testami opisanymi powyżej. Nie przedstawia
się tego jako jednego nieprzerwanego wykonania wszystkich końcowych notebooków.
Pełny patrol po feedbacku jest odróżniony od pierwotnego przejścia bez wskazówek.

Oryginalny raport, wypełnione notebooki, kod studenta i logi pozostają w jego
osobnej kopii. Rozwiązań nie dodano do checkoutu przeznaczonego dla studentów.

Lokalna gałąź `ag2` zawiera testy i rozwiązania instruktorskie. Są przeznaczone
do kontroli przez prowadzącego po niezależnym przejściu; nie należy ich udostępniać
agentowi wykonującemu zadania jak student. Prowadzący wykorzystał testy dopiero
do kontroli odpowiedzi i osobnych prób opisanych powyżej.
