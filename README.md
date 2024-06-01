# Techniki Programowania - Projekt 4
Projekt polega na wczytaniu, przetworzeniu oraz zwizualizowaniu sygnałów występujących w systemach robotycznych. Projekt będzie realizowany z wykorzystaniem programu CMake.


# Korzystanie z szablonu
## Pobranie projektu
W celu pobrania zależności (biblioteki Eigen oraz Matplot++), nalezy zaciągnąć repozytorium wraz z submodułami korzystając z flagi `--recursive`

`git clone --recursive https://github.com/marekjg/tp_projekt_4`

## Budowanie projektu
Korzystając z Visual Code i dodatku CMake tools, można buować i uruchamiać program z dolnego paska (sposób sprawdzony na windowsie). Można też z linii poleceń:

```bash
mkdir build
cd build
cmake ..
make -j16  # zmienić 16 na liczbę rdzeni
```

## Uruchomienie aplikacji
Tak jak budowanie, można wykorzystać CMake tools w Visual Studio Code albo znaleźć plik wykonywalny (pod nazwą z `add_executable` w pliku `CMakeLists.txt`).

# Zadanie projektowe
1. Uzupełnić kod zgodnie z polami TODO w plikach `PlanarQuadrotor`
2. Dodać bibliotekę do odtwarzania dźwięku i odtwarzać dźwięk który narasta wraz z wiekszym sterowaniem (input w klasie PlanarQuadrotor)
3. Dopasowanie macierzy Q i R w celu polepszenia dokładności sterowania
4. Optymalizacja macierzy Q/R w celu minimalizacji czasu przelotu
