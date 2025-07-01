#include <Servo.h>
#include <limits.h> // Für ULONG_MAX
#include <math.h>   // Für sqrt() Funktion und Standardabweichung

// Pin-Definitionen für den Farbsensor (ÜBERNOMMEN AUS DEINEM CODE)
const int S0 = 8;  // Verbindung der Farbsensorkontakte mit dem Arduino festlegen
const int S1 = 9;
const int S2 = 12;
const int S3 = 11;
const int signal = 10; // 'out' in deinem Code ist der Signal-Pin des Farbsensors

// Pin-Definitionen für die LEDs (ÜBERNOMMEN AUS DEINEM CODE)
int roteLED = 2;   // Verbindung der LEDs mit dem Arduino festlegen
int grueneLED = 3;
int blaueLED = 4;

// (Diese Variablen sind im komplexen Code nicht direkt für die Erkennung notwendig,
// werden aber der Vollständigkeit halber deklariert, um deinem Original zu entsprechen.)
int rot = 0;
int gruen = 0;
int blau = 0;

// Servo-Pin-Definitionen (wie im komplexen Code, angepasst um Konflikte zu vermeiden)
int servoPinRot = 5; // Beispiel-Pin für Servo 1
int servoPinGruen = 6; // Beispiel-Pin für Servo 2
int servoPinBlau = 7; // Beispiel-Pin für Servo 3

// Servo-Objekte deklarieren
Servo ServoRot;
Servo ServoGruen;
Servo ServoBlau;

// Servo-Konstanten
const int SERVO_ROT_FORWARD = 0;           // Eine Drehrichtung für den Endlos-Servo
const int SERVO_ROT_BACKWARD = 180;        // Andere Drehrichtung
const int SERVO_STOP_VALUE = 90;           // Wert, um den Endlos-Servo zu stoppen
const int SERVO_ROTATION_DURATION = 1000;  // Dauer der Servo-Aktion in Millisekunden (1 Sekunde)

// --- Erweiterte Kalibrierungsprofile für jede Farbe ---
// Speichert gelernten Durchschnitt, Rohwert-Bereich und Standardabweichung pro Kanal
struct ColorProfileV2 {
  int redAvg;   // Durchschnittlicher gemappter Rotwert (0-255)
  int greenAvg; // Durchschnittlicher gemappter Grünwert (0-255)
  int blueAvg;  // Durchschnittlicher gemappter Blauwert (0-255)

  // Gelerntes Minimum und Maximum der ROHWERTE für jeden Kanal während der Kalibrierung
  unsigned long minRedRaw;
  unsigned long maxRedRaw;
  unsigned long minGreenRaw;
  unsigned long maxGreenRaw;
  unsigned long minBlueRaw;
  unsigned long maxBlueRaw;

  // Standardabweichung der ROHWERTE für jeden Kanal
  float stdDevRed;
  float stdDevGreen;
  float stdDevBlue;

  // Dynamisch berechnete Toleranz für diese Farbe (basierend auf Standardabweichung)
  int dynamicTolerance;
};

// Farbprofile für Rot, Grün, Blau und Weiß (Förderband)
ColorProfileV2 redProfile;
ColorProfileV2 greenProfile;
ColorProfileV2 blueProfile;
ColorProfileV2 whiteProfile;

// Aktuell gemappte RGB-Werte (0-255 Bereich) - global zugänglich
int redMapped;
int greenMapped;
int blueMapped;

// --- Globale Variablen für den gesamten Rohwert-Bereich zur besseren Abbildung ---
// Diese Werte definieren die Bandbreite, die der Sensor insgesamt erfasst hat.
unsigned long globalMinRawPulse = ULONG_MAX; // Initialisiert mit dem größtmöglichen unsigned long Wert
unsigned long globalMaxRawPulse = 0;         // Initialisiert mit 0 (kleinstmöglicher Wert)

// Anzahl der Einzelmessungen, die für jeden Durchschnittswert (readRawColors) herangezogen werden
const int NUM_MEASUREMENTS_FOR_AVG = 20;

// Dauer der Kalibrierung pro Farbe in Millisekunden (10 Sekunden pro Farbe)
const unsigned long CALIBRATION_DURATION_MS = 10000;

// --- Hilfsfunktion zum Lesen von Rohfarbwerten (mit Mittelwertbildung) ---
// Diese Funktion liest jeden Farbkanal NUM_MEASUREMENTS_FOR_AVG mal aus
// und berechnet den Durchschnitt, um Rauschen zu reduzieren.
void readRawColors(unsigned long &r, unsigned long &g, unsigned long &b) {
  unsigned long sumR = 0, sumG = 0, sumB = 0;
  int validReadings = 0;

  for (int i = 0; i < NUM_MEASUREMENTS_FOR_AVG; i++) {
    unsigned long currentR_local, currentG_local, currentB_local;

    // Rotfilter einstellen und messen
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    currentR_local = pulseIn(signal, HIGH); // Misst die Impulsbreite, wenn Signal HIGH ist

    // Grünfilter einstellen und messen
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    currentG_local = pulseIn(signal, HIGH);

    // Blaufilter einstellen und messen
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    currentB_local = pulseIn(signal, HIGH);

    // Nur gültige Messwerte (pulseIn gibt 0 bei Timeout oder fehlendem Signal) addieren
    if (currentR_local > 0 && currentG_local > 0 && currentB_local > 0) {
      sumR += currentR_local;
      sumG += currentG_local;
      sumB += currentB_local;
      validReadings++;
    }
  }

  // Durchschnitt berechnen, wenn gültige Messwerte vorliegen
  if (validReadings > 0) {
    r = sumR / validReadings;
    g = sumG / validReadings;
    b = sumB / validReadings;
  } else {
    r = 0; g = 0; b = 0; // Fehlerfall: keine gültigen Messwerte
  }
}

// --- Hilfsfunktion zum Lesen des Clear-Kanals (für Weißkalibrierung, mit Mittelwertbildung) ---
// Der Clear-Kanal misst die Gesamtlichtintensität ohne Farbfilter.
unsigned long readClearRaw() {
  unsigned long sumClear = 0;
  int validReadings = 0;
  for (int i = 0; i < NUM_MEASUREMENTS_FOR_AVG; i++) {
    digitalWrite(S2, HIGH);
    digitalWrite(S3, LOW); // Clear-Filter
    unsigned long currentClear_local = pulseIn(signal, HIGH);
    if (currentClear_local > 0) {
      sumClear += currentClear_local;
      validReadings++;
    }
  }
  return (validReadings > 0) ? (sumClear / validReadings) : 0;
}

// --- Hilfsfunktion zur Berechnung der Standardabweichung ---
// Misst die Streuung der Datenpunkte um den Mittelwert.
float calculateStandardDeviation(unsigned long data[], int n, unsigned long mean) {
  if (n <= 1) return 0; // Standardabweichung nicht definierbar für 0 oder 1 Punkt

  float sumOfSqDiff = 0;
  for (int i = 0; i < n; i++) {
    // Sicherstellen, dass nur die tatsächlich gesammelten Stichproben verwendet werden
    // Optional: 0-Werte, die von pulseIn bei Timeout kommen, ignorieren, falls Array ungefüllt ist
    if (i < n && data[i] > 0) {
      sumOfSqDiff += pow((long)data[i] - (long)mean, 2); // Expliziter Cast zu long für pow
    }
  }
  return sqrt(sumOfSqDiff / (n - 1)); // Stichprobenstandardabweichung (n-1)
}


// --- Funktion zur Kalibrierung einer einzelnen Farbe (hochkomplex) ---
// Diese Funktion sammelt umfassende Daten über eine Farbe und füllt das ColorProfileV2.
void calibrateSingleColor(const char* colorName, ColorProfileV2 &profileToFill, bool isWhiteCalibration = false) {
  Serial.print("--- KALIBRIERUNG: "); Serial.print(colorName); Serial.println(" ---");
  Serial.print("Halte JETZT den "); Serial.print(colorName); Serial.println(" Klotz vor den Sensor.");
  Serial.println("Messung startet in 3 Sekunden... Halte EXTREM STABIL und im PRÄZISEN ABSTAND!");
  delay(3000);

  unsigned long startTime = millis();
  unsigned long redSum = 0, greenSum = 0, blueSum = 0;
  unsigned long clearSum = 0;
  unsigned long count = 0; // Anzahl der gemittelten Messzyklen

  // Arrays zum Speichern der gemittelten Rohwerte für die Standardabweichungsberechnung
  // Eine Schätzung: ca. 10ms pro readRawColors + Serial.print -> maxCalibrationSamples = 10000ms / 10ms = 1000 Samples.
  // Das Arduino hat begrenzten SRAM, daher muss dieser Wert realistisch bleiben.
  const int maxCalibrationSamples = 1000;
  unsigned long redSamples[maxCalibrationSamples];
  unsigned long greenSamples[maxCalibrationSamples];
  unsigned long blueSamples[maxCalibrationSamples];
  int sampleIndex = 0; // Zähler für die tatsächlich gesammelten Samples

  // Initialisiere die Min/Max-Rohwerte für dieses spezifische Profil
  profileToFill.minRedRaw = ULONG_MAX; profileToFill.maxRedRaw = 0;
  profileToFill.minGreenRaw = ULONG_MAX; profileToFill.maxGreenRaw = 0;
  profileToFill.minBlueRaw = ULONG_MAX; profileToFill.maxBlueRaw = 0;

  Serial.print("Kalibriere "); Serial.print(colorName); Serial.print("... ("); Serial.print(CALIBRATION_DURATION_MS / 1000); Serial.println(" Sekunden)");

  // Schleife für die Kalibrierungsdauer
  while (millis() - startTime < CALIBRATION_DURATION_MS && sampleIndex < maxCalibrationSamples) {
    unsigned long currentR, currentG, currentB;
    readRawColors(currentR, currentG, currentB); // Liest gemittelte Rohwerte
    unsigned long currentClear = 0;
    if (isWhiteCalibration) currentClear = readClearRaw(); // Liest gemittelten Clear-Wert für Weiß

    // Nur gültige (nicht-null) Messwerte verarbeiten
    if (currentR > 0 && currentG > 0 && currentB > 0 && (!isWhiteCalibration || currentClear > 0)) {
      redSum += currentR; greenSum += currentG; blueSum += currentB;
      if (isWhiteCalibration) clearSum += currentClear;
      
      // Speichere die gemittelten Rohwerte für die Standardabweichungsberechnung
      redSamples[sampleIndex] = currentR;
      greenSamples[sampleIndex] = currentG;
      blueSamples[sampleIndex] = currentB;
      sampleIndex++;
      count++; // Zählt die Anzahl der erfolgreich gesammelten "durchschnittlichen" Samples

      // Update lokale Min/Max-Rohwerte für diesen Farbtyp (für isWithinRawRange-Check)
      if (currentR < profileToFill.minRedRaw) profileToFill.minRedRaw = currentR;
      if (currentR > profileToFill.maxRedRaw) profileToFill.maxRedRaw = currentR;
      if (currentG < profileToFill.minGreenRaw) profileToFill.minGreenRaw = currentG;
      if (currentG > profileToFill.maxGreenRaw) profileToFill.maxGreenRaw = currentG;
      if (currentB < profileToFill.minBlueRaw) profileToFill.minBlueRaw = currentB;
      if (currentB > profileToFill.maxBlueRaw) profileToFill.maxBlueRaw = currentB;

      // Update globale Min/Max-Rohwerte für das gesamte Mapping-Spektrum (globalMinRawPulse, globalMaxRawPulse)
      if (currentR < globalMinRawPulse) globalMinRawPulse = currentR; if (currentR > globalMaxRawPulse) globalMaxRawPulse = currentR;
      if (currentG < globalMinRawPulse) globalMinRawPulse = currentG; if (currentG > globalMaxRawPulse) globalMaxRawPulse = currentG;
      if (currentB < globalMinRawPulse) globalMinRawPulse = currentB; if (currentB > globalMaxRawPulse) globalMaxRawPulse = currentB;
      if (isWhiteCalibration) { // Clear-Kanal auch für globale Min/Max betrachten, da er die Gesamtlichtintensität widerspiegelt
        if (currentClear < globalMinRawPulse) globalMinRawPulse = currentClear;
        if (currentClear > globalMaxRawPulse) globalMaxRawPulse = currentClear;
      }
    }
  }

  // Fehlerbehandlung: Wenn keine gültigen Messwerte gesammelt wurden
  if (count == 0) {
    Serial.println("FEHLER: Keine gültigen Messwerte während der Kalibrierung erhalten! Profilwerte und Toleranz auf Standard gesetzt.");
    profileToFill.redAvg = 0; profileToFill.greenAvg = 0; profileToFill.blueAvg = 0;
    profileToFill.dynamicTolerance = 50; // Hohe Standardtoleranz bei Fehler
    profileToFill.stdDevRed = 0; profileToFill.stdDevGreen = 0; profileToFill.stdDevBlue = 0;
    // Setze auch die lokalen Rohwert-Min/Max auf Standardwerte
    profileToFill.minRedRaw = 0; profileToFill.maxRedRaw = 255;
    profileToFill.minGreenRaw = 0; profileToFill.maxGreenRaw = 255;
    profileToFill.minBlueRaw = 0; profileToFill.maxBlueRaw = 255;
    return;
  }

  // Durchschnittliche Rohwerte berechnen
  unsigned long avgRedRaw = redSum / count;
  unsigned long avgGreenRaw = greenSum / count;
  unsigned long avgBlueRaw = blueSum / count;

  // Standardabweichungen der Rohwerte berechnen (nur für die tatsächlich gesammelten Samples)
  profileToFill.stdDevRed = calculateStandardDeviation(redSamples, sampleIndex, avgRedRaw);
  profileToFill.stdDevGreen = calculateStandardDeviation(greenSamples, sampleIndex, avgGreenRaw);
  profileToFill.stdDevBlue = calculateStandardDeviation(blueSamples, sampleIndex, avgBlueRaw);
  
  // Debug-Ausgabe der Rohwerte-Bereiche und Standardabweichungen
  Serial.print("  Rohwerte-Bereich (Lokal): R("); Serial.print(profileToFill.minRedRaw); Serial.print("-"); Serial.print(profileToFill.maxRedRaw); Serial.print("), G("); Serial.print(profileToFill.minGreenRaw); Serial.print("-"); Serial.print(profileToFill.maxGreenRaw); Serial.print("), B("); Serial.print(profileToFill.minBlueRaw); Serial.print("-"); Serial.print(profileToFill.maxBlueRaw); Serial.print(")");
  Serial.print(", StdDev: R="); Serial.print(profileToFill.stdDevRed, 2); Serial.print(", G="); Serial.print(profileToFill.stdDevGreen, 2); Serial.print(", B="); Serial.print(profileToFill.stdDevBlue, 2); Serial.println();


  // Sicherstellen, dass der globale Mapping-Bereich nicht zu klein ist oder uninitialisiert
  // Dies dient als Fallback, falls globalMinRawPulse/globalMaxRawPulse nicht sinnvoll aktualisiert wurden.
  unsigned long effectiveMinRaw = (globalMinRawPulse == ULONG_MAX) ? 80 : globalMinRawPulse;
  unsigned long effectiveMaxRaw = (globalMaxRawPulse == 0) ? 260 : globalMaxRawPulse;
  if (effectiveMaxRaw <= effectiveMinRaw) effectiveMaxRaw = effectiveMinRaw + 1; // Sicherstellen, dass Division durch Null vermieden wird

  // Durchschnittliche gemappte RGB-Werte für das Profil
  profileToFill.redAvg = map(avgRedRaw, effectiveMaxRaw, effectiveMinRaw, 0, 255);
  profileToFill.greenAvg = map(avgGreenRaw, effectiveMaxRaw, effectiveMinRaw, 0, 255);
  profileToFill.blueAvg = map(avgBlueRaw, effectiveMaxRaw, effectiveMinRaw, 0, 255);

  // Dynamische Toleranzberechnung: Basierend auf der maximalen Standardabweichung der Rohwerte
  // Ein höherer 'toleranceMultiplier' macht die Erkennung nachsichtiger (größere Toleranz).
  // Ein Wert von 2.0 bis 3.0 ist oft ein guter Startpunkt.
  float toleranceMultiplier = 2.5; // Experimentiere mit diesem Wert!
  profileToFill.dynamicTolerance = (int)(max(profileToFill.stdDevRed, max(profileToFill.stdDevGreen, profileToFill.stdDevBlue)) * toleranceMultiplier);
  if (profileToFill.dynamicTolerance < 20) profileToFill.dynamicTolerance = 20; // Mindesttoleranz, um überhaupt eine Erkennung zu ermöglichen

  // Ausgabe des fertigen Farbprofils
  Serial.print(colorName); Serial.print(" Profil: R_Avg="); Serial.print(profileToFill.redAvg);
  Serial.print(", G_Avg="); Serial.print(profileToFill.greenAvg);
  Serial.print(", B_Avg="); Serial.print(profileToFill.blueAvg);
  Serial.print(", Tol="); Serial.print(profileToFill.dynamicTolerance);
  Serial.print(", Roh_MinMax: R("); Serial.print(profileToFill.minRedRaw); Serial.print("-"); Serial.print(profileToFill.maxRedRaw);
  Serial.print("), G("); Serial.print(profileToFill.minGreenRaw); Serial.print("-"); Serial.print(profileToFill.maxGreenRaw);
  Serial.print("), B("); Serial.print(profileToFill.minBlueRaw); Serial.print("-"); Serial.print(profileToFill.maxBlueRaw); Serial.println(")");
}

// --- Arduino Setup Funktion ---
void setup() {
  // Servo-Pins binden (können auch entfernt werden, falls keine Servos verwendet werden)
  ServoRot.attach(servoPinRot);
  ServoGruen.attach(servoPinGruen);
  ServoBlau.attach(servoPinBlau);

  // Farbsensor-Pin-Modi setzen (ÜBERNOMMEN AUS DEINEM CODE)
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(signal, INPUT); // Der Signal-Pin ist ein Input

  // LED-Pin-Modi setzen (ÜBERNOMMEN AUS DEINEM CODE)
  pinMode(roteLED, OUTPUT);
  pinMode(grueneLED, OUTPUT);
  pinMode(blaueLED, OUTPUT);

  // Frequenzskalierung des Farbsensors auf 100 % setzen (ÜBERNOMMEN AUS DEINEM CODE)
  // Wenn deine bisherigen Tests mit HIGH/HIGH funktioniert haben, belasse es dabei.
  // Häufig ist S0=HIGH, S1=LOW (20% Skalierung) stabiler.
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);

  // Serielle Kommunikation starten
  Serial.begin(9600);
  Serial.println("Farbsensor gestartet - Komplexe Kalibrierung und Algorithmus.");
  Serial.println("---------------------------------------------------------------");

  Serial.println("--- KALIBRIERUNG STARTET ---");
  Serial.println("BITTE SEI SEHR PRÄZISE UND STABIL WÄHREND DER MESSUNGEN!");
  
  // Kalibriere jede Farbe einzeln und fülle ihr Profil
  calibrateSingleColor("ROT", redProfile);
  calibrateSingleColor("BLAU", blueProfile);
  calibrateSingleColor("GRUEN", greenProfile);
  calibrateSingleColor("WEISS (Förderband)", whiteProfile, true); // True für isWhiteCalibration des Clear-Kanals

  // Finalen globalen Rohwert-Mapping-Bereich überprüfen und ausgeben
  if (globalMaxRawPulse == 0 || globalMinRawPulse == ULONG_MAX || (globalMaxRawPulse - globalMinRawPulse) < 50) {
      Serial.println("WARNUNG: Globaler Rohwertbereich zu klein oder nicht erfasst. Setze Fallback-Mapping.");
      // Fallback-Werte, die für viele TCS3200-Sensoren passen könnten
      globalMinRawPulse = 80;
      globalMaxRawPulse = 260;
  }
  Serial.print("\nFinaler Globaler Rohwert-Mapping-Bereich: Min="); Serial.print(globalMinRawPulse); Serial.print(", Max="); Serial.println(globalMaxRawPulse);

  Serial.println("--- KALIBRIERUNG ABGESCHLOSSEN ---");
  Serial.println("\nFarberkennung läuft. Beobachte die Ausgaben...");
  Serial.println("---------------------------------------------------------------");
}

// --- Funktion zur Berechnung der "Distanz" zwischen zwei Farben (Euklidische Distanz im RGB-Raum) ---
// Gibt die "Unähnlichkeit" zweier Farben als numerischen Wert zurück.
int getColorDistance(int r1, int g1, int b1, int r2, int g2, int b2) {
  long dr = (long)r1 - r2;
  long dg = (long)g1 - g2;
  long db = (long)b1 - b2;
  return (int)sqrt((double)(dr * dr + dg * dg + db * db));
}

// --- Hilfsfunktion zur Überprüfung, ob aktuelle Rohwerte innerhalb der gelernten Range liegen ---
// Zusätzliche Prüfebene, um Fehlidentifikationen zu vermeiden.
bool isWithinRawRange(unsigned long currentR, unsigned long currentG, unsigned long currentB, const ColorProfileV2& profile) {
  // Überprüft, ob jeder der aktuellen Rohwerte innerhalb des gelernten Min/Max-Bereichs liegt.
  return (currentR >= profile.minRedRaw && currentR <= profile.maxRedRaw &&
          currentG >= profile.minGreenRaw && currentG <= profile.maxGreenRaw &&
          currentB >= profile.minBlueRaw && currentB <= profile.maxBlueRaw);
}


// --- Arduino Loop Funktion ---
void loop() {
  unsigned long currentRedRaw, currentGreenRaw, currentBlueRaw;
  readRawColors(currentRedRaw, currentGreenRaw, currentBlueRaw); // Liest aktuelle Rohwerte (gemittelt)

  // Mapping der Rohwerte zu 0-255 mit dem GELERNTEN globalen Bereich
  // Wichtig: Hoher pulseIn-Rohwert bedeutet weniger Licht (dunkler), daher Max und Min in map() vertauscht.
  redMapped = map(currentRedRaw, globalMaxRawPulse, globalMinRawPulse, 0, 255);
  greenMapped = map(currentGreenRaw, globalMaxRawPulse, globalMinRawPulse, 0, 255);
  blueMapped = map(currentBlueRaw, globalMaxRawPulse, globalMinRawPulse, 0, 255);

  // Sicherstellen, dass die gemappten Werte im Bereich 0-255 bleiben (Constrain)
  redMapped = constrain(redMapped, 0, 255);
  greenMapped = constrain(greenMapped, 0, 255);
  blueMapped = constrain(blueMapped, 0, 255);

  // Ausgabe der gemappten Werte auf den Variablen 'rot', 'gruen', 'blau' (wie in deinem Originalcode)
  // Dies sind die gemappten RGB-Werte, nicht die Rohwerte.
  rot = redMapped;
  gruen = greenMapped;
  blau = blueMapped;


  // --- Debug-Ausgabe der aktuellen Werte im Loop ---
  Serial.print("Raw: R="); Serial.print(currentRedRaw); Serial.print(", G="); Serial.print(currentGreenRaw); Serial.print(", B="); Serial.print(currentBlueRaw);
  Serial.print(" | Mapped: R="); Serial.print(redMapped); Serial.print(", G="); Serial.print(greenMapped); Serial.print(", B="); Serial.print(blueMapped);
  Serial.print(" -> ");

  // --- Farberkennungslogik mittels Distanz zu gelernten Profilen ---
  // Berechne die euklidische Distanz zu jedem kalibrierten Farbprofil
  int distRed = getColorDistance(redMapped, greenMapped, blueMapped, redProfile.redAvg, redProfile.greenAvg, redProfile.blueAvg);
  int distGreen = getColorDistance(redMapped, greenMapped, blueMapped, greenProfile.redAvg, greenProfile.greenAvg, greenProfile.blueAvg);
  int distBlue = getColorDistance(redMapped, greenMapped, blueMapped, blueProfile.redAvg, blueProfile.greenAvg, blueProfile.blueAvg);
  int distWhite = getColorDistance(redMapped, greenMapped, blueMapped, whiteProfile.redAvg, whiteProfile.greenAvg, whiteProfile.blueAvg);

  // Debug-Ausgabe der Distanzen und gelernten Toleranzen
  Serial.print("Dists: R="); Serial.print(distRed); Serial.print(" (Tol "); Serial.print(redProfile.dynamicTolerance); Serial.print(")");
  Serial.print(", G="); Serial.print(distGreen); Serial.print(" (Tol "); Serial.print(greenProfile.dynamicTolerance); Serial.print(")");
  Serial.print(", B="); Serial.print(distBlue); Serial.print(" (Tol "); Serial.print(blueProfile.dynamicTolerance); Serial.print(")");
  Serial.print(", W="); Serial.print(distWhite); Serial.print(" (Tol "); Serial.print(whiteProfile.dynamicTolerance); Serial.print(")");
  Serial.print(" -> ");

  String detectedColor = "UNBEKANNT";
  int bestDistance = 9999; // Initialisiere mit einem sehr hohen Wert

  // Prüfung der Farberkennung:
  // 1. Die euklidische Distanz zum Profil muss innerhalb der gelernten dynamischen Toleranz liegen.
  // 2. Die aktuellen ROHWERTE müssen innerhalb der gelernten Min/Max-Bereiche dieser Referenzfarbe liegen.
  // 3. Von allen Farben, die diese Kriterien erfüllen, wird diejenige mit der kleinsten Distanz ausgewählt.

  // Prüfe Rot
  if (distRed <= redProfile.dynamicTolerance && isWithinRawRange(currentRedRaw, currentGreenRaw, currentBlueRaw, redProfile)) {
    if (distRed < bestDistance) {
      bestDistance = distRed;
      detectedColor = "Rot";
    }
  }
  // Prüfe Grün
  if (distGreen <= greenProfile.dynamicTolerance && isWithinRawRange(currentRedRaw, currentGreenRaw, currentBlueRaw, greenProfile)) {
    if (distGreen < bestDistance) {
      bestDistance = distGreen;
      detectedColor = "Gruen";
    }
  }
  // Prüfe Blau
  if (distBlue <= blueProfile.dynamicTolerance && isWithinRawRange(currentRedRaw, currentGreenRaw, currentBlueRaw, blueProfile)) {
    if (distBlue < bestDistance) {
      bestDistance = distBlue;
      detectedColor = "Blau";
    }
  }
  // Prüfe Weiß (oft breitere Toleranzen, daher als letzte der Hauptfarben geprüft,
  // um Überschneidungen mit reineren Farben zu minimieren, falls deren Toleranz kleiner ist)
  if (distWhite <= whiteProfile.dynamicTolerance && isWithinRawRange(currentRedRaw, currentGreenRaw, currentBlueRaw, whiteProfile)) {
    if (distWhite < bestDistance) {
      bestDistance = distWhite;
      detectedColor = "WEISSES Förderband"; // Dies sollte die Logik sein, die du für das Förderband wolltest
    }
  }
  
  // Ausgabe des erkannten Zustands auf dem seriellen Monitor
  Serial.print("Erkannt: ");
  Serial.println(detectedColor);

  // Steuerung der LEDs
  digitalWrite(roteLED, LOW);
  digitalWrite(grueneLED, LOW);
  digitalWrite(blaueLED, LOW); // Alle LEDs zuerst ausschalten

  if (detectedColor == "Rot") {
    digitalWrite(roteLED, HIGH);
    ServoRot.write(SERVO_ROT_FORWARD); // Servoaktion
  } else if (detectedColor == "Gruen") {
    digitalWrite(grueneLED, HIGH);
    ServoGruen.write(SERVO_ROT_FORWARD); // Servoaktion
  } else if (detectedColor == "Blau") {
    digitalWrite(blaueLED, HIGH);
    ServoBlau.write(SERVO_ROT_FORWARD); // Servoaktion
  }
  // Für "WEISSES Förderband" oder "UNBEKANNT" bleiben alle LEDs aus und keine Servo-Aktion.

  delay(SERVO_ROTATION_DURATION); // Warte, bis Servo-Aktion beendet ist (falls aktiv)
  // Servos stoppen nach ihrer Aktion
  if (detectedColor == "Rot" || detectedColor == "Gruen" || detectedColor == "Blau") {
    ServoRot.write(SERVO_STOP_VALUE);
    ServoGruen.write(SERVO_STOP_VALUE);
    ServoBlau.write(SERVO_STOP_VALUE);
  }

  delay(300); // Eine kleine Verzögerung zwischen den gesamten Messzyklen, wie in deinem Originalcode
}