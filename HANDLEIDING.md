# Handleiding LiDAR Scanner Systeem

## Inhoudsopgave

### Voor Gebruikers
1. [Inleiding](#inleiding)
2. [Snelstart](#snelstart)
3. [Systeemoverzicht](#systeemoverzicht)
4. [Installatie & Setup](#installatie--setup)
5. [Gebruik](#gebruik)
6. [Output & Resultaten](#output--resultaten)
7. [Troubleshooting](#troubleshooting)

### Voor Ontwikkelaars
8. [Systeem Architectuur](#systeem-architectuur)
9. [Configuratie](#configuratie)
10. [Features Aan/Uit Zetten](#features-aanuit-zetten)
11. [Technische Details](#technische-details)

---

## Inleiding

### Doel van dit Document

Deze handleiding is primair bedoeld als **gebruikershandleiding** voor het LiDAR Scanner Systeem. Het document helpt gebruikers het systeem te installeren, op te starten en te gebruiken voor 3D scanning taken.

Daarnaast bevat dit document ook technische informatie voor **ontwikkelaars** die het systeem moeten onderhouden, aanpassen of uitbreiden. Deze informatie is duidelijk gemarkeerd en bevindt zich in de tweede helft van het document.

### Doelgroep

**Primaire doelgroep**: Eindgebruikers die het systeem willen gebruiken voor 3D scanning
- Gebruikers die een scan willen uitvoeren
- Gebruikers die het systeem moeten installeren en configureren
- Gebruikers die problemen moeten oplossen tijdens gebruik

**Secundaire doelgroep**: Ontwikkelaars die het systeem moeten onderhouden of uitbreiden
- Ontwikkelaars die features willen aanpassen
- Ontwikkelaars die het systeem moeten debuggen
- Ontwikkelaars die nieuwe functionaliteit willen toevoegen

### Documentstructuur

Het document is opgedeeld in twee delen:

**Deel 1 - Voor Gebruikers** (secties 1-7):
- Praktische instructies voor installatie en gebruik
- Hoe het systeem werkt
- Stap-voor-stap procedures
- Troubleshooting voor veelvoorkomende problemen

**Deel 2 - Voor Ontwikkelaars** (secties 8-11):
- Technische architectuurbeschrijving
- Configuratieopties en aanpassingen
- Service endpoints en data structuren
- Uitbreidingsmogelijkheden

### Wat is het LiDAR Scanner Systeem?

Het LiDAR Scanner Systeem is een geautomatiseerd 3D-scanning systeem dat gebruik maakt van ROS2 (Robot Operating System 2) en Docker containers. Het systeem maakt volledige 360-graden 3D-scans van objecten door een LiDAR sensor rond te draaien en op verschillende posities metingen te verrichten.

---

## Snelstart

Voor gebruikers die snel aan de slag willen, volg deze minimale stappen:

### Vereisten Controleren

1. **Hardware**:
   - Raspberry Pi 4 of nieuwer
   - LD14P LiDAR sensor (USB)
   - NEMA17 Stepper Motor met DRV8825 driver
   - Alle componenten correct aangesloten

2. **Software**:
   - Linux (Raspberry Pi OS aanbevolen)
   - Docker en Docker Compose geïnstalleerd

### Snelstart Stappen

1. **Output directory aanmaken**:
   ```bash
   mkdir -p /home/pi/lidar_output
   chmod 777 /home/pi/lidar_output
   ```

2. **Systeem starten**:
   ```bash
   docker-compose up
   ```

3. **Wachten op scan voltooiing**:
   - Het systeem wacht 60 seconden na opstarten
   - Voert automatisch een volledige scan uit
   - Genereert bestanden in `/home/pi/lidar_output/`

4. **Resultaten bekijken**:
   - `scan.pcd` - Point cloud data
   - `scan.obj` - 3D mesh (automatisch gegenereerd)

**Voor gedetailleerde instructies**, zie de secties [Installatie & Setup](#installatie--setup) en [Gebruik](#gebruik).

---

## Systeemoverzicht

### Hoofddoel

Het LiDAR Scanner Systeem is ontworpen om:

- **3D digitale modellen** te creëren van fysieke objecten door middel van LiDAR scanning
- **Automatische 360-graden scans** uit te voeren zonder handmatige interventie
- **Bruikbare 3D bestanden** te produceren (PCD en OBJ formaten) voor verdere analyse, visualisatie of 3D printing
- **Reproduceerbare resultaten** te leveren met consistente kwaliteit

### Wat doet het Systeem?

Het systeem voert automatisch een volledige 3D-scan uit van een object:

1. **Rotatie**: De stepper motor draait het scanplatform rond in discrete stappen
2. **Scanning**: Bij elke positie neemt de LiDAR sensor een 360-graden scan
3. **Data Verzameling**: Alle scan data wordt verzameld en opgeslagen
4. **Verwerking**: De verzamelde data wordt omgezet naar 3D coördinaten
5. **Output**: Twee bestanden worden gegenereerd:
   - **PCD bestand**: Point cloud data voor technische analyse
   - **OBJ bestand**: 3D mesh voor visualisatie en 3D printing

### Voor wie is dit Systeem?

Dit systeem is geschikt voor:

- **Ontwerpers** die fysieke objecten willen digitaliseren
- **3D Printing** workflows waarbij bestaande objecten moeten worden gescand en gereproduceerd

### Belangrijke Kenmerken

- **Volledig geautomatiseerd**: Start het systeem en het voert automatisch een volledige scan uit
- **Hoge resolutie**: Configureerbaar aantal stappen per rotatie (standaard: 1600 stappen)
- **Modulair ontwerp**: Services kunnen onafhankelijk worden aangepast of vervangen
- **Docker-gebaseerd**: Eenvoudige installatie en isolatie van componenten
- **ROS2 communicatie**: Betrouwbare service-gebaseerde architectuur

---

## Installatie & Setup

### Vereisten

#### Hardware Vereisten

Je hebt de volgende hardware nodig om het systeem te gebruiken:

- **Raspberry Pi** (aanbevolen: Raspberry Pi 4 of nieuwer)
  - GPIO toegang voor motor controle
  - USB poort voor LiDAR sensor
  - Minimaal 4GB RAM voor Docker containers

- **LD14P LiDAR Sensor**
  - USB aansluiting
  - Wordt gedetecteerd als `/dev/ttyUSB0` op Linux systemen

- **NEMA17 Stepper Motor**
  - Met DRV8825 stepper motor driver
  - Aangesloten op GPIO pins (zie GPIO configuratie hieronder)

- **Stroomvoorziening**
  - Voldoende vermogen voor Raspberry Pi (5V), LiDAR sensor (5V) en motors (12-24V)

#### Software Vereisten

- **Operating System**: Linux (aanbevolen: Raspberry Pi OS)
- **Docker**: Versie 20.10 of nieuwer
- **Docker Compose**: Versie 1.29 of nieuwer

**Let op**: ROS2 wordt automatisch geïnstalleerd in de Docker containers, je hoeft dit niet handmatig te installeren.

### Hardware Aansluitingen

#### GPIO Pin Mapping (BCM Mode)

**Stepper Motor (DRV8825)**:
- `GPIO 17` → DIR (Direction pin)
- `GPIO 27` → STEP (Step pulse pin)
- `GPIO 24` → M0 (Microstepping pin 0)
- `GPIO 22` → M1 (Microstepping pin 1)
- `GPIO 16` → M2 (Microstepping pin 2)

#### LiDAR Hardware

- Sluit de LD14P LiDAR aan op een USB poort
- Controleer of het apparaat wordt gedetecteerd: `ls -l /dev/ttyUSB0`
- Als het apparaat niet wordt gevonden, controleer USB verbinding en probeer andere poorten

### Installatie Stappen

#### 1. Docker en Docker Compose Installeren

Als Docker nog niet geïnstalleerd is:

```bash
# Update package list
sudo apt update

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group (vereist herlogin)
sudo usermod -aG docker $USER

# Install Docker Compose
sudo apt install docker-compose
```

#### 2. Output Directory Aanmaken

Het systeem heeft een directory nodig voor output bestanden:

```bash
mkdir -p /home/pi/lidar_output
chmod 777 /home/pi/lidar_output
```

**Let op**: Pas het pad aan als je een andere locatie wilt gebruiken. Update dan ook `docker-compose.yml` (zie ontwikkelaarssectie).

#### 3. Docker Images Bouwen

Navigeer naar de project directory en bouw de Docker images:

```bash
cd /pad/naar/lidar-scanner
docker-compose build
```

Dit kan enkele minuten duren bij de eerste keer.

#### 4. Hardware Verbindingen Controleren

Voordat je het systeem start:

1. Controleer LiDAR USB verbinding: `ls -l /dev/ttyUSB0`
2. Controleer stepper motor GPIO verbindingen
3. Controleer stroomvoorziening voor alle componenten

---

## Gebruik

### Een Scan Uitvoeren

Het systeem voert automatisch een volledige scan uit wanneer je het start. Volg deze stappen:

#### 1. Start het Systeem

```bash
# Start alle services
docker-compose up
```

Het systeem start alle benodigde services en wacht 60 seconden voordat de scan begint. Dit geeft tijd voor alle services om correct te initialiseren.

#### 2. Scan Proces

Tijdens de scan:

- De orchestrator voert automatisch `STEPS_PER_ROTATION` stappen uit (standaard: 1600)
- Bij elke stap:
  - De stepper motor beweegt naar de volgende positie (behalve bij stap 0)
  - De LiDAR neemt een 360-graden scan
  - De data wordt verzameld
- Je ziet voortgangsberichten in de console

**Duur**: Afhankelijk van het aantal stappen. Bij 1600 stappen duurt een volledige scan ongeveer 30-60 minuten.

#### 3. Data Verwerking

Na voltooiing van alle stappen:

- De stepper motor keert automatisch terug naar positie 0
- Alle verzamelde data wordt verwerkt
- Twee bestanden worden gegenereerd:
  - `scan.pcd` - Point cloud data
  - `scan.obj` - 3D mesh (automatisch geconverteerd)

#### 4. Systeem Stoppen

Druk `Ctrl+C` om het systeem te stoppen, of gebruik:

```bash
# In een andere terminal
docker-compose down
```

### Output Bestanden

Na voltooiing van de scan vind je de volgende bestanden in `/home/pi/lidar_output/`:

**scan.pcd** (Point Cloud Data):
- Formaat: ASCII PCD versie 0.7
- Inhoud: 3D punten met x, y, z coördinaten
- Gebruik: Technische analyse, point cloud processing
- Aantal punten: `STEPS_PER_ROTATION × 360` (bijv. 1600 × 360 = 576,000 punten)

**scan.obj** (3D Mesh):
- Formaat: Wavefront OBJ
- Inhoud: 3D mesh met vertices en faces
- Gebruik: 3D visualisatie, 3D printing, CAD software
- Automatisch gegenereerd vanuit PCD bestand

### Bestanden Openen

**PCD Bestanden**:
- CloudCompare (gratis, open source)
- PCL (Point Cloud Library) tools
- RViz2 (ROS2 3D viewer)

**OBJ Bestanden**:
- Elke 3D viewer
- CAD software die OBJ ondersteunt

### Background Mode

Voor langere scans kun je het systeem in de achtergrond draaien:

```bash
# Start in achtergrond
docker-compose up -d

# Volg logs
docker-compose logs -f

# Stop systeem
docker-compose down
```

---

## Output & Resultaten

### Bestandsformaten

#### PCD (Point Cloud Data)

Het PCD bestand bevat de ruwe 3D point cloud data:

- **Formaat**: ASCII PCD versie 0.7
- **Velden**: x, y, z (cartesische coördinaten in meters)
- **Aantal punten**: `STEPS_PER_ROTATION × 360`

**Voorbeeld PCD header**:
```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 576000
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 576000
DATA ascii
```

#### OBJ (3D Mesh)

Het OBJ bestand bevat een 3D mesh gegenereerd vanuit de point cloud:

- **Formaat**: Wavefront OBJ
- **Inhoud**: 3D mesh met vertices en faces
- **Gebruik**: Direct bruikbaar in 3D software

### Output Locatie

- **Volume Mount**: `/home/pi/lidar_output:/output`
- **PCD Bestand**: `/home/pi/lidar_output/scan.pcd`
- **OBJ Bestand**: `/home/pi/lidar_output/scan.obj`

### Automatische OBJ Conversie

Het systeem converteert automatisch de pointcloud naar OBJ formaat. Deze conversie:

- Gebeurt direct na het genereren van het PCD bestand
- Gebruikt Poisson surface reconstructie voor mesh generatie
- Verwijdert automatisch outliers en optimaliseert de mesh
- **Let op**: Deze feature is momenteel nog in ontwikkeling

Als het OBJ bestand niet wordt gegenereerd, zie de [Troubleshooting](#troubleshooting) sectie.

---

## Troubleshooting

### Veelvoorkomende Problemen

#### LiDAR Device Niet Gevonden

**Symptoom**: Container kan `/dev/ttyUSB0` niet vinden, of scan faalt met device errors.

**Oplossingen**:

1. Controleer USB verbinding:
   ```bash
   ls -l /dev/ttyUSB0
   ```

2. Als het apparaat niet wordt gevonden:
   - Controleer USB kabel
   - Probeer andere USB poorten
   - Controleer of LiDAR sensor aan staat

3. Controleer device permissions:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

4. Controleer USB devices:
   ```bash
   lsusb
   ```

#### Output Bestanden Niet Gegenereerd

**Symptoom**: Geen `scan.pcd` of `scan.obj` bestand na scan voltooiing.

**Oplossingen**:

1. Controleer output directory:
   ```bash
   ls -l /home/pi/lidar_output
   ```

2. Controleer schrijfrechten:
   ```bash
   chmod 777 /home/pi/lidar_output
   ```

3. Controleer logs:
   ```bash
   docker-compose logs data_processor
   docker-compose logs orchestrator
   ```

4. Controleer of scan volledig is voltooid:
   - Kijk naar orchestrator logs voor voltooiingsberichten
   - Controleer of alle stappen zijn uitgevoerd

**OBJ Bestand Ontbreekt**:

Als alleen `scan.pcd` bestaat maar `scan.obj` niet:
- Dit kan betekenen dat de automatische conversie nog niet volledig geïmplementeerd is
- Het PCD bestand is nog steeds bruikbaar voor point cloud processing
- Voor handmatige conversie, zie ontwikkelaarssectie

#### Systeem Start Niet

**Symptoom**: Containers crashen bij opstarten of starten niet.

**Oplossingen**:

1. Controleer Docker status:
   ```bash
   docker ps
   docker-compose ps
   ```

2. Controleer logs:
   ```bash
   docker-compose logs
   ```

3. Controleer disk ruimte:
   ```bash
   df -h
   ```

4. Controleer geheugen:
   ```bash
   free -h
   ```

5. Rebuild images:
   ```bash
   docker-compose build --no-cache
   ```

#### Scan Stopt Voortijdig

**Symptoom**: Scan stopt voordat alle stappen zijn voltooid.

**Oplossingen**:

1. Controleer orchestrator logs voor foutmeldingen
2. Controleer LiDAR verbinding (zie boven)
3. Controleer stepper motor verbindingen
4. Controleer stroomvoorziening

#### Stepper Motor Beweegt Niet

**Symptoom**: Motor beweegt niet of beweegt onregelmatig.

**Oplossingen**:

1. Controleer GPIO verbindingen
2. Controleer stroomvoorziening voor motor
3. Controleer step_motor logs:
   ```bash
   docker-compose logs step_motor
   ```

### Logs Analyseren

Voor gedetailleerde debugging:

```bash
# Alle logs bekijken
docker-compose logs

# Specifieke service logs
docker-compose logs orchestrator
docker-compose logs lidar
docker-compose logs step_motor
docker-compose logs data_processor

# Logs volgen in real-time
docker-compose logs -f

# Laatste 100 regels
docker-compose logs --tail=100
```

---

## Systeem Architectuur

> **Voor Ontwikkelaars**: Deze sectie beschrijft de technische architectuur van het systeem.

### Componenten Overzicht

Het systeem bestaat uit de volgende hoofdcomponenten:

#### 1. **lidar_data** (Hardware Driver)

- **Functie**: ROS2 launch service voor de LD14P LiDAR sensor hardware
- **Verantwoordelijkheid**: Directe communicatie met de LiDAR hardware via `/dev/ttyUSB0`
- **Output**: Publiceert LiDAR scan data naar het `/scan` ROS2 topic
- **Container**: `ros2_ldlidar`

#### 2. **lidar** (LiDAR Service)

- **Functie**: Verwerkt LiDAR scans en maakt deze beschikbaar via ROS2 services
- **Verantwoordelijkheid**:
  - Luistert naar `/scan` topic
  - Interpoleert scan data naar 360 punten (één per graad)
  - Biedt `/lidar/measure` service endpoint aan
- **Container**: `lidar`

#### 3. **step_motor** (Stepper Motor Controller)

- **Functie**: Bestuurt een NEMA17 stepper motor via DRV8825 driver
- **Verantwoordelijkheid**:
  - Rotatie van het scanplatform in discrete stappen
  - GPIO pin controle (BCM mode)
  - Microstepping configuratie (1/32)
- **GPIO Pins**:
  - `DIR_PIN = 17` (Direction)
  - `STEP_PIN = 27` (Step pulse)
  - `M0_PIN = 24`, `M1_PIN = 22`, `M2_PIN = 16` (Microstepping)
- **Container**: `step_motor`

#### 4. **servo_motor** (Servo Motor Controller) - Optioneel

- **Functie**: Bestuurt een continu servo motor voor verticale rotatie
- **Status**: Momenteel uitgeschakeld in `docker-compose.yml`
- **GPIO Pin**: `SERVO_PIN = 26` (PWM)
- **Container**: `servo_motor`

#### 5. **photogrammetry** (Fotogrammetrie Service) - Optioneel

- **Functie**: Neemt diepte-afbeeldingen op verschillende posities
- **Status**: Momenteel uitgeschakeld in `docker-compose.yml`
- **Container**: `photogrammetry`

#### 6. **data_processor** (Data Verwerking)

- **Functie**: Verwerkt verzamelde scan data en converteert naar PCD en OBJ formaten
- **Verantwoordelijkheid**:
  - Ontvangt alle scan data van de orchestrator
  - Converteert LiDAR metingen naar 3D cartesische coördinaten
  - Genereert PCD bestand in `/output/scan.pcd`
  - Converteert automatisch pointcloud naar OBJ formaat in `/output/scan.obj` (in ontwikkeling)
- **Container**: `data_processor`

#### 7. **orchestrator** (Hoofd Coördinator)

- **Functie**: Coördineert alle services voor een volledige scan
- **Verantwoordelijkheid**:
  - Start de scan sequentie
  - Roept services aan in de juiste volgorde
  - Verzamelt alle scan data
  - Start data verwerking na voltooiing
- **Container**: `orchestrator`

### ROS2 Service Communicatie

Alle services communiceren via ROS2 service calls met JSON payloads:

- **Service Type**: `scanner_pkg/srv/JsonIO`
- **Request**: JSON string met parameters (bijv. `{"step": 0}`)
- **Response**: JSON string met resultaten

### Data Flow

1. **Hardware Laag**: LiDAR hardware → `/scan` ROS2 topic
2. **Service Laag**: Orchestrator roept services aan:
   - `/step_motor/step` - Motor beweging
   - `/lidar/measure` - LiDAR scan
   - `/data/process` - Data verwerking
3. **Data Verwerking**: ScanData → PCD → OBJ conversie
4. **Output**: Bestanden in `/output/` directory

---

## Configuratie

> **Voor Ontwikkelaars**: Deze sectie beschrijft hoe je de systeemconfiguratie kunt aanpassen.

### Configuratie Bestand: `config/config.py`

Het hoofdconfiguratiebestand bevat de belangrijkste parameters:

```1:16:config/config.py
#general
STEPS_PER_ROTATION = 1600

#orchestrator

#lidar
LIDAR_ROTATIONS_PER_STEP = 16

#photogrammetry

#step_motor

#data processing
```

#### Belangrijke Parameters

- **`STEPS_PER_ROTATION`**:
  - Bepaalt de resolutie van de scan
  - Hogere waarde = meer detail maar langere scan tijd
  - Standaard: 1600 stappen

- **`LIDAR_ROTATIONS_PER_STEP`**:
  - Aantal LiDAR rotaties die worden uitgevoerd per step positie
  - Standaard: 16 rotaties

### Docker Compose Configuratie

Belangrijke instellingen in `docker-compose.yml`:

- **`ROS_DOMAIN_ID=0`**: ROS2 domain ID voor service communicatie
- **`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`**: ROS2 middleware implementatie
- **`network_mode: host`**: Alle containers gebruiken host networking voor ROS2 communicatie
- **`privileged: true`**: Nodig voor GPIO toegang

### ROS2 Middleware Instellingen

Het systeem gebruikt FastRTPS als ROS2 middleware. De configuratie wordt geladen vanuit:

- `/root/.ros/fastdds.xml` (in containers)
- `config/fastdds.xml` (in repository)

### Output Directory Aanpassen

Om een andere output directory te gebruiken, pas `docker-compose.yml` aan:

```yaml
volumes:
  - /jouw/pad/lidar_output:/output
```

---

## Features Aan/Uit Zetten

> **Voor Ontwikkelaars**: Deze sectie beschrijft hoe je optionele features kunt activeren of deactiveren.

### Servo Motor Activeren/Deactiveren

De servo motor service is standaard uitgeschakeld. We gebruiken normaal gezien de stepper motor voor de rotatie. Om deze te activeren:

1. Open `docker-compose.yml`
2. Zoek de `servo_motor` service sectie en verwijder de `#` commentaar tekens:

```71:87:docker-compose.yml
#  servo_motor:
#    build:
#      context: .
#      args:
#        BUILDPLATFORM: ${BUILDPLATFORM}
#      dockerfile: src/servo_motor/Dockerfile
#    container_name: servo_motor
#    environment:
#      - ROS_DOMAIN_ID=0
#      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#    stdin_open: true
#    tty: true
#    entrypoint: /entrypoint.sh
#    privileged: true          # Needed for GPIO access
#    devices:
#      - /dev/gpiomem:/dev/gpiomem   # Expose GPIO memory to container
#    network_mode: host
```

3. Activeer ook de servo motor calls in de orchestrator:

```29:29:src/orchestrator/orchestrator_service.py
self.servoMotorClient = ServiceClient(node_name="servo_motor", service_type=JsonIO)
```

```55:55:src/orchestrator/orchestrator_service.py
# self.servoMotorClient.call('/servo_motor/step', request=json.dumps(payload))
```

### Photogrammetry Activeren/Deactiveren

De photogrammetry service is standaard uitgeschakeld. Deze feature is nog totaal niet geïmplementeerd, maar bestaat omdat dit in het plan zit. Om deze te activeren:

1. Open `docker-compose.yml`
2. Zoek de `photogrammetry` service sectie en verwijder de `#` commentaar tekens:

```38:52:docker-compose.yml
#  photogrammetry:
#    build:
#      context: .
#      args:
#        BUILDPLATFORM: ${BUILDPLATFORM}
#      dockerfile: src/photogrammetry/Dockerfile
#    container_name: photogrammetry
#    environment:
#      - ROS_DOMAIN_ID=0
#      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#    stdin_open: true
#    tty: true
#    entrypoint: /entrypoint.sh
#    network_mode: host
```

3. Activeer photogrammetry calls in de orchestrator:

```30:30:src/orchestrator/orchestrator_service.py
# self.photogrammetryClient = ServiceClient(node_name="photogrammetry", service_type=JsonIO)
```

```57:60:src/orchestrator/orchestrator_service.py
def photogrammetry_measure(self, step: int) -> None:
  payload = {"step": step}
  # response = self.photogrammetryClient.call('/photogrammetry/measure', request=json.dumps(payload))
  # print("Photogrammetry response:", response)
```

### Standaard Actieve Services

De volgende services zijn standaard actief:

- `lidar_data` - LiDAR hardware driver
- `lidar` - LiDAR service
- `step_motor` - Stepper motor controller
- `data_processor` - Data verwerking
- `orchestrator` - Hoofd coördinator

### Impact van Uitschakelen van Services

- **Servo Motor uit**: Alleen horizontale scans mogelijk (2D/2.5D)
- **Photogrammetry uit**: Alleen LiDAR data wordt verzameld
- **LiDAR uit**: Systeem kan niet functioneren (kritieke service)
- **Step Motor uit**: Geen rotatie mogelijk, alleen statische scans

---

## Technische Details

> **Voor Ontwikkelaars**: Deze sectie bevat gedetailleerde technische informatie over service endpoints, data structuren en communicatie protocollen.

### ROS2 Service Endpoints

#### `/lidar/measure`

- **Service Type**: `scanner_pkg/srv/JsonIO`
- **Request**: `{"step": <int>}`
- **Response**: `{"laser_scans": [{"angle": float, "distance": float}, ...]}`
- **Beschrijving**: Neemt een LiDAR scan en retourneert 360 punten (één per graad)

#### `/step_motor/step`

- **Service Type**: `scanner_pkg/srv/JsonIO`
- **Request**: `{"step": <int>}`
- **Response**: `true` of `false`
- **Beschrijving**: Beweegt stepper motor één stap vooruit

#### `/servo_motor/step` (Optioneel)

- **Service Type**: `scanner_pkg/srv/JsonIO`
- **Request**: `{"step": <int>, "direction": "cw"|"ccw"}`
- **Response**: `"true"`
- **Beschrijving**: Beweegt servo motor voor gespecificeerde tijd

#### `/photogrammetry/measure` (Optioneel)

- **Service Type**: `scanner_pkg/srv/JsonIO`
- **Request**: `{"step": <int>}`
- **Response**: `true` of `false`
- **Beschrijving**: Neemt een diepte-afbeelding op de gespecificeerde positie

#### `/data/process`

- **Service Type**: `scanner_pkg/srv/JsonIO`
- **Request**: `ScanData` JSON (zie data structuren hieronder)
- **Response**: `true` of `false`
- **Beschrijving**: Verwerkt alle scan data en genereert PCD bestand

### Data Structuren (DTOs)

#### `LaserScan`

```4:6:domain/dto/laser_scan.py
class LaserScan(BaseModel):
    angle: float = 0.0 # Angle in radians
    distance: float = 0.0 # Distance in meters
```

#### `LidarResponse`

```7:8:domain/dto/lidar_response.py
class LidarResponse(BaseModel):
    laser_scans: List[LaserScan]
```

#### `ScanData`

```6:7:domain/dto/scan_data.py
class ScanData(BaseModel):
    lidar_data: Dict[int, List[LaserScan]] = {}
```

#### `Point`

```4:7:domain/dto/point.py
class Point(BaseModel):
    x: float = 0.0 # X coordinate in meters
    y: float = 0.0 # Y coordinate in meters
    z: float = 0.0 # Z coordinate in meters
```

### Service Communicatie Protocollen

Alle services gebruiken het `JsonIO` service type:

- **Request veld**: `request` (string) - Bevat JSON data
- **Response veld**: `response` (string) - Bevat JSON data

**Voorbeeld service call** (uit orchestrator):

```62:72:src/orchestrator/orchestrator_service.py
    def lidar_scan(self, step: int) -> None:
        payload = {"step": step}
        response = self.lidarClient.call('/lidar/measure', request=json.dumps(payload))

        lidar_response = json.loads(response.response)

        scans = lidar_response["laser_scans"]

        self._scan_data.lidar_data[step] = [
            LaserScan.parse_obj(p) for p in scans
        ]
```

### Coördinaten Systeem

Het systeem gebruikt een **sferisch naar cartesisch** conversie:

1. **LiDAR scan angle** (graden) → **beam_angle** (radialen)
2. **Step positie** → **servo_angle** (radialen): `2π × (step / STEPS_PER_ROTATION)`
3. **Sferische coördinaten** (r, beam_angle, servo_angle) → **Cartesische coördinaten** (x, y, z)

**Conversie formule**:

```47:65:src/data_processor/data_service.py
def laser_data_to_point(laser_scan: LaserScan, step: int) -> Point:
    # ---- SERVO ROTATION (around Z) ----
    servo_angle = 2.0 * math.pi * (float(step) / float(config.STEPS_PER_ROTATION))

    # ---- LIDAR SCAN ANGLE (convert DEGREES → RADIANS) ----
    beam_angle = math.radians(laser_scan.angle)

    r = laser_scan.distance

    # ---- SPHERICAL → CARTESIAN ----
    # beam angle = elevation
    horizontal = r * math.cos(beam_angle)  # radius in XY plane
    z = r * math.sin(beam_angle)

    # rotate around Z according to servo step
    x = horizontal * math.cos(servo_angle)
    y = horizontal * math.sin(servo_angle)

    return Point(x=x, y=y, z=z)
```

### Bestandsstructuur

```
lidar-scanner/
├── config/
│   ├── config.py           # Hoofdconfiguratie
│   └── fastdds.xml         # FastRTPS configuratie
├── domain/
│   ├── communication/      # ROS2 service client utilities
│   ├── dto/                # Data Transfer Objects
│   └── helpers/            # Helper functies
├── src/
│   ├── lidar/              # LiDAR service
│   ├── lidar_data/         # LiDAR hardware driver
│   ├── step_motor/         # Stepper motor service
│   ├── servo_motor/        # Servo motor service (optioneel)
│   ├── photogrammetry/     # Photogrammetry service (optioneel)
│   ├── data_processor/     # Data verwerking service
│   └── orchestrator/       # Hoofd coördinator
├── docker-compose.yml      # Docker Compose configuratie
└── HANDLEIDING.md          # Deze handleiding
```

### OBJ Conversie Proces

De automatische OBJ conversie gebruikt het `pointcloud_to_obj.py` script en voert de volgende stappen uit:

**Functies**:

- **Outlier verwijdering**: Verwijdert statistische outliers uit de point cloud
- **Voxel downsampling**: Verkleint grote point clouds voor betere verwerkingssnelheid
- **Normaal schatting**: Berekent normals voor mesh reconstructie
- **Poisson surface reconstructie**: Genereert 3D mesh van point cloud
- **Mesh cleaning**: Verwijdert degeneratieve triangles en duplicates

**Conversie Parameters** (standaardwaarden in `pointcloud_to_obj.py`):

```124:135:src/data_processor/pointcloud_to_obj.py
def pcd_to_obj(
    input_pcd_path: str,
    output_obj_path: str,
    depth: int = 9,
    width: int = 0,
    scale: float = 1.1,
    linear_fit: bool = False,
    remove_outliers: bool = True,
    outlier_nb_neighbors: int = 20,
    outlier_std_ratio: float = 2.0,
    voxel_size: float = 0.02
) -> bool:
```

**Handmatige Conversie** (indien automatische conversie nog niet actief):

```bash
docker exec data_processor python3 /root/ros2_ws/src/data_processor/pointcloud_to_obj.py /output/scan.pcd /output/scan.obj
```

---

## Conclusie

Dit LiDAR Scanner Systeem biedt een volledig geautomatiseerde oplossing voor 3D scanning met behulp van ROS2 en Docker. Door de modulaire architectuur kunnen componenten eenvoudig worden aangepast, toegevoegd of verwijderd.

Voor vragen of problemen:
- **Gebruikers**: Raadpleeg de [Troubleshooting](#troubleshooting) sectie
- **Ontwikkelaars**: Controleer de service logs en technische documentatie

---

Zie de [git commit geschiedenis](https://github.com/LidarPhotogrammetryScanner/Scanner/commits) voor wijzigingen.
