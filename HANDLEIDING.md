# Handleiding LiDAR Scanner Systeem

## Inhoudsopgave

1. [Inleiding](#inleiding)
1. [Systeem Architectuur](#systeem-architectuur)
1. [Vereisten](#vereisten)
1. [Installatie & Setup](#installatie--setup)
1. [Configuratie](#configuratie)
1. [Opstarten van het Systeem](#opstarten-van-het-systeem)
1. [Features Aan/Uit Zetten](#features-aanuit-zetten)
1. [Gebruik](#gebruik)
1. [Output & Resultaten](#output--resultaten)
1. [Troubleshooting](#troubleshooting)
1. [Technische Details](#technische-details)

---

## Inleiding

### Wat is het systeem?

Het LiDAR Scanner Systeem is een geautomatiseerd 3D-scanning systeem dat gebruik maakt van ROS2 (Robot Operating System 2) en Docker containers. Het systeem maakt een volledige 360-graden 3D-scan van objecten door een LiDAR sensor rond te draaien en op verschillende posities metingen te verrichten.

### Doel en Functionaliteit

Het hoofddoel van dit systeem is om:

- **3D Point Clouds** te genereren van objecten door middel van LiDAR scanning
- **Automatische rotatie** te bewerkstelligen met behulp van een stepper motor
- **Gedistribueerde verwerking** te gebruiken via ROS2 services in Docker containers
- **PCD (Point Cloud Data)** bestanden te produceren die kunnen worden gebruikt voor verdere analyse of visualisatie
- **OBJ (Object File)** bestanden te produceren die kunnen worden gebruikt voor 3D visualisatie

### Overzicht van de Architectuur

Het systeem bestaat uit meerdere onafhankelijke services die via ROS2 met elkaar communiceren:

- **Hardware drivers** voor LiDAR en motors
- **Service nodes** die specifieke taken uitvoeren
- **Een orchestrator** die alle services coördineert
- **Een data processor** die de verzamelde data omzet naar bruikbare formaten

---

## Systeem Architectuur

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

---

## Vereisten

### Hardware Vereisten

- **Raspberry Pi** (aanbevolen: Raspberry Pi 4 of nieuwer)

  - GPIO toegang voor motor controle
  - USB poort voor LiDAR sensor
  - Voldoende geheugen voor Docker containers

- **LD14P LiDAR Sensor**

  - USB aansluiting
  - Wordt gedetecteerd als `/dev/ttyUSB0`

- **NEMA17 Stepper Motor**

  - Met DRV8825 stepper motor driver
  - Aangesloten op GPIO pins (zie GPIO configuratie hieronder)

- **Optioneel (ipv Stepper Motor): Servo Motor**
  - DS3115 continu servo motor
  - Voor verticale rotatie (momenteel niet actief)

### Software Vereisten

- **Operating System**: Linux (aanbevolen: Raspberry Pi OS)
- **Docker**: Versie 20.10 of nieuwer
- **Docker Compose**: Versie 1.29 of nieuwer
- **ROS2**: Kilted versie (geïnstalleerd in Docker containers)

### Hardware Configuratie

#### GPIO Pin Mapping (BCM Mode)

**Stepper Motor (DRV8825):**

- `GPIO 17` → DIR (Direction pin)
- `GPIO 27` → STEP (Step pulse pin)
- `GPIO 24` → M0 (Microstepping pin 0)
- `GPIO 22` → M1 (Microstepping pin 1)
- `GPIO 16` → M2 (Microstepping pin 2)

**Servo Motor (Optioneel):**

- `GPIO 26` → PWM output voor servo controle

#### LiDAR Hardware

- USB device: `/dev/ttyUSB0`
- Wordt automatisch gemount in de `lidar_data` container

---

## Installatie & Setup

### 1. Docker Images Bouwen

Alle Docker images worden automatisch gebouwd wanneer je Docker Compose gebruikt. Om handmatig te bouwen:

```bash
# Bouw alle images
docker-compose build

# Bouw een specifieke service
docker-compose build lidar
```

### 1. Volume Mounts Configureren

Het systeem gebruikt een volume mount voor output bestanden. Controleer in `docker-compose.yml`:

```yaml
volumes:
  - /home/pi/lidar_output:/output
```

Zorg ervoor dat het pad `/home/pi/lidar_output` bestaat en schrijfrechten heeft:

```bash
mkdir -p /home/pi/lidar_output
chmod 777 /home/pi/lidar_output
```

### 2. Hardware Aansluitingen

1. **LiDAR Sensor**:

   - Sluit de LD14P LiDAR aan op een USB poort
   - Controleer of het apparaat wordt gedetecteerd: `ls -l /dev/ttyUSB0`

1. **Stepper Motor**:

   - Sluit de DRV8825 driver aan volgens de GPIO pin mapping
   - Controleer alle verbindingen voordat je het systeem start

1. **Power Supply**:
   - Zorg voor voldoende stroomvoorziening voor de motors
   - Controleer dat alle componenten correct zijn aangesloten

---

## Configuratie

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

---

## Opstarten van het Systeem

### Alle Services Starten

Het eenvoudigste is om alle services tegelijk te starten:

```bash
# Start alle services in de voorgrond (zie logs)
docker-compose up

# Start alle services in de achtergrond
docker-compose up -d

# Start en bouw images indien nodig
docker-compose up --build
```

### Individuele Services Starten

Je kunt ook individuele services starten:

```bash
# Start alleen de LiDAR hardware driver
docker-compose up lidar_data

# Start alleen de LiDAR service
docker-compose up lidar

# Start alleen de stepper motor service
docker-compose up step_motor

# Start alleen de data processor
docker-compose up data_processor

# Start alleen de orchestrator (start automatisch scan)
docker-compose up orchestrator
```

**Let op**: De orchestrator heeft de andere services nodig om te functioneren. Start altijd eerst de afhankelijke services.

### Services Stoppen

```bash
# Stop alle services
docker-compose down

# Stop en verwijder volumes
docker-compose down -v

# Stop een specifieke service
docker-compose stop lidar
```

### Logs Bekijken

```bash
# Logs van alle services
docker-compose logs

# Logs van een specifieke service
docker-compose logs lidar

# Volg logs in real-time
docker-compose logs -f

# Logs van laatste 100 regels
docker-compose logs --tail=100
```

### Background Mode

Voor productie gebruik, start services in detached mode:

```bash
docker-compose up -d
```

Controleer status:

```bash
docker-compose ps
```

---

## Features Aan/Uit Zetten

### Servo Motor Activeren/Deactiveren

De servo motor service is standaard uitgeschakeld. We gebruiken normaal gezien de stepper motor voor de rotatie. Om deze te activeren:

1. Open `docker-compose.yml`
1. Zoek de `servo_motor` service sectie en verwijder de `#` commentaar tekens:

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

1. Activeer ook de servo motor calls in de orchestrator:

```29:29:src/orchestrator/orchestrator_service.py
self.servoMotorClient = ServiceClient(node_name="servo_motor", service_type=JsonIO)
```

```55:55:src/orchestrator/orchestrator_service.py
# self.servoMotorClient.call('/servo_motor/step', request=json.dumps(payload))
```

### Photogrammetry Activeren/Deactiveren

De photogrammetry service is standaard uitgeschakeld. Deze feature is nog totaal niet geïmplementeerd, maar bestaat omdat dit in het plan zit. Om deze te activeren:

1. Open `docker-compose.yml`
1. Zoek de `photogrammetry` service sectie en verwijder de `#` commentaar tekens:

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

## Gebruik (WORK IN PROGRESS) # TODO

### Een Scan Uitvoeren

Het systeem voert automatisch een volledige scan uit wanneer de orchestrator start:

1. **Start alle services**:

```bash
docker-compose up
```

2. **Wacht op initialisatie**: De orchestrator wacht 60 seconden na opstarten voordat de scan begint:

```23:24:src/orchestrator/orchestrator_service.py
print("starting in 60 seconds")
time.sleep(60)
```

3. **Scan Proces**:

   - De orchestrator voert automatisch `STEPS_PER_ROTATION` stappen uit
   - Bij elke stap:
     - Stepper motor beweegt (behalve bij stap 0)
     - LiDAR neemt een scan
     - Data wordt verzameld

4. **Data Verwerking**:
   - Na voltooiing van alle stappen wordt de data verwerkt
   - Stepper motor keert terug naar positie 0
   - PCD bestand wordt gegenereerd
   - Pointcloud wordt automatisch geconverteerd naar OBJ formaat voor 3D visualisatie

### Scan Proces (Stap-voor-Stap)

Het scan proces volgt deze sequentie:

1. **Initialisatie** (60 seconden wachttijd):

```36:43:src/orchestrator/orchestrator_service.py
def run(self):

    self.lidar_scan(0)

    for i in range(config.STEPS_PER_ROTATION):
        self.measure(i)

    self.process_data()
```

2. **Eerste scan** op positie 0 (geen motor beweging)
3. **Voor elke volgende stap** (1 tot STEPS_PER_ROTATION):

```45:50:src/orchestrator/orchestrator_service.py
def measure(self, step: int) -> None:
    print("NOW PREPARE STEP {}".format(step + 1))
    if step != 0:
        self.step_step_motor(step)
    self.lidar_scan(step)
    print("DONE WITH STEP {}".format(step + 1))
```

- Stepper motor beweegt naar volgende positie
- LiDAR neemt scan met 360 punten
- Data wordt opgeslagen in `ScanData` object

4. **Na alle stappen**:

```74:76:src/orchestrator/orchestrator_service.py
def process_data(self):
    self.step_step_motor(0)
    self.dataClient.call('/data/process', request=self._scan_data.json())
```

- Stepper motor keert terug naar positie 0
- Alle verzamelde data wordt naar data_processor gestuurd
- PCD bestand wordt gegenereerd (`scan.pcd`)
- Pointcloud wordt automatisch geconverteerd naar OBJ formaat (`scan.obj`)

### Output Bestanden Locatie

Na voltooiing van de scan worden de volgende bestanden gegenereerd:

**PCD Bestand** (Point Cloud Data):

- **Container pad**: `/output/scan.pcd`
- **Host pad**: `/home/pi/lidar_output/scan.pcd` (via volume mount)

**OBJ Bestand** (3D Mesh):

- **Container pad**: `/output/scan.obj`
- **Host pad**: `/home/pi/lidar_output/scan.obj` (via volume mount)

### Automatische PCD naar OBJ Conversie

Het systeem converteert automatisch de gegenereerde pointcloud naar een OBJ bestand voor 3D visualisatie. Deze conversie gebeurt direct na het genereren van het PCD bestand door de `data_processor` service.

**Let op**: Deze feature is momenteel nog in ontwikkeling. De automatische conversie wordt geïmplementeerd in de `data_processor` service en gebruikt het `pointcloud_to_obj.py` script voor de mesh reconstructie.

**Handmatige Conversie** (indien automatische conversie nog niet actief):

Als de automatische conversie nog niet volledig geïmplementeerd is, kan je handmatig converteren:

```bash
# Voer conversie uit vanuit de container
docker exec data_processor python3 /root/ros2_ws/src/data_processor/pointcloud_to_obj.py /output/scan.pcd /output/scan.obj
```

**Conversie Parameters** (aanpasbaar in `pointcloud_to_obj.py`):

- `depth`: Poisson reconstructie diepte (standaard: 9)
- `remove_outliers`: Verwijder statistische outliers (standaard: True)
- `voxel_size`: Voxel grootte voor downsampling (standaard: 0.02)

---

## Output & Resultaten

### PCD Bestandsformaat

Het gegenereerde bestand is in **PCD (Point Cloud Data)** formaat versie 0.7:

- **Formaat**: ASCII
- **Velden**: x, y, z (cartesische coördinaten in meters)
- **Aantal punten**: `STEPS_PER_ROTATION × 360` (bijv. 1600 × 360 = 576,000 punten)

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

### Output Locatie

- **Volume Mount**: `/home/pi/lidar_output:/output`
- **PCD Bestand**: `scan.pcd` → `/home/pi/lidar_output/scan.pcd`
- **OBJ Bestand**: `scan.obj` → `/home/pi/lidar_output/scan.obj` (automatisch gegenereerd)

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

**OBJ Bestandsformaat**:

- **Formaat**: Wavefront OBJ
- **Inhoud**: 3D mesh met vertices en faces
- **Gebruik**: Kan worden geopend in 3D software zoals Blender, MeshLab, of andere 3D viewers

---

## Troubleshooting

### Veelvoorkomende Problemen

#### 1. LiDAR Device Niet Gevonden

**Symptoom**: `lidar_data` container kan `/dev/ttyUSB0` niet vinden

**Oplossingen**:

- Controleer USB verbinding: `ls -l /dev/ttyUSB0`
- Controleer device permissions: `sudo chmod 666 /dev/ttyUSB0`
- Controleer of device is gemount in container:

```27:28:docker-compose.yml
    devices:
      - "/dev/ttyUSB0:/dev/ttyUSB0"
```

- Probeer andere USB poorten
- Controleer USB kabel

**Debug commando's**:

```bash
# Controleer USB devices
lsusb

# Controleer tty devices
ls -l /dev/ttyUSB*

# Test in container
docker exec lidar_data ls -l /dev/ttyUSB0
```

#### 2. Output Bestand Niet Gegenereerd

**Symptoom**: Geen `scan.pcd` of `scan.obj` bestand in output directory

**Oplossingen**:

- Controleer volume mount pad bestaat: `ls -l /home/pi/lidar_output`
- Controleer schrijfrechten: `chmod 777 /home/pi/lidar_output`
- Controleer data_processor logs: `docker-compose logs data_processor`
- Controleer of orchestrator de scan heeft voltooid
- Controleer output pad in data_processor:

```37:39:src/data_processor/data_service.py
output_path = "/output/scan.pcd"
with open(output_path, "w") as f:
    f.write(pcd_file)
```

**OBJ Bestand Ontbreekt**:

- Als alleen `scan.pcd` bestaat maar `scan.obj` niet, kan dit betekenen dat de automatische conversie nog niet volledig geïmplementeerd is
- Voer handmatige conversie uit (zie sectie "PCD naar OBJ Conversie")
- Controleer of `pointcloud_to_obj.py` correct werkt: `docker exec data_processor python3 /root/ros2_ws/src/data_processor/pointcloud_to_obj.py --help`

---

## Technische Details

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

---

## Conclusie

Dit LiDAR Scanner Systeem biedt een volledig geautomatiseerde oplossing voor 3D scanning met behulp van ROS2 en Docker. Door de modulaire architectuur kunnen componenten eenvoudig worden aangepast, toegevoegd of verwijderd.

---

Zie de [git commit geschiedenis](https://github.com/LidarPhotogrammetryScanner/Scanner/commits) voor wijzigingen.
