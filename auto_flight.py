import serial
import struct
import time
import sys
import os
import pygame

# --- Configuration Globale ---
ENABLE_THROTTLE_TEST_LIMIT = False # Mettre à False pour utiliser la pleine poussée (1000-2000)
THROTTLE_TEST_MIN_VALUE = 1000
THROTTLE_TEST_MAX_VALUE = 1700

# --- Configuration et Constantes MSP ---
# SERIAL_PORT n'est plus défini ici, il sera détecté automatiquement
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200
MSP_ALTITUDE = 109
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107

# --- Configuration du Contrôle ---
RC_CHANNELS_COUNT = 8
current_rc_values = [1500] * RC_CHANNELS_COUNT
THROTTLE_MIN_EFFECTIVE = THROTTLE_TEST_MIN_VALUE if ENABLE_THROTTLE_TEST_LIMIT else 1000
THROTTLE_MAX_EFFECTIVE = THROTTLE_TEST_MAX_VALUE if ENABLE_THROTTLE_TEST_LIMIT else 2000

# Initialisation du throttle à la valeur minimale de sécurité
current_rc_values[2] = THROTTLE_MIN_EFFECTIVE
current_rc_values[4] = 1000  # AUX1 (Arm switch) désarmé

ARM_VALUE = 1800
DISARM_VALUE = 1000
# Sécurité d'armement basée sur la valeur minimale
THROTTLE_SAFETY_ARM = THROTTLE_MIN_EFFECTIVE + 50

# État du drone
is_armed_command = False

# --- Variables GPS ---
gps_fix = False
gps_num_sat = 0
gps_latitude = 0
gps_longitude = 0
gps_altitude = 0
gps_speed = 0
gps_ground_course = 0

# --- Configuration Verrouillage Yaw ---
yaw_locked = False # Yaw verrouillé par défaut au démarrage
YAW_LOCK_VALUE = 1500 # Valeur du yaw lorsqu'il est verrouillé
current_rc_values[3] = YAW_LOCK_VALUE # S'assurer que le yaw est à la valeur de verrouillage au démarrage

# --- Configuration Manette ---
AXIS_YAW = 0        # Joystick Gauche X (pour Yaw)
AXIS_THROTTLE = 1   # Joystick Gauche Y (pour Throttle)
AXIS_L2 = 2         # Gâchette L2 (non utilisée pour le throttle maintenant)
AXIS_ROLL = 3       # Joystick Droit X (pour Roll)
AXIS_PITCH = 4      # Joystick Droit Y (pour Pitch)
AXIS_R2 = 5         # Gâchette R2 (non utilisée pour le throttle maintenant)

BUTTON_ARM_DISARM = 4 # Souvent L1/LB
BUTTON_QUIT = 5       # Souvent R1/RB
BUTTON_AUTO_MODE = 2  # Souvent X ou Carré
BUTTON_ALTHOLD = 3    # Souvent Y ou Triangle (ex-BUTTON_FLIP)

JOYSTICK_DEADZONE = 0.08

# --- Configuration Vol Automatique (Inchangé) ---
TARGET_ALTITUDE_M = 1.0
LANDING_ALTITUDE_THRESHOLD_M = 0.20
TAKEOFF_END_ALTITUDE_ERROR_M = 0.25

STATE_MANUAL = 0
STATE_AUTO_TAKEOFF = 1
STATE_AUTO_HOVER = 2
STATE_AUTO_LANDING = 3
STATE_PERFORMING_FLIP = 4
current_flight_state = STATE_MANUAL

current_altitude_m = None
last_msp_request_time = 0
last_gps_request_time = 0
MSP_REQUEST_INTERVAL = 0.05
GPS_REQUEST_INTERVAL = 0.5  # Requête GPS moins fréquente

HOVER_THROTTLE_ESTIMATE = (THROTTLE_MIN_EFFECTIVE + THROTTLE_MAX_EFFECTIVE) // 2 + 50
KP_ALTITUDE = 20.0
MAX_AUTO_THROTTLE_ADJUST = 100
TAKEOFF_THROTTLE_CEILING = THROTTLE_MAX_EFFECTIVE + 50

# --- Configuration Flip (Inchangé) ---
FLIP_THROTTLE_PUNCH_DURATION = 0.2
FLIP_THROTTLE_PUNCH_VALUE = THROTTLE_MAX_EFFECTIVE + 100
FLIP_PITCH_DURATION = 0.45
FLIP_PITCH_VALUE = 2000
FLIP_RECOVERY_THROTTLE_DURATION = 0.3
FLIP_RECOVERY_THROTTLE_VALUE = HOVER_THROTTLE_ESTIMATE + 50
flip_start_time = 0
flip_phase = 0

ACRO_MODE_CHANNEL_INDEX = 5
ACRO_MODE_VALUE = 2000
STABLE_MODE_VALUE = 1000
previous_flight_mode_rc_value = STABLE_MODE_VALUE

# --- Variables globales pour la manette ---
joystick = None
joystick_connected = False
althold_active = False  # NOUVEAU: État du mode ALTHOLD

# --- Fonctions MSP (Inchangé) ---
def calculate_checksum(payload):
    chk = 0
    for b in payload:
        chk ^= b
    return chk

def send_msp_packet(ser, command, data):
    if data: data_size = len(data)
    else: data_size = 0
    header = b'$M<'
    payload_header = struct.pack('<BB', data_size, command)
    full_payload = payload_header + (data if data else b'')
    checksum_val = calculate_checksum(full_payload)
    packet = header + full_payload + struct.pack('<B', checksum_val)
    try:
        ser.write(packet)
    except Exception as e:
        print(f"Erreur d'écriture série: {e}")

def request_msp_data(ser, command):
    send_msp_packet(ser, command, None)

def parse_msp_response(ser_buffer):
    global current_altitude_m, gps_fix, gps_num_sat, gps_latitude, gps_longitude, gps_altitude, gps_speed, gps_ground_course
    idx = ser_buffer.find(b'$M>')
    if idx == -1: return ser_buffer
    if len(ser_buffer) < idx + 5: return ser_buffer[idx:]
    payload_size = ser_buffer[idx+3]
    cmd = ser_buffer[idx+4]
    if len(ser_buffer) < idx + 5 + payload_size + 1: return ser_buffer[idx:]
    full_packet_payload = ser_buffer[idx+3 : idx+5+payload_size]
    received_checksum = ser_buffer[idx+5+payload_size]
    calculated_checksum = calculate_checksum(full_packet_payload)
    if received_checksum == calculated_checksum:
        payload_data = ser_buffer[idx+5 : idx+5+payload_size]
        if cmd == MSP_ALTITUDE and payload_size >= 4:
            altitude_cm = struct.unpack('<i', payload_data[0:4])[0]
            current_altitude_m = float(altitude_cm) / 100.0
        elif cmd == MSP_RAW_GPS and payload_size >= 16:
            # Parse GPS data: fix, numSat, lat, lon, alt, speed, ground_course
            gps_fix = payload_data[0] != 0
            gps_num_sat = payload_data[1]
            gps_latitude = struct.unpack('<i', payload_data[2:6])[0] / 10000000.0
            gps_longitude = struct.unpack('<i', payload_data[6:10])[0] / 10000000.0
            gps_altitude = struct.unpack('<h', payload_data[10:12])[0]
            gps_speed = struct.unpack('<h', payload_data[12:14])[0]
            gps_ground_course = struct.unpack('<h', payload_data[14:16])[0] / 10.0
        return ser_buffer[idx+6+payload_size:]
    else:
        return ser_buffer[idx+1:]

# --- NOUVEAU: Initialisation Port Série ---
def initialize_serial():
    """Tente d'ouvrir ttyAMA0, puis ttyS0 si le premier échoue."""
    potential_ports = ['/dev/ttyAMA0', '/dev/ttyS0']
    ser = None
    
    for port in potential_ports:
        try:
            print(f"Tentative de connexion à {port}...")
            # timeout=0 pour des lectures non bloquantes
            ser = serial.Serial(port, BAUD_RATE, timeout=0) 
            print(f"Succès: Port série {port} ouvert.")
            return ser
        except serial.SerialException as e:
            print(f"Échec de l'ouverture de {port}: {e}")
    
    # Si la boucle se termine sans succès
    print("Erreur Critique: Impossible d'ouvrir un port série valide (AMA0 ou S0).")
    return None

# --- Logique de Contrôle Manette ---
def map_axis_to_rc(axis_value, min_rc=1000, max_rc=2000, inverted=False):
    if abs(axis_value) < JOYSTICK_DEADZONE: axis_value = 0.0
    if inverted: axis_value = -axis_value
    normalized_value = (axis_value + 1.0) / 2.0
    return int(min_rc + normalized_value * (max_rc - min_rc))

# --- Fonctions de Vol Automatique (Inchangées pour la plupart) ---
def manage_flip_sequence():
    global current_rc_values, current_flight_state, flip_start_time, flip_phase, previous_flight_mode_rc_value
    now = time.time()
    elapsed_time = now - flip_start_time
    if flip_phase == 1:
        current_rc_values[2] = FLIP_THROTTLE_PUNCH_VALUE
        current_rc_values[1] = 1500
        if elapsed_time >= FLIP_THROTTLE_PUNCH_DURATION:
            flip_phase = 2; flip_start_time = now
            print("Flip: Phase Pitch")
    elif flip_phase == 2:
        current_rc_values[1] = FLIP_PITCH_VALUE
        current_rc_values[2] = int((FLIP_THROTTLE_PUNCH_VALUE + HOVER_THROTTLE_ESTIMATE) / 2.2)
        if elapsed_time >= FLIP_PITCH_DURATION:
            flip_phase = 3; flip_start_time = now
            print("Flip: Phase Récupération")
            current_rc_values[1] = 1500
    elif flip_phase == 3:
        current_rc_values[1] = 1500
        current_rc_values[2] = FLIP_RECOVERY_THROTTLE_VALUE
        if elapsed_time >= FLIP_RECOVERY_THROTTLE_DURATION:
            print("Flip: Terminé. Retour au mode manuel.")
            current_rc_values[ACRO_MODE_CHANNEL_INDEX] = previous_flight_mode_rc_value
            current_flight_state = STATE_MANUAL
            flip_phase = 0
    else: current_flight_state = STATE_MANUAL

def manage_auto_flight_modes():
    global current_rc_values, current_flight_state, is_armed_command
    if current_altitude_m is None and current_flight_state not in [STATE_MANUAL, STATE_PERFORMING_FLIP]:
        print("ATTENTION: Pas de lecture d'altitude, retour manuel et désarmement!")
        current_flight_state = STATE_MANUAL; current_rc_values[2] = THROTTLE_MIN_EFFECTIVE
        current_rc_values[4] = DISARM_VALUE; is_armed_command = False
        return

    if current_flight_state not in [STATE_MANUAL, STATE_PERFORMING_FLIP]:
        current_rc_values[0] = 1500; current_rc_values[1] = 1500
        if yaw_locked:
            current_rc_values[3] = YAW_LOCK_VALUE

    if current_flight_state == STATE_AUTO_TAKEOFF:
        if current_altitude_m is None: return
        error_altitude = TARGET_ALTITUDE_M - current_altitude_m
        throttle_adjustment = KP_ALTITUDE * error_altitude
        throttle_adjustment = max(-MAX_AUTO_THROTTLE_ADJUST, min(MAX_AUTO_THROTTLE_ADJUST, throttle_adjustment))
        commanded_throttle = HOVER_THROTTLE_ESTIMATE + throttle_adjustment
        current_rc_values[2] = int(max(THROTTLE_MIN_EFFECTIVE, min(TAKEOFF_THROTTLE_CEILING, commanded_throttle)))
        if abs(error_altitude) < TAKEOFF_END_ALTITUDE_ERROR_M or current_altitude_m >= TARGET_ALTITUDE_M :
            current_flight_state = STATE_AUTO_HOVER; current_rc_values[2] = HOVER_THROTTLE_ESTIMATE
    elif current_flight_state == STATE_AUTO_HOVER:
        if current_altitude_m is None: return
        error_altitude = TARGET_ALTITUDE_M - current_altitude_m
        throttle_adjustment = KP_ALTITUDE * error_altitude
        throttle_adjustment = max(-MAX_AUTO_THROTTLE_ADJUST, min(MAX_AUTO_THROTTLE_ADJUST, throttle_adjustment))
        commanded_throttle = HOVER_THROTTLE_ESTIMATE + throttle_adjustment
        current_rc_values[2] = int(max(THROTTLE_MIN_EFFECTIVE, min(THROTTLE_MAX_EFFECTIVE, commanded_throttle)))
    elif current_flight_state == STATE_AUTO_LANDING:
        if current_altitude_m is None: return
        if current_altitude_m > LANDING_ALTITUDE_THRESHOLD_M:
            error_altitude = 0.0 - current_altitude_m
            throttle_adjustment = KP_ALTITUDE * 0.5 * error_altitude
            throttle_adjustment = max(-MAX_AUTO_THROTTLE_ADJUST, min(MAX_AUTO_THROTTLE_ADJUST, throttle_adjustment))
            landing_base_throttle = HOVER_THROTTLE_ESTIMATE - 70
            commanded_throttle = landing_base_throttle + throttle_adjustment
            current_rc_values[2] = int(max(THROTTLE_MIN_EFFECTIVE, min(THROTTLE_MAX_EFFECTIVE, commanded_throttle)))
        else:
            current_rc_values[2] = THROTTLE_MIN_EFFECTIVE; current_rc_values[4] = DISARM_VALUE
            is_armed_command = False; current_flight_state = STATE_MANUAL
    elif current_flight_state == STATE_PERFORMING_FLIP:
        manage_flip_sequence()

def handle_joystick_event(event):
    global current_rc_values, is_armed_command, joystick, joystick_connected, current_flight_state
    global flip_start_time, flip_phase, previous_flight_mode_rc_value
    global yaw_locked, althold_active

    if current_flight_state == STATE_MANUAL:
        if event.type == pygame.JOYAXISMOTION:
            if event.axis == AXIS_YAW:
                if not yaw_locked:
                    current_rc_values[3] = map_axis_to_rc(event.value)
            elif event.axis == AXIS_THROTTLE:
                # Contrôle du throttle avec le joystick gauche Y (inversé pour que haut = plus de puissance)
                current_rc_values[2] = map_axis_to_rc(event.value, THROTTLE_MIN_EFFECTIVE, THROTTLE_MAX_EFFECTIVE, inverted=True)
            elif event.axis == AXIS_ROLL:
                current_rc_values[0] = map_axis_to_rc(event.value)
            elif event.axis == AXIS_PITCH:
                current_rc_values[1] = map_axis_to_rc(event.value, inverted=True)
    
    # S'assurer que le yaw reste verrouillé si activé
    if yaw_locked:
        current_rc_values[3] = YAW_LOCK_VALUE

    if event.type == pygame.JOYBUTTONDOWN:
        if event.button == BUTTON_ARM_DISARM:
            if not is_armed_command:
                # Vérifie si la poussée actuelle (par défaut 1300) est inférieure à la sécurité (1350)
                if current_rc_values[2] <= THROTTLE_SAFETY_ARM:
                    current_rc_values[4] = ARM_VALUE; is_armed_command = True
                    print("\nCOMMANDE: ARMEMENT")
                else: print(f"\nSECURITE: Gaz ({current_rc_values[2]}) > {THROTTLE_SAFETY_ARM} pour armer.")
            else: 
                current_rc_values[4] = DISARM_VALUE; is_armed_command = False
                # Lors du désarmement, on remet la poussée au minimum de sécurité (1300)
                current_rc_values[2] = THROTTLE_MIN_EFFECTIVE
                current_flight_state = STATE_MANUAL 
                # Désactiver ALTHOLD lors du désarmement
                althold_active = False
                current_rc_values[5] = 1000  # AUX2 à 1000 (index 5 = canal 6 = AUX2)
                print("\nCOMMANDE: DESARMEMENT")

        elif event.button == BUTTON_AUTO_MODE:
            if not is_armed_command: print("\nINFO: Armez d'abord pour le mode auto."); return None
            if current_flight_state == STATE_MANUAL:
                if current_altitude_m is not None and current_altitude_m < (TARGET_ALTITUDE_M / 2):
                    current_flight_state = STATE_AUTO_TAKEOFF
                else: print("\nINFO: Décollage auto non initié (altitude?).")
            elif current_flight_state == STATE_AUTO_HOVER or current_flight_state == STATE_AUTO_TAKEOFF:
                current_flight_state = STATE_AUTO_LANDING
            elif current_flight_state == STATE_AUTO_LANDING:
                current_flight_state = STATE_AUTO_HOVER
        
        elif event.button == BUTTON_ALTHOLD:
            # Activer ALTHOLD mode (AUX2 à 1800)
            althold_active = True
            current_rc_values[5] = 1800  # AUX2 (index 5 = canal 6 = AUX2)
            print("\nINFO: Mode ALTHOLD ACTIVÉ")
        
        elif event.button == BUTTON_QUIT: return "quit"

    elif event.type == pygame.JOYBUTTONUP:
        if event.button == BUTTON_ALTHOLD:
            # Désactiver ALTHOLD mode (AUX2 à 1000)
            althold_active = False
            current_rc_values[5] = 1000  # AUX2 à 1000 (index 5 = canal 6 = AUX2)
            print("\nINFO: Mode ALTHOLD DÉSACTIVÉ")

    elif event.type == pygame.JOYDEVICEADDED:
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0); joystick.init(); joystick_connected = True
            print(f"\nManette '{joystick.get_name()}' connectée.")
    elif event.type == pygame.JOYDEVICEREMOVED:
        print("\nMANETTE DÉCONNECTÉE - ARRÊT DU SCRIPT POUR SÉCURITÉ")
        return "quit"  # Terminer le script immédiatement
    return None

def print_status():
    sys.stdout.write("\033[K") # Efface la ligne
    alt_str = f"{current_altitude_m:.2f}m" if current_altitude_m is not None else "N/A"
    state_list = ["MANUAL", "TAKEOFF", "HOVER", "LANDING", "FLIPPING"]
    state_str = state_list[current_flight_state] if 0 <= current_flight_state < len(state_list) else "UNKNOWN"
    
    throttle_mode_str = "TEST" if ENABLE_THROTTLE_TEST_LIMIT else "FULL"
    yaw_lock_str = "L" if yaw_locked else "U"
    althold_str = "ON" if althold_active else "OFF"
    
    # Format GPS status
    gps_fix_str = "FIX" if gps_fix else "NO"
    gps_status_str = f"GPS:{gps_fix_str}({gps_num_sat}sat)"

    status_line = (
        f"R:{current_rc_values[0]} P:{current_rc_values[1]} T:{current_rc_values[2]}({throttle_mode_str}) Y:{current_rc_values[3]}({yaw_lock_str}) | "
        f"ARM:{current_rc_values[4]}({'Y' if is_armed_command else 'N'}) | ACRO:{current_rc_values[ACRO_MODE_CHANNEL_INDEX]} | "
        f"ALTHOLD:{current_rc_values[5]}({althold_str}) | "
        f"Alt:{alt_str} | {gps_status_str} | St:{state_str}"
    )
    if current_flight_state == STATE_PERFORMING_FLIP:
        status_line += f" FlipPh:{flip_phase}"
    
    print(status_line, end="\r")
    sys.stdout.flush()

def main():
    global current_rc_values, is_armed_command, joystick, joystick_connected, current_flight_state
    global last_msp_request_time, last_gps_request_time, current_altitude_m
    global THROTTLE_MIN_EFFECTIVE, THROTTLE_MAX_EFFECTIVE, HOVER_THROTTLE_ESTIMATE, THROTTLE_SAFETY_ARM, TAKEOFF_THROTTLE_CEILING
    global yaw_locked, althold_active

    # --- Initialisation des valeurs dynamiques ---
    THROTTLE_MIN_EFFECTIVE = THROTTLE_TEST_MIN_VALUE if ENABLE_THROTTLE_TEST_LIMIT else 1000
    THROTTLE_MAX_EFFECTIVE = THROTTLE_TEST_MAX_VALUE if ENABLE_THROTTLE_TEST_LIMIT else 2000
    
    # Initialisation du throttle à la valeur minimale de sécurité
    current_rc_values[2] = THROTTLE_MIN_EFFECTIVE
    THROTTLE_SAFETY_ARM = THROTTLE_MIN_EFFECTIVE + 50 

    if ENABLE_THROTTLE_TEST_LIMIT:
        HOVER_THROTTLE_ESTIMATE = int(THROTTLE_TEST_MIN_VALUE + (THROTTLE_TEST_MAX_VALUE - THROTTLE_TEST_MIN_VALUE) * 0.75)
    else: HOVER_THROTTLE_ESTIMATE = 1500
    TAKEOFF_THROTTLE_CEILING = THROTTLE_MAX_EFFECTIVE + 50
    
    current_rc_values[3] = YAW_LOCK_VALUE 
    current_rc_values[5] = 1000  # AUX2 initialisé à 1000 (index 5 = canal 6 = AUX2)
    althold_active = False

    print("--- Script Contrôle Drone MSP (Contrôle Joystick Gauche) ---")
    if ENABLE_THROTTLE_TEST_LIMIT: print(f"!!! MODE TEST THROTTLE ACTIF: {THROTTLE_MIN_EFFECTIVE}-{THROTTLE_MAX_EFFECTIVE} !!!")
    else: print("!!! MODE PLEINE POUSSÉE ACTIF: 1000-2000 !!!")
    print(f"Contrôle: Joystick Gauche Y=Throttle, X=Yaw (verrouillé par défaut). Sécurité armement: <={THROTTLE_SAFETY_ARM}")
    print("IMPORTANT: Le script se terminera automatiquement si la manette se déconnecte!")
    print("Bouton Y: Mode ALTHOLD (maintenir appuyé = AUX2 à 1800, relâcher = AUX2 à 1000)")
    
    # --- Initialisation Pygame et Manette ---
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0); joystick.init(); joystick_connected = True
        print(f"Manette '{joystick.get_name()}' connectée.")
    else: 
        print("ERREUR: Aucune manette détectée. Le script nécessite une manette pour fonctionner.")
        pygame.quit()
        sys.exit(1)

    # --- NOUVEAU: Initialisation Port Série ---
    ser = initialize_serial()
    if ser is None:
        pygame.quit()
        sys.exit(1) # Quitter si aucun port série n'est trouvé

    ser_buffer = b''
    running = True
    try:
        while running:
            # 1. Gestion des événements manette
            for event in pygame.event.get():
                if handle_joystick_event(event) == "quit":
                    running = False; break
            if not running: break

            # 2. Communication MSP (Lecture Altitude et GPS)
            current_time = time.time()
            if current_time - last_msp_request_time > MSP_REQUEST_INTERVAL:
                request_msp_data(ser, MSP_ALTITUDE) 
                last_msp_request_time = current_time
            
            if current_time - last_gps_request_time > GPS_REQUEST_INTERVAL:
                request_msp_data(ser, MSP_RAW_GPS)
                last_gps_request_time = current_time
            
            if ser.in_waiting > 0:
                ser_buffer += ser.read(ser.in_waiting)
            ser_buffer = parse_msp_response(ser_buffer)

            # 3. Application de la logique de contrôle
            if joystick_connected:
                # Gestion des modes de vol automatiques
                if is_armed_command and current_flight_state != STATE_MANUAL:
                    manage_auto_flight_modes()

            # Sécurité Yaw Lock
            if yaw_locked:
                current_rc_values[3] = YAW_LOCK_VALUE

            # Gestion du désarmement inattendu pendant un mode auto
            if not is_armed_command and current_flight_state != STATE_MANUAL :
                current_flight_state = STATE_MANUAL
                current_rc_values[2] = THROTTLE_MIN_EFFECTIVE # Retour à la poussée par défaut

            # 4. Envoi des commandes RC et affichage
            if joystick_connected:
                # Clamping des valeurs RC
                for i in [0, 1, 4, 5, 6, 7]: # Roll, Pitch, AUX1-4
                    current_rc_values[i] = max(1000, min(2000, current_rc_values[i]))
                
                if not yaw_locked: # Yaw
                    current_rc_values[3] = max(1000, min(2000, current_rc_values[3]))

                # Envoi MSP
                payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                print_status()
            else:
                # Si on arrive ici, c'est que la manette était connectée mais ne l'est plus
                print("\nMANETTE DÉCONNECTÉE - ARRÊT IMMÉDIAT DU SCRIPT")
                running = False
            
            time.sleep(0.02) # Boucle à ~50Hz

    except KeyboardInterrupt: print("\nArrêt Ctrl+C.")
    except Exception as e: print(f"\nErreur inattendue: {e}")
    finally:
        print("\nNettoyage et commandes de sécurité finales...")
        final_rc = [1500]*RC_CHANNELS_COUNT
        # Utiliser la poussée minimale absolue (1000) pour la sécurité finale
        final_rc[2] = 1000 
        final_rc[4] = DISARM_VALUE
        final_rc[3] = YAW_LOCK_VALUE
        payload_final = b''.join(struct.pack('<H', int(v)) for v in final_rc)
        if 'ser' in locals() and ser is not None and ser.is_open:
            for _ in range(5): send_msp_packet(ser, MSP_SET_RAW_RC, payload_final); time.sleep(0.02)
            ser.close(); print("Port série fermé.")
        pygame.quit(); print("Pygame quitté. Script terminé.")

if __name__ == "__main__":
    main()