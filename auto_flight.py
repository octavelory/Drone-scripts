import serial
import struct
import time
import sys
import os
import pygame

# --- Configuration Globale ---
ENABLE_THROTTLE_TEST_LIMIT = True # Mettre à False pour utiliser la pleine poussée (1000-2000)
THROTTLE_TEST_MIN_VALUE = 1000
THROTTLE_TEST_MAX_VALUE = 1700

# --- Configuration et Constantes MSP ---
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200
MSP_ALTITUDE = 109

# --- Configuration du Contrôle ---
RC_CHANNELS_COUNT = 8
current_rc_values = [1500] * RC_CHANNELS_COUNT
THROTTLE_MIN_EFFECTIVE = THROTTLE_TEST_MIN_VALUE if ENABLE_THROTTLE_TEST_LIMIT else 1000
THROTTLE_MAX_EFFECTIVE = THROTTLE_TEST_MAX_VALUE if ENABLE_THROTTLE_TEST_LIMIT else 2000

current_rc_values[2] = THROTTLE_MIN_EFFECTIVE  # Throttle bas au démarrage
current_rc_values[4] = 1000  # AUX1 (Arm switch) désarmé

ARM_VALUE = 1800
DISARM_VALUE = 1000
THROTTLE_SAFETY_ARM = THROTTLE_MIN_EFFECTIVE + 100

# État du drone
is_armed_command = False

# --- NOUVEAU: Configuration Verrouillage Yaw ---
yaw_locked = True # Yaw verrouillé par défaut au démarrage
YAW_LOCK_VALUE = 1530 # Valeur du yaw lorsqu'il est verrouillé
current_rc_values[3] = YAW_LOCK_VALUE # S'assurer que le yaw est à la valeur de verrouillage au démarrage

# --- Configuration Manette ---
AXIS_YAW = 0        # Joystick Gauche X (pour Yaw)
AXIS_THROTTLE = 1   # Joystick Gauche Y (pour Throttle)
AXIS_ROLL = 3       # Joystick Droit X (pour Roll) - Souvent 2 ou 3 selon la manette
AXIS_PITCH = 4      # Joystick Droit Y (pour Pitch) - Souvent 3 ou 4 selon la manette
# Vérifiez ces axes avec `pygame.examples.joystick` si besoin

BUTTON_ARM_DISARM = 4 # Souvent L1/LB
BUTTON_QUIT = 5       # Souvent L2/LT (peut être un axe sur certaines manettes)
BUTTON_AUTO_MODE = 2  # Souvent X ou Carré
BUTTON_FLIP = 3       # Souvent Y ou Triangle
BUTTON_TOGGLE_YAW_LOCK = 6 # NOUVEAU: Souvent R1/RB. Choisir un bouton disponible.

JOYSTICK_DEADZONE = 0.08

# --- Configuration Vol Automatique ---
TARGET_ALTITUDE_M = 2.0
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
MSP_REQUEST_INTERVAL = 0.05

HOVER_THROTTLE_ESTIMATE = (THROTTLE_MIN_EFFECTIVE + THROTTLE_MAX_EFFECTIVE) // 2 + 50
KP_ALTITUDE = 20.0
MAX_AUTO_THROTTLE_ADJUST = 100
TAKEOFF_THROTTLE_CEILING = THROTTLE_MAX_EFFECTIVE + 50

# --- Configuration Flip ---
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

# --- Fonctions MSP ---
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
    global current_altitude_m
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
        return ser_buffer[idx+6+payload_size:]
    else:
        return ser_buffer[idx+1:]

# --- Logique de Contrôle Manette ---
def map_axis_to_rc(axis_value, min_rc=1000, max_rc=2000, inverted=False):
    if abs(axis_value) < JOYSTICK_DEADZONE: axis_value = 0.0
    if inverted: axis_value = -axis_value
    normalized_value = (axis_value + 1.0) / 2.0
    return int(min_rc + normalized_value * (max_rc - min_rc))

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
        if yaw_locked: # MODIFIÉ: Si yaw verrouillé, le maintenir à sa valeur de verrouillage
            current_rc_values[3] = YAW_LOCK_VALUE
        # Si le yaw n'est pas verrouillé, on pourrait vouloir le laisser contrôlable ou le mettre neutre.
        # Pour les modes auto, il est généralement préférable de le mettre neutre.
        else:
            current_rc_values[3] = 1500


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
    global yaw_locked # NOUVEAU

    if current_flight_state == STATE_MANUAL:
        if event.type == pygame.JOYAXISMOTION:
            if event.axis == AXIS_THROTTLE:
                current_rc_values[2] = map_axis_to_rc(event.value, min_rc=THROTTLE_MIN_EFFECTIVE, max_rc=THROTTLE_MAX_EFFECTIVE, inverted=True)
            elif event.axis == AXIS_YAW:
                if not yaw_locked: # MODIFIÉ: Contrôler le yaw seulement s'il n'est pas verrouillé
                    current_rc_values[3] = map_axis_to_rc(event.value)
                # Si yaw_locked est True, current_rc_values[3] reste à YAW_LOCK_VALUE
            elif event.axis == AXIS_ROLL:
                current_rc_values[0] = map_axis_to_rc(event.value)
            elif event.axis == AXIS_PITCH:
                current_rc_values[1] = map_axis_to_rc(event.value, inverted=False)
    
    # S'assurer que si le yaw est verrouillé, sa valeur est maintenue, même en mode manuel
    # Ceci est important si on sort d'un mode auto où le yaw aurait pu être modifié.
    if yaw_locked:
        current_rc_values[3] = YAW_LOCK_VALUE

    if event.type == pygame.JOYBUTTONDOWN:
        if event.button == BUTTON_ARM_DISARM:
            if not is_armed_command:
                if current_rc_values[2] <= THROTTLE_SAFETY_ARM:
                    current_rc_values[4] = ARM_VALUE; is_armed_command = True
                    print("\nCOMMANDE: ARMEMENT")
                else: print(f"\nSECURITE: Gaz ({current_rc_values[2]}) > {THROTTLE_SAFETY_ARM} pour armer.")
            else: 
                current_rc_values[4] = DISARM_VALUE; is_armed_command = False
                current_rc_values[2] = THROTTLE_MIN_EFFECTIVE
                current_flight_state = STATE_MANUAL 
                print("\nCOMMANDE: DESARMEMENT")
        
        elif event.button == BUTTON_TOGGLE_YAW_LOCK: # NOUVEAU
            yaw_locked = not yaw_locked
            if yaw_locked:
                current_rc_values[3] = YAW_LOCK_VALUE # Forcer le yaw à la valeur de verrouillage
                print("\nINFO: Yaw VERROUILLÉ")
            else:
                print("\nINFO: Yaw DÉVERROUILLÉ")
                # Optionnel: si on déverrouille, on pourrait vouloir que le yaw prenne la position actuelle du stick
                # Mais cela sera géré par le prochain événement JOYAXISMOTION pour AXIS_YAW

        elif event.button == BUTTON_AUTO_MODE:
            if not is_armed_command: print("\nINFO: Armez d'abord pour le mode auto."); return None
            if current_flight_state == STATE_MANUAL and current_rc_values[2] <= THROTTLE_SAFETY_ARM:
                if current_altitude_m is not None and current_altitude_m < (TARGET_ALTITUDE_M / 2):
                    current_flight_state = STATE_AUTO_TAKEOFF
                else: print("\nINFO: Décollage auto non initié (altitude?).")
            elif current_flight_state == STATE_AUTO_HOVER or current_flight_state == STATE_AUTO_TAKEOFF:
                current_flight_state = STATE_AUTO_LANDING
            elif current_flight_state == STATE_AUTO_LANDING:
                current_flight_state = STATE_AUTO_HOVER
        
        elif event.button == BUTTON_FLIP:
            if is_armed_command and current_flight_state == STATE_MANUAL:
                if current_altitude_m is not None and current_altitude_m > 1.5 :
                    print("!!! TENTATIVE DE FLIP !!!")
                    previous_flight_mode_rc_value = current_rc_values[ACRO_MODE_CHANNEL_INDEX]
                    current_rc_values[ACRO_MODE_CHANNEL_INDEX] = ACRO_MODE_VALUE
                    current_flight_state = STATE_PERFORMING_FLIP
                    flip_start_time = time.time(); flip_phase = 1
                    print("Flip: Phase Punch Gaz")
                else: print("Flip: Altitude trop basse ou non lue.")
            elif not is_armed_command: print("Flip: Drone non armé.")
            else: print(f"Flip: Non autorisé dans l'état {current_flight_state}.")
        
        elif event.button == BUTTON_QUIT: return "quit"

    elif event.type == pygame.JOYDEVICEADDED:
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0); joystick.init(); joystick_connected = True
            print(f"\nManette '{joystick.get_name()}' connectée.")
    elif event.type == pygame.JOYDEVICEREMOVED:
        joystick_connected = False; joystick = None
        print("\nManette déconnectée. Passage en mode manuel et désarmement.")
        current_flight_state = STATE_MANUAL
        current_rc_values[2] = THROTTLE_MIN_EFFECTIVE
        current_rc_values[4] = DISARM_VALUE
        is_armed_command = False
    return None

def print_status():
    sys.stdout.write("\033[K") # Efface la ligne
    alt_str = f"{current_altitude_m:.2f}m" if current_altitude_m is not None else "N/A"
    state_list = ["MANUAL", "TAKEOFF", "HOVER", "LANDING", "FLIPPING"]
    state_str = state_list[current_flight_state] if 0 <= current_flight_state < len(state_list) else "UNKNOWN"
    
    throttle_mode_str = "TEST" if ENABLE_THROTTLE_TEST_LIMIT else "FULL"
    yaw_lock_str = "L" if yaw_locked else "U" # MODIFIÉ: L pour Locked, U pour Unlocked

    status_line = (
        f"R:{current_rc_values[0]} P:{current_rc_values[1]} T:{current_rc_values[2]}({throttle_mode_str}) Y:{current_rc_values[3]}({yaw_lock_str}) | " # MODIFIÉ
        f"ARM:{current_rc_values[4]}({'Y' if is_armed_command else 'N'}) | ACRO:{current_rc_values[ACRO_MODE_CHANNEL_INDEX]} | "
        f"Alt:{alt_str} | St:{state_str}"
    )
    if current_flight_state == STATE_PERFORMING_FLIP:
        status_line += f" FlipPh:{flip_phase}"
    
    print(status_line, end="\r")
    sys.stdout.flush()

def main():
    global current_rc_values, is_armed_command, joystick, joystick_connected, current_flight_state
    global last_msp_request_time, current_altitude_m
    global THROTTLE_MIN_EFFECTIVE, THROTTLE_MAX_EFFECTIVE, HOVER_THROTTLE_ESTIMATE, THROTTLE_SAFETY_ARM, TAKEOFF_THROTTLE_CEILING
    global yaw_locked # NOUVEAU

    THROTTLE_MIN_EFFECTIVE = THROTTLE_TEST_MIN_VALUE if ENABLE_THROTTLE_TEST_LIMIT else 1000
    THROTTLE_MAX_EFFECTIVE = THROTTLE_TEST_MAX_VALUE if ENABLE_THROTTLE_TEST_LIMIT else 2000
    current_rc_values[2] = THROTTLE_MIN_EFFECTIVE
    THROTTLE_SAFETY_ARM = THROTTLE_MIN_EFFECTIVE + 100
    if ENABLE_THROTTLE_TEST_LIMIT:
        HOVER_THROTTLE_ESTIMATE = int(THROTTLE_TEST_MIN_VALUE + (THROTTLE_TEST_MAX_VALUE - THROTTLE_TEST_MIN_VALUE) * 0.75)
    else: HOVER_THROTTLE_ESTIMATE = 1500
    TAKEOFF_THROTTLE_CEILING = THROTTLE_MAX_EFFECTIVE + 50
    
    # S'assurer que le yaw est initialisé à sa valeur verrouillée
    current_rc_values[3] = YAW_LOCK_VALUE 
    yaw_locked = True # Redondant si déjà défini globalement, mais assure l'état initial

    print("--- Script Contrôle Drone MSP (Manette + Auto Alt + Flip + Yaw Lock) ---")
    if ENABLE_THROTTLE_TEST_LIMIT: print(f"!!! MODE TEST THROTTLE ACTIF: {THROTTLE_MIN_EFFECTIVE}-{THROTTLE_MAX_EFFECTIVE} !!!")
    else: print("!!! MODE PLEINE POUSSÉE ACTIF: 1000-2000 !!!")
    print("!!! FLIP EST EXTRÊMEMENT DANGEREUX !!!")
    
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0); joystick.init(); joystick_connected = True
        print(f"Manette '{joystick.get_name()}' connectée.")
    else: print("Aucune manette détectée.")

    ser_buffer = b''
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0) 
        print(f"Port série {SERIAL_PORT} ouvert.")
    except serial.SerialException as e:
        print(f"Erreur port série: {e}"); pygame.quit(); return

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if handle_joystick_event(event) == "quit":
                    running = False; break
            if not running: break

            if time.time() - last_msp_request_time > MSP_REQUEST_INTERVAL:
                request_msp_data(ser, MSP_ALTITUDE) 
                last_msp_request_time = time.time()
            
            if ser.in_waiting > 0:
                ser_buffer += ser.read(ser.in_waiting)
            ser_buffer = parse_msp_response(ser_buffer)

            # MODIFIÉ: S'assurer que le yaw est à YAW_LOCK_VALUE si yaw_locked est True
            # Ceci est une double sécurité, car handle_joystick_event devrait déjà le gérer.
            if yaw_locked:
                current_rc_values[3] = YAW_LOCK_VALUE

            if joystick_connected and is_armed_command and current_flight_state != STATE_MANUAL:
                manage_auto_flight_modes()
            
            if not is_armed_command and current_flight_state != STATE_MANUAL :
                current_flight_state = STATE_MANUAL
                current_rc_values[2] = THROTTLE_MIN_EFFECTIVE

            if joystick_connected:
                if current_flight_state == STATE_MANUAL:
                     current_rc_values[2] = max(THROTTLE_MIN_EFFECTIVE, min(THROTTLE_MAX_EFFECTIVE, current_rc_values[2]))
                
                for i in [0, 1, 4, 5, 6, 7]: # Tous sauf throttle et yaw (gérés spécifiquement)
                    current_rc_values[i] = max(1000, min(2000, current_rc_values[i]))
                # S'assurer que le yaw est aussi clampé s'il n'est pas verrouillé
                if not yaw_locked:
                    current_rc_values[3] = max(1000, min(2000, current_rc_values[3]))


                payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                print_status()
            else: 
                sys.stdout.write("\033[K"); print("Manette déconnectée...", end="\r"); sys.stdout.flush()
                if is_armed_command:
                    current_rc_values[2] = THROTTLE_MIN_EFFECTIVE; current_rc_values[4] = DISARM_VALUE
                    is_armed_command = False; current_flight_state = STATE_MANUAL
                    current_rc_values[3] = YAW_LOCK_VALUE # Verrouiller le yaw si manette déconnectée
                    yaw_locked = True
                    payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                    send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
            time.sleep(0.02)

    except KeyboardInterrupt: print("\nArrêt Ctrl+C.")
    except Exception as e: print(f"\nErreur inattendue: {e}")
    finally:
        print("\nNettoyage et commandes de sécurité finales...")
        final_rc = [1500]*RC_CHANNELS_COUNT
        final_rc[2] = THROTTLE_MIN_EFFECTIVE; final_rc[4] = DISARM_VALUE
        final_rc[3] = YAW_LOCK_VALUE # S'assurer que le yaw est neutre à la fin
        payload_final = b''.join(struct.pack('<H', int(v)) for v in final_rc)
        if 'ser' in locals() and ser.is_open:
            for _ in range(5): send_msp_packet(ser, MSP_SET_RAW_RC, payload_final); time.sleep(0.02)
            ser.close(); print("Port série fermé.")
        pygame.quit(); print("Pygame quitté. Script terminé.")

if __name__ == "__main__":
    main()