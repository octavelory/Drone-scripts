import serial
import struct
import time
import sys
import os
import pygame

# --- Configuration et Constantes MSP ---
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200
MSP_ALTITUDE = 109      # Pour lire l'altitude du baromètre (en cm)
# MSP_RANGEFINDER = 117 # Pour lire un télémètre laser/sonar (en cm) - Préférable pour basse altitude

# --- Configuration du Contrôle ---
RC_CHANNELS_COUNT = 8
current_rc_values = [1500] * RC_CHANNELS_COUNT
current_rc_values[2] = 1000  # Throttle bas au démarrage
current_rc_values[4] = 1000  # AUX1 (Arm switch) désarmé

ARM_VALUE = 1800
DISARM_VALUE = 1000

THROTTLE_MIN_OPERATIONAL = 1000
THROTTLE_MAX_OPERATIONAL = 2000 # Pour référence

THROTTLE_TEST_MIN = 1000
THROTTLE_TEST_MAX = 1100 # Limite pour les tests au sol / vol très bas
THROTTLE_SAFETY_ARM = 1100

# État du drone
is_armed_command = False

# --- Configuration Manette ---
AXIS_YAW = 0
AXIS_THROTTLE = 1
AXIS_ROLL = 3
AXIS_PITCH = 4
BUTTON_ARM_DISARM = 4
BUTTON_QUIT = 5
BUTTON_AUTO_MODE = 2 # Bouton pour décollage/atterrissage auto
JOYSTICK_DEADZONE = 0.08

# --- Configuration Vol Automatique ---
TARGET_ALTITUDE_M = 2.0
LANDING_ALTITUDE_THRESHOLD_M = 0.20 # Seuil pour considérer atterri (ajuster si Lidar/Baro)
TAKEOFF_END_ALTITUDE_ERROR_M = 0.25 # Marge d'erreur pour considérer décollage terminé

# États du vol automatique
STATE_MANUAL = 0
STATE_AUTO_TAKEOFF = 1
STATE_AUTO_HOVER = 2
STATE_AUTO_LANDING = 3
current_flight_state = STATE_MANUAL

current_altitude_m = None # Sera mis à jour par MSP
last_msp_request_time = 0
MSP_REQUEST_INTERVAL = 0.05 # Demander l'altitude toutes les 50ms

# --- Paramètres du Contrôleur P pour l'Altitude (À RÉGLER AVEC EXTRÊME PRUDENCE) ---
# HOVER_THROTTLE_ESTIMATE: Valeur de gaz (dans la plage THROTTLE_TEST_MIN/MAX)
# où le drone est supposé être en stationnaire.
# À trouver expérimentalement SANS HÉLICES en observant les moteurs, puis avec hélices très prudemment.
# Si votre drone décolle à 1500 en mode normal, et que votre plage de test est 1000-1400,
# cette valeur sera probablement proche de 1350-1380, mais CELA DÉPEND TOTALEMENT DE VOTRE DRONE.
HOVER_THROTTLE_ESTIMATE = 1350 # !!! VALEUR CRITIQUE À RÉGLER !!!
KP_ALTITUDE = 20.0           # !!! GAIN PROPORTIONNEL - COMMENCER TRÈS BAS (ex: 5-20) !!!
MAX_AUTO_THROTTLE_ADJUST = 100 # Ajustement max du throttle par le P-contrôleur (+/-)
# Pour le décollage, on peut autoriser un throttle un peu plus élevé temporairement
TAKEOFF_THROTTLE_CEILING = THROTTLE_TEST_MAX + 50 # Max throttle pendant le décollage auto

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
    # Recherche du début de trame MSP '$M>'
    idx = ser_buffer.find(b'$M>')
    if idx == -1:
        return ser_buffer # Pas de début de trame trouvé, garder le buffer

    # S'assurer qu'on a assez de données pour l'en-tête (préambule, taille, cmd)
    if len(ser_buffer) < idx + 5:
        return ser_buffer[idx:] # Trame incomplète, garder à partir du début trouvé

    payload_size = ser_buffer[idx+3]
    cmd = ser_buffer[idx+4]

    # S'assurer qu'on a la trame complète (préambule, taille, cmd, payload, checksum)
    if len(ser_buffer) < idx + 5 + payload_size + 1:
        return ser_buffer[idx:] # Trame incomplète

    full_packet_payload = ser_buffer[idx+3 : idx+5+payload_size] # size, cmd, data
    received_checksum = ser_buffer[idx+5+payload_size]
    calculated_checksum = calculate_checksum(full_packet_payload)

    if received_checksum == calculated_checksum:
        payload_data = ser_buffer[idx+5 : idx+5+payload_size]
        if cmd == MSP_ALTITUDE and payload_size >= 4:
            altitude_cm = struct.unpack('<i', payload_data[0:4])[0]
            current_altitude_m = float(altitude_cm) / 100.0
            # print(f"Alt MSP: {current_altitude_m:.2f}m") # Debug
        # elif cmd == MSP_RANGEFINDER and payload_size >= 2:
        #     distance_cm = struct.unpack('<H', payload_data[0:2])[0]
        #     current_altitude_m = float(distance_cm) / 100.0
        #     # print(f"Range MSP: {current_altitude_m:.2f}m") # Debug
        
        return ser_buffer[idx+6+payload_size:] # Retourner le reste du buffer
    else:
        # print("MSP Checksum error")
        return ser_buffer[idx+1:] # Erreur checksum, avancer d'un octet et réessayer

# --- Logique de Contrôle Manette ---
def map_axis_to_rc(axis_value, min_rc=1000, max_rc=2000, inverted=False):
    if abs(axis_value) < JOYSTICK_DEADZONE: axis_value = 0.0
    if inverted: axis_value = -axis_value
    normalized_value = (axis_value + 1.0) / 2.0
    return int(min_rc + normalized_value * (max_rc - min_rc))

def manage_auto_flight_modes():
    global current_rc_values, current_flight_state, is_armed_command
    
    if current_altitude_m is None and current_flight_state != STATE_MANUAL:
        print("ATTENTION: Pas de lecture d'altitude, retour en mode manuel et désarmement!")
        current_flight_state = STATE_MANUAL
        current_rc_values[2] = THROTTLE_TEST_MIN
        current_rc_values[4] = DISARM_VALUE
        is_armed_command = False
        return

    # --- Neutraliser Roll/Pitch/Yaw en mode auto ---
    if current_flight_state != STATE_MANUAL:
        current_rc_values[0] = 1500 # Roll
        current_rc_values[1] = 1500 # Pitch
        current_rc_values[3] = 1500 # Yaw

    # --- Logique des états ---
    if current_flight_state == STATE_AUTO_TAKEOFF:
        if current_altitude_m is None: return # Attendre une lecture d'altitude

        error_altitude = TARGET_ALTITUDE_M - current_altitude_m
        throttle_adjustment = KP_ALTITUDE * error_altitude
        throttle_adjustment = max(-MAX_AUTO_THROTTLE_ADJUST, min(MAX_AUTO_THROTTLE_ADJUST, throttle_adjustment))
        
        commanded_throttle = HOVER_THROTTLE_ESTIMATE + throttle_adjustment
        # Permettre de dépasser THROTTLE_TEST_MAX pour le décollage, mais avec un plafond
        current_rc_values[2] = int(max(THROTTLE_TEST_MIN, min(TAKEOFF_THROTTLE_CEILING, commanded_throttle)))

        if abs(error_altitude) < TAKEOFF_END_ALTITUDE_ERROR_M or current_altitude_m >= TARGET_ALTITUDE_M :
            print("\nINFO: Altitude cible décollage atteinte, passage en stationnaire auto.")
            current_flight_state = STATE_AUTO_HOVER
            current_rc_values[2] = HOVER_THROTTLE_ESTIMATE # Essayer de stabiliser
    
    elif current_flight_state == STATE_AUTO_HOVER:
        if current_altitude_m is None: return

        error_altitude = TARGET_ALTITUDE_M - current_altitude_m
        throttle_adjustment = KP_ALTITUDE * error_altitude
        throttle_adjustment = max(-MAX_AUTO_THROTTLE_ADJUST, min(MAX_AUTO_THROTTLE_ADJUST, throttle_adjustment))
        
        commanded_throttle = HOVER_THROTTLE_ESTIMATE + throttle_adjustment
        current_rc_values[2] = int(max(THROTTLE_TEST_MIN, min(THROTTLE_TEST_MAX, commanded_throttle)))

    elif current_flight_state == STATE_AUTO_LANDING:
        if current_altitude_m is None: return

        if current_altitude_m > LANDING_ALTITUDE_THRESHOLD_M:
            # Viser une descente douce. On peut cibler 0m ou simplement réduire le throttle.
            # Ici, on utilise le P-contrôleur pour viser 0m, mais avec un gain réduit pour la descente.
            error_altitude = 0.0 - current_altitude_m # Erreur sera négative
            throttle_adjustment = KP_ALTITUDE * 0.5 * error_altitude # Gain réduit pour descente
            throttle_adjustment = max(-MAX_AUTO_THROTTLE_ADJUST, min(MAX_AUTO_THROTTLE_ADJUST, throttle_adjustment))

            # Base de throttle pour la descente (un peu en dessous du hover)
            landing_base_throttle = HOVER_THROTTLE_ESTIMATE - 70 # Ajuster cette valeur
            commanded_throttle = landing_base_throttle + throttle_adjustment
            current_rc_values[2] = int(max(THROTTLE_TEST_MIN, min(THROTTLE_TEST_MAX, commanded_throttle)))
        else:
            print("\nINFO: Atterrissage détecté.")
            current_rc_values[2] = THROTTLE_TEST_MIN
            current_rc_values[4] = DISARM_VALUE
            is_armed_command = False
            current_flight_state = STATE_MANUAL

def handle_joystick_event(event):
    global current_rc_values, is_armed_command, joystick, joystick_connected, current_flight_state

    # --- Gestion des axes en mode MANUEL uniquement ---
    if current_flight_state == STATE_MANUAL:
        if event.type == pygame.JOYAXISMOTION:
            if event.axis == AXIS_THROTTLE:
                current_rc_values[2] = map_axis_to_rc(event.value, min_rc=THROTTLE_TEST_MIN, max_rc=THROTTLE_TEST_MAX, inverted=True)
            elif event.axis == AXIS_YAW:
                current_rc_values[3] = map_axis_to_rc(event.value)
            elif event.axis == AXIS_ROLL:
                current_rc_values[0] = map_axis_to_rc(event.value)
            elif event.axis == AXIS_PITCH:
                current_rc_values[1] = map_axis_to_rc(event.value, inverted=False)

    # --- Gestion des boutons (affecte tous les modes) ---
    if event.type == pygame.JOYBUTTONDOWN:
        if event.button == BUTTON_ARM_DISARM:
            if not is_armed_command:
                if current_rc_values[2] <= THROTTLE_SAFETY_ARM:
                    current_rc_values[4] = ARM_VALUE; is_armed_command = True
                    print("\nCOMMANDE: ARMEMENT")
                else:
                    print(f"\nSECURITE: Gaz ({current_rc_values[2]}) > {THROTTLE_SAFETY_ARM} pour armer.")
            else: # Si armé, ce bouton désarme
                current_rc_values[4] = DISARM_VALUE; is_armed_command = False
                current_rc_values[2] = THROTTLE_TEST_MIN
                current_flight_state = STATE_MANUAL # Forcer manuel au désarmement
                print("\nCOMMANDE: DESARMEMENT")
        
        elif event.button == BUTTON_AUTO_MODE:
            if not is_armed_command:
                print("\nINFO: Armez d'abord pour le mode auto.")
                return None

            if current_flight_state == STATE_MANUAL and current_rc_values[2] <= THROTTLE_SAFETY_ARM:
                if current_altitude_m is not None and current_altitude_m < (TARGET_ALTITUDE_M / 2): # S'assurer qu'on est bien au sol
                    print("\nCOMMANDE: DECOLLAGE AUTO -> 2m")
                    current_flight_state = STATE_AUTO_TAKEOFF
                else:
                    print("\nINFO: Décollage auto non initié (altitude non lue ou trop haute).")
            elif current_flight_state == STATE_AUTO_HOVER or current_flight_state == STATE_AUTO_TAKEOFF:
                print("\nCOMMANDE: ATTERRISSAGE AUTO")
                current_flight_state = STATE_AUTO_LANDING
            elif current_flight_state == STATE_AUTO_LANDING: # Appuyer à nouveau pendant l'atterrissage
                print("\nCOMMANDE: Annulation atterrissage -> HOVER AUTO")
                current_flight_state = STATE_AUTO_HOVER # Tenter de reprendre le hover
            else:
                print(f"\nINFO: Mode auto non géré depuis l'état {current_flight_state}")

        elif event.button == BUTTON_QUIT:
            return "quit"

    # --- Gestion connexion/déconnexion manette ---
    elif event.type == pygame.JOYDEVICEADDED:
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0); joystick.init(); joystick_connected = True
            print(f"\nManette '{joystick.get_name()}' connectée.")
    elif event.type == pygame.JOYDEVICEREMOVED:
        joystick_connected = False; joystick = None
        print("\nManette déconnectée. Passage en mode manuel et désarmement.")
        current_flight_state = STATE_MANUAL
        current_rc_values[2] = THROTTLE_TEST_MIN
        current_rc_values[4] = DISARM_VALUE
        is_armed_command = False
    return None

def print_status():
    sys.stdout.write("\033[K")
    alt_str = f"{current_altitude_m:.2f}m" if current_altitude_m is not None else "N/A"
    state_str = ["MANUAL", "TAKEOFF", "HOVER", "LANDING"][current_flight_state]
    status_line = (
        f"R:{current_rc_values[0]} P:{current_rc_values[1]} T:{current_rc_values[2]} Y:{current_rc_values[3]} | "
        f"AUX1(Arm):{current_rc_values[4]} | Armed:{'Y' if is_armed_command else 'N'} | "
        f"Alt:{alt_str} | State:{state_str}"
    )
    print(status_line, end="\r")
    sys.stdout.flush()

def main():
    global current_rc_values, is_armed_command, joystick, joystick_connected, current_flight_state
    global last_msp_request_time, current_altitude_m
    
    print("--- Script Contrôle Drone MSP (Manette + Auto Alt) ---")
    print("!!! ATTENTION: SCRIPT EXPÉRIMENTAL ET DANGEREUX !!!")
    print(f"!!! TEST SANS HÉLICES D'ABORD. KP_ALTITUDE={KP_ALTITUDE}, HOVER_THROTTLE={HOVER_THROTTLE_ESTIMATE} !!!")
    print(f"!!! THROTTLE LIMITÉ À {THROTTLE_TEST_MIN}-{THROTTLE_TEST_MAX} (sauf décollage auto: {TAKEOFF_THROTTLE_CEILING}) !!!")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0); joystick.init(); joystick_connected = True
        print(f"Manette '{joystick.get_name()}' connectée.")
        current_rc_values[2] = THROTTLE_TEST_MIN
    else:
        print("Aucune manette détectée.")

    ser_buffer = b''
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0) # Timeout non bloquant
        print(f"Port série {SERIAL_PORT} ouvert.")
    except serial.SerialException as e:
        print(f"Erreur port série: {e}"); pygame.quit(); return

    running = True
    try:
        while running:
            # --- Gestion Événements Pygame ---
            for event in pygame.event.get():
                if handle_joystick_event(event) == "quit":
                    running = False; break
            if not running: break

            # --- Communication MSP (Lecture Altitude) ---
            if time.time() - last_msp_request_time > MSP_REQUEST_INTERVAL:
                request_msp_data(ser, MSP_ALTITUDE) # ou MSP_RANGEFINDER
                last_msp_request_time = time.time()
            
            if ser.in_waiting > 0:
                ser_buffer += ser.read(ser.in_waiting)
            ser_buffer = parse_msp_response(ser_buffer)


            # --- Logique de Vol Automatique ---
            if joystick_connected and is_armed_command and current_flight_state != STATE_MANUAL:
                manage_auto_flight_modes()
            
            # --- Sécurité: si désarmé, s'assurer que le throttle est bas et état manuel ---
            if not is_armed_command and current_flight_state != STATE_MANUAL :
                current_flight_state = STATE_MANUAL
                current_rc_values[2] = THROTTLE_TEST_MIN


            # --- Envoi Commandes RC ---
            if joystick_connected: # Toujours envoyer si manette connectée, même si désarmé (pour AUX)
                # Clamp final des valeurs RC (surtout throttle en mode manuel)
                if current_flight_state == STATE_MANUAL:
                     current_rc_values[2] = max(THROTTLE_TEST_MIN, min(THROTTLE_TEST_MAX, current_rc_values[2]))

                payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                print_status()
            else: # Pas de manette
                sys.stdout.write("\033[K"); print("Manette déconnectée...", end="\r"); sys.stdout.flush()
                # S'assurer du désarmement si la manette est perdue
                if is_armed_command:
                    current_rc_values[2] = THROTTLE_TEST_MIN; current_rc_values[4] = DISARM_VALUE
                    is_armed_command = False; current_flight_state = STATE_MANUAL
                    payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                    send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)


            time.sleep(0.02) # Boucle principale ~50Hz

    except KeyboardInterrupt: print("\nArrêt Ctrl+C.")
    except Exception as e: print(f"\nErreur inattendue: {e}")
    finally:
        print("\nNettoyage et commandes de sécurité finales...")
        final_rc = [1500]*RC_CHANNELS_COUNT
        final_rc[2] = THROTTLE_TEST_MIN; final_rc[4] = DISARM_VALUE
        payload_final = b''.join(struct.pack('<H', int(v)) for v in final_rc)
        if 'ser' in locals() and ser.is_open:
            for _ in range(5): send_msp_packet(ser, MSP_SET_RAW_RC, payload_final); time.sleep(0.02)
            ser.close(); print("Port série fermé.")
        pygame.quit(); print("Pygame quitté. Script terminé.")

if __name__ == "__main__":
    main()