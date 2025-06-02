import serial
import struct
import time
import sys
import pygame # pygame est utilisé pour la manette

# --- Configuration Globale ---
ENABLE_THROTTLE_TEST_LIMIT = True # Mettre à False pour utiliser la pleine poussée (1000-2000)
THROTTLE_MIN_VALUE = 1000 # Valeur minimale absolue pour les gaz (correspond à min_command)
THROTTLE_MAX_VALUE = 2000 # Valeur maximale absolue pour les gaz

# Définir les limites effectives des gaz en fonction du mode test
THROTTLE_MIN_EFFECTIVE = THROTTLE_MIN_VALUE
THROTTLE_MAX_EFFECTIVE = THROTTLE_MAX_VALUE
if ENABLE_THROTTLE_TEST_LIMIT:
    THROTTLE_MAX_EFFECTIVE = 1600 # Limite pour les tests, ajustez si besoin

# --- Configuration et Constantes MSP ---
SERIAL_PORT = '/dev/ttyAMA0'  # VÉRIFIEZ CECI ! Doit correspondre à l'UART connecté au FC
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200
MSP_ALTITUDE = 109 # Pour affichage uniquement

# --- Configuration du Contrôle ---
# Betaflight map AETR1234: Ch1=Roll, Ch2=Pitch, Ch3=Throttle, Ch4=Yaw, Ch5=AUX1...
# Indices Python (0-based):
RC_CHAN_ROLL = 0
RC_CHAN_PITCH = 1
RC_CHAN_THROTTLE = 2
RC_CHAN_YAW = 3
RC_CHAN_AUX1_ARM = 4
RC_CHAN_AUX2_ANGLE_MODE = 5
# RC_CHAN_AUX3 = 6
# RC_CHAN_AUX4 = 7
RC_CHANNELS_COUNT = 8 # Envoyer 8 canaux, même si seuls les 6 premiers sont activement utilisés

# Initialisation des valeurs RC
current_rc_values = [1500] * RC_CHANNELS_COUNT
current_rc_values[RC_CHAN_THROTTLE] = THROTTLE_MIN_EFFECTIVE # Throttle bas au démarrage
current_rc_values[RC_CHAN_AUX1_ARM] = 1000  # AUX1 (Arm switch) désarmé (valeur basse)

# Valeurs pour les switchs AUX
ARM_VALUE = 1800       # Valeur pour armer (dans la plage 1525-2100 de Betaflight)
DISARM_VALUE = 1000    # Valeur pour désarmer

ANGLE_MODE_ON_VALUE = 1800  # Valeur pour activer le mode Angle (dans la plage 1525-2100)
ANGLE_MODE_OFF_VALUE = 1000 # Valeur pour désactiver le mode Angle (Acro par défaut)

THROTTLE_SAFETY_ARM_MAX = THROTTLE_MIN_EFFECTIVE + 50 # Gaz doivent être en dessous pour armer

# État du drone
is_armed_command = False
angle_mode_active = True # Activer le mode Angle par défaut pour la sécurité
current_rc_values[RC_CHAN_AUX2_ANGLE_MODE] = ANGLE_MODE_ON_VALUE if angle_mode_active else ANGLE_MODE_OFF_VALUE

# --- Configuration Verrouillage Yaw ---
yaw_locked = True
YAW_LOCK_VALUE = 1500
current_rc_values[RC_CHAN_YAW] = YAW_LOCK_VALUE

# --- Configuration Manette ---
# ATTENTION: Ces axes peuvent varier selon votre manette.
# Utilisez `pygame.examples.joystick` ou un outil similaire pour les identifier.
AXIS_ROLL = 3       # Joystick Droit X (pour Roll)
AXIS_PITCH = 4      # Joystick Droit Y (pour Pitch) - Inverser si besoin dans map_axis_to_rc
AXIS_YAW = 0        # Joystick Gauche X (pour Yaw)
AXIS_THROTTLE = 1   # Joystick Gauche Y (pour Throttle) - Inverser si besoin

BUTTON_ARM_DISARM = 4      # L1/LB
BUTTON_TOGGLE_ANGLE_MODE = 0 # Souvent Bouton A (Xbox) ou X (Playstation)
BUTTON_TOGGLE_YAW_LOCK = 6 # R1/RB
BUTTON_QUIT = 5            # L2/LT (peut être un axe sur certaines manettes, ajustez)

JOYSTICK_DEADZONE = 0.08

# --- Variables pour MSP et Pygame ---
current_altitude_m = None
last_msp_request_time = 0
MSP_REQUEST_INTERVAL = 0.1 # Demander l'altitude toutes les 100ms

joystick = None
joystick_connected = False

# --- Fonctions MSP ---
def calculate_checksum(payload_bytes):
    chk = 0
    for b_val in payload_bytes:
        chk ^= b_val
    return chk

def send_msp_packet(ser_conn, command, data_payload):
    payload_size = len(data_payload) if data_payload else 0
    header = b'$M<'
    msp_payload_header = struct.pack('<BB', payload_size, command)
    full_msp_payload = msp_payload_header + (data_payload if data_payload else b'')
    checksum_val = calculate_checksum(full_msp_payload)
    packet = header + full_msp_payload + struct.pack('<B', checksum_val)
    try:
        ser_conn.write(packet)
    except Exception as e:
        print(f"Erreur d'écriture série: {e}")

def request_msp_data(ser_conn, command):
    send_msp_packet(ser_conn, command, None)

def parse_msp_response(ser_buffer_local):
    global current_altitude_m
    # Recherche du début d'un message MSP valide
    idx = ser_buffer_local.find(b'$M>')
    if idx == -1:
        return ser_buffer_local # Pas de début de message trouvé

    # Vérifier si on a assez de données pour lire l'entête du payload
    if len(ser_buffer_local) < idx + 5: # $M> + size + cmd
        return ser_buffer_local[idx:] # Garder le début partiel pour la prochaine fois

    payload_size = ser_buffer_local[idx+3]
    cmd = ser_buffer_local[idx+4]

    # Vérifier si on a assez de données pour lire le payload complet et le checksum
    if len(ser_buffer_local) < idx + 5 + payload_size + 1: # + checksum
        return ser_buffer_local[idx:] # Pas assez de données, garder pour la prochaine fois

    # Extraire le payload et le checksum
    full_packet_payload_bytes = ser_buffer_local[idx+3 : idx+5+payload_size] # size, cmd, data
    received_checksum = ser_buffer_local[idx+5+payload_size]
    
    calculated_checksum = calculate_checksum(full_packet_payload_bytes)

    if received_checksum == calculated_checksum:
        payload_data_bytes = ser_buffer_local[idx+5 : idx+5+payload_size] # Uniquement les données
        if cmd == MSP_ALTITUDE and payload_size >= 4:
            # L'altitude est un entier signé de 32 bits en cm
            altitude_cm = struct.unpack('<i', payload_data_bytes[0:4])[0]
            current_altitude_m = float(altitude_cm) / 100.0
        # Consommer le message traité du buffer
        return ser_buffer_local[idx + 5 + payload_size + 1:]
    else:
        # Checksum invalide, jeter le début du message et essayer de resynchroniser
        print("MSP Checksum error")
        return ser_buffer_local[idx+1:]


# --- Logique de Contrôle Manette ---
def map_axis_to_rc(axis_value, min_rc=1000, max_rc=2000, invert_axis=False):
    if abs(axis_value) < JOYSTICK_DEADZONE:
        axis_value = 0.0
    if invert_axis:
        axis_value = -axis_value
    # Normaliser l'axe de -1..1 à 0..1
    normalized_value = (axis_value + 1.0) / 2.0
    return int(min_rc + normalized_value * (max_rc - min_rc))

def handle_joystick_event(event):
    global current_rc_values, is_armed_command, joystick, joystick_connected
    global angle_mode_active, yaw_locked

    if event.type == pygame.JOYAXISMOTION:
        if event.axis == AXIS_THROTTLE:
            # Le throttle est souvent inversé sur les manettes (haut = -1.0)
            current_rc_values[RC_CHAN_THROTTLE] = map_axis_to_rc(event.value, 
                                                                 min_rc=THROTTLE_MIN_EFFECTIVE, 
                                                                 max_rc=THROTTLE_MAX_EFFECTIVE, 
                                                                 invert_axis=True)
        elif event.axis == AXIS_YAW:
            if not yaw_locked:
                current_rc_values[RC_CHAN_YAW] = map_axis_to_rc(event.value)
        elif event.axis == AXIS_ROLL:
            current_rc_values[RC_CHAN_ROLL] = map_axis_to_rc(event.value)
        elif event.axis == AXIS_PITCH:
            # Le pitch peut aussi nécessiter une inversion selon la manette/préférence
            current_rc_values[RC_CHAN_PITCH] = map_axis_to_rc(event.value, invert_axis=False)

    if yaw_locked: # S'assurer que le yaw reste verrouillé si actif
        current_rc_values[RC_CHAN_YAW] = YAW_LOCK_VALUE

    if event.type == pygame.JOYBUTTONDOWN:
        if event.button == BUTTON_ARM_DISARM:
            if not is_armed_command:
                if current_rc_values[RC_CHAN_THROTTLE] <= THROTTLE_SAFETY_ARM_MAX:
                    current_rc_values[RC_CHAN_AUX1_ARM] = ARM_VALUE
                    is_armed_command = True
                    print("\nCOMMANDE: ARMEMENT")
                else:
                    print(f"\nSECURITE: Gaz ({current_rc_values[RC_CHAN_THROTTLE]}) trop hauts pour armer (max {THROTTLE_SAFETY_ARM_MAX}).")
            else:
                current_rc_values[RC_CHAN_AUX1_ARM] = DISARM_VALUE
                is_armed_command = False
                current_rc_values[RC_CHAN_THROTTLE] = THROTTLE_MIN_EFFECTIVE # Couper les gaz au désarmement
                print("\nCOMMANDE: DESARMEMENT")

        elif event.button == BUTTON_TOGGLE_ANGLE_MODE:
            angle_mode_active = not angle_mode_active
            current_rc_values[RC_CHAN_AUX2_ANGLE_MODE] = ANGLE_MODE_ON_VALUE if angle_mode_active else ANGLE_MODE_OFF_VALUE
            print(f"\nINFO: Mode Angle {'ACTIVÉ' if angle_mode_active else 'DÉSACTIVÉ (ACRO)'}")

        elif event.button == BUTTON_TOGGLE_YAW_LOCK:
            yaw_locked = not yaw_locked
            if yaw_locked:
                current_rc_values[RC_CHAN_YAW] = YAW_LOCK_VALUE
                print("\nINFO: Yaw VERROUILLÉ")
            else:
                # Au déverrouillage, le yaw prendra la valeur du stick au prochain JOYAXISMOTION
                print("\nINFO: Yaw DÉVERROUILLÉ")

        elif event.button == BUTTON_QUIT:
            return "quit"

    elif event.type == pygame.JOYDEVICEADDED:
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            joystick_connected = True
            print(f"\nManette '{joystick.get_name()}' connectée.")
    elif event.type == pygame.JOYDEVICEREMOVED:
        joystick_connected = False
        joystick = None
        print("\nManette déconnectée. Désarmement et gaz au minimum.")
        current_rc_values[RC_CHAN_THROTTLE] = THROTTLE_MIN_EFFECTIVE
        current_rc_values[RC_CHAN_AUX1_ARM] = DISARM_VALUE
        is_armed_command = False
        # Optionnel: remettre le yaw lock et angle mode par défaut
        yaw_locked = True
        current_rc_values[RC_CHAN_YAW] = YAW_LOCK_VALUE
        angle_mode_active = True
        current_rc_values[RC_CHAN_AUX2_ANGLE_MODE] = ANGLE_MODE_ON_VALUE

    return None

def print_status():
    # Efface la ligne précédente pour une impression propre
    sys.stdout.write("\033[K")
    
    alt_str = f"{current_altitude_m:.2f}m" if current_altitude_m is not None else "N/A"
    arm_str = "ARMED" if is_armed_command else "DISARMED"
    angle_str = "ANGLE" if angle_mode_active else "ACRO"
    yaw_l_str = "LOCKED" if yaw_locked else "UNLOCKED"
    thr_mode_str = "TEST" if ENABLE_THROTTLE_TEST_LIMIT else "FULL"

    status_line = (
        f"T:{current_rc_values[RC_CHAN_THROTTLE]}({thr_mode_str}) "
        f"R:{current_rc_values[RC_CHAN_ROLL]} P:{current_rc_values[RC_CHAN_PITCH]} Y:{current_rc_values[RC_CHAN_YAW]}({yaw_l_str}) | "
        f"{arm_str} (AUX1:{current_rc_values[RC_CHAN_AUX1_ARM]}) | "
        f"{angle_str} (AUX2:{current_rc_values[RC_CHAN_AUX2_ANGLE_MODE]}) | "
        f"Alt:{alt_str}"
    )
    print(status_line, end="\r")
    sys.stdout.flush()

def main():
    global current_rc_values, is_armed_command, joystick, joystick_connected
    global last_msp_request_time, current_altitude_m, ser_buffer
    global angle_mode_active, yaw_locked # Assurer qu'elles sont globales

    print("--- Script Contrôle Drone MSP Simplifié (Angle Mode + Yaw Lock) ---")
    if ENABLE_THROTTLE_TEST_LIMIT:
        print(f"!!! MODE TEST THROTTLE ACTIF: {THROTTLE_MIN_EFFECTIVE}-{THROTTLE_MAX_EFFECTIVE} !!!")
    else:
        print(f"!!! MODE PLEINE POUSSÉE ACTIF: {THROTTLE_MIN_EFFECTIVE}-{THROTTLE_MAX_EFFECTIVE} !!!")
    print(f"Vérifiez vos assignations de boutons/axes de manette!")
    print(f"Port série: {SERIAL_PORT} à {BAUD_RATE} bauds.")
    print("Appuyez sur le bouton 'QUIT' pour arrêter.")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        joystick_connected = True
        print(f"Manette '{joystick.get_name()}' connectée.")
    else:
        print("Aucune manette détectée. Le script ne pourra pas envoyer de commandes RC.")
        # On pourrait quitter ici, mais laissons-le tourner pour voir les messages MSP si le port série s'ouvre.

    ser_buffer = b''
    ser = None # Initialiser à None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) # timeout non bloquant
        print(f"Port série {SERIAL_PORT} ouvert.")
    except serial.SerialException as e:
        print(f"Erreur à l'ouverture du port série {SERIAL_PORT}: {e}")
        print("Le script va continuer sans communication série (pour démo manette).")
        # pygame.quit() # Décommentez si vous voulez quitter si le port série échoue
        # return

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: # Permet de fermer la fenêtre Pygame si elle existe
                    running = False; break
                if joystick_connected: # Traiter les événements de la manette seulement si elle est connectée
                    if handle_joystick_event(event) == "quit":
                        running = False; break
            if not running:
                break

            current_time = time.time()
            if ser and ser.is_open and (current_time - last_msp_request_time > MSP_REQUEST_INTERVAL):
                request_msp_data(ser, MSP_ALTITUDE)
                last_msp_request_time = current_time
            
            if ser and ser.is_open and ser.in_waiting > 0:
                ser_buffer += ser.read(ser.in_waiting)
            
            if ser_buffer: # Ne parser que s'il y a des données
                 ser_buffer = parse_msp_response(ser_buffer)


            # S'assurer que les valeurs RC sont dans les limites 1000-2000
            # Le throttle est déjà géré par map_axis_to_rc avec THROTTLE_MIN/MAX_EFFECTIVE
            current_rc_values[RC_CHAN_THROTTLE] = max(THROTTLE_MIN_VALUE, min(THROTTLE_MAX_VALUE, current_rc_values[RC_CHAN_THROTTLE]))
            for i in [RC_CHAN_ROLL, RC_CHAN_PITCH, RC_CHAN_YAW, RC_CHAN_AUX1_ARM, RC_CHAN_AUX2_ANGLE_MODE]:
                 current_rc_values[i] = max(1000, min(2000, current_rc_values[i]))
            # Les autres canaux AUX restent à 1500 par défaut

            if joystick_connected: # N'envoyer des commandes que si la manette est là
                if ser and ser.is_open:
                    payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                    send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                print_status()
            else:
                # Si la manette n'est pas connectée, afficher un message et ne pas envoyer de commandes RC
                # Les valeurs RC de sécurité (désarmé, gaz bas) sont définies dans JOYDEVICEREMOVED
                sys.stdout.write("\033[K") # Efface la ligne
                print("Manette déconnectée... Attente de reconnexion.", end="\r")
                sys.stdout.flush()
                # S'assurer que le drone est désarmé si la manette se déconnecte en vol
                if is_armed_command:
                    print("\nURGENCE: Manette déconnectée en vol! Tentative de désarmement.")
                    current_rc_values[RC_CHAN_THROTTLE] = THROTTLE_MIN_EFFECTIVE
                    current_rc_values[RC_CHAN_AUX1_ARM] = DISARM_VALUE
                    is_armed_command = False
                    if ser and ser.is_open:
                        payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                        send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc) # Envoyer une dernière commande de désarmement


            time.sleep(0.02) # Boucle de contrôle à environ 50Hz

    except KeyboardInterrupt:
        print("\nArrêt demandé par l'utilisateur (Ctrl+C).")
    except Exception as e:
        print(f"\nErreur inattendue dans la boucle principale: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nNettoyage et commandes de sécurité finales...")
        # Préparer un paquet de désarmement final
        final_rc_values = [1500] * RC_CHANNELS_COUNT
        final_rc_values[RC_CHAN_THROTTLE] = THROTTLE_MIN_EFFECTIVE
        final_rc_values[RC_CHAN_AUX1_ARM] = DISARM_VALUE # Désarmé
        final_rc_values[RC_CHAN_YAW] = YAW_LOCK_VALUE # Yaw neutre/verrouillé
        final_rc_values[RC_CHAN_AUX2_ANGLE_MODE] = ANGLE_MODE_ON_VALUE # Angle mode ON pour sécurité
        
        payload_final = b''.join(struct.pack('<H', int(v)) for v in final_rc_values[:RC_CHANNELS_COUNT])

        if ser and ser.is_open:
            print("Envoi des commandes de désarmement finales...")
            for _ in range(5): # Envoyer plusieurs fois pour s'assurer de la réception
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_final)
                time.sleep(0.02)
            ser.close()
            print("Port série fermé.")
        
        pygame.quit()
        print("Pygame quitté. Script terminé.")

if __name__ == "__main__":
    main()