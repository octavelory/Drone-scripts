import serial
import struct
import time
import sys
import pygame

# --- Configuration Globale ---
SERIAL_PORT = '/dev/ttyAMA0'  # Adaptez si nécessaire pour RPi 5 (peut être ttyS0 ou ttyAMA0)
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200

# --- Configuration du Contrôle ---
# Ordre standard des canaux : Roll, Pitch, Throttle, Yaw, AUX1, AUX2, ...
# Indices (commençant à 0):   0,    1,     2,        3,   4,    5, ...
RC_CHANNELS_COUNT = 8  # Nombre total de canaux à envoyer (même si non tous utilisés)
THROTTLE_CHANNEL_INDEX = 2
ARM_CHANNEL_INDEX = 4  # AUX1 pour l'armement

# Valeurs RC pour la poussée
THROTTLE_MIN = 1000  # Poussée minimale / neutre
THROTTLE_MAX = 1300  # Poussée maximale verticale

# Valeurs RC pour l'armement
ARM_VALUE = 1800
DISARM_VALUE = 1000
# Le drone ne s'armera que si la poussée est en dessous de cette valeur
THROTTLE_SAFETY_ARM_MAX = 1050 # Un peu au-dessus de THROTTLE_MIN pour la sécurité

# --- Configuration Manette ---
# Pour la plupart des manettes, l'axe Y du joystick gauche est l'axe 1
AXIS_THROTTLE = 1   # Joystick Gauche Y (pour Throttle)
# Choisissez un bouton pour armer/désarmer, par exemple L1/LB
BUTTON_ARM_DISARM = 4 # Souvent L1/LB sur manettes type PS/Xbox
# Choisissez un bouton pour quitter proprement
BUTTON_QUIT = 6       # Souvent R1/RB ou un autre bouton facilement accessible

JOYSTICK_DEADZONE = 0.08 # Zone morte pour éviter les dérives du joystick

# --- Variables Globales ---
current_rc_values = [1500] * RC_CHANNELS_COUNT # Initialise tous les canaux à neutre (1500)
current_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN # Poussée au minimum au démarrage
current_rc_values[ARM_CHANNEL_INDEX] = DISARM_VALUE     # Désarmé au démarrage

is_armed = False
joystick = None
joystick_connected = False

# --- Fonctions MSP ---
def calculate_checksum(payload):
    """Calcule le checksum pour un payload MSP."""
    chk = 0
    for b in payload:
        chk ^= b
    return chk

def send_msp_packet(ser, command, data):
    """Envoie un paquet MSP au FC."""
    if not ser or not ser.is_open:
        print("Erreur: Port série non ouvert.", file=sys.stderr)
        return

    data_size = len(data) if data else 0
    header = b'$M<'
    payload_header = struct.pack('<BB', data_size, command)
    full_payload = payload_header + (data if data else b'')
    checksum_val = calculate_checksum(full_payload)
    packet = header + full_payload + struct.pack('<B', checksum_val)
    try:
        ser.write(packet)
    except Exception as e:
        print(f"Erreur d'écriture série: {e}", file=sys.stderr)

# --- Logique de Contrôle Manette ---
def map_axis_to_rc(axis_value, min_rc, max_rc, inverted=False):
    """Mappe la valeur d'un axe de joystick (-1 à 1) à une plage RC."""
    if abs(axis_value) < JOYSTICK_DEADZONE:
        axis_value = 0.0
    if inverted:
        axis_value = -axis_value # Inverser si nécessaire (souvent pour la poussée)

    # Normaliser la valeur de l'axe de -1..1 à 0..1
    normalized_value = (axis_value + 1.0) / 2.0
    rc_value = int(min_rc + normalized_value * (max_rc - min_rc))
    return max(min_rc, min(max_rc, rc_value)) # S'assurer que la valeur reste dans les bornes

def handle_joystick_events():
    """Gère les événements du joystick."""
    global current_rc_values, is_armed, joystick, joystick_connected

    action_taken = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return "quit"

        if event.type == pygame.JOYDEVICEADDED:
            if pygame.joystick.get_count() > 0 and not joystick_connected:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                joystick_connected = True
                print(f"\nManette '{joystick.get_name()}' connectée.")
                action_taken = True

        elif event.type == pygame.JOYDEVICEREMOVED:
            if joystick_connected:
                print("\nManette déconnectée.")
                joystick_connected = False
                joystick = None
                # Sécurité : désarmer si la manette est déconnectée
                if is_armed:
                    print("Désarmement automatique (manette déconnectée).")
                    current_rc_values[ARM_CHANNEL_INDEX] = DISARM_VALUE
                    current_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN
                    is_armed = False
                action_taken = True

        if not joystick_connected:
            continue

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == AXIS_THROTTLE:
                # L'axe Y du joystick est souvent -1 (haut) à 1 (bas).
                # On veut que "haut" = plus de poussée, donc on inverse.
                throttle_value = map_axis_to_rc(event.value, THROTTLE_MIN, THROTTLE_MAX, inverted=True)
                current_rc_values[THROTTLE_CHANNEL_INDEX] = throttle_value
                action_taken = True

        elif event.type == pygame.JOYBUTTONDOWN:
            if event.button == BUTTON_ARM_DISARM:
                if not is_armed:
                    if current_rc_values[THROTTLE_CHANNEL_INDEX] <= THROTTLE_SAFETY_ARM_MAX:
                        current_rc_values[ARM_CHANNEL_INDEX] = ARM_VALUE
                        is_armed = True
                        print("\nCOMMANDE: ARMEMENT")
                    else:
                        print(f"\nSÉCURITÉ: Poussée ({current_rc_values[THROTTLE_CHANNEL_INDEX]}) trop haute pour armer (max {THROTTLE_SAFETY_ARM_MAX}).")
                else:
                    current_rc_values[ARM_CHANNEL_INDEX] = DISARM_VALUE
                    current_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN # Poussée à zéro au désarmement
                    is_armed = False
                    print("\nCOMMANDE: DÉSARMEMENT")
                action_taken = True
            
            elif event.button == BUTTON_QUIT:
                print("\nBouton Quitter pressé.")
                return "quit"
    
    if action_taken:
        print_status()
    return None

def print_status():
    """Affiche l'état actuel des commandes."""
    arm_status_str = "ARMÉ" if is_armed else "DÉSARMÉ"
    status_line = (
        f"Poussée: {current_rc_values[THROTTLE_CHANNEL_INDEX]:04d} | "
        f"Armement (AUX1): {current_rc_values[ARM_CHANNEL_INDEX]:04d} ({arm_status_str}) | "
        f"Manette: {'OK' if joystick_connected else 'NON CONNECTÉE'}"
    )
    sys.stdout.write("\r" + status_line + "   ") # Espaces pour effacer la fin de la ligne précédente
    sys.stdout.flush()

def main():
    global joystick, joystick_connected, current_rc_values, is_armed

    print("--- Script Contrôle Poussée Drone MSP (Simplifié) ---")
    print(f"Contrôle de poussée: {THROTTLE_MIN} - {THROTTLE_MAX}")
    print(f"Port série: {SERIAL_PORT} @ {BAUD_RATE}bps")
    print(f"Joystick: Axe {AXIS_THROTTLE} pour Poussée, Bouton {BUTTON_ARM_DISARM} pour Armer/Désarmer, Bouton {BUTTON_QUIT} pour Quitter.")
    print("ATTENTION: Ce script contrôle directement les moteurs. Soyez prudent !")
    print("Assurez-vous que les hélices sont retirées lors des premiers tests.")

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        joystick_connected = True
        print(f"Manette '{joystick.get_name()}' détectée au démarrage.")
    else:
        print("Aucune manette détectée au démarrage. Connectez une manette.")

    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) # timeout court pour ne pas bloquer
        print(f"Port série {SERIAL_PORT} ouvert.")
    except serial.SerialException as e:
        print(f"Erreur ouverture port série {SERIAL_PORT}: {e}", file=sys.stderr)
        print("Vérifiez que le port est correct et que vous avez les permissions (sudo?).", file=sys.stderr)
        print("Sur Raspberry Pi, désactivez la console série si elle utilise ttyAMA0/ttyS0.", file=sys.stderr)
        pygame.quit()
        return

    running = True
    last_send_time = time.time()

    try:
        while running:
            if handle_joystick_events() == "quit":
                running = False
                break

            # Envoyer les commandes RC à une fréquence régulière (ex: 50Hz)
            current_time = time.time()
            if current_time - last_send_time >= 0.02: # 50Hz
                # S'assurer que les autres canaux restent neutres ou à leur valeur par défaut
                # Roll, Pitch, Yaw à 1500 (neutre)
                current_rc_values[0] = 1500 # Roll
                current_rc_values[1] = 1500 # Pitch
                current_rc_values[3] = 1500 # Yaw
                # Les autres AUX peuvent rester à 1500 ou une valeur par défaut
                for i in range(5, RC_CHANNELS_COUNT):
                    current_rc_values[i] = 1500

                # Préparer et envoyer le paquet MSP
                payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                last_send_time = current_time
                
                # Afficher le statut uniquement si la manette est connectée pour éviter le spam
                if joystick_connected:
                    print_status()
                elif not joystick_connected and is_armed: # Sécurité si la manette se déconnecte en vol
                    print("\nManette déconnectée en vol! Désarmement d'urgence!")
                    current_rc_values[ARM_CHANNEL_INDEX] = DISARM_VALUE
                    current_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN
                    is_armed = False
                    # Envoyer immédiatement la commande de désarmement
                    payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                    send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                    print_status()

            time.sleep(0.005) # Petite pause pour ne pas surcharger le CPU

    except KeyboardInterrupt:
        print("\nArrêt par Ctrl+C.")
    except Exception as e:
        print(f"\nErreur inattendue: {e}", file=sys.stderr)
    finally:
        print("\nNettoyage et commandes de sécurité finales...")
        if ser and ser.is_open:
            # Envoyer une commande de désarmement et poussée minimale plusieurs fois
            final_rc_values = [1500] * RC_CHANNELS_COUNT
            final_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN
            final_rc_values[ARM_CHANNEL_INDEX] = DISARM_VALUE
            payload_final = b''.join(struct.pack('<H', int(v)) for v in final_rc_values)
            print("Envoi des commandes de désarmement finales...")
            for _ in range(10): # Envoyer plusieurs fois pour s'assurer de la réception
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_final)
                time.sleep(0.02)
            ser.close()
            print("Port série fermé.")
        pygame.quit()
        print("Pygame quitté. Script terminé.")

if __name__ == "__main__":
    main()