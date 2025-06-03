import serial
import struct
import time
import sys
import pygame

# --- Configuration Globale ---
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200

# --- Configuration du Contrôle ---
RC_CHANNELS_COUNT = 8
THROTTLE_CHANNEL_INDEX = 2
ARM_CHANNEL_INDEX = 4

THROTTLE_MIN = 1000
THROTTLE_MAX = 1300

ARM_VALUE = 1800
DISARM_VALUE = 1000
THROTTLE_SAFETY_ARM_MAX = 1050 # Doit être >= THROTTLE_MIN

# --- Configuration Manette ---
AXIS_THROTTLE = 1
BUTTON_ARM_DISARM = 4
BUTTON_QUIT = 6
JOYSTICK_DEADZONE = 0.08

# --- Variables Globales ---
current_rc_values = [1500] * RC_CHANNELS_COUNT
current_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN
current_rc_values[ARM_CHANNEL_INDEX] = DISARM_VALUE

is_armed = False
joystick = None
joystick_connected = False

# --- Fonctions MSP ---
def calculate_checksum(payload):
    chk = 0
    for b in payload:
        chk ^= b
    return chk

def send_msp_packet(ser, command, data):
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
def map_axis_to_rc_throttle(axis_value_raw, min_rc, max_rc, inverted=True):
    """
    Mappe la valeur d'un axe de joystick pour la poussée (throttle).
    Comportement spécifique pour la poussée :
    - Joystick au centre (valeur d'axe ~0.0) ou en position basse : donne min_rc (1000).
    - Joystick en position haute : donne max_rc (1300).
    - Seule la moitié supérieure de la course du joystick (du centre vers le haut)
      module la poussée de min_rc à max_rc.
    """
    axis_val = axis_value_raw
    if abs(axis_val) < JOYSTICK_DEADZONE:
        axis_val = 0.0

    if inverted:
        # Pour un stick Y standard: -1 (haut), 0 (centre), +1 (bas)
        # Après inversion: axis_val devient +1 (haut), 0 (centre), -1 (bas)
        axis_val = -axis_val

    # À ce stade, après inversion (si applicable):
    # Stick physique HAUT   => axis_val est proche de +1.0
    # Stick physique CENTRE => axis_val est proche de  0.0
    # Stick physique BAS    => axis_val est proche de -1.0

    if axis_val <= 0:  # Joystick au centre (0.0) ou en position basse (valeurs négatives)
        return min_rc
    else:  # Joystick en position haute (axis_val est entre 0.0 et +1.0)
        # Mapper la plage [0.0, 1.0] de axis_val vers [min_rc, max_rc]
        # La proportion de la course active (moitié supérieure) est axis_val (qui va de 0 à 1 ici)
        # L'augmentation de poussée par rapport à min_rc est axis_val * (max_rc - min_rc)
        rc_value = int(min_rc + axis_val * (max_rc - min_rc))
        # S'assurer que la valeur reste dans les bornes définies (devrait déjà l'être si axis_val <= 1.0)
        return max(min_rc, min(max_rc, rc_value))

def handle_joystick_events():
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
                # Utiliser la fonction de mappage spécifique pour la poussée
                throttle_value = map_axis_to_rc_throttle(event.value, THROTTLE_MIN, THROTTLE_MAX, inverted=True)
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
                    current_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN
                    is_armed = False
                    print("\nCOMMANDE: DÉSARMEMENT")
                action_taken = True
            
            elif event.button == BUTTON_QUIT:
                print("\nBouton Quitter pressé.")
                return "quit"
    
    if action_taken: # Afficher le statut seulement si une action pertinente a eu lieu
        print_status()
    return None

def print_status():
    arm_status_str = "ARMÉ" if is_armed else "DÉSARMÉ"
    status_line = (
        f"Poussée: {current_rc_values[THROTTLE_CHANNEL_INDEX]:04d} | "
        f"Armement (AUX1): {current_rc_values[ARM_CHANNEL_INDEX]:04d} ({arm_status_str}) | "
        f"Manette: {'OK' if joystick_connected else 'NON CONNECTÉE'}"
    )
    sys.stdout.write("\r" + status_line + "   ")
    sys.stdout.flush()

def main():
    global joystick, joystick_connected, current_rc_values, is_armed

    print("--- Script Contrôle Poussée Drone MSP (Simplifié & Corrigé) ---")
    print(f"Contrôle de poussée: {THROTTLE_MIN} (centre/bas joystick) - {THROTTLE_MAX} (haut joystick)")
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
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"Port série {SERIAL_PORT} ouvert.")
    except serial.SerialException as e:
        print(f"Erreur ouverture port série {SERIAL_PORT}: {e}", file=sys.stderr)
        pygame.quit()
        return

    running = True
    last_send_time = time.time()

    try:
        while running:
            if handle_joystick_events() == "quit":
                running = False
                break

            current_time = time.time()
            if current_time - last_send_time >= 0.02: # 50Hz
                current_rc_values[0] = 1500 # Roll
                current_rc_values[1] = 1500 # Pitch
                current_rc_values[3] = 1500 # Yaw
                for i in range(5, RC_CHANNELS_COUNT):
                    current_rc_values[i] = 1500

                payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                last_send_time = current_time
                
                if joystick_connected: # Afficher le statut en continu si la manette est là
                    print_status()
                elif not joystick_connected and is_armed:
                    print("\nManette déconnectée en vol! Désarmement d'urgence!")
                    current_rc_values[ARM_CHANNEL_INDEX] = DISARM_VALUE
                    current_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN
                    is_armed = False
                    payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                    send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                    print_status() # Mettre à jour le statut affiché

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nArrêt par Ctrl+C.")
    except Exception as e:
        print(f"\nErreur inattendue: {e}", file=sys.stderr)
    finally:
        print("\nNettoyage et commandes de sécurité finales...")
        if ser and ser.is_open:
            final_rc_values = [1500] * RC_CHANNELS_COUNT
            final_rc_values[THROTTLE_CHANNEL_INDEX] = THROTTLE_MIN
            final_rc_values[ARM_CHANNEL_INDEX] = DISARM_VALUE
            payload_final = b''.join(struct.pack('<H', int(v)) for v in final_rc_values)
            print("Envoi des commandes de désarmement finales...")
            for _ in range(10):
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_final)
                time.sleep(0.02)
            ser.close()
            print("Port série fermé.")
        pygame.quit()
        print("Pygame quitté. Script terminé.")

if __name__ == "__main__":
    main()