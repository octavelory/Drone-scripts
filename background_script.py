import serial
import struct
import time
import sys
import os

# --- Configuration et Constantes MSP ---
SERIAL_PORT = '/dev/ttyAMA0'  # Ou /dev/ttyUSB0 si vous utilisez un adaptateur USB-Série
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200
MSP_RC_TUNING = 111 # Pour lire la configuration des canaux (pas utilisé activement ici mais utile pour debug)
MSP_STATUS_EX = 150 # Pour lire l'état d'armement du FC (plus fiable)

# --- Configuration du Contrôle ---
# Ordre des canaux RC (Ajustez selon votre configuration FC: AETR1234 est courant)
# Index:    0     1        2          3       4      5      6      7
#           Roll, Pitch,   Throttle,  Yaw,    AUX1,  AUX2,  AUX3,  AUX4
# Valeurs: 1000 (min) - 1500 (milieu) - 2000 (max)
RC_CHANNELS_COUNT = 8 # Envoyer 8 canaux par défaut
current_rc_values = [1500] * RC_CHANNELS_COUNT
current_rc_values[2] = 1000  # Throttle bas au démarrage
current_rc_values[4] = 1000  # AUX1 (Arm switch) désarmé

# Valeurs pour l'armement/désarmement sur AUX1 (canal 5)
ARM_VALUE = 1800
DISARM_VALUE = 1000
THROTTLE_MIN = 1000
THROTTLE_MAX = 2000
THROTTLE_SAFETY_ARM = 1100 # Le throttle doit être en dessous pour pouvoir armer

# Incréments pour le contrôle
STEP_THROTTLE = 25
STEP_RPY = 75 # Roll, Pitch, Yaw step
CENTER_RPY_SPEED = 50 # Vitesse à laquelle roll/pitch/yaw retournent au centre

# État du drone
is_armed_command = False # Commande d'armement envoyée par le script
fc_is_armed = False # État d'armement réel lu depuis le FC (pas implémenté dans cette version pour la lecture)

# --- Configuration du Terminal pour lecture non-bloquante (Linux/macOS) ---
fd = sys.stdin.fileno()
# Sauvegarder les anciens paramètres du terminal
try:
    import termios
    import tty
    old_settings = termios.tcgetattr(fd)
except ImportError:
    print("ERREUR: termios/tty non disponible. Ce script est conçu pour Linux/macOS.")
    print("Sur Windows, vous pourriez utiliser msvcrt.getch().")
    sys.exit(1)

def setup_terminal_nonblocking():
    try:
        tty.setraw(sys.stdin.fileno())
        # Rendre sys.stdin non bloquant
        import fcntl
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    except Exception as e:
        print(f"Erreur de configuration du terminal: {e}")
        restore_terminal()
        sys.exit(1)

def restore_terminal():
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    # Restaurer le mode bloquant (optionnel, car old_settings devrait le faire)
    import fcntl
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl & ~os.O_NONBLOCK)
    print("\nTerminal restauré.")

def get_key_non_blocking():
    try:
        return sys.stdin.read(1)
    except (IOError, BlockingIOError): # BlockingIOError si O_NONBLOCK est actif et rien à lire
        return None
    except Exception: # Autres erreurs potentielles
        return None

# --- Fonctions MSP ---
def calculate_checksum(payload):
    chk = 0
    for b in payload:
        chk ^= b
    return chk

def send_msp_packet(ser, command, data):
    if data:
        data_size = len(data)
    else:
        data_size = 0

    header = b'$M<'
    payload_header = struct.pack('<BB', data_size, command)
    full_payload = payload_header
    if data:
        full_payload += data

    checksum = calculate_checksum(full_payload)
    packet = header + full_payload + struct.pack('<B', checksum)
    try:
        ser.write(packet)
        # print(f"Sent: {packet.hex()}") # Debug
    except Exception as e:
        print(f"Erreur d'écriture série: {e}")

# --- Logique de Contrôle ---
def update_rc_values(key):
    global current_rc_values, is_armed_command

    # --- Contrôles de vol ---
    # Throttle
    if key == 'w':  # Augmenter Throttle
        if is_armed_command: # On ne peut augmenter les gaz que si armé
            current_rc_values[2] = min(THROTTLE_MAX, current_rc_values[2] + STEP_THROTTLE)
    elif key == 's':  # Diminuer Throttle
        current_rc_values[2] = max(THROTTLE_MIN, current_rc_values[2] - STEP_THROTTLE)

    # Pitch (Elevator)
    elif key == 'i':  # Pitch avant
        current_rc_values[1] = max(1000, current_rc_values[1] - STEP_RPY)
    elif key == 'k':  # Pitch arrière
        current_rc_values[1] = min(2000, current_rc_values[1] + STEP_RPY)

    # Roll (Aileron)
    elif key == 'j':  # Roll gauche
        current_rc_values[0] = max(1000, current_rc_values[0] - STEP_RPY)
    elif key == 'l':  # Roll droite
        current_rc_values[0] = min(2000, current_rc_values[0] + STEP_RPY)

    # Yaw (Rudder)
    elif key == 'a':  # Yaw gauche
        current_rc_values[3] = max(1000, current_rc_values[3] - STEP_RPY)
    elif key == 'd':  # Yaw droite
        current_rc_values[3] = min(2000, current_rc_values[3] + STEP_RPY)

    # --- Commandes Système ---
    elif key == ' ':  # Espace pour Armer/Désarmer
        if not is_armed_command:
            if current_rc_values[2] <= THROTTLE_SAFETY_ARM:
                current_rc_values[4] = ARM_VALUE  # Arm
                is_armed_command = True
                print("COMMANDE: ARMEMENT")
            else:
                print(f"SECURITE: Gaz trop hauts ({current_rc_values[2]}) pour armer. Baissez les gaz en dessous de {THROTTLE_SAFETY_ARM}.")
        else:
            current_rc_values[4] = DISARM_VALUE  # Disarm
            current_rc_values[2] = THROTTLE_MIN # Couper les gaz au désarmement
            is_armed_command = False
            print("COMMANDE: DESARMEMENT")

    elif key == 'c': # Centrer Roll/Pitch/Yaw
        current_rc_values[0] = 1500 # Roll
        current_rc_values[1] = 1500 # Pitch
        current_rc_values[3] = 1500 # Yaw
        print("COMMANDE: CENTRAGE RPY")

    # S'assurer que les valeurs restent dans les limites
    for i in range(RC_CHANNELS_COUNT):
        current_rc_values[i] = max(1000, min(2000, current_rc_values[i]))

def auto_center_rpy():
    """ Ramène progressivement Roll, Pitch, Yaw au centre si aucune touche n'est active pour eux. """
    # Pour une version simple, on peut juste les remettre à 1500 si aucune touche RPY n'est pressée.
    # Une version plus avancée impliquerait de savoir si une touche RPY *vient* d'être relâchée.
    # Ici, on va supposer que si update_rc_values n'a pas modifié RPY, on les centre.
    # Cette fonction est appelée si AUCUNE touche n'est pressée.
    if current_rc_values[0] > 1500: current_rc_values[0] = max(1500, current_rc_values[0] - CENTER_RPY_SPEED)
    elif current_rc_values[0] < 1500: current_rc_values[0] = min(1500, current_rc_values[0] + CENTER_RPY_SPEED)

    if current_rc_values[1] > 1500: current_rc_values[1] = max(1500, current_rc_values[1] - CENTER_RPY_SPEED)
    elif current_rc_values[1] < 1500: current_rc_values[1] = min(1500, current_rc_values[1] + CENTER_RPY_SPEED)

    if current_rc_values[3] > 1500: current_rc_values[3] = max(1500, current_rc_values[3] - CENTER_RPY_SPEED)
    elif current_rc_values[3] < 1500: current_rc_values[3] = min(1500, current_rc_values[3] + CENTER_RPY_SPEED)


def print_status():
    # Effacer la ligne précédente (ou plusieurs lignes) pour un affichage propre
    sys.stdout.write("\033[K") # Efface la ligne courante
    # sys.stdout.write("\033[F" * N) # Remonte N lignes puis efface
    
    status_str = (
        f"Roll: {current_rc_values[0]:4} Pitch: {current_rc_values[1]:4} "
        f"Throttle: {current_rc_values[2]:4} Yaw: {current_rc_values[3]:4} | "
        f"AUX1(Arm): {current_rc_values[4]:4} | Armed: {'YES' if is_armed_command else 'NO '}"
    )
    print(status_str, end="\r") # \r pour retour chariot, écrase la ligne
    sys.stdout.flush()


def main():
    global current_rc_values, is_armed_command
    print("--- Script de contrôle drone via MSP (Clavier) ---")
    print("ATTENTION: COMMENCEZ TOUJOURS SANS HÉLICES !!!")
    print("Commandes:")
    print("  w/s: Throttle haut/bas")
    print("  i/k: Pitch avant/arrière")
    print("  j/l: Roll gauche/droite")
    print("  a/d: Yaw gauche/droite")
    print("  ESPACE: Armer / Désarmer (gaz bas pour armer)")
    print("  c: Centrer Roll/Pitch/Yaw")
    print("  q: Quitter (Kill Switch - désarme et coupe les gaz)")
    print("----------------------------------------------------")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05) # timeout plus court
        print(f"Port série {SERIAL_PORT} ouvert.")
    except serial.SerialException as e:
        print(f"Erreur: Impossible d'ouvrir le port série {SERIAL_PORT}. {e}")
        return

    setup_terminal_nonblocking()
    last_key_time = time.time()

    try:
        while True:
            key = get_key_non_blocking()

            if key:
                last_key_time = time.time()
                if key == 'q':  # Kill switch / Quitter
                    print("\nKILL SWITCH / QUITTER")
                    is_armed_command = False # Forcer le désarmement logique
                    current_rc_values[2] = THROTTLE_MIN # Throttle min
                    current_rc_values[4] = DISARM_VALUE # Commande de désarmement
                    # Envoyer plusieurs fois pour s'assurer que le FC reçoit
                    for _ in range(5):
                        payload_rc = b''.join(struct.pack('<H', int(val)) for val in current_rc_values[:RC_CHANNELS_COUNT])
                        send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
                        time.sleep(0.02)
                    break
                update_rc_values(key)
            else:
                # Si aucune touche n'est pressée pendant un court instant, centrer RPY
                if time.time() - last_key_time > 0.1: # Délai avant de commencer à centrer
                     auto_center_rpy()


            # Préparer et envoyer le paquet MSP_SET_RAW_RC
            # S'assurer d'envoyer le bon nombre de canaux, complétant avec 1500 si nécessaire
            rc_to_send = list(current_rc_values)
            while len(rc_to_send) < RC_CHANNELS_COUNT:
                rc_to_send.append(1500)
            
            payload_rc = b''
            for channel_val in rc_to_send[:RC_CHANNELS_COUNT]: # Envoyer seulement les N premiers canaux
                payload_rc += struct.pack('<H', int(channel_val))

            send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc)
            print_status()

            # Lire les données du FC (optionnel, pour lire l'état d'armement par exemple)
            # response = ser.read(100) # Lire ce qui est disponible
            # if response:
            #    pass # Analyser la réponse MSP ici (plus complexe)

            time.sleep(0.02)  # Boucle de contrôle à ~50Hz (MSP s'attend à des mises à jour régulières)

    except KeyboardInterrupt:
        print("\nArrêt du script (Ctrl+C).")
    except Exception as e:
        print(f"\nUne erreur est survenue: {e}")
    finally:
        restore_terminal()
        print("Envoi des commandes de sécurité finales...")
        # S'assurer que le drone est désarmé et throttle bas
        final_rc_values = [1500] * RC_CHANNELS_COUNT
        final_rc_values[2] = THROTTLE_MIN
        final_rc_values[4] = DISARM_VALUE

        payload_rc_final = b''.join(struct.pack('<H', int(val)) for val in final_rc_values[:RC_CHANNELS_COUNT])

        if 'ser' in locals() and ser.is_open:
            for _ in range(5): # Envoyer plusieurs fois
                send_msp_packet(ser, MSP_SET_RAW_RC, payload_rc_final)
                time.sleep(0.02)
            print("Fermeture du port série.")
            ser.close()
        print("Script terminé.")

if __name__ == "__main__":
    main()