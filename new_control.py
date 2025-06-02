import pygame
import serial
import struct
import time
import sys

# --- Configuration ---
# ATTENTION: Adaptez SERIAL_PORT au port série de votre Raspberry Pi connecté au FC
# Exemples: '/dev/ttyS0', '/dev/serial0', '/dev/ttyAMA0', '/dev/ttyUSB0'
# Vérifiez quel UART de votre FC (UART1 ou UART5 selon votre dump) est connecté au RPi
SERIAL_PORT = '/dev/serial0'  # À MODIFIER SI NÉCESSAIRE
BAUD_RATE = 115200

# Valeurs RC (basées sur votre dump)
RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000
THROTTLE_MIN_ARMED = 1070 # min_throttle de votre dump
THROTTLE_MAX_TEST_MODE = 1500

# Canaux RC (0-indexed) selon map AETR1234
# Roll, Pitch, Throttle, Yaw, Aux1, Aux2, Aux3, Aux4 ...
# Betaflight attend 8 canaux minimum pour MSP_SET_RAW_RC, peut en gérer jusqu'à 18
NUM_RC_CHANNELS = 8 # Nous enverrons 8 canaux

# Mapping des canaux Betaflight (AETR1234)
# Betaflight Channel Order (0-indexed for our array)
CH_ROLL = 0
CH_PITCH = 1
CH_THROTTLE = 2
CH_YAW = 3
CH_AUX1 = 4 # Arm/Disarm
CH_AUX2 = 5 # Mode (non utilisé activement ici, mis à MID)
CH_AUX3 = 6 # (non utilisé activement ici, mis à MID)
CH_AUX4 = 7 # (non utilisé activement ici, mis à MID)

# Valeurs pour AUX1 (Arm/Disarm)
AUX_ARM_VALUE = 1800  # Doit être dans la plage 1525-2100 de votre dump
AUX_DISARM_VALUE = 1000 # Doit être en dehors de la plage 1525-2100

# --- Initialisation Pygame ---
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Aucune manette détectée !")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Manette détectée : {joystick.get_name()}")

# Mapping des axes et boutons de la manette (selon votre description)
# joystick gauche: gauche droite = axe 0, haut bas = axe 1
# joystick droit: gauche droite = axe 3, haut bas = axe 4
# bouton L1 = bouton 4 (arm/disarm)
# bouton R1 = bouton 5 (emergency stop)
# bouton X = bouton 2 (non utilisé ici)
# bouton Y = bouton 3 (yaw lock toggle)
# bouton A = bouton 0 (test mode toggle)
# bouton B = bouton 1 (non utilisé ici)

AXIS_YAW = 0        # Joystick gauche horizontal
AXIS_THROTTLE = 1   # Joystick gauche vertical
AXIS_ROLL = 3       # Joystick droit horizontal
AXIS_PITCH = 4      # Joystick droit vertical

BUTTON_ARM_DISARM = 4 # L1
BUTTON_EMERGENCY_STOP = 5 # R1
BUTTON_YAW_LOCK = 3   # Y
BUTTON_TEST_MODE = 0  # A

# --- Initialisation Série ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) # timeout réduit pour ne pas bloquer
    print(f"Port série {SERIAL_PORT} ouvert à {BAUD_RATE} bauds.")
except serial.SerialException as e:
    print(f"Erreur à l'ouverture du port série {SERIAL_PORT}: {e}")
    print("Vérifiez que le port est correct, non utilisé par une autre application (ex: Betaflight Configurator),")
    print("et que vous avez les permissions (ajoutez votre utilisateur au groupe 'dialout': sudo usermod -a -G dialout $USER).")
    pygame.quit()
    sys.exit()

# --- Fonctions MSP ---
MSP_SET_RAW_RC = 200

def send_msp_command(command_id, data):
    payload_size = len(data)
    header = struct.pack('<3sBB', b'$M<', payload_size, command_id)
    payload_format = '<' + 'H' * (payload_size // 2) # H for unsigned short (2 bytes)
    payload_bytes = struct.pack(payload_format, *data)

    checksum = 0
    checksum ^= payload_size
    checksum ^= command_id
    for byte_val in payload_bytes:
        checksum ^= byte_val

    checksum_byte = struct.pack('<B', checksum)
    message = header + payload_bytes + checksum_byte
    
    try:
        ser.write(message)
    except serial.SerialException as e:
        print(f"Erreur d'écriture série: {e}")
        # Potentiellement gérer la réouverture du port ou l'arrêt propre

# --- Variables d'état ---
armed = False
yaw_locked = True # Yaw bloqué par défaut
test_mode_active = False
emergency_stop_engaged = False

# Pour la détection de front des boutons (éviter actions multiples)
prev_button_arm_disarm_state = False
prev_button_yaw_lock_state = False
prev_button_test_mode_state = False

# Initial RC values
rc_channels = [RC_MID] * NUM_RC_CHANNELS
rc_channels[CH_THROTTLE] = RC_MIN
rc_channels[CH_AUX1] = AUX_DISARM_VALUE

# --- Boucle principale ---
running = True
print("\nDébut du contrôle. Appuyez sur L1 pour armer (gaz à zéro!). R1 pour arrêt d'urgence.")
print("Y pour bloquer/débloquer le Yaw. A pour activer/désactiver le mode test (poussée limitée).")

try:
    while running:
        pygame.event.pump() # Important pour que Pygame traite les événements internes

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # Gérer la déconnexion de la manette (optionnel mais recommandé pour la robustesse)
            # if event.type == pygame.JOYDEVICEREMOVED:
            #     print("Manette déconnectée !")
            #     running = False # Ou engager un failsafe

        # --- Lecture des boutons ---
        current_button_arm_disarm_state = joystick.get_button(BUTTON_ARM_DISARM)
        current_button_yaw_lock_state = joystick.get_button(BUTTON_YAW_LOCK)
        current_button_test_mode_state = joystick.get_button(BUTTON_TEST_MODE)
        emergency_stop_pressed = joystick.get_button(BUTTON_EMERGENCY_STOP)

        # --- Logique de contrôle ---

        # 1. Arrêt d'urgence (R1) - Priorité maximale
        if emergency_stop_pressed:
            if not emergency_stop_engaged:
                print("ARRÊT D'URGENCE ENGAGÉ !")
                emergency_stop_engaged = True
            armed = False
            rc_channels[CH_THROTTLE] = RC_MIN
            rc_channels[CH_AUX1] = AUX_DISARM_VALUE
            # Forcer les autres commandes à neutre pour la sécurité
            rc_channels[CH_ROLL] = RC_MID
            rc_channels[CH_PITCH] = RC_MID
            rc_channels[CH_YAW] = RC_MID
            send_msp_command(MSP_SET_RAW_RC, rc_channels[:NUM_RC_CHANNELS])
            time.sleep(0.02) # 50Hz
            continue # Passer le reste de la boucle si arrêt d'urgence
        elif emergency_stop_engaged and not emergency_stop_pressed:
            # L'arrêt d'urgence reste engagé jusqu'à ce que l'utilisateur ré-arme explicitement
            # ou que le script soit redémarré. Pour l'instant, on le désengage si le bouton est relâché
            # MAIS le drone reste désarmé.
            print("Arrêt d'urgence relâché, drone reste DÉSARMÉ.")
            emergency_stop_engaged = False # Permet de reprendre le contrôle normal après relâchement

        # 2. Armement/Désarmement (L1)
        if current_button_arm_disarm_state and not prev_button_arm_disarm_state:
            # Lire la position actuelle du stick des gaz
            throttle_stick_value_raw = joystick.get_axis(AXIS_THROTTLE) # -1 (haut) à 1 (bas)
            # Convertir en % (0% en bas, 100% en haut)
            throttle_stick_percentage = (-throttle_stick_value_raw + 1) / 2 

            if not armed:
                if throttle_stick_percentage < 0.05: # Gaz à moins de 5% pour armer
                    armed = True
                    rc_channels[CH_AUX1] = AUX_ARM_VALUE
                    print("DRONE ARMÉ")
                else:
                    print("Impossible d'armer : Gaz non à zéro !")
            else:
                armed = False
                rc_channels[CH_AUX1] = AUX_DISARM_VALUE
                print("DRONE DÉSARMÉ")
        prev_button_arm_disarm_state = current_button_arm_disarm_state

        # 3. Yaw Lock (Y)
        if current_button_yaw_lock_state and not prev_button_yaw_lock_state:
            yaw_locked = not yaw_locked
            print(f"Yaw Lock : {'Activé' if yaw_locked else 'Désactivé'}")
        prev_button_yaw_lock_state = current_button_yaw_lock_state

        # 4. Test Mode (A)
        if current_button_test_mode_state and not prev_button_test_mode_state:
            test_mode_active = not test_mode_active
            print(f"Mode Test (Poussée limitée à {THROTTLE_MAX_TEST_MODE}µs) : {'Activé' if test_mode_active else 'Désactivé'}")
        prev_button_test_mode_state = current_button_test_mode_state
        
        # --- Lecture des Axes ---
        # Joystick gauche vertical (Throttle): axe 1. Manette -1 (haut) à +1 (bas)
        # Inverser et mapper: -1 (haut manette) -> RC_MAX, +1 (bas manette) -> RC_MIN
        raw_throttle = joystick.get_axis(AXIS_THROTTLE)
        rc_throttle = int(RC_MIN + ((-raw_throttle + 1) / 2) * (RC_MAX - RC_MIN))

        # Joystick gauche horizontal (Yaw): axe 0. Manette -1 (gauche) à +1 (droite)
        # Mapper: -1 -> RC_MIN, 0 -> RC_MID, +1 -> RC_MAX
        raw_yaw = joystick.get_axis(AXIS_YAW)
        rc_yaw = int(RC_MID + (raw_yaw / 2) * (RC_MAX - RC_MIN)) # /2 car l'amplitude est de RC_MID à RC_MAX ou RC_MIN

        # Joystick droit vertical (Pitch): axe 4. Manette -1 (haut) à +1 (bas)
        # Stick haut = drone avance (pique du nez). Betaflight: valeur RC plus élevée pour Pitch = cabrer.
        # Donc, -1 (haut manette) -> RC_MAX (pitch arrière/cabrer), +1 (bas manette) -> RC_MIN (pitch avant/piquer)
        # Si on veut stick haut = piquer du nez (plus intuitif pour certains):
        # -1 (haut manette) -> RC_MIN (piquer), +1 (bas manette) -> RC_MAX (cabrer)
        # Je vais utiliser: stick haut = piquer du nez (valeur RC basse)
        raw_pitch = joystick.get_axis(AXIS_PITCH)
        # rc_pitch = int(RC_MID + ((-raw_pitch / 2) * (RC_MAX - RC_MIN))) # Stick haut = RC_MAX (cabrer)
        rc_pitch = int(RC_MID + ((raw_pitch / 2) * (RC_MAX - RC_MIN))) # Stick haut = RC_MIN (piquer)
                                                                      # Correction: raw_pitch est -1 (haut) à 1 (bas)
                                                                      # (-raw_pitch + 1)/2 -> 1 (haut) à 0 (bas)
                                                                      # Donc pour stick haut = piquer (RC_MIN)
                                                                      # et stick bas = cabrer (RC_MAX)
                                                                      # rc_pitch = int(RC_MIN + ((-raw_pitch + 1) / 2) * (RC_MAX - RC_MIN))
                                                                      # Non, c'est l'inverse.
                                                                      # Si raw_pitch = -1 (haut), on veut RC_MIN (ou proche de RC_MIN pour piquer)
                                                                      # Si raw_pitch = 1 (bas), on veut RC_MAX (ou proche de RC_MAX pour cabrer)
                                                                      # Donc: (raw_pitch + 1)/2 -> 0 (haut) à 1 (bas)
                                                                      # rc_pitch = int(RC_MIN + ((raw_pitch + 1) / 2) * (RC_MAX - RC_MIN))
                                                                      # Testons: raw_pitch = -1 (haut) => (0/2) * (delta) + RC_MIN = RC_MIN (piquer)
                                                                      #          raw_pitch =  1 (bas) => (2/2) * (delta) + RC_MIN = RC_MAX (cabrer)
                                                                      #          raw_pitch =  0 (milieu) => (1/2) * (delta) + RC_MIN = RC_MID
        rc_pitch = int(RC_MIN + ((raw_pitch + 1) / 2) * (RC_MAX - RC_MIN))


        # Joystick droit horizontal (Roll): axe 3. Manette -1 (gauche) à +1 (droite)
        # Mapper: -1 -> RC_MIN, 0 -> RC_MID, +1 -> RC_MAX
        raw_roll = joystick.get_axis(AXIS_ROLL)
        rc_roll = int(RC_MID + (raw_roll / 2) * (RC_MAX - RC_MIN))

        # --- Application des logiques spécifiques ---
        if yaw_locked:
            rc_yaw = RC_MID

        if not armed:
            rc_throttle = RC_MIN # Sécurité: si désarmé, gaz au minimum
        else:
            # S'assurer que les gaz ne descendent pas en dessous de min_throttle quand armé
            # (sauf si on veut couper les moteurs en vol, ce qui est géré par le désarmement)
            if rc_throttle < THROTTLE_MIN_ARMED :
                 rc_throttle = THROTTLE_MIN_ARMED
            # Appliquer la limite du mode test si actif
            if test_mode_active:
                rc_throttle = min(rc_throttle, THROTTLE_MAX_TEST_MODE)


        # --- Assemblage des canaux RC ---
        rc_channels[CH_ROLL] = rc_roll
        rc_channels[CH_PITCH] = rc_pitch
        rc_channels[CH_THROTTLE] = rc_throttle
        rc_channels[CH_YAW] = rc_yaw
        # rc_channels[CH_AUX1] est déjà géré par l'armement/désarmement
        rc_channels[CH_AUX2] = RC_MID # Pas de mode de vol actif sur AUX2 pour l'instant
        rc_channels[CH_AUX3] = RC_MID
        rc_channels[CH_AUX4] = RC_MID
        
        # --- Envoi de la commande MSP ---
        send_msp_command(MSP_SET_RAW_RC, rc_channels[:NUM_RC_CHANNELS])

        # Affichage des valeurs (optionnel, pour le debug)
        # print(f"Armed: {armed} | YawLock: {yaw_locked} | TestMode: {test_mode_active} | EStop: {emergency_stop_engaged}")
        # print(f"R:{rc_roll} P:{rc_pitch} T:{rc_throttle} Y:{rc_yaw} A1:{rc_channels[CH_AUX1]}")

        time.sleep(0.02) # Boucle à environ 50Hz

except KeyboardInterrupt:
    print("\nInterruption clavier, désarmement et fermeture...")
except Exception as e:
    print(f"\nUne erreur est survenue: {e}")
finally:
    # Action de sécurité à la fermeture : envoyer une commande de désarmement
    print("Envoi de la commande de désarmement finale.")
    final_rc_values = [RC_MID] * NUM_RC_CHANNELS
    final_rc_values[CH_THROTTLE] = RC_MIN
    final_rc_values[CH_AUX1] = AUX_DISARM_VALUE
    if 'ser' in locals() and ser.is_open:
        send_msp_command(MSP_SET_RAW_RC, final_rc_values[:NUM_RC_CHANNELS])
        time.sleep(0.1) # Laisser le temps à la commande de passer
        ser.close()
        print("Port série fermé.")
    pygame.quit()
    print("Pygame quitté.")
    print("Script terminé.")