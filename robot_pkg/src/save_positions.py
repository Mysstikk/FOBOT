import os
import time
import math
from dynamixel_sdk import *

# --- Configuración de Control de Tabla (Serie X) ---
ADDR_TORQUE_ENABLE      = 64               
ADDR_PRESENT_POSITION   = 132              

# --- Configuración de la Red de Motores ---
PROTOCOL_VERSION        = 2.0
DXL_IDS                 = [1, 2, 3, 4, 5, 6]
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyUSB0'

TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0
TICKS_PER_REV           = 4096.0 # Resolución de encoder para XM430, XM540, XC330
# ---------------------------------------

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def setup():
    if not portHandler.openPort():
        print("Error: No se pudo abrir el puerto.")
        quit()
    if not portHandler.setBaudRate(BAUDRATE):
        print("Error: No se pudo configurar el baudrate.")
        quit()

def set_torque_all(enable):
    estado_texto = "BLOQUEADO" if enable == TORQUE_ENABLE else "DESBLOQUEADO"
    print(f"\nCambiando estado a {estado_texto}...")
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, enable)

def to_signed_32(value):
    """Convierte un entero sin signo de 32 bits a un entero con signo."""
    if value > 2147483647:
        return value - 4294967296
    return value

def ticks_a_radianes(ticks):
    """
    Convierte ticks a radianes. 
    Asume que la posición central (2048) equivale a 0 radianes.
    """
    return (ticks) * (2.0 * math.pi / TICKS_PER_REV)

def read_all_positions_rad():
    """Lee las posiciones de todos los motores y devuelve una lista en radianes."""
    posiciones_rad = []
    
    for dxl_id in DXL_IDS:
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
        
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f"[ID:{dxl_id}] Error en la lectura.")
            posiciones_rad.append(None)
        else:
            # 1. Recuperamos el valor real con signo (ej: 4294967265 -> -31)
            signed_ticks = to_signed_32(dxl_present_position)
            
            # 2. Convertimos a radianes
            rad = ticks_a_radianes(signed_ticks)
            
            posiciones_rad.append(round(rad, 4))
            
    return posiciones_rad

def main():
    setup()
    set_torque_all(TORQUE_DISABLE)
    
    print("\n" + "="*50)
    print(" MODO DE GRABACIÓN MÚLTIPLE (EN RADIANES)")
    print("="*50)

    nombre_archivo = "/home/gromep/robot_ws/src/robot_pkg/src/trayectoria_dynamixel_rad.csv"

    if not os.path.exists(nombre_archivo):
        '''with open(nombre_archivo, "w") as f:
            header = ",".join([f"ID_{i}_rad" for i in DXL_IDS])
            f.write(header + "\n")'''

    with open(nombre_archivo, "w") as f:
        captura = 1
        while True:
            comando = input(f"Captura #{captura} (Enter para grabar, 'q' para salir): ")
            if comando.lower() == 'q':
                break
            
            pos_rad = read_all_positions_rad()
            
            if None not in pos_rad:
                linea_csv = ",".join(map(str, pos_rad))
                f.write(linea_csv + "\n")
                print(f" > Captura #{captura}: [{linea_csv}]")
                captura += 1
            else:
                print(" > Error de lectura. Intenta de nuevo.")
    
    set_torque_all(TORQUE_ENABLE)
    portHandler.closePort()
    print(f"\nArchivo guardado: '{nombre_archivo}'")

if __name__ == '__main__':
    main()
