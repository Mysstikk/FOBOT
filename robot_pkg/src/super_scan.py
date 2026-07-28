import os
from dynamixel_sdk import * # Importar la librería del SDK

# ================= CONFIGURACIÓN =================
# Cambia esto por tu puerto (ej: 'COM3' en Windows o '/dev/ttyUSB0' en Linux)
DEVICENAME = '/dev/ttyUSB0'  

# Configuración específica para tus AX-12A
BAUDRATE = 1000000         # 1 Mbps
PROTOCOL_VERSION = 2.0     # ¡CRUCIAL! AX-12A usa Protocolo 1.0

# Rango de escaneo (buscará del ID 0 al 25)
# Como tienes IDs 17 y 18, necesitamos llegar al menos hasta el 20.
MAX_ID_SCAN = 25 
# =================================================

def scan_motors():
    print("--------------------------------------------------")
    print(f"Iniciando escaneo en {DEVICENAME} a {BAUDRATE} baudios")
    print(f"Usando PROTOCOLO {PROTOCOL_VERSION}")
    print("--------------------------------------------------")

    # 1. Inicializar PortHandler y PacketHandler
    # Aquí es donde le decimos al código que use el Protocolo 1.0
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # 2. Abrir el puerto
    if portHandler.openPort():
        print(f"[OK] Puerto {DEVICENAME} abierto correctamente.")
    else:
        print(f"[ERROR] No se pudo abrir el puerto {DEVICENAME}.")
        print("Asegúrate de que el Dynamixel Wizard esté CERRADO.")
        return

    # 3. Configurar el Baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print(f"[OK] Baudrate cambiado a {BAUDRATE}.")
    else:
        print("[ERROR] No se pudo cambiar el baudrate.")
        portHandler.closePort()
        return

    print("\nBuscando motores... (Esto puede tardar unos segundos)\n")
    
    found_count = 0

    # 4. Bucle de escaneo (Ping)
    for dxl_id in range(MAX_ID_SCAN):
        # Intentamos hacer PING al ID actual
        # ping() devuelve: numero_modelo, resultado_com, error
        model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)

        if dxl_comm_result == COMM_SUCCESS:
            print(f" -> ¡MOTOR ENCONTRADO! ID: {dxl_id} (Modelo: {model_number})")
            found_count += 1
        elif dxl_comm_result != COMM_SUCCESS:
            # Si quieres ver el intento fallido, descomenta la línea de abajo:
            # print(f". ID {dxl_id} sin respuesta")
            pass

    print("\n--------------------------------------------------")
    print(f"Escaneo finalizado. Se encontraron {found_count} motores.")
    print("--------------------------------------------------")

    # 5. Cerrar puerto
    portHandler.closePort()

if __name__ == '__main__':
    scan_motors()
