import time
import math
import serial
import struct
import random
import serial.tools.list_ports
import telemetry_pb2
import platform # İşletim sistemini anlamak için
from pymavlink import mavutil

# --- AYARLAR ---
SIMULATION_MODE = False  
XBEE_BAUD_RATE = 9600
PIXHAWK_BAUD_RATE = 57600

# İşletim Sistemine Göre Port Tahmini
system_os = platform.system()

if system_os == "Linux":
    # Raspberry Pi Varsayılanları
    PIXHAWK_PORT_STR = '/dev/ttyACM0' 
    DEFAULT_XBEE_PORT = '/dev/ttyUSB0'
else:
    # Windows Varsayılanları
    PIXHAWK_PORT_STR = 'COM9'
    DEFAULT_XBEE_PORT = 'COM5'

# Simülasyon Merkezi
SIM_CENTER_LAT = 40.8085
SIM_CENTER_LON = 29.3562

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]

def request_message_interval(master, message_id, frequency_hz):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

def get_flight_mode(master):
    try:
        return master.flightmode if master.flightmode else "UNKNOWN"
    except:
        return "UNKNOWN"

def generate_simulation_data(state_cache, elapsed_time):
    # ... (Simülasyon fonksiyonu aynı kalıyor) ...
    radius = 0.003
    speed_factor = 0.1
    state_cache['lat'] = SIM_CENTER_LAT + math.sin(elapsed_time * speed_factor) * radius
    state_cache['lon'] = SIM_CENTER_LON + math.cos(elapsed_time * speed_factor) * radius
    state_cache['alt'] = 100 + math.sin(elapsed_time * 0.2) * 10

    angle = (elapsed_time * speed_factor) % (2 * math.pi)
    state_cache['heading'] = (math.degrees(angle) + 90) % 360
    state_cache['speed'] = 12.0 + random.uniform(-0.5, 0.5)
    state_cache['climb'] = math.cos(elapsed_time * 0.2) * 2

    battery_drain = (elapsed_time / 3600) * 20
    state_cache['voltage'] = max(14.0, 24.0 - battery_drain)
    state_cache['current'] = 15.0 + random.uniform(-2, 5)
    state_cache['battery_remaining'] = max(0, 100 - (elapsed_time / 10))

    state_cache['roll'] = math.sin(elapsed_time) * 5 + random.uniform(-0.5, 0.5)
    state_cache['pitch'] = math.cos(elapsed_time) * 2 + random.uniform(-0.5, 0.5)
    state_cache['vib_x'] = random.uniform(0, 2.5)
    state_cache['vib_y'] = random.uniform(0, 2.5)
    state_cache['vib_z'] = random.uniform(5, 15.0)

    state_cache['satellites'] = 14
    state_cache['hdop'] = 0.7 + random.uniform(0, 0.1)
    state_cache['fix_type'] = 3
    
    modes = ["LOITER", "AUTO", "GUIDED"]
    mode_index = int(elapsed_time / 20) % len(modes)
    state_cache['flight_mode'] = modes[mode_index]

def connect_pixhawk(port):
    print(f"Pixhawk aranıyor: {port}...")
    try:
        master = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD_RATE, autoreconnect=True)
        master.wait_heartbeat()
        print("Pixhawk Bağlandı!")
        
        msgs = [
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION
        ]
        for msg_id in msgs:
            request_message_interval(master, msg_id, 4)
            
        return master
    except Exception as e:
        print(f"Pixhawk bağlantı hatası: {e}")
        return None

def main():
    # XBee Port Seçimi (Otomatik veya Varsayılan)
    available_ports = list_serial_ports()
    xbee_port = None

    if not available_ports:
        print("UYARI: Port bulunamadı!")
        if SIMULATION_MODE:
            xbee_port = "TEST_LOOPBACK"
    else:
        # Pixhawk portu haricindeki ilk portu XBee varsayalım
        # Eğer listede varsa öncelikli olarak DEFAULT_XBEE_PORT'u ara
        if DEFAULT_XBEE_PORT in available_ports:
            xbee_port = DEFAULT_XBEE_PORT
        else:
            # Bulamazsa Pixhawk olmayan ilk cihazı al
            xbee_port = next((p for p in available_ports if p != PIXHAWK_PORT_STR), available_ports[0])

    # Eğer port hala bulunamadıysa varsayılanı kullan (Riskli ama dener)
    if xbee_port is None and not SIMULATION_MODE:
        xbee_port = DEFAULT_XBEE_PORT

    print(f"Sistem: {system_os}")
    print(f"Mod: {'SİMÜLASYON' if SIMULATION_MODE else 'GERÇEK VERİ'}")
    print(f"XBee Hedef: {xbee_port}")
    print(f"Pixhawk Hedef: {PIXHAWK_PORT_STR}")

    current_data = {
        'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
        'speed': 0.0, 'heading': 0.0,
        'battery_remaining': 0.0, 'voltage': 0.0, 'current': 0.0,
        'roll': 0.0, 'pitch': 0.0,
        'climb': 0.0,
        'satellites': 0, 'hdop': 99.9, 'fix_type': 0,
        'flight_mode': 'MANUAL',
        'vib_x': 0.0, 'vib_y': 0.0, 'vib_z': 0.0
    }

    mav_master = None
    ser_xbee = None

    try:
        if xbee_port and xbee_port != "TEST_LOOPBACK":
            # XBee bağlantısını denerken hata olursa program çökmesin
            try:
                ser_xbee = serial.Serial(xbee_port, XBEE_BAUD_RATE, timeout=1)
            except Exception as e:
                print(f"XBee Bağlantı Hatası: {e}")
        
        if not SIMULATION_MODE:
            mav_master = connect_pixhawk(PIXHAWK_PORT_STR)
            if not mav_master: 
                print("Pixhawk bağlanamadı, tekrar deneniyor...")
                # Döngü içinde tekrar dener, burada çıkış yapmıyoruz

        print("Sistem Başladı. Veri akışı yapılıyor...\n")
        start_time = time.time()
        last_send_time = time.time()

        while True:
            now = time.time()

            if SIMULATION_MODE:
                generate_simulation_data(current_data, now - start_time)
                time.sleep(0.05)
            
            elif mav_master:
                msg = mav_master.recv_match(blocking=False)
                if msg:
                    msg_type = msg.get_type()
                    # ... (Veri işleme blokları aynı) ...
                    if msg_type == 'GLOBAL_POSITION_INT':
                        current_data['lat'] = msg.lat / 1e7
                        current_data['lon'] = msg.lon / 1e7
                        current_data['alt'] = msg.alt / 1000.0
                        current_data['heading'] = msg.hdg / 100.0
                        current_data['climb'] = msg.vz / 100.0

                    elif msg_type == 'VFR_HUD':
                        current_data['speed'] = msg.groundspeed
                        current_data['climb'] = msg.climb

                    elif msg_type == 'ATTITUDE':
                        current_data['roll'] = math.degrees(msg.roll)
                        current_data['pitch'] = math.degrees(msg.pitch)

                    elif msg_type == 'SYS_STATUS':
                        current_data['voltage'] = msg.voltage_battery / 1000.0
                        current_data['current'] = msg.current_battery / 100.0
                        current_data['battery_remaining'] = msg.battery_remaining

                    elif msg_type == 'GPS_RAW_INT':
                        current_data['fix_type'] = msg.fix_type
                        current_data['satellites'] = msg.satellites_visible
                        current_data['hdop'] = msg.eph / 100.0

                    elif msg_type == 'VIBRATION':
                        current_data['vib_x'] = msg.vibration_x
                        current_data['vib_y'] = msg.vibration_y
                        current_data['vib_z'] = msg.vibration_z

                    current_data['flight_mode'] = get_flight_mode(mav_master)

            if now - last_send_time > 0.5:
                flight_data = telemetry_pb2.FlightData()
                
                flight_data.latitude = current_data['lat']
                flight_data.longitude = current_data['lon']
                flight_data.altitude = current_data['alt']
                flight_data.speed = current_data['speed']
                flight_data.heading = current_data['heading']
                flight_data.battery = current_data['battery_remaining']
                flight_data.timestamp = int(now * 1000)
                flight_data.roll = current_data['roll']
                flight_data.pitch = current_data['pitch']
                flight_data.vertical_speed = current_data['climb']
                flight_data.current = current_data['current']
                flight_data.voltage = current_data['voltage']
                flight_data.gps_satellites = int(current_data['satellites'])
                flight_data.gps_hdop = current_data['hdop']
                flight_data.gps_fix_type = int(current_data['fix_type'])
                flight_data.flight_mode = str(current_data['flight_mode'])
                flight_data.vibration_x = current_data['vib_x']
                flight_data.vibration_y = current_data['vib_y']
                flight_data.vibration_z = current_data['vib_z']

                serialized_data = flight_data.SerializeToString()
                length_prefix = struct.pack('>I', len(serialized_data))
                
                if ser_xbee and ser_xbee.is_open:
                    ser_xbee.write(length_prefix + serialized_data)
                
                print(flight_data)
                last_send_time = time.time()

    except KeyboardInterrupt:
        print("\nÇıkış yapılıyor...")
        if ser_xbee and ser_xbee.is_open: ser_xbee.close()
        if mav_master: mav_master.close()
    except Exception as e:
        print(f"Genel Hata: {e}")

if __name__ == "__main__":
    main()