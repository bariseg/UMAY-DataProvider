import time
import math
import serial
import struct
import random
import serial.tools.list_ports
import telemetry_pb2
from pymavlink import mavutil

# --- AYARLAR ---
SIMULATION_MODE = False  # <-- DİKKAT: Gerçek uçuş için bunu False yapın!
XBEE_BAUD_RATE = 9600
PIXHAWK_BAUD_RATE = 57600
PIXHAWK_PORT_STR = 'COM9'  # Gerçek bağlantı için portu kontrol edin

# Simülasyon için başlangıç merkezi (Gebze Teknik Üni civarı)
SIM_CENTER_LAT = 40.8085
SIM_CENTER_LON = 29.3562

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]

def request_message_interval(master, message_id, frequency_hz):
    """Pixhawk'tan belirli mesajları belirli sıklıkla ister."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

def get_flight_mode(master):
    """Mevcut uçuş modunu string olarak döndürür."""
    try:
        # master.flightmode özelliği pymavlink tarafından heartbeat geldikçe güncellenir
        return master.flightmode if master.flightmode else "UNKNOWN"
    except:
        return "UNKNOWN"

def generate_simulation_data(state_cache, elapsed_time):
    """
    Cube Orange yokken verileri matematiksel olarak üretir.
    Dairesel bir rota çizer ve sensör gürültüsü ekler.
    """
    # 1. Konum (Dairesel Hareket)
    radius = 0.003  # Yaklaşık 300-400 metre yarıçap
    speed_factor = 0.1
    state_cache['lat'] = SIM_CENTER_LAT + math.sin(elapsed_time * speed_factor) * radius
    state_cache['lon'] = SIM_CENTER_LON + math.cos(elapsed_time * speed_factor) * radius
    state_cache['alt'] = 100 + math.sin(elapsed_time * 0.2) * 10  # 90m ile 110m arası dalgalanma

    # 2. Yön ve Hız
    # Heading, dairesel hareketin teğeti
    angle = (elapsed_time * speed_factor) % (2 * math.pi)
    state_cache['heading'] = (math.degrees(angle) + 90) % 360
    state_cache['speed'] = 12.0 + random.uniform(-0.5, 0.5)
    state_cache['climb'] = math.cos(elapsed_time * 0.2) * 2  # Tırmanma hızı

    # 3. Batarya ve Akım
    # Batarya zamanla azalır
    battery_drain = (elapsed_time / 3600) * 20 # Saatte 20V düşüş (abartı ama görülsün diye)
    state_cache['voltage'] = max(14.0, 24.0 - battery_drain) # 6S batarya simülasyonu
    state_cache['current'] = 15.0 + random.uniform(-2, 5) # 15 Amper civarı
    state_cache['battery_remaining'] = max(0, 100 - (elapsed_time / 10)) # %

    # 4. IMU ve Titreşim
    state_cache['roll'] = math.sin(elapsed_time) * 5 + random.uniform(-0.5, 0.5)
    state_cache['pitch'] = math.cos(elapsed_time) * 2 + random.uniform(-0.5, 0.5)
    state_cache['vib_x'] = random.uniform(0, 2.5)
    state_cache['vib_y'] = random.uniform(0, 2.5)
    state_cache['vib_z'] = random.uniform(5, 15.0) # Z ekseni yerçekimi + titreşim

    # 5. GPS Durumu
    state_cache['satellites'] = 14
    state_cache['hdop'] = 0.7 + random.uniform(0, 0.1)
    state_cache['fix_type'] = 3 # 3D Fix

    # 6. Mod
    modes = ["LOITER", "AUTO", "GUIDED"]
    # Her 20 saniyede bir mod değiştiriyormuş gibi yap
    mode_index = int(elapsed_time / 20) % len(modes)
    state_cache['flight_mode'] = modes[mode_index]

def connect_pixhawk(port):
    print(f"Pixhawk aranıyor: {port}...")
    try:
        master = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD_RATE, autoreconnect=True)
        master.wait_heartbeat()
        print("Pixhawk Bağlandı!")
        
        # Gerekli mesajları talep et
        msgs = [
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, # Konum, Hız
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,            # Roll, Pitch
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,             # Airspeed, Heading, Altitude
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,          # Voltaj, Akım, Batarya %
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,         # Uydu sayısı, Fix type, HDOP
            mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION            # Titreşim verileri
        ]
        for msg_id in msgs:
            request_message_interval(master, msg_id, 4) # 4 Hz
            
        return master
    except Exception as e:
        print(f"Pixhawk bağlantı hatası: {e}")
        return None

def main():
    # XBee Port Seçimi
    available_ports = list_serial_ports()
    if not available_ports:
        print("HATA: Port bulunamadı!")
        # Simülasyon modundaysak port olmasa da çalışsın mı? 
        # Hayır, veriyi gönderecek bir yer lazım. Ama test için 'COM1' sallayabiliriz.
        if not SIMULATION_MODE: return
        xbee_port = "TEST_LOOPBACK"
    else:
        # Pixhawk portu haricindekini bulmaya çalış
        if SIMULATION_MODE:
             # Simülasyonda Pixhawk portunu elemeye gerek yok, XBee'yi seç
             xbee_port = available_ports[0] 
        else:
             xbee_port = next((p for p in available_ports if p != PIXHAWK_PORT_STR), available_ports[0])

    xbee_port = "COM5"
    print(f"Mod: {'SİMÜLASYON' if SIMULATION_MODE else 'GERÇEK VERİ'}")
    print(f"XBee Hedef: {xbee_port}")

    # Veri Önbelleği (Tüm proto alanları için)
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
        # Bağlantılar
        if not xbee_port == "TEST_LOOPBACK":
            ser_xbee = serial.Serial(xbee_port, XBEE_BAUD_RATE, timeout=1)
        
        if not SIMULATION_MODE:
            mav_master = connect_pixhawk(PIXHAWK_PORT_STR)
            if not mav_master: return

        print("Sistem Başladı. Veri akışı yapılıyor...\n")
        start_time = time.time()
        last_send_time = time.time()

        while True:
            now = time.time()

            # --- VERİ TOPLAMA (SİMÜLASYON veya GERÇEK) ---
            if SIMULATION_MODE:
                # Simülasyon fonksiyonunu çağır
                generate_simulation_data(current_data, now - start_time)
                time.sleep(0.05) # Döngüyü çok yormamak için
            
            else:
                # Gerçek MAVLink okuma
                msg = mav_master.recv_match(blocking=False)
                if msg:
                    msg_type = msg.get_type()

                    if msg_type == 'GLOBAL_POSITION_INT':
                        current_data['lat'] = msg.lat / 1e7
                        current_data['lon'] = msg.lon / 1e7
                        current_data['alt'] = msg.alt / 1000.0 # MSL
                        current_data['heading'] = msg.hdg / 100.0
                        current_data['climb'] = msg.vz / 100.0 # dikey hız cm/s -> m/s (ters olabilir, kontrol et)
                        # Not: MAVLink'te pozitif vz aşağı yönlü olabilir, VFR_HUD climb daha güvenilirdir.

                    elif msg_type == 'VFR_HUD':
                        current_data['speed'] = msg.groundspeed
                        current_data['climb'] = msg.climb # m/s

                    elif msg_type == 'ATTITUDE':
                        current_data['roll'] = math.degrees(msg.roll)
                        current_data['pitch'] = math.degrees(msg.pitch)

                    elif msg_type == 'SYS_STATUS':
                        current_data['voltage'] = msg.voltage_battery / 1000.0 # mV -> V
                        current_data['current'] = msg.current_battery / 100.0  # cA -> A
                        current_data['battery_remaining'] = msg.battery_remaining # %

                    elif msg_type == 'GPS_RAW_INT':
                        current_data['fix_type'] = msg.fix_type
                        current_data['satellites'] = msg.satellites_visible
                        current_data['hdop'] = msg.eph / 100.0 # Genellikle cm gelir -> metre

                    elif msg_type == 'VIBRATION':
                        current_data['vib_x'] = msg.vibration_x
                        current_data['vib_y'] = msg.vibration_y
                        current_data['vib_z'] = msg.vibration_z

                    # Uçuş modunu heartbeat'ten otomatik alıyoruz
                    current_data['flight_mode'] = get_flight_mode(mav_master)

            if now - last_send_time > 0.5:
                flight_data = telemetry_pb2.FlightData()
                
                # Temel Veriler
                flight_data.latitude = current_data['lat']
                flight_data.longitude = current_data['lon']
                flight_data.altitude = current_data['alt']
                flight_data.speed = current_data['speed']
                flight_data.heading = current_data['heading']
                flight_data.battery = current_data['battery_remaining'] # Proto'da battery % olarak düşündüm
                flight_data.timestamp = int(now * 1000)
                flight_data.roll = current_data['roll']
                flight_data.pitch = current_data['pitch']

                # EKSTRA VERİLER (Proto dosyasındaki yeniler)
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

                # Target (Görüntü işleme hedefleri)
                # Burası şimdilik boş, ileride kamera verisiyle doldurulabilir
                # flight_data.targets... 

                # Paketleme
                serialized_data = flight_data.SerializeToString()
                length_prefix = struct.pack('>I', len(serialized_data))
                
                if ser_xbee and ser_xbee.is_open:
                    ser_xbee.write(length_prefix + serialized_data)
                
                # Bilgi Ekranı
                mode_str = "SIM" if SIMULATION_MODE else "REAL"
                """ print(
                    f"[{mode_str}] Mod:{flight_data.flight_mode} "
                    f"Sat:{flight_data.gps_satellites} "
                    f"Bat:{flight_data.voltage:.1f}V "
                    f"VibZ:{flight_data.vibration_z:.1f} "
                    f"Lat:{flight_data.latitude:.4f}"
                ) """
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